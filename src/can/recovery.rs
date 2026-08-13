use super::protocol::{
    CommandPhase, CommandReason, CommandResult, RecoveryLogData, RecoveryOpcode, RecoverySource,
    RecoveryStatus, RecoveryStatusCode,
};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PendingRecovery {
    pub transaction_id: u8,
    pub command: u8,
    pub opcode: RecoveryOpcode,
    pub source: RecoverySource,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RecoverySession {
    pending: Option<PendingRecovery>,
}

impl RecoverySession {
    pub const fn new() -> Self {
        Self { pending: None }
    }

    pub fn start(&mut self, pending: PendingRecovery) -> bool {
        if self.pending.is_some() {
            false
        } else {
            self.pending = Some(pending);
            true
        }
    }

    pub fn cancel(&mut self, transaction_id: u8) {
        if self
            .pending
            .is_some_and(|pending| pending.transaction_id == transaction_id)
        {
            self.pending = None;
        }
    }

    pub fn apply_status(&mut self, status: RecoveryStatus) -> Option<CommandResult> {
        let pending = self.pending?;
        if status.transfer_id != pending.transaction_id
            || status.source != pending.source
            || status.opcode != pending.opcode
        {
            return None;
        }
        let (phase, reason, final_result) = match status.status {
            RecoveryStatusCode::Dumping => (CommandPhase::Accepted, CommandReason::None, false),
            RecoveryStatusCode::Ready | RecoveryStatusCode::Complete => {
                (CommandPhase::Completed, CommandReason::None, true)
            }
            RecoveryStatusCode::Aborted if pending.opcode == RecoveryOpcode::StopLogDump => {
                (CommandPhase::Completed, CommandReason::None, true)
            }
            RecoveryStatusCode::Busy => (CommandPhase::Rejected, CommandReason::Busy, true),
            RecoveryStatusCode::InvalidState => {
                (CommandPhase::Rejected, CommandReason::InvalidState, true)
            }
            RecoveryStatusCode::InvalidArgument => {
                (CommandPhase::Rejected, CommandReason::InvalidArgument, true)
            }
            RecoveryStatusCode::SourceUnavailable => {
                (CommandPhase::Failed, CommandReason::DeviceUnavailable, true)
            }
            RecoveryStatusCode::IoError => {
                (CommandPhase::Failed, CommandReason::PersistenceError, true)
            }
            RecoveryStatusCode::Aborted => (
                CommandPhase::Failed,
                CommandReason::InterruptedByEmergency,
                true,
            ),
            RecoveryStatusCode::InternalError => {
                (CommandPhase::Failed, CommandReason::InternalError, true)
            }
        };
        if final_result {
            self.pending = None;
        }
        Some(CommandResult {
            transaction_id: pending.transaction_id,
            command: pending.command,
            phase,
            reason,
            detail: status.total_size,
        })
    }

    pub const fn active_source(&self) -> Option<RecoverySource> {
        match self.pending {
            Some(pending) => Some(pending.source),
            None => None,
        }
    }

    pub fn fail(&mut self, reason: CommandReason, detail: u32) -> Option<CommandResult> {
        let pending = self.pending.take()?;
        Some(CommandResult {
            transaction_id: pending.transaction_id,
            command: pending.command,
            phase: CommandPhase::Failed,
            reason,
            detail,
        })
    }

    pub fn fail_matching(
        &mut self,
        transaction_id: u8,
        reason: CommandReason,
        detail: u32,
    ) -> Option<CommandResult> {
        if self
            .pending
            .is_none_or(|pending| pending.transaction_id != transaction_id)
        {
            return None;
        }
        self.fail(reason, detail)
    }

    pub fn interrupt_with_stop(&mut self, pending: PendingRecovery) -> Option<CommandResult> {
        let interrupted = self.fail(CommandReason::InterruptedByEmergency, 0);
        self.pending = Some(pending);
        interrupted
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RecoveryChunk {
    pub transfer_id: u8,
    pub source: RecoverySource,
    pub offset: u32,
    pub data_length: u8,
    pub data: [u8; 16],
    pub end_of_file: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RecoveryResume {
    pub transfer_id: u8,
    pub source: RecoverySource,
    pub offset: u32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RecoveryAssemblyError {
    NotStarted,
    WrongTransfer,
    SequenceGap,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RecoveryAssembler {
    active: bool,
    transfer_id: u8,
    source: RecoverySource,
    offset: u32,
    total_size: Option<u32>,
    expected_sequence: Option<u8>,
    data: [u8; 16],
    data_length: u8,
}

impl RecoveryAssembler {
    pub const fn new() -> Self {
        Self {
            active: false,
            transfer_id: 0,
            source: RecoverySource::InternalFlash,
            offset: 0,
            total_size: None,
            expected_sequence: None,
            data: [0; 16],
            data_length: 0,
        }
    }

    pub fn start(
        &mut self,
        transfer_id: u8,
        source: RecoverySource,
        offset: u32,
        requested_length: u32,
    ) {
        *self = Self {
            active: true,
            transfer_id,
            source,
            offset,
            total_size: (requested_length != 0).then_some(offset.saturating_add(requested_length)),
            expected_sequence: None,
            data: [0; 16],
            data_length: 0,
        };
    }

    pub fn set_total_size(&mut self, transfer_id: u8, total_size: u32) {
        if self.active && self.transfer_id == transfer_id && total_size != 0 {
            self.total_size = Some(
                self.total_size
                    .map_or(total_size, |end| end.min(total_size)),
            );
        }
    }

    pub fn stop(&mut self) -> Option<RecoveryChunk> {
        if !self.active {
            return None;
        }
        self.finish(self.transfer_id)
    }

    pub fn push(
        &mut self,
        fragment: RecoveryLogData,
    ) -> Result<Option<RecoveryChunk>, RecoveryAssemblyError> {
        if !self.active {
            return Err(RecoveryAssemblyError::NotStarted);
        }
        if fragment.transfer_id != self.transfer_id {
            return Err(RecoveryAssemblyError::WrongTransfer);
        }
        if self
            .expected_sequence
            .is_some_and(|expected| fragment.sequence != expected)
        {
            let expected = self.expected_sequence.unwrap();
            let missed = fragment.sequence.wrapping_sub(expected);
            self.offset = self
                .offset
                .saturating_add(u32::from(self.data_length))
                .saturating_add(u32::from(missed) * 6);
            self.data = [0; 16];
            self.data_length = 0;
            self.active = false;
            return Err(RecoveryAssemblyError::SequenceGap);
        }
        self.expected_sequence = Some(fragment.sequence.wrapping_add(1));
        let fragment_length = self.total_size.map_or(6, |total_size| {
            total_size
                .saturating_sub(self.offset.saturating_add(u32::from(self.data_length)))
                .min(6) as usize
        });
        let mut source = 0usize;
        let mut emitted = None;
        while source < fragment_length {
            let available = usize::from(16 - self.data_length);
            let copied = available.min(fragment_length - source);
            let destination = usize::from(self.data_length);
            self.data[destination..destination + copied]
                .copy_from_slice(&fragment.data[source..source + copied]);
            self.data_length += copied as u8;
            source += copied;
            if self.data_length == 16 {
                emitted = Some(self.take_chunk(self.reached_end() && source == fragment_length));
            }
        }
        if emitted.is_none() && self.reached_end() {
            emitted = Some(self.take_chunk(true));
        }
        Ok(emitted)
    }

    pub fn finish(&mut self, transfer_id: u8) -> Option<RecoveryChunk> {
        if !self.active || self.transfer_id != transfer_id {
            return None;
        }
        if self.data_length == 0 {
            return Some(self.take_chunk(true));
        }
        Some(self.take_chunk(true))
    }

    pub fn abort(&mut self) -> Option<RecoveryResume> {
        if self.transfer_id == 0 {
            return None;
        }
        let resume = RecoveryResume {
            transfer_id: self.transfer_id,
            source: self.source,
            offset: self.offset.saturating_add(u32::from(self.data_length)),
        };
        self.active = false;
        self.data = [0; 16];
        self.data_length = 0;
        Some(resume)
    }

    fn reached_end(&self) -> bool {
        self.total_size.is_some_and(|total_size| {
            self.offset.saturating_add(u32::from(self.data_length)) >= total_size
        })
    }

    fn take_chunk(&mut self, end_of_file: bool) -> RecoveryChunk {
        let chunk = RecoveryChunk {
            transfer_id: self.transfer_id,
            source: self.source,
            offset: self.offset,
            data_length: self.data_length,
            data: self.data,
            end_of_file,
        };
        self.offset = self.offset.saturating_add(u32::from(self.data_length));
        self.data = [0; 16];
        self.data_length = 0;
        if end_of_file {
            self.active = false;
        }
        chunk
    }
}

impl Default for RecoveryAssembler {
    fn default() -> Self {
        Self::new()
    }
}

impl Default for RecoverySession {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pending() -> PendingRecovery {
        PendingRecovery {
            transaction_id: 7,
            command: b'f',
            opcode: RecoveryOpcode::StartLogDump,
            source: RecoverySource::InternalFlash,
        }
    }

    fn status(status: RecoveryStatusCode) -> RecoveryStatus {
        RecoveryStatus {
            opcode: RecoveryOpcode::StartLogDump,
            transfer_id: 7,
            status,
            source: RecoverySource::InternalFlash,
            total_size: 100,
        }
    }

    #[test]
    fn dumping_then_complete_reports_lifecycle() {
        let mut session = RecoverySession::new();
        assert!(session.start(pending()));
        assert_eq!(
            session
                .apply_status(status(RecoveryStatusCode::Dumping))
                .unwrap()
                .phase,
            CommandPhase::Accepted
        );
        assert_eq!(
            session
                .apply_status(status(RecoveryStatusCode::Complete))
                .unwrap()
                .phase,
            CommandPhase::Completed
        );
        assert!(session.start(pending()));
    }

    #[test]
    fn mismatched_transfer_is_ignored() {
        let mut session = RecoverySession::new();
        session.start(pending());
        let mut mismatch = status(RecoveryStatusCode::Complete);
        mismatch.transfer_id = 8;
        assert_eq!(session.apply_status(mismatch), None);
        assert!(!session.start(pending()));
    }

    #[test]
    fn fragments_are_packed_without_losing_remainder() {
        let mut assembler = RecoveryAssembler::new();
        assembler.start(7, RecoverySource::InternalFlash, 100, 18);
        assert_eq!(
            assembler.push(fragment(0, [0, 1, 2, 3, 4, 5])).unwrap(),
            None
        );
        assert_eq!(
            assembler.push(fragment(1, [6, 7, 8, 9, 10, 11])).unwrap(),
            None
        );
        let chunk = assembler
            .push(fragment(2, [12, 13, 14, 15, 16, 17]))
            .unwrap()
            .unwrap();
        assert_eq!(chunk.offset, 100);
        assert_eq!(chunk.data_length, 16);
        assert!(!chunk.end_of_file);
        assert_eq!(
            chunk.data,
            [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]
        );
        let final_chunk = assembler.finish(7).unwrap();
        assert_eq!(final_chunk.offset, 116);
        assert_eq!(final_chunk.data_length, 2);
        assert_eq!(&final_chunk.data[..2], &[16, 17]);
        assert!(final_chunk.end_of_file);
    }

    #[test]
    fn sequence_gap_is_reported_and_partial_chunk_is_not_emitted() {
        let mut assembler = RecoveryAssembler::new();
        assembler.start(7, RecoverySource::InternalFlash, 0, 0);
        assembler.push(fragment(10, [1; 6])).unwrap();
        assert_eq!(
            assembler.push(fragment(12, [2; 6])),
            Err(RecoveryAssemblyError::SequenceGap)
        );
        assert_eq!(assembler.finish(7), None);
    }

    #[test]
    fn exact_multiple_emits_an_explicit_eof_marker() {
        let mut assembler = RecoveryAssembler::new();
        assembler.start(7, RecoverySource::InternalFlash, 0, 0);
        assembler.push(fragment(0, [1; 6])).unwrap();
        assembler.push(fragment(1, [2; 6])).unwrap();
        let chunk = assembler.push(fragment(2, [3; 6])).unwrap().unwrap();
        assert_eq!(chunk.data_length, 16);
        assert!(!chunk.end_of_file);
        let eof = assembler.finish(7).unwrap();
        assert_eq!(eof.offset, 16);
        assert_eq!(eof.data_length, 2);
        assert!(eof.end_of_file);

        let mut exact = RecoveryAssembler::new();
        exact.start(7, RecoverySource::InternalFlash, 0, 16);
        exact.push(fragment(0, [1; 6])).unwrap();
        exact.push(fragment(1, [2; 6])).unwrap();
        let final_chunk = exact.push(fragment(2, [3; 6])).unwrap().unwrap();
        assert_eq!(final_chunk.data_length, 16);
        assert!(final_chunk.end_of_file);
    }

    fn fragment(sequence: u8, data: [u8; 6]) -> RecoveryLogData {
        RecoveryLogData {
            transfer_id: 7,
            sequence,
            data,
        }
    }
}
