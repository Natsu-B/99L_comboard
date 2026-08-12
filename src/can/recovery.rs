use super::protocol::{
    CommandPhase, CommandReason, CommandResult, RecoveryOpcode, RecoverySource, RecoveryStatus,
    RecoveryStatusCode,
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
        if status.transfer_id != pending.transaction_id || status.source != pending.source {
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
}
