use super::protocol::{CommandResult, GenericCommandRequest};

pub const MAX_PENDING_TRANSACTIONS: usize = 16;
pub const RESULT_CACHE_CAPACITY: usize = 16;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RegisterResult {
    Forward,
    DuplicatePending,
    Replay(CommandResult),
    ProtocolError,
    Busy,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct PendingEntry {
    request: GenericCommandRequest,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct CachedEntry {
    request: GenericCommandRequest,
    result: CommandResult,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TransactionTracker {
    pending: [Option<PendingEntry>; MAX_PENDING_TRANSACTIONS],
    results: [Option<CachedEntry>; RESULT_CACHE_CAPACITY],
    next_result: usize,
}

impl TransactionTracker {
    pub const fn new() -> Self {
        Self {
            pending: [None; MAX_PENDING_TRANSACTIONS],
            results: [None; RESULT_CACHE_CAPACITY],
            next_result: 0,
        }
    }

    pub fn register(&mut self, request: GenericCommandRequest) -> RegisterResult {
        if let Some(entry) = self
            .pending
            .iter()
            .flatten()
            .find(|entry| entry.request.transaction_id == request.transaction_id)
        {
            return if entry.request == request {
                self.results
                    .iter()
                    .flatten()
                    .find(|cached| cached.request == request)
                    .map_or(RegisterResult::DuplicatePending, |cached| {
                        RegisterResult::Replay(cached.result)
                    })
            } else {
                RegisterResult::ProtocolError
            };
        }

        if let Some(entry) = self
            .results
            .iter()
            .flatten()
            .find(|entry| entry.request.transaction_id == request.transaction_id)
        {
            return if entry.request == request {
                RegisterResult::Replay(entry.result)
            } else {
                RegisterResult::ProtocolError
            };
        }

        let Some(slot) = self.pending.iter_mut().find(|entry| entry.is_none()) else {
            return RegisterResult::Busy;
        };
        *slot = Some(PendingEntry { request });
        RegisterResult::Forward
    }

    pub fn apply_result(&mut self, result: CommandResult) -> bool {
        let Some(index) = self.pending.iter().position(|entry| {
            entry.is_some_and(|entry| {
                entry.request.transaction_id == result.transaction_id
                    && entry.request.command == result.command
            })
        }) else {
            return false;
        };
        let request = self.pending[index].unwrap().request;
        if let Some(entry) = self.results.iter_mut().flatten().find(|entry| {
            entry.request.transaction_id == request.transaction_id
                && entry.request.command == request.command
        }) {
            entry.result = result;
        } else {
            self.results[self.next_result] = Some(CachedEntry { request, result });
            self.next_result = (self.next_result + 1) % RESULT_CACHE_CAPACITY;
        }
        if result.phase.is_final() {
            self.pending[index] = None;
        }
        true
    }

    pub fn pending_count(&self) -> usize {
        self.pending.iter().flatten().count()
    }
}

impl Default for TransactionTracker {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::can::protocol::{CommandPhase, CommandReason};

    fn request(transaction_id: u8, command: u8) -> GenericCommandRequest {
        GenericCommandRequest {
            transaction_id,
            command,
            args: [0; 6],
        }
    }

    fn result(request: GenericCommandRequest, phase: CommandPhase) -> CommandResult {
        CommandResult {
            transaction_id: request.transaction_id,
            command: request.command,
            phase,
            reason: CommandReason::None,
            detail: 0,
        }
    }

    #[test]
    fn duplicate_pending_is_not_forwarded_again() {
        let mut tracker = TransactionTracker::new();
        let request = request(1, 2);
        assert_eq!(tracker.register(request), RegisterResult::Forward);
        assert_eq!(tracker.register(request), RegisterResult::DuplicatePending);
        assert_eq!(tracker.pending_count(), 1);
    }

    #[test]
    fn same_id_with_different_payload_is_protocol_error() {
        let mut tracker = TransactionTracker::new();
        assert_eq!(tracker.register(request(1, 2)), RegisterResult::Forward);
        assert_eq!(
            tracker.register(request(1, 3)),
            RegisterResult::ProtocolError
        );
    }

    #[test]
    fn accepted_stays_pending_and_final_result_is_replayed() {
        let mut tracker = TransactionTracker::new();
        let request = request(1, 2);
        tracker.register(request);
        assert!(tracker.apply_result(result(request, CommandPhase::Accepted)));
        assert_eq!(tracker.pending_count(), 1);
        assert_eq!(
            tracker.register(request),
            RegisterResult::Replay(result(request, CommandPhase::Accepted))
        );
        let final_result = result(request, CommandPhase::Completed);
        assert!(tracker.apply_result(final_result));
        assert_eq!(tracker.pending_count(), 0);
        assert_eq!(
            tracker.register(request),
            RegisterResult::Replay(final_result)
        );
    }

    #[test]
    fn seventeenth_pending_request_is_busy() {
        let mut tracker = TransactionTracker::new();
        for transaction_id in 1..=16 {
            assert_eq!(
                tracker.register(request(transaction_id, 2)),
                RegisterResult::Forward
            );
        }
        assert_eq!(tracker.register(request(17, 2)), RegisterResult::Busy);
    }

    #[test]
    fn result_cache_keeps_at_least_sixteen_entries() {
        let mut tracker = TransactionTracker::new();
        for transaction_id in 1..=16 {
            let request = request(transaction_id, 2);
            tracker.register(request);
            tracker.apply_result(result(request, CommandPhase::Completed));
        }
        for transaction_id in 1..=16 {
            assert!(matches!(
                tracker.register(request(transaction_id, 2)),
                RegisterResult::Replay(_)
            ));
        }
    }

    #[test]
    fn transmit_failure_is_cached_instead_of_reusing_the_id() {
        let mut tracker = TransactionTracker::new();
        let request = request(1, 2);
        assert_eq!(tracker.register(request), RegisterResult::Forward);
        let failed = CommandResult {
            transaction_id: 1,
            command: 2,
            phase: CommandPhase::Failed,
            reason: CommandReason::Timeout,
            detail: 0,
        };
        assert!(tracker.apply_result(failed));
        assert_eq!(tracker.register(request), RegisterResult::Replay(failed));
    }
}
