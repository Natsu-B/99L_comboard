#[cfg(target_arch = "xtensa")]
use crate::payload::LoraFrame;

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub(crate) enum LoRaTxSource {
    Periodic,
    Recovery,
    GroundTimeRequest,
    CommandResult,
    EmergencyResult,
}

#[cfg(target_arch = "xtensa")]
#[derive(Clone, Copy, Debug)]
pub(crate) struct LoRaTxEnvelope {
    pub frame: LoraFrame,
    pub source: LoRaTxSource,
    #[cfg(feature = "lora-timing-debug")]
    pub requested_at_us: u64,
    pub interval_events: u16,
    pub event_revision: u32,
}

#[cfg(target_arch = "xtensa")]
impl LoRaTxEnvelope {
    pub(crate) const fn queued(
        frame: LoraFrame,
        source: LoRaTxSource,
        requested_at_us: u64,
    ) -> Self {
        #[cfg(not(feature = "lora-timing-debug"))]
        let _ = requested_at_us;
        Self {
            frame,
            source,
            #[cfg(feature = "lora-timing-debug")]
            requested_at_us,
            interval_events: 0,
            event_revision: 0,
        }
    }

    pub(crate) const fn periodic(
        frame: LoraFrame,
        requested_at_us: u64,
        interval_events: u16,
        event_revision: u32,
    ) -> Self {
        #[cfg(not(feature = "lora-timing-debug"))]
        let _ = requested_at_us;
        Self {
            frame,
            source: LoRaTxSource::Periodic,
            #[cfg(feature = "lora-timing-debug")]
            requested_at_us,
            interval_events,
            event_revision,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct DeadlineAdvance {
    pub next_deadline_us: u64,
    pub missed_slots: u32,
}

pub(crate) fn advance_deadline(
    deadline_us: u64,
    now_us: u64,
    interval_us: u64,
) -> Option<DeadlineAdvance> {
    if interval_us == 0 {
        return None;
    }
    if deadline_us > now_us {
        return Some(DeadlineAdvance {
            next_deadline_us: deadline_us,
            missed_slots: 0,
        });
    }
    let elapsed_intervals = now_us.checked_sub(deadline_us)? / interval_us;
    let skipped = elapsed_intervals.checked_add(1)?;
    let advance_us = skipped.checked_mul(interval_us)?;
    let next_deadline_us = deadline_us.checked_add(advance_us)?;
    if next_deadline_us <= now_us {
        return None;
    }
    Some(DeadlineAdvance {
        next_deadline_us,
        missed_slots: if skipped > u32::MAX as u64 {
            u32::MAX
        } else {
            skipped as u32
        },
    })
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum GroundTimeQueueAction {
    IgnoreDuplicate,
    Enqueue,
    ReplaceOldest,
}

pub(crate) fn ground_time_queue_action(
    previous_request_id: Option<u8>,
    request_id: u8,
    queue_is_full: bool,
) -> GroundTimeQueueAction {
    if previous_request_id == Some(request_id) {
        GroundTimeQueueAction::IgnoreDuplicate
    } else if queue_is_full {
        GroundTimeQueueAction::ReplaceOldest
    } else {
        GroundTimeQueueAction::Enqueue
    }
}

pub(crate) fn should_clear_periodic_events(
    source: LoRaTxSource,
    transmitted: bool,
    interval_events: u16,
) -> bool {
    source == LoRaTxSource::Periodic && transmitted && interval_events != 0
}

pub(crate) const fn is_higher_priority(candidate: LoRaTxSource, selected: LoRaTxSource) -> bool {
    (candidate as u8) > (selected as u8)
}

pub(crate) const fn should_defer_preempted(source: LoRaTxSource) -> bool {
    !matches!(source, LoRaTxSource::Periodic)
}

pub(crate) const fn preempted_periodic_missed_slots(source: LoRaTxSource) -> u32 {
    match source {
        LoRaTxSource::Periodic => 1,
        _ => 0,
    }
}

pub(crate) const fn recovery_next_eligible_at_us(
    completed_at_us: u64,
    minimum_gap_us: u64,
) -> Option<u64> {
    completed_at_us.checked_add(minimum_gap_us)
}

pub(crate) const fn recovery_allowed(
    recovery_sent_since_periodic: bool,
    periodic_due: bool,
) -> bool {
    !recovery_sent_since_periodic || !periodic_due
}

pub(crate) const fn periodic_due_for_reselection(
    selected_source: LoRaTxSource,
    periodic_deadline_valid: bool,
    deadline_reached: bool,
) -> bool {
    matches!(selected_source, LoRaTxSource::Periodic)
        || (periodic_deadline_valid && deadline_reached)
}

pub(crate) const fn deferred_recovery_blocked(
    deferred_source: LoRaTxSource,
    recovery_sent_since_periodic: bool,
    periodic_due: bool,
) -> bool {
    matches!(deferred_source, LoRaTxSource::Recovery)
        && !recovery_allowed(recovery_sent_since_periodic, periodic_due)
}

pub(crate) const fn update_recovery_fairness(
    recovery_sent_since_periodic: bool,
    attempted_source: LoRaTxSource,
) -> bool {
    match attempted_source {
        LoRaTxSource::Recovery => true,
        LoRaTxSource::Periodic => false,
        _ => recovery_sent_since_periodic,
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct PeriodicDeadlineSelection {
    pub deadline_us: u64,
    pub retry_selected: bool,
}

pub(crate) const fn select_periodic_deadline(
    regular_deadline_us: u64,
    regular_deadline_valid: bool,
    retry_at_us: Option<u64>,
) -> PeriodicDeadlineSelection {
    match retry_at_us {
        Some(retry_at_us) if !regular_deadline_valid || retry_at_us < regular_deadline_us => {
            PeriodicDeadlineSelection {
                deadline_us: retry_at_us,
                retry_selected: true,
            }
        }
        _ => PeriodicDeadlineSelection {
            deadline_us: regular_deadline_us,
            retry_selected: false,
        },
    }
}

pub(crate) const fn regular_deadline_after_selection(
    regular_deadline_us: u64,
    retry_selected: bool,
    interval_us: u64,
) -> Option<u64> {
    if retry_selected {
        Some(regular_deadline_us)
    } else {
        regular_deadline_us.checked_add(interval_us)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct PeriodicDeadlineConsumption {
    pub next_regular_deadline_us: Option<u64>,
    pub retry_at_us: Option<u64>,
}

pub(crate) const fn consume_periodic_selection(
    regular_deadline_us: u64,
    retry_selected: bool,
    interval_us: u64,
) -> PeriodicDeadlineConsumption {
    PeriodicDeadlineConsumption {
        next_regular_deadline_us: regular_deadline_after_selection(
            regular_deadline_us,
            retry_selected,
            interval_us,
        ),
        retry_at_us: None,
    }
}

pub(crate) const fn periodic_retry_after_nonperiodic_attempt(
    retry_at_us: Option<u64>,
    completed_at_us: u64,
    attempted_source: LoRaTxSource,
) -> Option<u64> {
    if matches!(attempted_source, LoRaTxSource::Periodic) {
        return retry_at_us;
    }
    match retry_at_us {
        Some(retry_at_us) if retry_at_us <= completed_at_us => None,
        retry_at_us => retry_at_us,
    }
}

pub(crate) const fn periodic_retry_after_recovery_us(
    completed_at_us: u64,
    minimum_gap_us: u64,
    displaced_regular_slot: bool,
) -> Option<u64> {
    if displaced_regular_slot {
        completed_at_us.checked_add(minimum_gap_us)
    } else {
        None
    }
}

pub(crate) const fn recovery_displaced_regular_slot(
    selected_source: LoRaTxSource,
    higher_source: LoRaTxSource,
    regular_periodic_selected: bool,
) -> bool {
    regular_periodic_selected
        && matches!(selected_source, LoRaTxSource::Periodic)
        && matches!(higher_source, LoRaTxSource::Recovery)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn deadline_keeps_500_ms_phase_after_280_ms_transmission() {
        assert_eq!(
            advance_deadline(1_000_000, 780_000, 500_000),
            Some(DeadlineAdvance {
                next_deadline_us: 1_000_000,
                missed_slots: 0,
            })
        );
    }

    #[test]
    fn overdue_slot_is_skipped_without_burst() {
        assert_eq!(
            advance_deadline(500_000, 630_000, 500_000),
            Some(DeadlineAdvance {
                next_deadline_us: 1_000_000,
                missed_slots: 1,
            })
        );
    }

    #[test]
    fn scheduler_recovers_after_a_long_timeout() {
        assert_eq!(
            advance_deadline(500_000, 2_130_000, 500_000),
            Some(DeadlineAdvance {
                next_deadline_us: 2_500_000,
                missed_slots: 4,
            })
        );
        assert_eq!(
            advance_deadline(500_000, 1_300_000, 500_000),
            Some(DeadlineAdvance {
                next_deadline_us: 1_500_000,
                missed_slots: 2,
            })
        );
    }

    #[test]
    fn deadline_rejects_zero_and_unrepresentable_future() {
        assert_eq!(advance_deadline(1, 2, 0), None);
        assert_eq!(
            advance_deadline(u64::MAX - 10, u64::MAX - 5, 10),
            Some(DeadlineAdvance {
                next_deadline_us: u64::MAX,
                missed_slots: 1,
            })
        );
        assert_eq!(advance_deadline(0, u64::MAX, 1), None);
        assert_eq!(advance_deadline(u64::MAX, u64::MAX, 1), None);
    }

    #[test]
    fn missed_slot_count_saturates_without_deadline_overflow() {
        assert_eq!(
            advance_deadline(0, u64::from(u32::MAX) * 2, 1),
            Some(DeadlineAdvance {
                next_deadline_us: u64::from(u32::MAX) * 2 + 1,
                missed_slots: u32::MAX,
            })
        );
    }

    #[test]
    fn source_priority_and_ground_time_policy_are_explicit() {
        assert!(LoRaTxSource::EmergencyResult > LoRaTxSource::CommandResult);
        assert!(LoRaTxSource::CommandResult > LoRaTxSource::GroundTimeRequest);
        assert!(LoRaTxSource::GroundTimeRequest > LoRaTxSource::Recovery);
        assert!(LoRaTxSource::Recovery > LoRaTxSource::Periodic);
        assert!(is_higher_priority(
            LoRaTxSource::EmergencyResult,
            LoRaTxSource::CommandResult
        ));
        assert!(!is_higher_priority(
            LoRaTxSource::GroundTimeRequest,
            LoRaTxSource::CommandResult
        ));
        assert_eq!(
            ground_time_queue_action(Some(7), 7, true),
            GroundTimeQueueAction::IgnoreDuplicate
        );
        assert_eq!(
            ground_time_queue_action(Some(7), 8, false),
            GroundTimeQueueAction::Enqueue
        );
        assert_eq!(
            ground_time_queue_action(Some(7), 8, true),
            GroundTimeQueueAction::ReplaceOldest
        );
    }

    #[test]
    fn recovery_gap_leaves_a_due_periodic_opportunity() {
        let recovery_eligible = recovery_next_eligible_at_us(9_900_000, 200_000).unwrap();
        let periodic_deadline = 10_000_000;
        assert!(periodic_deadline < recovery_eligible);
        assert_eq!(recovery_next_eligible_at_us(u64::MAX, 1), None);
    }

    #[test]
    fn recovery_fairness_depends_on_periodic_due_state() {
        assert!(recovery_allowed(false, false));
        assert!(recovery_allowed(false, true));
        assert!(recovery_allowed(true, false));
        assert!(!recovery_allowed(true, true));

        let after_recovery = update_recovery_fairness(false, LoRaTxSource::Recovery);
        assert!(after_recovery);
        let after_command = update_recovery_fairness(after_recovery, LoRaTxSource::CommandResult);
        assert!(after_command);
        assert!(!update_recovery_fairness(
            after_command,
            LoRaTxSource::Periodic
        ));

        assert!(periodic_due_for_reselection(
            LoRaTxSource::Periodic,
            true,
            false
        ));
        assert!(deferred_recovery_blocked(
            LoRaTxSource::Recovery,
            true,
            true
        ));
        assert!(!deferred_recovery_blocked(
            LoRaTxSource::Recovery,
            true,
            false
        ));
    }

    #[test]
    fn controlled_retry_uses_recovery_gap_without_moving_regular_phase() {
        let retry_at = periodic_retry_after_recovery_us(1_000_000, 200_000, true).unwrap();
        assert_eq!(retry_at, 1_200_000);
        let selection = select_periodic_deadline(1_500_000, true, Some(retry_at));
        assert_eq!(
            selection,
            PeriodicDeadlineSelection {
                deadline_us: 1_200_000,
                retry_selected: true,
            }
        );
        assert_eq!(
            regular_deadline_after_selection(1_500_000, selection.retry_selected, 500_000),
            Some(1_500_000)
        );
        assert_eq!(
            consume_periodic_selection(1_500_000, true, 500_000),
            PeriodicDeadlineConsumption {
                next_regular_deadline_us: Some(1_500_000),
                retry_at_us: None,
            }
        );
        assert_eq!(
            periodic_retry_after_recovery_us(u64::MAX, 200_000, true),
            None
        );
        assert_eq!(
            periodic_retry_after_recovery_us(1_000_000, 200_000, false),
            None
        );
    }

    #[test]
    fn regular_periodic_selection_wins_valid_ties_and_clears_retry() {
        let selection = select_periodic_deadline(1_000_000, true, Some(1_000_000));
        assert!(!selection.retry_selected);
        assert_eq!(
            regular_deadline_after_selection(1_000_000, false, 500_000),
            Some(1_500_000)
        );
        assert_eq!(regular_deadline_after_selection(u64::MAX, false, 1), None);
        assert!(recovery_displaced_regular_slot(
            LoRaTxSource::Periodic,
            LoRaTxSource::Recovery,
            true
        ));
        assert!(!recovery_displaced_regular_slot(
            LoRaTxSource::Periodic,
            LoRaTxSource::CommandResult,
            true
        ));
    }

    #[test]
    fn valid_retry_wins_when_regular_deadline_overflowed() {
        let selection = select_periodic_deadline(u64::MAX - 400_000, false, Some(u64::MAX));
        assert_eq!(
            selection,
            PeriodicDeadlineSelection {
                deadline_us: u64::MAX,
                retry_selected: true,
            }
        );
    }

    #[test]
    fn nonperiodic_attempt_discards_only_expired_retry() {
        assert_eq!(
            periodic_retry_after_nonperiodic_attempt(
                Some(1_200_000),
                1_500_000,
                LoRaTxSource::CommandResult,
            ),
            None
        );
        assert_eq!(
            periodic_retry_after_nonperiodic_attempt(
                Some(1_700_000),
                1_500_000,
                LoRaTxSource::EmergencyResult,
            ),
            Some(1_700_000)
        );
        assert_eq!(
            periodic_retry_after_nonperiodic_attempt(
                Some(1_200_000),
                1_500_000,
                LoRaTxSource::Periodic,
            ),
            Some(1_200_000)
        );
    }

    #[test]
    fn preempted_periodic_slot_is_not_deferred_into_a_burst() {
        assert!(!should_defer_preempted(LoRaTxSource::Periodic));
        assert_eq!(preempted_periodic_missed_slots(LoRaTxSource::Periodic), 1);
        assert!(should_defer_preempted(LoRaTxSource::Recovery));
        assert_eq!(preempted_periodic_missed_slots(LoRaTxSource::Recovery), 0);
        assert!(should_defer_preempted(LoRaTxSource::GroundTimeRequest));
        assert!(should_defer_preempted(LoRaTxSource::CommandResult));
    }

    #[test]
    fn periodic_events_clear_only_after_success() {
        assert!(!should_clear_periodic_events(
            LoRaTxSource::Periodic,
            false,
            1
        ));
        assert!(!should_clear_periodic_events(
            LoRaTxSource::CommandResult,
            true,
            1
        ));
        assert!(!should_clear_periodic_events(
            LoRaTxSource::Periodic,
            true,
            0
        ));
        assert!(should_clear_periodic_events(
            LoRaTxSource::Periodic,
            true,
            1
        ));
    }
}
