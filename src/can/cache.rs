use super::protocol::{
    AirspeedTelemetry, AttitudeTiltTelemetry, CanRxMessage, CommandResult, ControlTelemetry,
    DescentCoreTelemetry, KinematicsTelemetry, LpsTelemetry, MissionEvent, MissionStatusTelemetry,
    PowerTimeTelemetry, RecoveryLogData, RecoveryStatus,
};

pub const FRESHNESS_100_HZ_MS: u64 = 30;
pub const FRESHNESS_25_HZ_MS: u64 = 120;
pub const FRESHNESS_10_HZ_MS: u64 = 300;

pub fn observed_sequence(identifier: u16, data: &[u8]) -> Option<u8> {
    match identifier {
        0x020 | 0x100..=0x104 | 0x107..=0x109 => data.first().copied(),
        0x106 => data.get(1).copied(),
        _ => None,
    }
}

pub const fn sequence_gap(previous: Option<u8>, current: u8) -> bool {
    match previous {
        Some(previous) => current != previous.wrapping_add(1),
        None => false,
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Freshness {
    Missing,
    Fresh,
    Stale,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Latest<T: Copy> {
    value: Option<T>,
    received_at_ms: u64,
}

impl<T: Copy> Latest<T> {
    pub const fn new() -> Self {
        Self {
            value: None,
            received_at_ms: 0,
        }
    }

    pub fn update(&mut self, value: T, received_at_ms: u64) {
        self.value = Some(value);
        self.received_at_ms = received_at_ms;
    }

    pub const fn value(self) -> Option<T> {
        self.value
    }

    pub const fn received_at_ms(self) -> Option<u64> {
        if self.value.is_some() {
            Some(self.received_at_ms)
        } else {
            None
        }
    }

    pub fn freshness(self, now_ms: u64, maximum_age_ms: u64) -> Freshness {
        match self.value {
            None => Freshness::Missing,
            Some(_)
                if now_ms >= self.received_at_ms
                    && now_ms - self.received_at_ms <= maximum_age_ms =>
            {
                Freshness::Fresh
            }
            Some(_) => Freshness::Stale,
        }
    }
}

impl<T: Copy> Default for Latest<T> {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CacheUpdate {
    Telemetry,
    CommandResult(CommandResult),
    TimeRequest { request_id: u8 },
    MissionEvent { event: MissionEvent, new_flags: u16 },
    DuplicateMissionEvent,
    RecoveryStatus(RecoveryStatus),
    RecoveryLogData(RecoveryLogData),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CanCache {
    pub kinematics: Latest<KinematicsTelemetry>,
    pub control: Latest<ControlTelemetry>,
    pub mission_status: Latest<MissionStatusTelemetry>,
    pub power_time: Latest<PowerTimeTelemetry>,
    pub descent_core: Latest<DescentCoreTelemetry>,
    pub attitude_tilt: Latest<AttitudeTiltTelemetry>,
    pub lps: Latest<LpsTelemetry>,
    pub airspeed: Latest<AirspeedTelemetry>,
    pub recovery_status: Latest<RecoveryStatus>,
    pub recovery_log_data: Latest<RecoveryLogData>,
    pub command_result: Latest<CommandResult>,
    pub time_request: Latest<u8>,
    pub mission_event: Latest<MissionEvent>,
    event_flags_latched: u16,
    event_revision: u32,
    last_event_sequence: Option<u8>,
}

impl CanCache {
    pub const fn new() -> Self {
        Self {
            kinematics: Latest::new(),
            control: Latest::new(),
            mission_status: Latest::new(),
            power_time: Latest::new(),
            descent_core: Latest::new(),
            attitude_tilt: Latest::new(),
            lps: Latest::new(),
            airspeed: Latest::new(),
            recovery_status: Latest::new(),
            recovery_log_data: Latest::new(),
            command_result: Latest::new(),
            time_request: Latest::new(),
            mission_event: Latest::new(),
            event_flags_latched: 0,
            event_revision: 0,
            last_event_sequence: None,
        }
    }

    pub fn update(&mut self, message: CanRxMessage, received_at_ms: u64) -> CacheUpdate {
        match message {
            CanRxMessage::CommandResult(value) => {
                self.command_result.update(value, received_at_ms);
                CacheUpdate::CommandResult(value)
            }
            CanRxMessage::TimeRequest { request_id } => {
                self.time_request.update(request_id, received_at_ms);
                CacheUpdate::TimeRequest { request_id }
            }
            CanRxMessage::MissionEvent(event) => {
                if self.last_event_sequence == Some(event.sequence) {
                    return CacheUpdate::DuplicateMissionEvent;
                }
                let new_flags = event.flags & !self.event_flags_latched;
                self.event_flags_latched |= event.flags;
                self.event_revision = self.event_revision.wrapping_add(1);
                self.last_event_sequence = Some(event.sequence);
                self.mission_event.update(event, received_at_ms);
                CacheUpdate::MissionEvent { event, new_flags }
            }
            CanRxMessage::Kinematics(value) => {
                self.kinematics.update(value, received_at_ms);
                CacheUpdate::Telemetry
            }
            CanRxMessage::Control(value) => {
                self.control.update(value, received_at_ms);
                CacheUpdate::Telemetry
            }
            CanRxMessage::MissionStatus(value) => {
                self.mission_status.update(value, received_at_ms);
                CacheUpdate::Telemetry
            }
            CanRxMessage::PowerTime(value) => {
                self.power_time.update(value, received_at_ms);
                CacheUpdate::Telemetry
            }
            CanRxMessage::DescentCore(value) => {
                self.descent_core.update(value, received_at_ms);
                CacheUpdate::Telemetry
            }
            CanRxMessage::RecoveryStatus(value) => {
                self.recovery_status.update(value, received_at_ms);
                CacheUpdate::RecoveryStatus(value)
            }
            CanRxMessage::RecoveryLogData(value) => {
                self.recovery_log_data.update(value, received_at_ms);
                CacheUpdate::RecoveryLogData(value)
            }
            CanRxMessage::AttitudeTilt(value) => {
                self.attitude_tilt.update(value, received_at_ms);
                CacheUpdate::Telemetry
            }
            CanRxMessage::Lps(value) => {
                self.lps.update(value, received_at_ms);
                CacheUpdate::Telemetry
            }
            CanRxMessage::Airspeed(value) => {
                self.airspeed.update(value, received_at_ms);
                CacheUpdate::Telemetry
            }
        }
    }

    pub const fn event_flags_latched(&self) -> u16 {
        self.event_flags_latched
    }

    pub const fn event_snapshot(&self) -> (u16, u32) {
        (self.event_flags_latched, self.event_revision)
    }

    pub fn clear_event_flags(&mut self, flags: u16, revision: u32) {
        if self.event_revision == revision {
            self.event_flags_latched &= !flags;
        }
    }
}

impl Default for CanCache {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::can::protocol::MissionState;

    fn event(sequence: u8, flags: u16) -> MissionEvent {
        MissionEvent {
            sequence,
            flags,
            state: MissionState::Control,
            elapsed: 1,
            detail: 2,
        }
    }

    #[test]
    fn freshness_is_tracked_per_can_id() {
        let mut cache = CanCache::new();
        cache.update(
            CanRxMessage::Airspeed(AirspeedTelemetry {
                sequence: 1,
                airspeed: 2,
            }),
            100,
        );
        cache.update(
            CanRxMessage::Lps(LpsTelemetry {
                sequence: 1,
                pressure: 2,
                temperature: 3,
            }),
            100,
        );
        assert_eq!(
            cache.airspeed.freshness(130, FRESHNESS_100_HZ_MS),
            Freshness::Fresh
        );
        assert_eq!(
            cache.airspeed.freshness(131, FRESHNESS_100_HZ_MS),
            Freshness::Stale
        );
        assert_eq!(
            cache.lps.freshness(220, FRESHNESS_25_HZ_MS),
            Freshness::Fresh
        );
        assert_eq!(
            cache.lps.freshness(221, FRESHNESS_25_HZ_MS),
            Freshness::Stale
        );
        assert_eq!(
            cache.control.freshness(0, FRESHNESS_100_HZ_MS),
            Freshness::Missing
        );
        assert_eq!(
            cache.airspeed.freshness(99, FRESHNESS_100_HZ_MS),
            Freshness::Stale
        );
    }

    #[test]
    fn observed_sequence_offsets_and_wrap_are_checked() {
        assert_eq!(observed_sequence(0x100, &[7]), Some(7));
        assert_eq!(observed_sequence(0x106, &[9, 8]), Some(8));
        assert_eq!(observed_sequence(0x011, &[7]), None);
        assert!(!sequence_gap(Some(255), 0));
        assert!(sequence_gap(Some(10), 12));
    }

    #[test]
    fn mission_event_is_or_latched_and_duplicates_are_suppressed() {
        let mut cache = CanCache::new();
        assert_eq!(
            cache.update(CanRxMessage::MissionEvent(event(7, 0x0003)), 1),
            CacheUpdate::MissionEvent {
                event: event(7, 0x0003),
                new_flags: 0x0003
            }
        );
        assert_eq!(
            cache.update(CanRxMessage::MissionEvent(event(7, 0x8000)), 2),
            CacheUpdate::DuplicateMissionEvent
        );
        assert_eq!(
            cache.update(CanRxMessage::MissionEvent(event(8, 0x0006)), 3),
            CacheUpdate::MissionEvent {
                event: event(8, 0x0006),
                new_flags: 0x0004
            }
        );
        assert_eq!(cache.event_flags_latched(), 0x0007);
        let (_, revision) = cache.event_snapshot();
        cache.clear_event_flags(0x0003, revision);
        assert_eq!(cache.event_flags_latched(), 0x0004);
    }

    #[test]
    fn event_sequence_wrap_is_not_suppressed() {
        let mut cache = CanCache::new();
        cache.update(CanRxMessage::MissionEvent(event(255, 1)), 1);
        assert!(matches!(
            cache.update(CanRxMessage::MissionEvent(event(0, 2)), 2),
            CacheUpdate::MissionEvent { .. }
        ));
    }

    #[test]
    fn event_arriving_during_send_is_not_cleared() {
        let mut cache = CanCache::new();
        cache.update(CanRxMessage::MissionEvent(event(1, 1)), 0);
        let (flags, revision) = cache.event_snapshot();
        cache.update(CanRxMessage::MissionEvent(event(2, 1)), 1);
        cache.clear_event_flags(flags, revision);
        assert_eq!(cache.event_flags_latched(), 1);
    }
}
