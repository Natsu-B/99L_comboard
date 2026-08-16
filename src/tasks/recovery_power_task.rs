use core::sync::atomic::Ordering;

use embassy_time::{Duration, Instant, Timer};

use crate::{
    can::{cache::Freshness, protocol::PowerTimeTelemetry},
    constants::RECOVERY_POWER_FRESHNESS_MS,
    state::{CAN_CACHE, RECOVERY_BEACON_ACTIVE},
};

const BATTERY_STALE_RAW: u8 = 253;
const BATTERY_UNAVAILABLE_RAW: u8 = 255;
const TIME_UNAVAILABLE_RAW: u16 = 0xfff1;
const TIME_STALE_RAW: u16 = 0xfffa;
const REFRESH_INTERVAL_MS: u64 = 100;

fn unavailable_snapshot() -> PowerTimeTelemetry {
    PowerTimeTelemetry {
        sequence: 0,
        logic_voltage: BATTERY_UNAVAILABLE_RAW,
        motor_voltage: BATTERY_UNAVAILABLE_RAW,
        descent_elapsed: TIME_UNAVAILABLE_RAW,
        recovery_elapsed: TIME_UNAVAILABLE_RAW,
        flags: 0,
    }
}

fn stale_snapshot(previous: PowerTimeTelemetry) -> PowerTimeTelemetry {
    PowerTimeTelemetry {
        sequence: previous.sequence,
        logic_voltage: BATTERY_STALE_RAW,
        motor_voltage: BATTERY_STALE_RAW,
        descent_elapsed: previous.descent_elapsed,
        recovery_elapsed: TIME_STALE_RAW,
        flags: previous.flags,
    }
}

#[embassy_executor::task]
pub async fn recovery_power_cache_task() {
    let mut active_previous = false;
    let mut snapshot = unavailable_snapshot();
    let mut snapshot_source_received_at_ms: Option<u64> = None;
    let mut last_refresh_received_at_ms: Option<u64> = None;

    loop {
        let active = RECOVERY_BEACON_ACTIVE.load(Ordering::Relaxed);
        let now_ms = Instant::now().as_millis();

        if active && !active_previous {
            snapshot = unavailable_snapshot();
            snapshot_source_received_at_ms = None;
            let mut cache = CAN_CACHE.lock().await;
            cache.power_time.update(snapshot, now_ms);
            last_refresh_received_at_ms = Some(now_ms);
        } else if !active && active_previous {
            let mut cache = CAN_CACHE.lock().await;
            cache.clear_recovery_power_snapshot();
            snapshot_source_received_at_ms = None;
            last_refresh_received_at_ms = None;
        } else if active {
            let mut cache = CAN_CACHE.lock().await;
            let current = cache.power_time.value();
            let current_received_at_ms = cache.power_time.received_at_ms();

            let externally_updated = current.is_some()
                && current_received_at_ms.is_some()
                && current_received_at_ms != last_refresh_received_at_ms;
            if externally_updated {
                snapshot = current.expect("checked is_some");
                snapshot_source_received_at_ms = current_received_at_ms;
            }

            let source_age_ms = snapshot_source_received_at_ms
                .and_then(|received_at_ms| now_ms.checked_sub(received_at_ms));
            if source_age_ms.is_some_and(|age| age > RECOVERY_POWER_FRESHNESS_MS) {
                snapshot = stale_snapshot(snapshot);
                snapshot_source_received_at_ms = None;
            }

            // A5の通常300 ms freshness判定を通すため受信時刻だけを延長する。
            // raw値自体はMissionのwake snapshotから一切外挿しない。
            cache.power_time.update(snapshot, now_ms);
            last_refresh_received_at_ms = Some(now_ms);
            debug_assert_eq!(
                cache.power_time.freshness(now_ms, REFRESH_INTERVAL_MS * 3),
                Freshness::Fresh
            );
        }

        active_previous = active;
        Timer::after(Duration::from_millis(REFRESH_INTERVAL_MS)).await;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn recovery_fallback_raw_codes_match_vault() {
        let unavailable = unavailable_snapshot();
        assert_eq!(unavailable.logic_voltage, 255);
        assert_eq!(unavailable.motor_voltage, 255);
        assert_eq!(unavailable.recovery_elapsed, 0xfff1);

        let stale = stale_snapshot(PowerTimeTelemetry {
            sequence: 9,
            logic_voltage: 100,
            motor_voltage: 120,
            descent_elapsed: 77,
            recovery_elapsed: 12,
            flags: 0x85,
        });
        assert_eq!(stale.sequence, 9);
        assert_eq!(stale.logic_voltage, 253);
        assert_eq!(stale.motor_voltage, 253);
        assert_eq!(stale.recovery_elapsed, 0xfffa);
        assert_eq!(stale.flags, 0x85);
    }
}
