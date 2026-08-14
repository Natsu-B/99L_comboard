#[path = "../src/lora_scheduler.rs"]
mod lora_scheduler;
#[path = "../src/lora_timing.rs"]
mod lora_timing;

use lora_scheduler::LoRaTxSource;
use lora_timing::{
    AUX_LOW_DURATION, FLUSH_TO_AUX_LOW, FLUSH_TO_TX_COMPLETE, IDLE_GAP, INITIAL_AUX_WAIT,
    POST_GUARD_AUX_WAIT, QUEUE_WAIT, REQUEST_TO_WRITE_START, RX_GUARD_WAIT, SAMPLES_PER_REPORT,
    TX_TOTAL, TimingCollector, TxTimingTrace, UART_FLUSH, UART_WRITE, WRITE_START_INTERVAL,
    reselect_prepared_trace,
};

fn complete_trace(requested_at_us: u64, source: LoRaTxSource) -> TxTimingTrace {
    let mut trace = TxTimingTrace::new(requested_at_us, source);
    trace.transmit_started_at_us = Some(requested_at_us + 1);
    trace.initial_aux_ready_at_us = Some(requested_at_us + 10);
    trace.rx_guard_done_at_us = Some(requested_at_us + 20);
    trace.post_guard_aux_ready_at_us = Some(requested_at_us + 25);
    trace.uart_write_started_at_us = Some(requested_at_us + 30);
    trace.uart_write_finished_at_us = Some(requested_at_us + 40);
    trace.uart_flush_finished_at_us = Some(requested_at_us + 50);
    trace.aux_low_at_us = Some(requested_at_us + 60);
    trace.aux_high_at_us = Some(requested_at_us + 160);
    trace.completed_at_us = Some(requested_at_us + 160);
    trace
}

#[test]
fn synthetic_trace_has_chronological_metrics() {
    let mut collector = TimingCollector::new();
    let mut report = None;
    for index in 0..SAMPLES_PER_REPORT {
        report = collector.record(complete_trace(
            u64::from(index) * 500,
            LoRaTxSource::Periodic,
        ));
    }
    let report = report.unwrap();
    assert_eq!(report.metrics[QUEUE_WAIT].average, 1);
    assert_eq!(report.metrics[INITIAL_AUX_WAIT].average, 9);
    assert_eq!(report.metrics[RX_GUARD_WAIT].average, 10);
    assert_eq!(report.metrics[REQUEST_TO_WRITE_START].average, 30);
    assert_eq!(report.metrics[UART_WRITE].average, 10);
    assert_eq!(report.metrics[UART_FLUSH].average, 10);
    assert_eq!(report.metrics[FLUSH_TO_AUX_LOW].average, 10);
    assert_eq!(report.metrics[AUX_LOW_DURATION].average, 100);
    assert_eq!(report.metrics[FLUSH_TO_TX_COMPLETE].average, 110);
    assert_eq!(report.metrics[TX_TOTAL].average, 159);
    assert_eq!(report.metrics[WRITE_START_INTERVAL].average, 500);
    assert_eq!(report.metrics[IDLE_GAP].average, 370);
    assert_eq!(report.sample_count(), 10);
    assert_eq!(report.source_samples(LoRaTxSource::Periodic), 10);
    assert_eq!(report.source_tx_total(LoRaTxSource::Periodic).average, 159);
    assert_eq!(report.invalid_timestamp_count, 0);
}

#[test]
fn timestamp_reversal_is_counted_without_underflow() {
    let mut collector = TimingCollector::new();
    let mut report = None;
    for index in 0..SAMPLES_PER_REPORT {
        let mut trace = complete_trace(u64::from(index) * 500, LoRaTxSource::CommandResult);
        trace.uart_write_finished_at_us = trace.uart_write_started_at_us.map(|value| value - 1);
        report = collector.record(trace);
    }
    let report = report.unwrap();
    assert_eq!(report.metrics[UART_WRITE].count, 0);
    assert_eq!(report.invalid_timestamp_count, 10);
}

#[test]
fn priority_reordering_does_not_reverse_request_intervals() {
    let mut collector = TimingCollector::new();
    let report = (0..5u64)
        .find_map(|pair| {
            let actual_base = pair * 1_000;
            let mut emergency = complete_trace(actual_base + 200, LoRaTxSource::EmergencyResult);
            emergency.requested_at_us = pair * 1_000 + 100;
            assert!(collector.record(emergency).is_none());

            let mut older_b1 = complete_trace(actual_base + 500, LoRaTxSource::GroundTimeRequest);
            older_b1.requested_at_us = pair * 1_000;
            collector.record(older_b1)
        })
        .unwrap();
    assert_eq!(report.metrics[lora_timing::REQUEST_INTERVAL].count, 8);
    assert_eq!(report.metrics[lora_timing::REQUEST_INTERVAL].average, 1_000);
    assert_eq!(report.invalid_timestamp_count, 0);
}

#[test]
fn priority_reselection_preserves_remaining_guard_timing() {
    let mut collector = TimingCollector::new();
    let report = (0..SAMPLES_PER_REPORT)
        .find_map(|index| {
            let base = u64::from(index) * 1_000;
            let mut prepared = TxTimingTrace::new(base, LoRaTxSource::Periodic);
            prepared.transmit_started_at_us = Some(base);
            prepared.initial_aux_ready_at_us = Some(base + 10);
            prepared.rx_guard_done_at_us = Some(base + 200);
            prepared.post_guard_aux_ready_at_us = Some(base + 210);

            let mut selected =
                reselect_prepared_trace(prepared, base + 100, LoRaTxSource::EmergencyResult);
            selected.uart_write_started_at_us = Some(base + 210);
            selected.uart_write_finished_at_us = Some(base + 220);
            selected.uart_flush_finished_at_us = Some(base + 230);
            selected.aux_low_at_us = Some(base + 232);
            selected.aux_high_at_us = Some(base + 500);
            selected.completed_at_us = Some(base + 500);
            collector.record(selected)
        })
        .unwrap();

    assert_eq!(report.metrics[QUEUE_WAIT].average, 0);
    assert_eq!(report.metrics[INITIAL_AUX_WAIT].average, 0);
    assert_eq!(report.metrics[RX_GUARD_WAIT].average, 100);
    assert_eq!(report.metrics[POST_GUARD_AUX_WAIT].average, 10);
    let stage_sum = report.metrics[QUEUE_WAIT].average
        + report.metrics[INITIAL_AUX_WAIT].average
        + report.metrics[RX_GUARD_WAIT].average
        + report.metrics[POST_GUARD_AUX_WAIT].average;
    assert_eq!(report.metrics[REQUEST_TO_WRITE_START].average, stage_sum);
    assert_eq!(report.invalid_timestamp_count, 0);
}

#[test]
fn source_and_aux_counters_are_reported() {
    let sources = [
        LoRaTxSource::EmergencyResult,
        LoRaTxSource::CommandResult,
        LoRaTxSource::GroundTimeRequest,
        LoRaTxSource::Recovery,
        LoRaTxSource::Periodic,
    ];
    let mut collector = TimingCollector::new();
    let mut report = None;
    for index in 0..SAMPLES_PER_REPORT {
        let mut trace = complete_trace(u64::from(index) * 500, sources[index as usize % 5]);
        trace.aux_low_not_observed = index == 0;
        trace.periodic_missed_slots = index;
        report = collector.record(trace);
    }
    let report = report.unwrap();
    for source in sources {
        assert_eq!(report.source_samples(source), 2);
    }
    assert_eq!(report.aux_low_not_observed, 1);
    assert_eq!(report.periodic_missed_slots, 45);
    assert_eq!(report.emergency_queue_wait.average, 30);
}
