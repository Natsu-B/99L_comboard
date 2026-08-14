use crate::lora_scheduler::LoRaTxSource;

pub(crate) const SAMPLES_PER_REPORT: u32 = 10;
const SOURCE_COUNT: usize = 5;

const fn source_index(source: LoRaTxSource) -> usize {
    match source {
        LoRaTxSource::EmergencyResult => 0,
        LoRaTxSource::CommandResult => 1,
        LoRaTxSource::GroundTimeRequest => 2,
        LoRaTxSource::Recovery => 3,
        LoRaTxSource::Periodic => 4,
    }
}

pub(crate) const REQUEST_INTERVAL: usize = 0;
pub(crate) const REQUEST_TO_WRITE_START: usize = 1;
pub(crate) const QUEUE_WAIT: usize = 2;
pub(crate) const INITIAL_AUX_WAIT: usize = 3;
pub(crate) const RX_GUARD_WAIT: usize = 4;
pub(crate) const POST_GUARD_AUX_WAIT: usize = 5;
pub(crate) const UART_WRITE: usize = 6;
pub(crate) const UART_FLUSH: usize = 7;
pub(crate) const FLUSH_TO_AUX_LOW: usize = 8;
pub(crate) const AUX_LOW_DURATION: usize = 9;
pub(crate) const FLUSH_TO_TX_COMPLETE: usize = 10;
pub(crate) const TX_TOTAL: usize = 11;
pub(crate) const WRITE_START_INTERVAL: usize = 12;
pub(crate) const TX_COMPLETE_INTERVAL: usize = 13;
pub(crate) const IDLE_GAP: usize = 14;
pub(crate) const METRIC_COUNT: usize = 15;

#[derive(Clone, Copy, Debug)]
pub(crate) struct TxTimingTrace {
    pub source: LoRaTxSource,
    pub requested_at_us: u64,
    pub transmit_started_at_us: Option<u64>,
    pub initial_aux_ready_at_us: Option<u64>,
    pub rx_guard_done_at_us: Option<u64>,
    pub post_guard_aux_ready_at_us: Option<u64>,
    pub uart_write_started_at_us: Option<u64>,
    pub uart_write_finished_at_us: Option<u64>,
    pub uart_flush_finished_at_us: Option<u64>,
    pub aux_low_at_us: Option<u64>,
    pub aux_high_at_us: Option<u64>,
    pub completed_at_us: Option<u64>,
    pub aux_low_not_observed: bool,
    pub periodic_missed_slots: u32,
}

impl TxTimingTrace {
    pub(crate) const fn new(requested_at_us: u64, source: LoRaTxSource) -> Self {
        Self {
            source,
            requested_at_us,
            transmit_started_at_us: None,
            initial_aux_ready_at_us: None,
            rx_guard_done_at_us: None,
            post_guard_aux_ready_at_us: None,
            uart_write_started_at_us: None,
            uart_write_finished_at_us: None,
            uart_flush_finished_at_us: None,
            aux_low_at_us: None,
            aux_high_at_us: None,
            completed_at_us: None,
            aux_low_not_observed: false,
            periodic_missed_slots: 0,
        }
    }
}

pub(crate) fn reselect_prepared_trace(
    prepared: TxTimingTrace,
    requested_at_us: u64,
    source: LoRaTxSource,
) -> TxTimingTrace {
    let floor = |timestamp: Option<u64>| timestamp.map(|value| value.max(requested_at_us));
    let mut trace = TxTimingTrace::new(requested_at_us, source);
    trace.transmit_started_at_us = Some(requested_at_us);
    trace.initial_aux_ready_at_us = floor(prepared.initial_aux_ready_at_us);
    trace.rx_guard_done_at_us = floor(prepared.rx_guard_done_at_us);
    trace.post_guard_aux_ready_at_us = floor(prepared.post_guard_aux_ready_at_us);
    trace
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct MetricSummary {
    pub count: u32,
    pub min: u64,
    pub max: u64,
    pub average: u64,
}

#[derive(Clone, Copy)]
struct MetricStats {
    count: u32,
    min: u64,
    max: u64,
    total: u64,
}

impl MetricStats {
    const fn new() -> Self {
        Self {
            count: 0,
            min: u64::MAX,
            max: 0,
            total: 0,
        }
    }

    fn record(&mut self, value: u64) {
        self.count = self.count.saturating_add(1);
        self.min = self.min.min(value);
        self.max = self.max.max(value);
        self.total = self.total.saturating_add(value);
    }

    fn summary(self) -> MetricSummary {
        if self.count == 0 {
            return MetricSummary {
                count: 0,
                min: 0,
                max: 0,
                average: 0,
            };
        }
        MetricSummary {
            count: self.count,
            min: self.min,
            max: self.max,
            average: self.total / u64::from(self.count),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct TimingReport {
    pub metrics: [MetricSummary; METRIC_COUNT],
    source_samples: [u32; SOURCE_COUNT],
    source_tx_total: [MetricSummary; SOURCE_COUNT],
    pub emergency_queue_wait: MetricSummary,
    pub aux_low_not_observed: u32,
    pub invalid_timestamp_count: u32,
    pub periodic_missed_slots: u32,
}

impl TimingReport {
    pub(crate) const fn source_samples(&self, source: LoRaTxSource) -> u32 {
        self.source_samples[source_index(source)]
    }

    pub(crate) const fn source_tx_total(&self, source: LoRaTxSource) -> MetricSummary {
        self.source_tx_total[source_index(source)]
    }

    pub(crate) fn sample_count(&self) -> u32 {
        self.source_samples.iter().sum()
    }
}

pub(crate) struct TimingCollector {
    metrics: [MetricStats; METRIC_COUNT],
    source_samples: [u32; SOURCE_COUNT],
    source_tx_total: [MetricStats; SOURCE_COUNT],
    emergency_queue_wait: MetricStats,
    previous_requested_at_us: [Option<u64>; SOURCE_COUNT],
    previous_write_started_at_us: Option<u64>,
    previous_completed_at_us: Option<u64>,
    sample_count: u32,
    aux_low_not_observed: u32,
    invalid_timestamp_count: u32,
    periodic_missed_slots: u32,
}

impl TimingCollector {
    pub(crate) const fn new() -> Self {
        Self {
            metrics: [MetricStats::new(); METRIC_COUNT],
            source_samples: [0; SOURCE_COUNT],
            source_tx_total: [MetricStats::new(); SOURCE_COUNT],
            emergency_queue_wait: MetricStats::new(),
            previous_requested_at_us: [None; SOURCE_COUNT],
            previous_write_started_at_us: None,
            previous_completed_at_us: None,
            sample_count: 0,
            aux_low_not_observed: 0,
            invalid_timestamp_count: 0,
            periodic_missed_slots: 0,
        }
    }

    pub(crate) fn record(&mut self, trace: TxTimingTrace) -> Option<TimingReport> {
        self.sample_count = self.sample_count.saturating_add(1);
        let source_index = source_index(trace.source);
        self.source_samples[source_index] = self.source_samples[source_index].saturating_add(1);
        self.periodic_missed_slots = self
            .periodic_missed_slots
            .saturating_add(trace.periodic_missed_slots);
        if trace.aux_low_not_observed {
            self.aux_low_not_observed = self.aux_low_not_observed.saturating_add(1);
        }

        let previous_requested_at_us = self.previous_requested_at_us[source_index];
        self.record_delta(
            REQUEST_INTERVAL,
            Some(trace.requested_at_us),
            previous_requested_at_us,
        );
        self.previous_requested_at_us[source_index] = Some(trace.requested_at_us);
        self.record_delta(
            REQUEST_TO_WRITE_START,
            trace.uart_write_started_at_us,
            Some(trace.requested_at_us),
        );
        self.record_delta(
            QUEUE_WAIT,
            trace.transmit_started_at_us,
            Some(trace.requested_at_us),
        );
        self.record_delta(
            INITIAL_AUX_WAIT,
            trace.initial_aux_ready_at_us,
            trace.transmit_started_at_us,
        );
        self.record_delta(
            RX_GUARD_WAIT,
            trace.rx_guard_done_at_us,
            trace.initial_aux_ready_at_us,
        );
        self.record_delta(
            POST_GUARD_AUX_WAIT,
            trace.post_guard_aux_ready_at_us,
            trace.rx_guard_done_at_us,
        );
        self.record_delta(
            UART_WRITE,
            trace.uart_write_finished_at_us,
            trace.uart_write_started_at_us,
        );
        self.record_delta(
            UART_FLUSH,
            trace.uart_flush_finished_at_us,
            trace.uart_write_finished_at_us,
        );
        self.record_delta(
            FLUSH_TO_AUX_LOW,
            trace.aux_low_at_us,
            trace.uart_flush_finished_at_us,
        );
        self.record_delta(AUX_LOW_DURATION, trace.aux_high_at_us, trace.aux_low_at_us);
        self.record_delta(
            FLUSH_TO_TX_COMPLETE,
            trace.completed_at_us,
            trace.uart_flush_finished_at_us,
        );
        self.record_delta(
            TX_TOTAL,
            trace.completed_at_us,
            trace.transmit_started_at_us,
        );
        self.record_delta(
            WRITE_START_INTERVAL,
            trace.uart_write_started_at_us,
            self.previous_write_started_at_us,
        );
        if trace.uart_write_started_at_us.is_some() {
            self.previous_write_started_at_us = trace.uart_write_started_at_us;
        }
        self.record_delta(
            TX_COMPLETE_INTERVAL,
            trace.completed_at_us,
            self.previous_completed_at_us,
        );
        self.record_delta(
            IDLE_GAP,
            trace.uart_write_started_at_us,
            self.previous_completed_at_us,
        );
        if trace.completed_at_us.is_some() {
            self.previous_completed_at_us = trace.completed_at_us;
        }

        if let (Some(completed), Some(started)) =
            (trace.completed_at_us, trace.transmit_started_at_us)
        {
            self.record_source_delta(source_index, completed, started);
        }
        if trace.source == LoRaTxSource::EmergencyResult
            && let (Some(write_started), requested) =
                (trace.uart_write_started_at_us, trace.requested_at_us)
        {
            self.record_emergency_queue_delta(write_started, requested);
        }

        if self.sample_count < SAMPLES_PER_REPORT {
            return None;
        }

        let report = TimingReport {
            metrics: self.metrics.map(MetricStats::summary),
            source_samples: self.source_samples,
            source_tx_total: self.source_tx_total.map(MetricStats::summary),
            emergency_queue_wait: self.emergency_queue_wait.summary(),
            aux_low_not_observed: self.aux_low_not_observed,
            invalid_timestamp_count: self.invalid_timestamp_count,
            periodic_missed_slots: self.periodic_missed_slots,
        };
        self.reset_window();
        Some(report)
    }

    fn record_delta(&mut self, metric: usize, later: Option<u64>, earlier: Option<u64>) {
        if let (Some(later), Some(earlier)) = (later, earlier) {
            if let Some(delta) = later.checked_sub(earlier) {
                self.metrics[metric].record(delta);
            } else {
                self.invalid_timestamp_count = self.invalid_timestamp_count.saturating_add(1);
            }
        }
    }

    fn record_source_delta(&mut self, source_index: usize, later: u64, earlier: u64) {
        if let Some(delta) = later.checked_sub(earlier) {
            self.source_tx_total[source_index].record(delta);
        } else {
            self.invalid_timestamp_count = self.invalid_timestamp_count.saturating_add(1);
        }
    }

    fn record_emergency_queue_delta(&mut self, later: u64, earlier: u64) {
        if let Some(delta) = later.checked_sub(earlier) {
            self.emergency_queue_wait.record(delta);
        } else {
            self.invalid_timestamp_count = self.invalid_timestamp_count.saturating_add(1);
        }
    }

    fn reset_window(&mut self) {
        self.metrics = [MetricStats::new(); METRIC_COUNT];
        self.source_samples = [0; SOURCE_COUNT];
        self.source_tx_total = [MetricStats::new(); SOURCE_COUNT];
        self.emergency_queue_wait = MetricStats::new();
        self.sample_count = 0;
        self.aux_low_not_observed = 0;
        self.invalid_timestamp_count = 0;
        self.periodic_missed_slots = 0;
    }
}
