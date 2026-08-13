use embassy_time::{Duration, with_timeout};
use esp_hal::{
    Async,
    uart::{RxError, TxError, Uart},
};
use esp_println::println;

type GnssUart = Uart<'static, Async>;

pub const GLL_DELETE: &[u8] = &[
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2A,
];
pub const GSA_DELETE: &[u8] = &[
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x31,
];
pub const GSV_DELETE: &[u8] = &[
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x38,
];
pub const VTG_DELETE: &[u8] = &[
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x46,
];
pub const MEAS_RATE: &[u8] = &[
    0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01, 0x00, 0x7A, 0x12,
];
pub const QZSS_L1S_ENABLE: &[u8] = &[
    0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 0x00, // version
    0x01, // layer: RAM
    0x00, 0x00, // reserved
    0x14, 0x00, 0x31, 0x10, // CFG-SIGNAL-QZSS_L1S_ENA
    0x01, // true
    0xF0, 0x49, // checksum
];
// pub const SLAS_EN: &[u8] = &[
//     0xB5, 0x62, 0x06, 0x8D, 0x04, 0x00, 0x01, 0x00, 0x00, 0x00, 0x98, 0x27,
// ];
pub const GST_ENABLE_UART1: &[u8] = &[
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x07, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x07, 0x59,
];
pub const DYNAMIC_MODEL_AIRBORNE_4G: &[u8] = &[
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x08, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27,
    0x00, 0x00, 0x0A, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x5E, 0x01, 0x00, 0x3C, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x8B, 0xC4,
];
pub const UART_BAUD: &[u8] = &[
    0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0xC2,
    0x01, 0x00, 0x03, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xBC, 0x5E,
];

const INIT_COMMANDS: &[&[u8]] = &[
    GLL_DELETE,
    GSA_DELETE,
    GSV_DELETE,
    VTG_DELETE,
    MEAS_RATE,
    QZSS_L1S_ENABLE,
    DYNAMIC_MODEL_AIRBORNE_4G,
    GST_ENABLE_UART1,
    UART_BAUD,
];

async fn send_cmd(tx: &mut GnssUart, mut data: &[u8]) -> Result<bool, TxError> {
    while !data.is_empty() {
        match tx.write_async(data).await {
            Ok(0) => return Ok(false),
            Err(error) => return Err(error),
            Ok(written) => {
                data = &data[written..];
            }
        }
    }
    Ok(true)
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum AckOutcome {
    Ack,
    Nak,
}

struct AckParser {
    bytes: [u8; 10],
    length: usize,
}

impl AckParser {
    const fn new() -> Self {
        Self {
            bytes: [0; 10],
            length: 0,
        }
    }

    fn push(&mut self, byte: u8, expected_class: u8, expected_id: u8) -> Option<AckOutcome> {
        if self.length == 0 && byte != 0xb5 {
            return None;
        }
        if self.length == 1 && byte != 0x62 {
            self.length = usize::from(byte == 0xb5);
            self.bytes[0] = byte;
            return None;
        }
        self.bytes[self.length] = byte;
        self.length += 1;
        if self.length < self.bytes.len() {
            return None;
        }
        self.length = 0;
        let mut checksum_a = 0u8;
        let mut checksum_b = 0u8;
        for value in &self.bytes[2..8] {
            checksum_a = checksum_a.wrapping_add(*value);
            checksum_b = checksum_b.wrapping_add(checksum_a);
        }
        if self.bytes[2] != 0x05
            || !matches!(self.bytes[3], 0x00 | 0x01)
            || self.bytes[4..6] != [0x02, 0x00]
            || self.bytes[6] != expected_class
            || self.bytes[7] != expected_id
            || self.bytes[8] != checksum_a
            || self.bytes[9] != checksum_b
        {
            return None;
        }
        Some(if self.bytes[3] == 0x01 {
            AckOutcome::Ack
        } else {
            AckOutcome::Nak
        })
    }
}

async fn wait_for_ack(
    uart: &mut GnssUart,
    expected_class: u8,
    expected_id: u8,
) -> Result<AckOutcome, RxError> {
    let mut parser = AckParser::new();
    let mut bytes = [0u8; 32];
    loop {
        let length = uart.read_async(&mut bytes).await?;
        for byte in &bytes[..length] {
            if let Some(outcome) = parser.push(*byte, expected_class, expected_id) {
                return Ok(outcome);
            }
        }
    }
}

#[derive(Debug)]
pub enum GnssSettingError {
    NoWriteProgress,
    Transmit(TxError),
    Receive(RxError),
    AckTimeout { class: u8, id: u8 },
    NegativeAck { class: u8, id: u8 },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct GnssSettingReport {
    pub acknowledged_commands: u8,
    pub final_baud_ack_unverified: bool,
}

pub async fn gnss_setting(tx: &mut GnssUart) -> Result<GnssSettingReport, GnssSettingError> {
    let mut acknowledged_commands = 0u8;
    for (index, &cmd) in INIT_COMMANDS.iter().enumerate() {
        match send_cmd(tx, cmd).await {
            Ok(true) => {}
            Ok(false) => {
                println!("GNSS UART write made no progress");
                return Err(GnssSettingError::NoWriteProgress);
            }
            Err(error) => {
                println!("GNSS UART write error: {:?}", error);
                return Err(GnssSettingError::Transmit(error));
            }
        }
        if let Err(error) = tx.flush_async().await {
            println!("GNSS UART flush error: {:?}", error);
            return Err(GnssSettingError::Transmit(error));
        }
        if index + 1 == INIT_COMMANDS.len() {
            continue;
        }
        let class = cmd[2];
        let id = cmd[3];
        match with_timeout(Duration::from_millis(500), wait_for_ack(tx, class, id)).await {
            Ok(Ok(AckOutcome::Ack)) => {
                acknowledged_commands = acknowledged_commands.saturating_add(1)
            }
            Ok(Ok(AckOutcome::Nak)) => {
                return Err(GnssSettingError::NegativeAck { class, id });
            }
            Ok(Err(error)) => return Err(GnssSettingError::Receive(error)),
            Err(_) => return Err(GnssSettingError::AckTimeout { class, id }),
        }
    }
    Ok(GnssSettingReport {
        acknowledged_commands,
        final_baud_ack_unverified: true,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ack(id: u8, positive: bool) -> [u8; 10] {
        let mut bytes = [
            0xb5,
            0x62,
            0x05,
            if positive { 1 } else { 0 },
            2,
            0,
            0x06,
            id,
            0,
            0,
        ];
        for value in bytes[2..8].to_vec() {
            bytes[8] = bytes[8].wrapping_add(value);
            bytes[9] = bytes[9].wrapping_add(bytes[8]);
        }
        bytes
    }

    #[test]
    fn ack_and_nak_are_distinguished_across_noise() {
        let mut parser = AckParser::new();
        assert_eq!(parser.push(b'$', 0x06, 0x01), None);
        let result = ack(0x01, true)
            .into_iter()
            .find_map(|byte| parser.push(byte, 0x06, 0x01));
        assert_eq!(result, Some(AckOutcome::Ack));
        let result = ack(0x01, false)
            .into_iter()
            .find_map(|byte| parser.push(byte, 0x06, 0x01));
        assert_eq!(result, Some(AckOutcome::Nak));
    }

    #[test]
    fn wrong_ack_target_and_checksum_are_ignored() {
        let mut parser = AckParser::new();
        let mut bytes = ack(0x08, true);
        assert_eq!(
            bytes
                .into_iter()
                .find_map(|byte| parser.push(byte, 0x06, 0x01)),
            None
        );
        bytes = ack(0x01, true);
        bytes[8] ^= 1;
        assert_eq!(
            bytes
                .into_iter()
                .find_map(|byte| parser.push(byte, 0x06, 0x01)),
            None
        );
    }
}
