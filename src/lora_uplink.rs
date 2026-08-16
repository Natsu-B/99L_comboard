use crate::can::protocol::{GenericCommandRequest, TimeSource};

pub const UPLINK_FRAME_LENGTH: usize = 11;
const UPLINK_HEADER: u8 = 0x55;
const PARA_FREE: u8 = 0x20;
const PARA_HOLD: u8 = 0x21;
const PARA_MOVE_RELATIVE: u8 = 0x22;
const SET_PARA_OPEN: u8 = 0x23;
const SET_PARA_CLOSE: u8 = 0x24;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum UplinkKind {
    MissionGeneric = 0,
    ActuatorEmergency = 1,
    LiftoffDetectionEmergency = 2,
    ComBoardLocal = 3,
    GroundTimeResponse = 4,
}

impl UplinkKind {
    const fn decode(raw: u8) -> Option<Self> {
        match raw {
            0 => Some(Self::MissionGeneric),
            1 => Some(Self::ActuatorEmergency),
            2 => Some(Self::LiftoffDetectionEmergency),
            3 => Some(Self::ComBoardLocal),
            4 => Some(Self::GroundTimeResponse),
            _ => None,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum UplinkCommand {
    MissionGeneric(GenericCommandRequest),
    ActuatorEmergency {
        transaction_id: u8,
    },
    LiftoffDetectionEmergency {
        transaction_id: u8,
    },
    ComBoardLocal {
        transaction_id: u8,
        command: u8,
        args: [u8; 6],
    },
    GroundTimeResponse {
        request_id: u8,
        source: TimeSource,
        unix_seconds: u32,
        milliseconds: u16,
    },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum UplinkDecodeError {
    InvalidHeader,
    InvalidChecksum,
    InvalidKind,
    ReservedTransactionId,
    InvalidField,
}

fn is_deprecated_parachute_command(command: u8) -> bool {
    matches!(
        command,
        PARA_FREE | PARA_HOLD | PARA_MOVE_RELATIVE | SET_PARA_OPEN | SET_PARA_CLOSE
    )
}

pub fn decode_uplink(
    bytes: &[u8; UPLINK_FRAME_LENGTH],
) -> Result<UplinkCommand, UplinkDecodeError> {
    if bytes[0] != UPLINK_HEADER {
        return Err(UplinkDecodeError::InvalidHeader);
    }
    if xor_checksum(&bytes[..UPLINK_FRAME_LENGTH - 1]) != bytes[UPLINK_FRAME_LENGTH - 1] {
        return Err(UplinkDecodeError::InvalidChecksum);
    }
    let kind = UplinkKind::decode(bytes[1]).ok_or(UplinkDecodeError::InvalidKind)?;
    if bytes[2] == 0 {
        return Err(UplinkDecodeError::ReservedTransactionId);
    }
    let mut args = [0; 6];
    args.copy_from_slice(&bytes[4..10]);
    match kind {
        UplinkKind::MissionGeneric => {
            if is_deprecated_parachute_command(bytes[3]) {
                return Err(UplinkDecodeError::InvalidField);
            }
            Ok(UplinkCommand::MissionGeneric(GenericCommandRequest {
                transaction_id: bytes[2],
                command: bytes[3],
                args,
            }))
        }
        UplinkKind::ActuatorEmergency | UplinkKind::LiftoffDetectionEmergency => {
            if bytes[3..10].iter().any(|byte| *byte != 0) {
                return Err(UplinkDecodeError::InvalidField);
            }
            if kind == UplinkKind::ActuatorEmergency {
                Ok(UplinkCommand::ActuatorEmergency {
                    transaction_id: bytes[2],
                })
            } else {
                Ok(UplinkCommand::LiftoffDetectionEmergency {
                    transaction_id: bytes[2],
                })
            }
        }
        UplinkKind::ComBoardLocal => Ok(UplinkCommand::ComBoardLocal {
            transaction_id: bytes[2],
            command: bytes[3],
            args,
        }),
        UplinkKind::GroundTimeResponse => {
            let source = TimeSource::decode(bytes[3]).ok_or(UplinkDecodeError::InvalidField)?;
            let milliseconds = u16::from_le_bytes([bytes[8], bytes[9]]);
            if milliseconds > 999 {
                return Err(UplinkDecodeError::InvalidField);
            }
            Ok(UplinkCommand::GroundTimeResponse {
                request_id: bytes[2],
                source,
                unix_seconds: u32::from_le_bytes([bytes[4], bytes[5], bytes[6], bytes[7]]),
                milliseconds,
            })
        }
    }
}

fn xor_checksum(bytes: &[u8]) -> u8 {
    bytes.iter().fold(0, |checksum, byte| checksum ^ byte)
}

pub struct UplinkFrameBuffer {
    bytes: [u8; UPLINK_FRAME_LENGTH],
    length: usize,
}

impl UplinkFrameBuffer {
    pub const fn new() -> Self {
        Self {
            bytes: [0; UPLINK_FRAME_LENGTH],
            length: 0,
        }
    }

    pub fn push(&mut self, byte: u8) -> Option<Result<UplinkCommand, UplinkDecodeError>> {
        if self.length == 0 {
            if byte != UPLINK_HEADER {
                return None;
            }
            self.bytes[0] = byte;
            self.length = 1;
            return None;
        }

        self.bytes[self.length] = byte;
        self.length += 1;
        if self.length != UPLINK_FRAME_LENGTH {
            return None;
        }

        let result = decode_uplink(&self.bytes);
        if result.is_ok() {
            self.length = 0;
        } else if let Some(next_header) = self.bytes[1..]
            .iter()
            .position(|value| *value == UPLINK_HEADER)
        {
            let next_header = next_header + 1;
            self.bytes.copy_within(next_header.., 0);
            self.length = UPLINK_FRAME_LENGTH - next_header;
        } else {
            self.length = 0;
        }
        Some(result)
    }

    pub fn reset(&mut self) {
        self.length = 0;
    }
}

impl Default for UplinkFrameBuffer {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn golden(name: &str) -> Vec<u8> {
        let line = include_str!("../testdata/99l_protocol_golden_vectors.txt")
            .lines()
            .find(|line| line.starts_with(name) && line.as_bytes().get(name.len()) == Some(&b'='))
            .unwrap();
        let hex = &line[name.len() + 1..];
        (0..hex.len())
            .step_by(2)
            .map(|index| u8::from_str_radix(&hex[index..index + 2], 16).unwrap())
            .collect()
    }

    fn as_frame(bytes: Vec<u8>) -> [u8; UPLINK_FRAME_LENGTH] {
        bytes.try_into().unwrap()
    }

    #[test]
    fn canonical_vectors_decode() {
        assert!(matches!(
            decode_uplink(&as_frame(golden("UPLINK_GENERIC"))),
            Ok(UplinkCommand::MissionGeneric(GenericCommandRequest {
                transaction_id: 0x2a,
                command: 0x13,
                args: [0x85, 0xff, 0, 0, 0, 0]
            }))
        ));
        assert!(matches!(
            decode_uplink(&as_frame(golden("UPLINK_EMERGENCY"))),
            Ok(UplinkCommand::ActuatorEmergency {
                transaction_id: 0x2b
            })
        ));
        assert!(matches!(
            decode_uplink(&as_frame(golden("UPLINK_TIME"))),
            Ok(UplinkCommand::GroundTimeResponse {
                request_id: 7,
                source: TimeSource::Ground,
                unix_seconds: 0x12345678,
                milliseconds: 999
            })
        ));
    }

    #[test]
    fn deprecated_parachute_commands_are_rejected() {
        for command in [
            PARA_FREE,
            PARA_HOLD,
            PARA_MOVE_RELATIVE,
            SET_PARA_OPEN,
            SET_PARA_CLOSE,
        ] {
            let mut bytes = as_frame(golden("UPLINK_GENERIC"));
            bytes[3] = command;
            bytes[4..10].fill(0);
            bytes[10] = xor_checksum(&bytes[..10]);
            assert_eq!(decode_uplink(&bytes), Err(UplinkDecodeError::InvalidField));
        }

        for command in [0x25, 0x26] {
            let mut bytes = as_frame(golden("UPLINK_GENERIC"));
            bytes[3] = command;
            bytes[4..10].fill(0);
            bytes[10] = xor_checksum(&bytes[..10]);
            assert!(matches!(
                decode_uplink(&bytes),
                Ok(UplinkCommand::MissionGeneric(GenericCommandRequest { command: decoded, .. })) if decoded == command
            ));
        }
    }

    #[test]
    fn payload_header_byte_does_not_break_fixed_length_frame() {
        let mut bytes = as_frame(golden("UPLINK_GENERIC"));
        bytes[5] = UPLINK_HEADER;
        bytes[10] = xor_checksum(&bytes[..10]);
        let mut buffer = UplinkFrameBuffer::new();
        let result = bytes.into_iter().find_map(|byte| buffer.push(byte));
        assert!(matches!(result, Some(Ok(UplinkCommand::MissionGeneric(_)))));
    }

    #[test]
    fn checksum_failure_resynchronizes_at_next_header() {
        let mut invalid = as_frame(golden("UPLINK_GENERIC"));
        invalid[10] ^= 1;
        let valid = as_frame(golden("UPLINK_TIME"));
        let mut buffer = UplinkFrameBuffer::new();
        let mut outputs = Vec::new();
        for byte in invalid.into_iter().chain(valid) {
            if let Some(result) = buffer.push(byte) {
                outputs.push(result);
            }
        }
        assert_eq!(outputs.len(), 2);
        assert_eq!(outputs[0], Err(UplinkDecodeError::InvalidChecksum));
        assert!(matches!(
            outputs[1],
            Ok(UplinkCommand::GroundTimeResponse { .. })
        ));
    }

    #[test]
    fn malformed_fields_are_rejected() {
        let mut bytes = as_frame(golden("UPLINK_EMERGENCY"));
        bytes[2] = 0;
        bytes[10] = xor_checksum(&bytes[..10]);
        assert_eq!(
            decode_uplink(&bytes),
            Err(UplinkDecodeError::ReservedTransactionId)
        );

        bytes = as_frame(golden("UPLINK_EMERGENCY"));
        bytes[4] = 1;
        bytes[10] = xor_checksum(&bytes[..10]);
        assert_eq!(decode_uplink(&bytes), Err(UplinkDecodeError::InvalidField));

        bytes = as_frame(golden("UPLINK_TIME"));
        bytes[8..10].copy_from_slice(&1000_u16.to_le_bytes());
        bytes[10] = xor_checksum(&bytes[..10]);
        assert_eq!(decode_uplink(&bytes), Err(UplinkDecodeError::InvalidField));
    }
}
