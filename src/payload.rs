use crate::can::protocol::{CommandPhase, CommandReason};

pub const LORA_PREFIX: [u8; 3] = [0x00, 0x00, 0x04];
pub const MAX_APPLICATION_LENGTH: usize = 24;
pub const MAX_LORA_FRAME_LENGTH: usize = LORA_PREFIX.len() + MAX_APPLICATION_LENGTH;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum PacketHeader {
    CommandReceive = 0xa0,
    LiftoffDetection = 0xa1,
    EngineBurn = 0xa2,
    Control = 0xa3,
    Descent = 0xa4,
    RecoveryBeacon = 0xa5,
    RecoveryLogData = 0xa6,
    CommandResult = 0xb0,
    GroundTimeRequest = 0xb1,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FlightTelemetry {
    pub header: PacketHeader,
    pub status: u16,
    pub roll: u16,
    pub roll_rate: u16,
    pub tilt_magnitude: u8,
    pub tilt_direction: u16,
    pub fin_angle: u8,
    pub fin_rate: u16,
    pub pressure: u16,
    pub temperature: u8,
    pub airspeed: u8,
    pub requested_torque: u16,
    pub elapsed: u8,
    pub east: u16,
    pub north: u16,
    pub height: u16,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CommandReceiveTelemetry {
    pub status: u32,
    pub motor_profile: u8,
    pub tilt_magnitude: u8,
    pub tilt_direction: u16,
    pub fin_mode: u8,
    pub para_mode: u8,
    pub fin_angle: u8,
    pub para_angle: u8,
    pub pressure: u16,
    pub temperature: u8,
    pub airspeed: u8,
    pub logic_voltage: u8,
    pub motor_voltage: u8,
    pub east: u16,
    pub north: u16,
    pub height: u16,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DescentTelemetry {
    pub status: u16,
    pub pressure: u16,
    pub temperature: u8,
    pub para_angle: u8,
    pub elapsed: u16,
    pub east: u16,
    pub north: u16,
    pub height: u16,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RecoveryBeacon {
    pub logic_voltage: u8,
    pub motor_voltage: u8,
    pub east: u16,
    pub north: u16,
    pub height: u16,
    pub elapsed: u16,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RecoveryLogPacket {
    pub transfer_id: u8,
    pub source: bool,
    pub end_of_file: bool,
    pub offset: u32,
    pub data_length: u8,
    pub data: [u8; 16],
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CommandResultPacket {
    pub transaction_id: u8,
    pub command: u8,
    pub phase: CommandPhase,
    pub reason: CommandReason,
    pub detail: u32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ApplicationPacket {
    CommandReceive(CommandReceiveTelemetry),
    Flight(FlightTelemetry),
    Descent(DescentTelemetry),
    RecoveryBeacon(RecoveryBeacon),
    RecoveryLogData(RecoveryLogPacket),
    CommandResult(CommandResultPacket),
    GroundTimeRequest { request_id: u8 },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LoraFrame {
    bytes: [u8; MAX_LORA_FRAME_LENGTH],
    length: usize,
}

impl LoraFrame {
    pub fn as_bytes(&self) -> &[u8] {
        &self.bytes[..self.length]
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EncodeError {
    InvalidHeader,
    OutOfRange,
    ReservedTransactionId,
}

impl ApplicationPacket {
    pub fn encode(self) -> Result<LoraFrame, EncodeError> {
        let mut frame = LoraFrame {
            bytes: [0; MAX_LORA_FRAME_LENGTH],
            length: 0,
        };
        frame.bytes[..LORA_PREFIX.len()].copy_from_slice(&LORA_PREFIX);
        let app = &mut frame.bytes[LORA_PREFIX.len()..];
        let mut writer = BitWriter::new(app);

        match self {
            Self::CommandReceive(value) => encode_command_receive(&mut writer, value)?,
            Self::Flight(value) => encode_flight(&mut writer, value)?,
            Self::Descent(value) => encode_descent(&mut writer, value)?,
            Self::RecoveryBeacon(value) => encode_recovery_beacon(&mut writer, value)?,
            Self::RecoveryLogData(value) => encode_recovery_log(&mut writer, value)?,
            Self::CommandResult(value) => encode_command_result(&mut writer, value)?,
            Self::GroundTimeRequest { request_id } => {
                writer.write(PacketHeader::GroundTimeRequest as u32, 8)?;
                writer.write(request_id as u32, 8)?;
            }
        }

        let application_without_checksum = writer.byte_length();
        let checksum = xor_checksum(&app[..application_without_checksum]);
        app[application_without_checksum] = checksum;
        frame.length = LORA_PREFIX.len() + application_without_checksum + 1;
        Ok(frame)
    }
}

fn encode_command_receive(
    writer: &mut BitWriter<'_>,
    value: CommandReceiveTelemetry,
) -> Result<(), EncodeError> {
    writer.write(PacketHeader::CommandReceive as u32, 8)?;
    writer.write(value.status, 24)?;
    writer.write(value.motor_profile as u32, 8)?;
    writer.write(value.tilt_magnitude as u32, 7)?;
    writer.write(value.tilt_direction as u32, 9)?;
    writer.write(value.fin_mode as u32, 4)?;
    writer.write(value.para_mode as u32, 4)?;
    writer.write(value.fin_angle as u32, 8)?;
    writer.write(value.para_angle as u32, 8)?;
    writer.write(value.pressure as u32, 11)?;
    writer.write(value.temperature as u32, 8)?;
    writer.write(value.airspeed as u32, 8)?;
    writer.write(value.logic_voltage as u32, 8)?;
    writer.write(value.motor_voltage as u32, 8)?;
    writer.write(value.east as u32, 16)?;
    writer.write(value.north as u32, 16)?;
    writer.write(value.height as u32, 9)?;
    writer.write(0, 4)
}

fn encode_flight(writer: &mut BitWriter<'_>, value: FlightTelemetry) -> Result<(), EncodeError> {
    if !matches!(
        value.header,
        PacketHeader::LiftoffDetection | PacketHeader::EngineBurn | PacketHeader::Control
    ) {
        return Err(EncodeError::InvalidHeader);
    }
    writer.write(value.header as u32, 8)?;
    writer.write(value.status as u32, 16)?;
    writer.write(value.roll as u32, 16)?;
    writer.write(value.roll_rate as u32, 16)?;
    writer.write(value.tilt_magnitude as u32, 7)?;
    writer.write(value.tilt_direction as u32, 9)?;
    writer.write(value.fin_angle as u32, 8)?;
    writer.write(value.fin_rate as u32, 16)?;
    writer.write(value.pressure as u32, 11)?;
    writer.write(value.temperature as u32, 8)?;
    writer.write(value.airspeed as u32, 8)?;
    writer.write(value.requested_torque as u32, 12)?;
    writer.write(value.elapsed as u32, 8)?;
    writer.write(value.east as u32, 16)?;
    writer.write(value.north as u32, 16)?;
    writer.write(value.height as u32, 9)
}

fn encode_descent(writer: &mut BitWriter<'_>, value: DescentTelemetry) -> Result<(), EncodeError> {
    writer.write(PacketHeader::Descent as u32, 8)?;
    writer.write(value.status as u32, 13)?;
    writer.write(value.pressure as u32, 11)?;
    writer.write(value.temperature as u32, 8)?;
    writer.write(value.para_angle as u32, 8)?;
    writer.write(value.elapsed as u32, 16)?;
    writer.write(value.east as u32, 16)?;
    writer.write(value.north as u32, 16)?;
    writer.write(value.height as u32, 9)?;
    writer.write(0, 7)
}

fn encode_recovery_beacon(
    writer: &mut BitWriter<'_>,
    value: RecoveryBeacon,
) -> Result<(), EncodeError> {
    writer.write(PacketHeader::RecoveryBeacon as u32, 8)?;
    writer.write(value.logic_voltage as u32, 8)?;
    writer.write(value.motor_voltage as u32, 8)?;
    writer.write(value.east as u32, 16)?;
    writer.write(value.north as u32, 16)?;
    writer.write(value.height as u32, 9)?;
    writer.write(value.elapsed as u32, 16)?;
    writer.write(0, 7)
}

fn encode_recovery_log(
    writer: &mut BitWriter<'_>,
    value: RecoveryLogPacket,
) -> Result<(), EncodeError> {
    if value.offset > 0x00ff_ffff || value.data_length > 16 {
        return Err(EncodeError::OutOfRange);
    }
    writer.write(PacketHeader::RecoveryLogData as u32, 8)?;
    writer.write(value.transfer_id as u32, 8)?;
    let meta = u8::from(value.source) | (u8::from(value.end_of_file) << 1);
    writer.write(meta as u32, 8)?;
    writer.write(value.offset, 24)?;
    writer.write(value.data_length as u32, 8)?;
    for byte in value.data {
        writer.write(byte as u32, 8)?;
    }
    Ok(())
}

fn encode_command_result(
    writer: &mut BitWriter<'_>,
    value: CommandResultPacket,
) -> Result<(), EncodeError> {
    if value.transaction_id == 0 {
        return Err(EncodeError::ReservedTransactionId);
    }
    writer.write(PacketHeader::CommandResult as u32, 8)?;
    writer.write(value.transaction_id as u32, 8)?;
    writer.write(value.command as u32, 8)?;
    writer.write(value.phase as u32, 8)?;
    writer.write(value.reason as u32, 8)?;
    writer.write(value.detail, 32)
}

pub fn xor_checksum(bytes: &[u8]) -> u8 {
    bytes.iter().fold(0, |checksum, byte| checksum ^ byte)
}

struct BitWriter<'a> {
    bytes: &'a mut [u8],
    bit_offset: usize,
}

impl<'a> BitWriter<'a> {
    fn new(bytes: &'a mut [u8]) -> Self {
        bytes.fill(0);
        Self {
            bytes,
            bit_offset: 0,
        }
    }

    fn write(&mut self, value: u32, bit_count: u8) -> Result<(), EncodeError> {
        if bit_count > 32
            || self.bit_offset + bit_count as usize > self.bytes.len() * 8
            || (bit_count < 32 && value >= (1_u32 << bit_count))
        {
            return Err(EncodeError::OutOfRange);
        }
        for bit in 0..bit_count as usize {
            if value & (1 << bit) != 0 {
                let destination = self.bit_offset + bit;
                self.bytes[destination / 8] |= 1 << (destination % 8);
            }
        }
        self.bit_offset += bit_count as usize;
        Ok(())
    }

    fn byte_length(&self) -> usize {
        self.bit_offset.div_ceil(8)
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

    #[test]
    fn canonical_flight_vector_matches() {
        let frame = ApplicationPacket::Flight(FlightTelemetry {
            header: PacketHeader::Control,
            status: 0xa55a,
            roll: 0xfffe,
            roll_rate: 1234,
            tilt_magnitude: 20,
            tilt_direction: 280,
            fin_angle: 120,
            fin_rate: 0xffce,
            pressure: 0x42a,
            temperature: 0x46,
            airspeed: 0x3d,
            requested_torque: 0xf85,
            elapsed: 0x7b,
            east: 0xfff4,
            north: 0x0022,
            height: 0x28,
        })
        .encode()
        .unwrap();
        assert_eq!(frame.as_bytes(), golden("LORA_FLIGHT"));
    }

    #[test]
    fn remaining_canonical_vectors_match() {
        let command_receive = ApplicationPacket::CommandReceive(CommandReceiveTelemetry {
            status: 0xabcdef,
            motor_profile: 3,
            tilt_magnitude: 27,
            tilt_direction: 281,
            fin_mode: 3,
            para_mode: 1,
            fin_angle: 120,
            para_angle: 60,
            pressure: 0x42a,
            temperature: 0x46,
            airspeed: 0xf9,
            logic_voltage: 0xa0,
            motor_voltage: 0xdc,
            east: 0x8001,
            north: 0x8001,
            height: 0x1f1,
        })
        .encode()
        .unwrap();
        assert_eq!(command_receive.as_bytes(), golden("LORA_COMMAND_RECEIVE"));

        let descent = ApplicationPacket::Descent(DescentTelemetry {
            status: 0x1a55,
            pressure: 0x7f7,
            temperature: 0xf7,
            para_angle: 0xf7,
            elapsed: 0xfffa,
            east: 0x8002,
            north: 0xffff,
            height: 0x1f2,
        })
        .encode()
        .unwrap();
        assert_eq!(descent.as_bytes(), golden("LORA_DESCENT"));

        let recovery = ApplicationPacket::RecoveryBeacon(RecoveryBeacon {
            logic_voltage: 0xa0,
            motor_voltage: 0xf0,
            east: 0x0064,
            north: 0xff9c,
            height: 0x03c,
            elapsed: 0x000c,
        })
        .encode()
        .unwrap();
        assert_eq!(recovery.as_bytes(), golden("LORA_RECOVERY"));

        let mut data = [0; 16];
        data[..3].copy_from_slice(&[0xde, 0xad, 0xbe]);
        let log = ApplicationPacket::RecoveryLogData(RecoveryLogPacket {
            transfer_id: 0x34,
            source: true,
            end_of_file: true,
            offset: 0x012345,
            data_length: 3,
            data,
        })
        .encode()
        .unwrap();
        assert_eq!(log.as_bytes(), golden("LORA_LOG_DATA"));

        let result = ApplicationPacket::CommandResult(CommandResultPacket {
            transaction_id: 0x2a,
            command: 0x13,
            phase: CommandPhase::Failed,
            reason: CommandReason::InterruptedByEmergency,
            detail: 0x12345678,
        })
        .encode()
        .unwrap();
        assert_eq!(result.as_bytes(), golden("LORA_COMMAND_RESULT"));

        let request = ApplicationPacket::GroundTimeRequest { request_id: 7 }
            .encode()
            .unwrap();
        assert_eq!(request.as_bytes(), golden("LORA_TIME_REQUEST"));
    }

    #[test]
    fn out_of_range_raw_values_are_rejected() {
        let frame = ApplicationPacket::Flight(FlightTelemetry {
            header: PacketHeader::Control,
            status: 0,
            roll: 0,
            roll_rate: 0,
            tilt_magnitude: 128,
            tilt_direction: 0,
            fin_angle: 0,
            fin_rate: 0,
            pressure: 0,
            temperature: 0,
            airspeed: 0,
            requested_torque: 0,
            elapsed: 0,
            east: 0,
            north: 0,
            height: 0,
        });
        assert_eq!(frame.encode(), Err(EncodeError::OutOfRange));

        let mut semantic_error = match frame {
            ApplicationPacket::Flight(value) => value,
            _ => unreachable!(),
        };
        semantic_error.tilt_magnitude = 121;
        assert!(ApplicationPacket::Flight(semantic_error).encode().is_ok());
        semantic_error.tilt_magnitude = 127;
        assert!(ApplicationPacket::Flight(semantic_error).encode().is_ok());
    }
}
