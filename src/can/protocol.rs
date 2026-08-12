pub const CAN_ID_ACTUATOR_EMERGENCY_STOP: u16 = 0x001;
pub const CAN_ID_LIFTOFF_EMERGENCY_STOP: u16 = 0x002;
pub const CAN_ID_RECOVERY_CONTROL: u16 = 0x008;
pub const CAN_ID_GENERIC_COMMAND_REQUEST: u16 = 0x010;
pub const CAN_ID_COMMAND_RESULT: u16 = 0x011;
pub const CAN_ID_TIME_REQUEST: u16 = 0x012;
pub const CAN_ID_TIME_RESPONSE: u16 = 0x013;
pub const CAN_ID_MISSION_EVENT: u16 = 0x020;
pub const CAN_ID_KINEMATICS: u16 = 0x100;
pub const CAN_ID_CONTROL: u16 = 0x101;
pub const CAN_ID_MISSION_STATUS: u16 = 0x102;
pub const CAN_ID_POWER_TIME: u16 = 0x103;
pub const CAN_ID_DESCENT_CORE: u16 = 0x104;
pub const CAN_ID_RECOVERY_STATUS: u16 = 0x105;
pub const CAN_ID_RECOVERY_LOG_DATA: u16 = 0x106;
pub const CAN_ID_ATTITUDE_TILT: u16 = 0x107;
pub const CAN_ID_LPS: u16 = 0x108;
pub const CAN_ID_AIRSPEED: u16 = 0x109;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum MissionState {
    CommandReceive = 0,
    LiftoffDetection = 1,
    EngineBurn = 2,
    Control = 3,
    Descent = 4,
    Unknown = 0xff,
}

impl MissionState {
    pub const fn decode(raw: u8) -> Option<Self> {
        match raw {
            0 => Some(Self::CommandReceive),
            1 => Some(Self::LiftoffDetection),
            2 => Some(Self::EngineBurn),
            3 => Some(Self::Control),
            4 => Some(Self::Descent),
            0xff => Some(Self::Unknown),
            _ => None,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum CommandPhase {
    Accepted = 0,
    Completed = 1,
    Rejected = 2,
    Failed = 3,
}

impl CommandPhase {
    pub const fn decode(raw: u8) -> Option<Self> {
        match raw {
            0 => Some(Self::Accepted),
            1 => Some(Self::Completed),
            2 => Some(Self::Rejected),
            3 => Some(Self::Failed),
            _ => None,
        }
    }

    pub const fn is_final(self) -> bool {
        !matches!(self, Self::Accepted)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum CommandReason {
    None = 0,
    Busy = 1,
    InvalidState = 2,
    InvalidArgument = 3,
    NotConfigured = 4,
    DeviceUnavailable = 5,
    Timeout = 6,
    Stall = 7,
    ProtocolError = 8,
    InterruptedByEmergency = 9,
    PersistenceError = 10,
    InternalError = 11,
    NotSupported = 12,
    SafetyInterlock = 13,
    AlreadySatisfied = 14,
}

impl CommandReason {
    pub const fn decode(raw: u8) -> Option<Self> {
        match raw {
            0 => Some(Self::None),
            1 => Some(Self::Busy),
            2 => Some(Self::InvalidState),
            3 => Some(Self::InvalidArgument),
            4 => Some(Self::NotConfigured),
            5 => Some(Self::DeviceUnavailable),
            6 => Some(Self::Timeout),
            7 => Some(Self::Stall),
            8 => Some(Self::ProtocolError),
            9 => Some(Self::InterruptedByEmergency),
            10 => Some(Self::PersistenceError),
            11 => Some(Self::InternalError),
            12 => Some(Self::NotSupported),
            13 => Some(Self::SafetyInterlock),
            14 => Some(Self::AlreadySatisfied),
            _ => None,
        }
    }
}

macro_rules! wire_enum {
    ($name:ident { $($value:ident = $raw:expr),+ $(,)? }) => {
        #[derive(Clone, Copy, Debug, PartialEq, Eq)]
        #[repr(u8)]
        pub enum $name {
            $($value = $raw),+
        }

        impl $name {
            pub const fn decode(raw: u8) -> Option<Self> {
                match raw {
                    $($raw => Some(Self::$value),)+
                    _ => None,
                }
            }
        }
    };
}

wire_enum!(FinMode {
    Coast = 0,
    Brake = 1,
    RollControl = 2,
    ZeroHold = 3,
    Disabled = 4,
    StoppedByLimit = 5,
    Unknown = 15,
});

wire_enum!(ParaMode {
    Idle = 0,
    Opening = 1,
    HoldingOpen = 2,
    Closing = 3,
    PowerOff = 4,
    Fault = 5,
    Unknown = 15,
});

wire_enum!(TimeSource {
    Invalid = 0,
    Gnss = 1,
    Ground = 2,
});

wire_enum!(RecoveryOpcode {
    QueryStatus = 0,
    Start = 1,
    Read = 2,
    Abort = 3,
});

wire_enum!(RecoverySource {
    MissionFlash = 0,
    CommunicationSd = 1,
});

wire_enum!(RecoveryStatusCode {
    Ready = 0,
    Active = 1,
    Completed = 2,
    InvalidRequest = 3,
    Busy = 4,
    NotAvailable = 5,
    OutOfRange = 6,
    IoError = 7,
    Aborted = 8,
    Timeout = 9,
});

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct GenericCommandRequest {
    pub transaction_id: u8,
    pub command: u8,
    pub args: [u8; 6],
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RecoveryControl {
    pub opcode: RecoveryOpcode,
    pub source: RecoverySource,
    pub transfer_id: u8,
    pub offset: u32,
    pub length: u32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CanTxMessage {
    ActuatorEmergencyStop {
        transaction_id: u8,
    },
    LiftoffEmergencyStop {
        transaction_id: u8,
    },
    RecoveryControl(RecoveryControl),
    GenericCommandRequest(GenericCommandRequest),
    TimeResponse {
        request_id: u8,
        source: TimeSource,
        unix_seconds: u32,
        milliseconds: u16,
    },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CommandResult {
    pub transaction_id: u8,
    pub command: u8,
    pub phase: CommandPhase,
    pub reason: CommandReason,
    pub detail: u32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MissionEvent {
    pub sequence: u8,
    pub flags: u16,
    pub state: MissionState,
    pub elapsed: u16,
    pub detail: u16,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct KinematicsTelemetry {
    pub sequence: u8,
    pub roll: u16,
    pub roll_rate: u16,
    pub fin_angle: u8,
    pub fin_rate: u16,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ControlTelemetry {
    pub sequence: u8,
    pub requested_torque: u16,
    pub elapsed: u8,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MissionStatusTelemetry {
    pub sequence: u8,
    pub state: MissionState,
    pub status: u16,
    pub config: u8,
    pub fin_mode: FinMode,
    pub para_mode: ParaMode,
    pub para_angle: u8,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PowerTimeTelemetry {
    pub sequence: u8,
    pub logic_voltage: u8,
    pub motor_voltage: u8,
    pub descent_elapsed: u16,
    pub recovery_elapsed: u16,
    pub flags: u8,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DescentCoreTelemetry {
    pub sequence: u8,
    pub status: u16,
    pub para_angle: u8,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RecoveryStatus {
    pub opcode: RecoveryOpcode,
    pub transfer_id: u8,
    pub status: RecoveryStatusCode,
    pub source: RecoverySource,
    pub total_size: u32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RecoveryLogData {
    pub transfer_id: u8,
    pub sequence: u8,
    pub data: [u8; 6],
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AttitudeTiltTelemetry {
    pub sequence: u8,
    pub magnitude: u8,
    pub direction: u16,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LpsTelemetry {
    pub sequence: u8,
    pub pressure: u16,
    pub temperature: u8,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AirspeedTelemetry {
    pub sequence: u8,
    pub airspeed: u8,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CanRxMessage {
    CommandResult(CommandResult),
    TimeRequest { request_id: u8 },
    MissionEvent(MissionEvent),
    Kinematics(KinematicsTelemetry),
    Control(ControlTelemetry),
    MissionStatus(MissionStatusTelemetry),
    PowerTime(PowerTimeTelemetry),
    DescentCore(DescentCoreTelemetry),
    RecoveryStatus(RecoveryStatus),
    RecoveryLogData(RecoveryLogData),
    AttitudeTilt(AttitudeTiltTelemetry),
    Lps(LpsTelemetry),
    Airspeed(AirspeedTelemetry),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CanDecodeError {
    UnknownId(u16),
    InvalidDlc {
        id: u16,
        expected: usize,
        actual: usize,
    },
    InvalidField {
        id: u16,
        index: usize,
        value: u8,
    },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CanEncodeError {
    ReservedTransactionId,
    OutOfRange,
}

impl CanTxMessage {
    pub const fn id(self) -> u16 {
        match self {
            Self::ActuatorEmergencyStop { .. } => CAN_ID_ACTUATOR_EMERGENCY_STOP,
            Self::LiftoffEmergencyStop { .. } => CAN_ID_LIFTOFF_EMERGENCY_STOP,
            Self::RecoveryControl(_) => CAN_ID_RECOVERY_CONTROL,
            Self::GenericCommandRequest(_) => CAN_ID_GENERIC_COMMAND_REQUEST,
            Self::TimeResponse { .. } => CAN_ID_TIME_RESPONSE,
        }
    }

    pub const fn dlc(self) -> usize {
        match self {
            Self::ActuatorEmergencyStop { .. } | Self::LiftoffEmergencyStop { .. } => 1,
            Self::RecoveryControl(_)
            | Self::GenericCommandRequest(_)
            | Self::TimeResponse { .. } => 8,
        }
    }

    pub fn encode_payload(self, out: &mut [u8; 8]) -> Result<usize, CanEncodeError> {
        out.fill(0);
        match self {
            Self::ActuatorEmergencyStop { transaction_id }
            | Self::LiftoffEmergencyStop { transaction_id } => {
                require_transaction_id(transaction_id)?;
                out[0] = transaction_id;
            }
            Self::RecoveryControl(control) => {
                if control.offset > 0x00ff_ffff || control.length > 0x00ff_ffff {
                    return Err(CanEncodeError::OutOfRange);
                }
                out[0] = control.opcode as u8 | ((control.source as u8) << 4);
                out[1] = control.transfer_id;
                put_u24_le(&mut out[2..5], control.offset);
                put_u24_le(&mut out[5..8], control.length);
            }
            Self::GenericCommandRequest(request) => {
                require_transaction_id(request.transaction_id)?;
                out[0] = request.transaction_id;
                out[1] = request.command;
                out[2..].copy_from_slice(&request.args);
            }
            Self::TimeResponse {
                request_id,
                source,
                unix_seconds,
                milliseconds,
            } => {
                if milliseconds > 999 {
                    return Err(CanEncodeError::OutOfRange);
                }
                out[0] = request_id;
                out[1] = source as u8;
                out[2..6].copy_from_slice(&unix_seconds.to_le_bytes());
                out[6..8].copy_from_slice(&milliseconds.to_le_bytes());
            }
        }
        Ok(self.dlc())
    }
}

impl CanRxMessage {
    pub fn decode_standard(id: u16, data: &[u8]) -> Result<Self, CanDecodeError> {
        match id {
            CAN_ID_COMMAND_RESULT => decode_command_result(id, data),
            CAN_ID_TIME_REQUEST => {
                require_dlc(id, data, 1)?;
                require_received_transaction_id(id, 0, data[0])?;
                Ok(Self::TimeRequest {
                    request_id: data[0],
                })
            }
            CAN_ID_MISSION_EVENT => {
                require_dlc(id, data, 8)?;
                let state = decode_field(id, 3, data[3], MissionState::decode)?;
                Ok(Self::MissionEvent(MissionEvent {
                    sequence: data[0],
                    flags: get_u16(&data[1..3]),
                    state,
                    elapsed: get_u16(&data[4..6]),
                    detail: get_u16(&data[6..8]),
                }))
            }
            CAN_ID_KINEMATICS => {
                require_dlc(id, data, 8)?;
                Ok(Self::Kinematics(KinematicsTelemetry {
                    sequence: data[0],
                    roll: get_u16(&data[1..3]),
                    roll_rate: get_u16(&data[3..5]),
                    fin_angle: data[5],
                    fin_rate: get_u16(&data[6..8]),
                }))
            }
            CAN_ID_CONTROL => {
                require_dlc(id, data, 4)?;
                require_reserved_zero(id, 2, data[2], 0xf0)?;
                Ok(Self::Control(ControlTelemetry {
                    sequence: data[0],
                    requested_torque: get_u16(&data[1..3]),
                    elapsed: data[3],
                }))
            }
            CAN_ID_MISSION_STATUS => {
                require_dlc(id, data, 8)?;
                let state = decode_field(id, 1, data[1], MissionState::decode)?;
                let fin_mode = decode_field(id, 5, data[5], FinMode::decode)?;
                let para_mode = decode_field(id, 6, data[6], ParaMode::decode)?;
                Ok(Self::MissionStatus(MissionStatusTelemetry {
                    sequence: data[0],
                    state,
                    status: get_u16(&data[2..4]),
                    config: data[4],
                    fin_mode,
                    para_mode,
                    para_angle: data[7],
                }))
            }
            CAN_ID_POWER_TIME => {
                require_dlc(id, data, 8)?;
                Ok(Self::PowerTime(PowerTimeTelemetry {
                    sequence: data[0],
                    logic_voltage: data[1],
                    motor_voltage: data[2],
                    descent_elapsed: get_u16(&data[3..5]),
                    recovery_elapsed: get_u16(&data[5..7]),
                    flags: data[7],
                }))
            }
            CAN_ID_DESCENT_CORE => {
                require_dlc(id, data, 4)?;
                require_reserved_zero(id, 2, data[2], 0xe0)?;
                Ok(Self::DescentCore(DescentCoreTelemetry {
                    sequence: data[0],
                    status: get_u16(&data[1..3]),
                    para_angle: data[3],
                }))
            }
            CAN_ID_RECOVERY_STATUS => {
                require_dlc(id, data, 8)?;
                let opcode = decode_field(id, 0, data[0], RecoveryOpcode::decode)?;
                let status = decode_field(id, 2, data[2], RecoveryStatusCode::decode)?;
                let source = decode_field(id, 3, data[3], RecoverySource::decode)?;
                Ok(Self::RecoveryStatus(RecoveryStatus {
                    opcode,
                    transfer_id: data[1],
                    status,
                    source,
                    total_size: u32::from_le_bytes([data[4], data[5], data[6], data[7]]),
                }))
            }
            CAN_ID_RECOVERY_LOG_DATA => {
                require_dlc(id, data, 8)?;
                let mut payload = [0; 6];
                payload.copy_from_slice(&data[2..8]);
                Ok(Self::RecoveryLogData(RecoveryLogData {
                    transfer_id: data[0],
                    sequence: data[1],
                    data: payload,
                }))
            }
            CAN_ID_ATTITUDE_TILT => {
                require_dlc(id, data, 3)?;
                let packed = get_u16(&data[1..3]);
                Ok(Self::AttitudeTilt(AttitudeTiltTelemetry {
                    sequence: data[0],
                    magnitude: (packed & 0x7f) as u8,
                    direction: packed >> 7,
                }))
            }
            CAN_ID_LPS => {
                require_dlc(id, data, 4)?;
                require_reserved_zero(id, 2, data[2], 0xf8)?;
                Ok(Self::Lps(LpsTelemetry {
                    sequence: data[0],
                    pressure: get_u16(&data[1..3]),
                    temperature: data[3],
                }))
            }
            CAN_ID_AIRSPEED => {
                require_dlc(id, data, 2)?;
                Ok(Self::Airspeed(AirspeedTelemetry {
                    sequence: data[0],
                    airspeed: data[1],
                }))
            }
            _ => Err(CanDecodeError::UnknownId(id)),
        }
    }
}

fn decode_command_result(id: u16, data: &[u8]) -> Result<CanRxMessage, CanDecodeError> {
    require_dlc(id, data, 8)?;
    require_received_transaction_id(id, 0, data[0])?;
    let phase = decode_field(id, 2, data[2], CommandPhase::decode)?;
    let reason = decode_field(id, 3, data[3], CommandReason::decode)?;
    Ok(CanRxMessage::CommandResult(CommandResult {
        transaction_id: data[0],
        command: data[1],
        phase,
        reason,
        detail: u32::from_le_bytes([data[4], data[5], data[6], data[7]]),
    }))
}

fn require_transaction_id(transaction_id: u8) -> Result<(), CanEncodeError> {
    if transaction_id == 0 {
        Err(CanEncodeError::ReservedTransactionId)
    } else {
        Ok(())
    }
}

fn require_received_transaction_id(
    id: u16,
    index: usize,
    transaction_id: u8,
) -> Result<(), CanDecodeError> {
    if transaction_id == 0 {
        Err(CanDecodeError::InvalidField {
            id,
            index,
            value: transaction_id,
        })
    } else {
        Ok(())
    }
}

fn decode_field<T>(
    id: u16,
    index: usize,
    raw: u8,
    decode: impl FnOnce(u8) -> Option<T>,
) -> Result<T, CanDecodeError> {
    decode(raw).ok_or(CanDecodeError::InvalidField {
        id,
        index,
        value: raw,
    })
}

fn require_reserved_zero(
    id: u16,
    index: usize,
    value: u8,
    reserved_mask: u8,
) -> Result<(), CanDecodeError> {
    if value & reserved_mask == 0 {
        Ok(())
    } else {
        Err(CanDecodeError::InvalidField { id, index, value })
    }
}

fn require_dlc(id: u16, data: &[u8], expected: usize) -> Result<(), CanDecodeError> {
    if data.len() == expected {
        Ok(())
    } else {
        Err(CanDecodeError::InvalidDlc {
            id,
            expected,
            actual: data.len(),
        })
    }
}

fn get_u16(data: &[u8]) -> u16 {
    u16::from_le_bytes([data[0], data[1]])
}

fn put_u24_le(out: &mut [u8], value: u32) {
    let bytes = value.to_le_bytes();
    out.copy_from_slice(&bytes[..3]);
}

#[cfg(test)]
mod tests {
    use super::*;

    fn golden(name: &str) -> Vec<u8> {
        let line = include_str!("../../testdata/99l_protocol_golden_vectors.txt")
            .lines()
            .find(|line| line.starts_with(name) && line.as_bytes().get(name.len()) == Some(&b'='))
            .unwrap();
        let hex = &line[name.len() + 1..];
        (0..hex.len())
            .step_by(2)
            .map(|index| u8::from_str_radix(&hex[index..index + 2], 16).unwrap())
            .collect()
    }

    fn encoded(message: CanTxMessage) -> Vec<u8> {
        let mut bytes = [0; 8];
        let length = message.encode_payload(&mut bytes).unwrap();
        bytes[..length].to_vec()
    }

    #[test]
    fn canonical_can_tx_vectors_match() {
        assert_eq!(
            encoded(CanTxMessage::ActuatorEmergencyStop {
                transaction_id: 0x2a
            }),
            golden("CAN_001")
        );
        assert_eq!(
            encoded(CanTxMessage::LiftoffEmergencyStop {
                transaction_id: 0x2b
            }),
            golden("CAN_002")
        );
        assert_eq!(
            encoded(CanTxMessage::RecoveryControl(RecoveryControl {
                opcode: RecoveryOpcode::Read,
                source: RecoverySource::CommunicationSd,
                transfer_id: 0x34,
                offset: 0x012345,
                length: 0x000456,
            })),
            golden("CAN_008")
        );
        assert_eq!(
            encoded(CanTxMessage::GenericCommandRequest(GenericCommandRequest {
                transaction_id: 0x2a,
                command: 0x13,
                args: [0x85, 0xff, 0, 0, 0, 0],
            })),
            golden("CAN_010")
        );
        assert_eq!(
            encoded(CanTxMessage::TimeResponse {
                request_id: 7,
                source: TimeSource::Ground,
                unix_seconds: 0x12345678,
                milliseconds: 999,
            }),
            golden("CAN_013")
        );
    }

    #[test]
    fn canonical_can_rx_vectors_decode() {
        for (name, id) in [
            ("CAN_011", CAN_ID_COMMAND_RESULT),
            ("CAN_012", CAN_ID_TIME_REQUEST),
            ("CAN_020", CAN_ID_MISSION_EVENT),
            ("CAN_100", CAN_ID_KINEMATICS),
            ("CAN_101", CAN_ID_CONTROL),
            ("CAN_102", CAN_ID_MISSION_STATUS),
            ("CAN_103", CAN_ID_POWER_TIME),
            ("CAN_104", CAN_ID_DESCENT_CORE),
            ("CAN_105", CAN_ID_RECOVERY_STATUS),
            ("CAN_106", CAN_ID_RECOVERY_LOG_DATA),
            ("CAN_107", CAN_ID_ATTITUDE_TILT),
            ("CAN_108", CAN_ID_LPS),
            ("CAN_109", CAN_ID_AIRSPEED),
        ] {
            assert!(
                CanRxMessage::decode_standard(id, &golden(name)).is_ok(),
                "{name}"
            );
        }
    }

    #[test]
    fn malformed_reserved_bits_are_rejected() {
        assert!(matches!(
            CanRxMessage::decode_standard(CAN_ID_CONTROL, &[0, 0, 0x10, 0]),
            Err(CanDecodeError::InvalidField { .. })
        ));
        assert!(matches!(
            CanRxMessage::decode_standard(CAN_ID_DESCENT_CORE, &[0, 0, 0x20, 0]),
            Err(CanDecodeError::InvalidField { .. })
        ));
        assert!(matches!(
            CanRxMessage::decode_standard(CAN_ID_LPS, &[0, 0, 0x08, 0]),
            Err(CanDecodeError::InvalidField { .. })
        ));
    }

    #[test]
    fn malformed_enums_and_ids_are_rejected() {
        let mut status = golden("CAN_102");
        status[5] = 6;
        assert!(CanRxMessage::decode_standard(CAN_ID_MISSION_STATUS, &status).is_err());
        status[5] = FinMode::Coast as u8;
        status[6] = 6;
        assert!(CanRxMessage::decode_standard(CAN_ID_MISSION_STATUS, &status).is_err());

        let mut recovery = golden("CAN_105");
        recovery[2] = 10;
        assert!(CanRxMessage::decode_standard(CAN_ID_RECOVERY_STATUS, &recovery).is_err());
        recovery[2] = RecoveryStatusCode::Ready as u8;
        recovery[3] = 2;
        assert!(CanRxMessage::decode_standard(CAN_ID_RECOVERY_STATUS, &recovery).is_err());

        assert!(CanRxMessage::decode_standard(CAN_ID_TIME_REQUEST, &[0]).is_err());
        let mut result = golden("CAN_011");
        result[0] = 0;
        assert!(CanRxMessage::decode_standard(CAN_ID_COMMAND_RESULT, &result).is_err());
        assert_eq!(
            CanTxMessage::ActuatorEmergencyStop { transaction_id: 0 }.encode_payload(&mut [0; 8]),
            Err(CanEncodeError::ReservedTransactionId)
        );
    }

    #[test]
    fn unknown_mission_state_is_valid() {
        let mut status = golden("CAN_102");
        status[1] = MissionState::Unknown as u8;
        assert!(CanRxMessage::decode_standard(CAN_ID_MISSION_STATUS, &status).is_ok());
    }
}
