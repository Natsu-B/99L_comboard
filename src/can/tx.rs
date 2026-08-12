use embassy_time::{Duration, with_timeout};
use esp_hal::{
    Async,
    twai::{self, EspTwaiError, EspTwaiFrame, StandardId},
};

use super::protocol::{CanEncodeError, CanTxMessage};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CanFrameError {
    InvalidId(u16),
    InvalidPayloadLength(usize),
    InvalidPayload(CanEncodeError),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CanTxError {
    FrameCreateFailed,
    TransmitFailed,
    TimedOutUnknownState,
    BusOff,
}

pub async fn transmit_message_with_timeout(
    can: &mut twai::Twai<'static, Async>,
    message: CanTxMessage,
    timeout: Duration,
) -> Result<(), CanTxError> {
    let frame = create_frame_from_message(message).map_err(|_| CanTxError::FrameCreateFailed)?;
    transmit_frame_with_timeout(can, &frame, timeout).await
}

pub fn create_frame_from_message(msg: CanTxMessage) -> Result<EspTwaiFrame, CanFrameError> {
    let mut payload = [0u8; 8];
    let len = msg
        .encode_payload(&mut payload)
        .map_err(CanFrameError::InvalidPayload)?;
    let id = StandardId::new(msg.id()).ok_or(CanFrameError::InvalidId(msg.id()))?;
    EspTwaiFrame::new(id, &payload[..len]).ok_or(CanFrameError::InvalidPayloadLength(len))
}

pub async fn transmit_frame_with_timeout(
    can: &mut twai::Twai<'static, Async>,
    frame: &EspTwaiFrame,
    timeout: Duration,
) -> Result<(), CanTxError> {
    match with_timeout(timeout, can.transmit_async(frame)).await {
        Ok(Ok(())) => Ok(()),
        Ok(Err(EspTwaiError::BusOff)) => Err(CanTxError::BusOff),
        Ok(Err(_)) => Err(CanTxError::TransmitFailed),
        Err(_) => Err(CanTxError::TimedOutUnknownState),
    }
}
