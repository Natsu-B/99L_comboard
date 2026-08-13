pub const REFERENCE_LATITUDE_E7: i32 = 402_428_650;
pub const REFERENCE_LONGITUDE_E7: i32 = 1_400_104_500;

pub const GNSS_COORDINATE_UNAVAILABLE: u16 = 0x8000;
pub const GNSS_COORDINATE_NO_FIX: u16 = 0x8001;
pub const GNSS_COORDINATE_STALE: u16 = 0x8002;
pub const GNSS_COORDINATE_OUT_OF_RANGE: u16 = 0x8003;
pub const GNSS_COORDINATE_INVALID: u16 = 0x8004;
pub const GNSS_COORDINATE_RECEIVER_ERROR: u16 = 0x8005;
pub const GNSS_HEIGHT_UNAVAILABLE: u16 = 496;
pub const GNSS_HEIGHT_NO_FIX: u16 = 497;
pub const GNSS_HEIGHT_STALE: u16 = 498;
pub const GNSS_HEIGHT_OUT_OF_RANGE: u16 = 499;
pub const GNSS_HEIGHT_INVALID: u16 = 500;
pub const GNSS_HEIGHT_RECEIVER_ERROR: u16 = 501;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EncodedPosition {
    pub east: u16,
    pub north: u16,
    pub height: u16,
}

pub fn encode_position(latitude_e7: i32, longitude_e7: i32, height_m: f32) -> EncodedPosition {
    let north = round_to_i32(
        (f64::from(latitude_e7) - f64::from(REFERENCE_LATITUDE_E7)) * 111_039.303_376
            / 10_000_000.0,
    );
    let east = round_to_i32(
        (f64::from(longitude_e7) - f64::from(REFERENCE_LONGITUDE_E7)) * 85_090.557_487
            / 10_000_000.0,
    );
    EncodedPosition {
        east: encode_coordinate(east),
        north: encode_coordinate(north),
        height: encode_height(height_m),
    }
}

fn round_to_i32(value: f64) -> i32 {
    if value >= 0.0 {
        (value + 0.5) as i32
    } else {
        (value - 0.5) as i32
    }
}

fn encode_coordinate(value: i32) -> u16 {
    if !(-32_752..=32_767).contains(&value) {
        GNSS_COORDINATE_OUT_OF_RANGE
    } else {
        value as i16 as u16
    }
}

fn encode_height(value_m: f32) -> u16 {
    if !value_m.is_finite() {
        return GNSS_HEIGHT_INVALID;
    }
    if !(-100.0..=2_375.0).contains(&value_m) {
        return GNSS_HEIGHT_OUT_OF_RANGE;
    }
    let raw = if value_m >= -100.0 {
        ((value_m + 100.0) / 5.0 + 0.5) as i32
    } else {
        ((value_m + 100.0) / 5.0 - 0.5) as i32
    };
    if (0..=495).contains(&raw) {
        raw as u16
    } else {
        GNSS_HEIGHT_OUT_OF_RANGE
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reference_position_encodes_zero_and_expected_height() {
        assert_eq!(
            encode_position(REFERENCE_LATITUDE_E7, REFERENCE_LONGITUDE_E7, 100.0),
            EncodedPosition {
                east: 0,
                north: 0,
                height: 40
            }
        );
    }

    #[test]
    fn negative_coordinate_uses_twos_complement() {
        let position = encode_position(
            REFERENCE_LATITUDE_E7 - 90,
            REFERENCE_LONGITUDE_E7 - 118,
            -100.0,
        );
        assert_eq!(position.east, 0xffff);
        assert_eq!(position.north, 0xffff);
        assert_eq!(position.height, 0);
    }

    #[test]
    fn reserved_coordinate_region_is_never_numeric() {
        assert_eq!(encode_coordinate(-32_753), GNSS_COORDINATE_OUT_OF_RANGE);
        assert_eq!(encode_coordinate(-32_752), 0x8010);
    }

    #[test]
    fn longitude_difference_does_not_overflow_i32() {
        let position = encode_position(REFERENCE_LATITUDE_E7, -1_800_000_000, 0.0);
        assert_eq!(position.east, GNSS_COORDINATE_OUT_OF_RANGE);
    }

    #[test]
    fn height_checks_physical_boundary_before_rounding() {
        assert_eq!(encode_height(-100.01), GNSS_HEIGHT_OUT_OF_RANGE);
        assert_eq!(encode_height(2_375.01), GNSS_HEIGHT_OUT_OF_RANGE);
    }
}
