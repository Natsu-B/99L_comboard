use core::str::FromStr;

#[derive(Debug, PartialEq)]
pub enum GgaParseError {
    InvalidStart,
    MissingField,
    InvalidChecksum,
    ParseError,
    InvalidTalker,
    InvalidHemisphere,
    InvalidUnit,
}

pub type NmeaParseError = GgaParseError;

#[derive(Debug, Default, Clone, Copy, PartialEq)]
#[repr(u8)]
pub enum FixQuality {
    #[default]
    Invalid = 0,
    Gps = 1,
    Dgps = 2,
    Unknown(u8),
}

impl From<u8> for FixQuality {
    fn from(v: u8) -> Self {
        match v {
            0 => FixQuality::Invalid,
            1 => FixQuality::Gps,
            2 => FixQuality::Dgps,
            x => FixQuality::Unknown(x),
        }
    }
}

#[derive(Debug, Default, PartialEq)]
pub struct UtcTime {
    pub hour: u8,
    pub minute: u8,
    pub second: f32,
}

#[derive(Debug, Default, PartialEq)]
pub struct GgaData {
    pub utc_time: Option<UtcTime>,
    pub latitude: Option<i32>,
    pub longitude: Option<i32>,
    pub fix_quality: FixQuality,
    pub satellites: u8,
    pub altitude: Option<f32>,
    pub geoid_sep: Option<f32>,
}

impl GgaData {
    pub fn ellipsoid_height(&self) -> Option<f32> {
        if let (Some(alt), Some(geoid)) = (self.altitude, self.geoid_sep) {
            Some(alt + geoid)
        } else {
            None
        }
    }
}

fn parse_num<T: FromStr>(bytes: &[u8]) -> Result<T, GgaParseError> {
    let s = core::str::from_utf8(bytes).map_err(|_| GgaParseError::ParseError)?;
    s.parse::<T>().map_err(|_| GgaParseError::ParseError)
}

fn parse_optional_f32(bytes: &[u8]) -> Result<Option<f32>, GgaParseError> {
    if bytes.is_empty() {
        Ok(None)
    } else {
        Ok(Some(parse_num::<f32>(bytes)?))
    }
}

fn parse_utc_time(bytes: &[u8]) -> Result<UtcTime, GgaParseError> {
    if bytes.len() < 6 {
        return Err(GgaParseError::ParseError);
    }

    let value = UtcTime {
        hour: parse_num::<u8>(&bytes[0..2])?,
        minute: parse_num::<u8>(&bytes[2..4])?,
        second: parse_num::<f32>(&bytes[4..])?,
    };
    if value.hour >= 24 || value.minute >= 60 || !(0.0..60.0).contains(&value.second) {
        return Err(GgaParseError::ParseError);
    }
    Ok(value)
}

fn nmea_to_decimal(nmea_val: f64, hemisphere: u8) -> Result<f64, GgaParseError> {
    let degrees = (nmea_val / 100.0) as i32 as f64;
    let minutes = nmea_val - degrees * 100.0;
    let maximum_degrees = if matches!(hemisphere, b'N' | b'S') {
        90.0
    } else if matches!(hemisphere, b'E' | b'W') {
        180.0
    } else {
        return Err(GgaParseError::InvalidHemisphere);
    };
    if nmea_val < 0.0
        || !(0.0..60.0).contains(&minutes)
        || degrees > maximum_degrees
        || (degrees == maximum_degrees && minutes != 0.0)
    {
        return Err(GgaParseError::ParseError);
    }
    let decimal = degrees + minutes / 60.0;

    match hemisphere {
        b'N' | b'E' => Ok(decimal),
        b'S' | b'W' => Ok(-decimal),
        _ => Err(GgaParseError::InvalidHemisphere),
    }
}

#[cfg(test)]
mod validation_tests {
    use super::*;

    #[test]
    fn utc_ranges_are_validated() {
        assert!(parse_utc_time(b"235959.9").is_ok());
        assert!(parse_utc_time(b"240000").is_err());
        assert!(parse_utc_time(b"126000").is_err());
    }

    #[test]
    fn coordinate_minutes_and_degrees_are_validated() {
        assert!(nmea_to_decimal(4_059.999, b'N').is_ok());
        assert!(nmea_to_decimal(4_060.0, b'N').is_err());
        assert!(nmea_to_decimal(9_001.0, b'N').is_err());
        assert!(nmea_to_decimal(18_001.0, b'E').is_err());
    }
}

fn f64_to_gps_i32(degrees: f64) -> i32 {
    let scaled = degrees * 10_000_000.0;

    if scaled >= 0.0 {
        (scaled + 0.5) as i32
    } else {
        (scaled - 0.5) as i32
    }
}

fn verify_checksum(sentence: &[u8]) -> Result<usize, GgaParseError> {
    if sentence.is_empty() || sentence[0] != b'$' {
        return Err(GgaParseError::InvalidStart);
    }

    let star_idx = sentence
        .iter()
        .position(|&c| c == b'*')
        .ok_or(GgaParseError::MissingField)?;

    if star_idx + 3 > sentence.len() {
        return Err(GgaParseError::ParseError);
    }

    let hex_bytes = &sentence[star_idx + 1..star_idx + 3];
    let hex_str = core::str::from_utf8(hex_bytes).map_err(|_| GgaParseError::ParseError)?;
    let expected = u8::from_str_radix(hex_str, 16).map_err(|_| GgaParseError::ParseError)?;

    let mut calc = 0u8;
    for &b in &sentence[1..star_idx] {
        calc ^= b;
    }

    if calc != expected {
        return Err(GgaParseError::InvalidChecksum);
    }

    Ok(star_idx)
}

pub fn parse_gga(sentence: &[u8]) -> Result<GgaData, GgaParseError> {
    let star_idx = verify_checksum(sentence)?;
    let data_bytes = &sentence[1..star_idx];
    let mut fields = data_bytes.split(|&b| b == b',');

    let talker = fields.next().ok_or(GgaParseError::MissingField)?;
    if talker.len() < 5 || &talker[talker.len() - 3..] != b"GGA" {
        return Err(GgaParseError::InvalidTalker);
    }

    let time_bytes = fields.next().ok_or(GgaParseError::MissingField)?;
    let lat_bytes = fields.next().ok_or(GgaParseError::MissingField)?;
    let ns_bytes = fields.next().ok_or(GgaParseError::MissingField)?;
    let lon_bytes = fields.next().ok_or(GgaParseError::MissingField)?;
    let ew_bytes = fields.next().ok_or(GgaParseError::MissingField)?;
    let fix_bytes = fields.next().ok_or(GgaParseError::MissingField)?;
    let sat_bytes = fields.next().ok_or(GgaParseError::MissingField)?;
    let _hdop = fields.next().ok_or(GgaParseError::MissingField)?;
    let alt_bytes = fields.next().ok_or(GgaParseError::MissingField)?;
    let alt_unit = fields.next().ok_or(GgaParseError::MissingField)?;
    let geoid_bytes = fields.next().ok_or(GgaParseError::MissingField)?;

    if !alt_unit.is_empty() && alt_unit != b"M" {
        return Err(GgaParseError::InvalidUnit);
    }

    let mut data = GgaData::default();

    if !time_bytes.is_empty() {
        data.utc_time = Some(parse_utc_time(time_bytes)?);
    }

    if !lat_bytes.is_empty() && !ns_bytes.is_empty() {
        let raw = parse_num::<f64>(lat_bytes)?;
        let decimal = nmea_to_decimal(raw, ns_bytes[0])?;
        data.latitude = Some(f64_to_gps_i32(decimal));
    }

    if !lon_bytes.is_empty() && !ew_bytes.is_empty() {
        let raw = parse_num::<f64>(lon_bytes)?;
        let decimal = nmea_to_decimal(raw, ew_bytes[0])?;
        data.longitude = Some(f64_to_gps_i32(decimal));
    }

    if !fix_bytes.is_empty() {
        let v = parse_num::<u8>(fix_bytes)?;
        data.fix_quality = FixQuality::from(v);
    }

    if !sat_bytes.is_empty() {
        data.satellites = parse_num::<u8>(sat_bytes)?;
    }

    if !alt_bytes.is_empty() {
        data.altitude = Some(parse_num::<f32>(alt_bytes)?);
    }
    if !geoid_bytes.is_empty() {
        data.geoid_sep = Some(parse_num::<f32>(geoid_bytes)?);
    }

    Ok(data)
}

#[derive(Debug, Default, PartialEq)]
pub struct RmcData {
    pub speed_kmh: Option<f32>,
    pub true_course: Option<f32>,
}

pub fn parse_rmc_movement(sentence: &[u8]) -> Result<RmcData, GgaParseError> {
    let star_idx = verify_checksum(sentence)?;
    let data_bytes = &sentence[1..star_idx];
    let mut fields = data_bytes.split(|&b| b == b',');

    let talker = fields.next().ok_or(GgaParseError::MissingField)?;
    if talker.len() < 5 || &talker[talker.len() - 3..] != b"RMC" {
        return Err(GgaParseError::InvalidTalker);
    }

    let _time = fields.next().ok_or(GgaParseError::MissingField)?;

    let status = fields.next().ok_or(GgaParseError::MissingField)?;
    if status != b"A" {
        return Err(GgaParseError::ParseError);
    }

    let speed_bytes = fields.nth(4).ok_or(GgaParseError::MissingField)?;
    let course_bytes = fields.next().ok_or(GgaParseError::MissingField)?;

    let mut data = RmcData::default();

    if !speed_bytes.is_empty() {
        let speed_knots = parse_num::<f32>(speed_bytes)?;
        data.speed_kmh = Some(speed_knots * 1.852);
    }

    if !course_bytes.is_empty() {
        data.true_course = Some(parse_num::<f32>(course_bytes)?);
    }

    Ok(data)
}

#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub struct GstData {
    pub range_rms: Option<f32>,
    pub std_major: Option<f32>,
    pub std_minor: Option<f32>,
    pub orient: Option<f32>,
    pub std_lat: Option<f32>,
    pub std_long: Option<f32>,
    pub std_alt: Option<f32>,
}

pub fn parse_gst(sentence: &[u8]) -> Result<GstData, GgaParseError> {
    let star_idx = verify_checksum(sentence)?;
    let data_bytes = &sentence[1..star_idx];
    let mut fields = data_bytes.split(|&b| b == b',');

    let talker = fields.next().ok_or(GgaParseError::MissingField)?;
    if talker.len() < 5 || &talker[talker.len() - 3..] != b"GST" {
        return Err(GgaParseError::InvalidTalker);
    }

    let _time = fields.next().ok_or(GgaParseError::MissingField)?;
    let range_rms = fields.next().ok_or(GgaParseError::MissingField)?;
    let std_major = fields.next().ok_or(GgaParseError::MissingField)?;
    let std_minor = fields.next().ok_or(GgaParseError::MissingField)?;
    let orient = fields.next().ok_or(GgaParseError::MissingField)?;
    let std_lat = fields.next().ok_or(GgaParseError::MissingField)?;
    let std_long = fields.next().ok_or(GgaParseError::MissingField)?;
    let std_alt = fields.next().ok_or(GgaParseError::MissingField)?;

    Ok(GstData {
        range_rms: parse_optional_f32(range_rms)?,
        std_major: parse_optional_f32(std_major)?,
        std_minor: parse_optional_f32(std_minor)?,
        orient: parse_optional_f32(orient)?,
        std_lat: parse_optional_f32(std_lat)?,
        std_long: parse_optional_f32(std_long)?,
        std_alt: parse_optional_f32(std_alt)?,
    })
}
