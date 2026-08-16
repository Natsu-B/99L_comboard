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

#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub struct UtcTime {
    pub hour: u8,
    pub minute: u8,
    pub second: f32,
}

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct UtcDate {
    pub year: u16,
    pub month: u8,
    pub day: u8,
}

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct UtcDateTime {
    pub date: UtcDate,
    pub hour: u8,
    pub minute: u8,
    pub second: u8,
    pub milliseconds: u16,
    pub unix_seconds: u32,
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

fn parse_two_digits(bytes: &[u8]) -> Result<u8, GgaParseError> {
    if bytes.len() != 2 || !bytes.iter().all(u8::is_ascii_digit) {
        return Err(GgaParseError::ParseError);
    }
    Ok((bytes[0] - b'0') * 10 + (bytes[1] - b'0'))
}

fn parse_utc_hms_millis(bytes: &[u8]) -> Result<(u8, u8, u8, u16), GgaParseError> {
    if bytes.len() < 6 {
        return Err(GgaParseError::ParseError);
    }
    let hour = parse_two_digits(&bytes[0..2])?;
    let minute = parse_two_digits(&bytes[2..4])?;
    let second = parse_two_digits(&bytes[4..6])?;
    if hour >= 24 || minute >= 60 || second >= 60 {
        return Err(GgaParseError::ParseError);
    }

    let mut milliseconds = 0u16;
    if bytes.len() > 6 {
        if bytes[6] != b'.' {
            return Err(GgaParseError::ParseError);
        }
        let fraction = &bytes[7..];
        if fraction.is_empty() || !fraction.iter().all(u8::is_ascii_digit) {
            return Err(GgaParseError::ParseError);
        }
        let digits = fraction.len().min(3);
        for &digit in &fraction[..digits] {
            milliseconds = milliseconds * 10 + u16::from(digit - b'0');
        }
        for _ in digits..3 {
            milliseconds *= 10;
        }
    }
    Ok((hour, minute, second, milliseconds))
}

const fn leap_year(year: u16) -> bool {
    year % 4 == 0 && (year % 100 != 0 || year % 400 == 0)
}

const fn days_in_month(year: u16, month: u8) -> Option<u8> {
    match month {
        1 | 3 | 5 | 7 | 8 | 10 | 12 => Some(31),
        4 | 6 | 9 | 11 => Some(30),
        2 => Some(if leap_year(year) { 29 } else { 28 }),
        _ => None,
    }
}

fn parse_rmc_date(bytes: &[u8]) -> Result<UtcDate, GgaParseError> {
    if bytes.len() != 6 {
        return Err(GgaParseError::ParseError);
    }
    let day = parse_two_digits(&bytes[0..2])?;
    let month = parse_two_digits(&bytes[2..4])?;
    let year_2d = parse_two_digits(&bytes[4..6])?;
    // RMCは2桁年なので、u-blox M10世代の運用範囲に合わせて
    // 80..99=1980..1999、00..79=2000..2079として解釈する。
    let year = if year_2d >= 80 {
        1900 + u16::from(year_2d)
    } else {
        2000 + u16::from(year_2d)
    };
    let maximum_day = days_in_month(year, month).ok_or(GgaParseError::ParseError)?;
    if day == 0 || day > maximum_day {
        return Err(GgaParseError::ParseError);
    }
    Ok(UtcDate { year, month, day })
}

fn unix_seconds(date: UtcDate, hour: u8, minute: u8, second: u8) -> Result<u32, GgaParseError> {
    if date.year < 1970 {
        return Err(GgaParseError::ParseError);
    }
    let mut days = 0u64;
    for year in 1970..date.year {
        days += if leap_year(year) { 366 } else { 365 };
    }
    for month in 1..date.month {
        days += u64::from(days_in_month(date.year, month).ok_or(GgaParseError::ParseError)?);
    }
    days += u64::from(date.day - 1);
    let seconds = days
        .checked_mul(86_400)
        .and_then(|value| value.checked_add(u64::from(hour) * 3_600))
        .and_then(|value| value.checked_add(u64::from(minute) * 60))
        .and_then(|value| value.checked_add(u64::from(second)))
        .ok_or(GgaParseError::ParseError)?;
    u32::try_from(seconds).map_err(|_| GgaParseError::ParseError)
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

    fn sentence(body: &str) -> std::string::String {
        let checksum = body.as_bytes().iter().fold(0u8, |acc, byte| acc ^ byte);
        std::format!("${body}*{checksum:02X}")
    }

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

    #[test]
    fn rmc_datetime_converts_to_unix_time() {
        let rmc = sentence("GNRMC,123519.250,A,4807.038,N,01131.000,E,0.0,0.0,230394,,,A");
        let parsed = parse_rmc_datetime(rmc.as_bytes()).unwrap();
        assert_eq!(
            parsed.date,
            UtcDate {
                year: 1994,
                month: 3,
                day: 23
            }
        );
        assert_eq!((parsed.hour, parsed.minute, parsed.second), (12, 35, 19));
        assert_eq!(parsed.milliseconds, 250);
        assert_eq!(parsed.unix_seconds, 764_426_119);
    }

    #[test]
    fn rmc_void_status_still_supplies_time() {
        let rmc = sentence("GNRMC,000000.005,V,,,,,,,010100,,,N");
        let parsed = parse_rmc_datetime(rmc.as_bytes()).unwrap();
        assert_eq!(parsed.unix_seconds, 946_684_800);
        assert_eq!(parsed.milliseconds, 5);
    }

    #[test]
    fn rmc_datetime_rejects_invalid_calendar_date() {
        let rmc = sentence("GNRMC,000000,A,,,,,,,290223,,,N");
        assert!(parse_rmc_datetime(rmc.as_bytes()).is_err());
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

pub fn parse_rmc_datetime(sentence: &[u8]) -> Result<UtcDateTime, GgaParseError> {
    let star_idx = verify_checksum(sentence)?;
    let data_bytes = &sentence[1..star_idx];
    let mut fields = data_bytes.split(|&b| b == b',');

    let talker = fields.next().ok_or(GgaParseError::MissingField)?;
    if talker.len() < 5 || &talker[talker.len() - 3..] != b"RMC" {
        return Err(GgaParseError::InvalidTalker);
    }
    let time_bytes = fields.next().ok_or(GgaParseError::MissingField)?;
    let status = fields.next().ok_or(GgaParseError::MissingField)?;
    if status != b"A" && status != b"V" {
        return Err(GgaParseError::ParseError);
    }
    // latitude, N/S, longitude, E/W, speed, courseをskipする。
    for _ in 0..6 {
        fields.next().ok_or(GgaParseError::MissingField)?;
    }
    let date_bytes = fields.next().ok_or(GgaParseError::MissingField)?;

    let (hour, minute, second, milliseconds) = parse_utc_hms_millis(time_bytes)?;
    let date = parse_rmc_date(date_bytes)?;
    Ok(UtcDateTime {
        date,
        hour,
        minute,
        second,
        milliseconds,
        unix_seconds: unix_seconds(date, hour, minute, second)?,
    })
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
