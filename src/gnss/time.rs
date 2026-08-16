#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GnssDateTime {
    pub unix_seconds: u32,
    pub milliseconds: u16,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GnssTimeParseError {
    InvalidStart,
    MissingField,
    InvalidChecksum,
    InvalidTalker,
    ParseError,
    OutOfRange,
}

fn verify_checksum(sentence: &[u8]) -> Result<usize, GnssTimeParseError> {
    if sentence.first().copied() != Some(b'$') {
        return Err(GnssTimeParseError::InvalidStart);
    }
    let star = sentence
        .iter()
        .position(|value| *value == b'*')
        .ok_or(GnssTimeParseError::MissingField)?;
    if star + 3 > sentence.len() {
        return Err(GnssTimeParseError::ParseError);
    }
    let expected = core::str::from_utf8(&sentence[star + 1..star + 3])
        .ok()
        .and_then(|value| u8::from_str_radix(value, 16).ok())
        .ok_or(GnssTimeParseError::ParseError)?;
    let actual = sentence[1..star]
        .iter()
        .fold(0u8, |checksum, value| checksum ^ value);
    if actual != expected {
        return Err(GnssTimeParseError::InvalidChecksum);
    }
    Ok(star)
}

fn parse_u8(field: &[u8]) -> Result<u8, GnssTimeParseError> {
    core::str::from_utf8(field)
        .map_err(|_| GnssTimeParseError::ParseError)?
        .parse::<u8>()
        .map_err(|_| GnssTimeParseError::ParseError)
}

fn parse_time(field: &[u8]) -> Result<(u8, u8, u8, u16), GnssTimeParseError> {
    if field.len() < 6 {
        return Err(GnssTimeParseError::ParseError);
    }
    let hour = parse_u8(&field[0..2])?;
    let minute = parse_u8(&field[2..4])?;
    let second = parse_u8(&field[4..6])?;
    if hour >= 24 || minute >= 60 || second > 60 {
        return Err(GnssTimeParseError::OutOfRange);
    }

    let milliseconds = if field.len() == 6 {
        0
    } else {
        if field[6] != b'.' {
            return Err(GnssTimeParseError::ParseError);
        }
        let fraction = &field[7..];
        if fraction.is_empty() || fraction.iter().any(|digit| !digit.is_ascii_digit()) {
            return Err(GnssTimeParseError::ParseError);
        }
        let mut value = 0u16;
        for index in 0..3 {
            value *= 10;
            if let Some(digit) = fraction.get(index) {
                value += u16::from(*digit - b'0');
            }
        }
        value
    };
    Ok((hour, minute, second, milliseconds))
}

fn leap_year(year: u16) -> bool {
    year % 4 == 0 && (year % 100 != 0 || year % 400 == 0)
}

fn days_in_month(year: u16, month: u8) -> Option<u8> {
    match month {
        1 | 3 | 5 | 7 | 8 | 10 | 12 => Some(31),
        4 | 6 | 9 | 11 => Some(30),
        2 => Some(if leap_year(year) { 29 } else { 28 }),
        _ => None,
    }
}

fn parse_date(field: &[u8]) -> Result<(u16, u8, u8), GnssTimeParseError> {
    if field.len() != 6 {
        return Err(GnssTimeParseError::ParseError);
    }
    let day = parse_u8(&field[0..2])?;
    let month = parse_u8(&field[2..4])?;
    let short_year = parse_u8(&field[4..6])?;
    // NMEA RMCは2桁年なので、GPS運用で一般的な1980..2079へ写像する。
    let year = if short_year >= 80 {
        1900 + u16::from(short_year)
    } else {
        2000 + u16::from(short_year)
    };
    let maximum_day = days_in_month(year, month).ok_or(GnssTimeParseError::OutOfRange)?;
    if day == 0 || day > maximum_day {
        return Err(GnssTimeParseError::OutOfRange);
    }
    Ok((year, month, day))
}

fn days_before_year(year: u16) -> u64 {
    let year = u64::from(year);
    let before = year - 1;
    365 * before + before / 4 - before / 100 + before / 400
}

fn days_before_month(year: u16, month: u8) -> u64 {
    let mut days = 0u64;
    let mut current = 1u8;
    while current < month {
        days += u64::from(days_in_month(year, current).unwrap_or(0));
        current += 1;
    }
    days
}

fn to_unix_seconds(
    year: u16,
    month: u8,
    day: u8,
    hour: u8,
    minute: u8,
    second: u8,
) -> Result<u32, GnssTimeParseError> {
    if year < 1970 {
        return Err(GnssTimeParseError::OutOfRange);
    }
    let days = days_before_year(year) - days_before_year(1970) + days_before_month(year, month)
        + u64::from(day - 1);
    let leap_second_carry = u64::from(second == 60);
    let normal_second = u64::from(second.min(59));
    let seconds = days
        .checked_mul(86_400)
        .and_then(|value| value.checked_add(u64::from(hour) * 3_600))
        .and_then(|value| value.checked_add(u64::from(minute) * 60))
        .and_then(|value| value.checked_add(normal_second + leap_second_carry))
        .ok_or(GnssTimeParseError::OutOfRange)?;
    u32::try_from(seconds).map_err(|_| GnssTimeParseError::OutOfRange)
}

pub fn parse_rmc_datetime(sentence: &[u8]) -> Result<GnssDateTime, GnssTimeParseError> {
    let star = verify_checksum(sentence)?;
    let mut fields = sentence[1..star].split(|value| *value == b',');
    let talker = fields.next().ok_or(GnssTimeParseError::MissingField)?;
    if talker.len() < 5 || &talker[talker.len() - 3..] != b"RMC" {
        return Err(GnssTimeParseError::InvalidTalker);
    }
    let time = fields.next().ok_or(GnssTimeParseError::MissingField)?;
    let _status = fields.next().ok_or(GnssTimeParseError::MissingField)?;
    let _latitude = fields.next().ok_or(GnssTimeParseError::MissingField)?;
    let _north_south = fields.next().ok_or(GnssTimeParseError::MissingField)?;
    let _longitude = fields.next().ok_or(GnssTimeParseError::MissingField)?;
    let _east_west = fields.next().ok_or(GnssTimeParseError::MissingField)?;
    let _speed = fields.next().ok_or(GnssTimeParseError::MissingField)?;
    let _course = fields.next().ok_or(GnssTimeParseError::MissingField)?;
    let date = fields.next().ok_or(GnssTimeParseError::MissingField)?;

    let (hour, minute, second, milliseconds) = parse_time(time)?;
    let (year, month, day) = parse_date(date)?;
    Ok(GnssDateTime {
        unix_seconds: to_unix_seconds(year, month, day, hour, minute, second)?,
        milliseconds,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sentence(payload: &str) -> std::string::String {
        let checksum = payload
            .as_bytes()
            .iter()
            .fold(0u8, |value, byte| value ^ byte);
        std::format!("${payload}*{checksum:02X}")
    }

    #[test]
    fn parses_rmc_date_and_time_without_position_fix_dependency() {
        let value = sentence("GNRMC,123519.250,V,,,,,,,230826,,,N");
        let parsed = parse_rmc_datetime(value.as_bytes()).unwrap();
        assert_eq!(parsed.milliseconds, 250);
        assert_eq!(parsed.unix_seconds, 1_787_488_519);
    }

    #[test]
    fn rejects_invalid_date_and_checksum() {
        let invalid_date = sentence("GNRMC,000000.000,A,,,,,,,310226,,,N");
        assert_eq!(
            parse_rmc_datetime(invalid_date.as_bytes()),
            Err(GnssTimeParseError::OutOfRange)
        );
        assert_eq!(
            parse_rmc_datetime(b"$GNRMC,000000.000,A,,,,,,,010126,,,N*00"),
            Err(GnssTimeParseError::InvalidChecksum)
        );
    }
}
