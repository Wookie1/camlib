//! GRBL 1.1h arithmetic, as the controller computes it (binary32 floats).
//!
//! A posted GRBL program is checked by decoding its words exactly as GRBL
//! does: its own eight-digit decimal reader, binary32 unit scaling, and
//! coordinates kept in binary32 machine space (work offset added).

/// Pinned GRBL1.1h decimal arithmetic model. First8decimal digits (including
/// leading zeros), then binary32 scaling; not the correctly rounded strtof rule.
/// Independently expressed from the observed numerical specification; the probe
/// runs the original reference function separately, outside product code.
pub fn grbl_number(text: &str) -> Result<f64, String> {
    if text.is_empty() || text.len() > 79 {
        return Err("invalid GRBL number".into());
    }
    let negative = text.starts_with('-');
    let unsigned = text.strip_prefix(['-', '+']).unwrap_or(text);
    let mut decimal = false;
    let mut places_before = 0i32;
    let mut digits = 0usize;
    let mut coefficient = 0u32;
    for ch in unsigned.bytes() {
        if ch == b'.' && !decimal {
            decimal = true;
            continue;
        }
        if !ch.is_ascii_digit() {
            return Err("invalid GRBL decimal syntax".into());
        }
        if !decimal {
            places_before += 1;
        }
        if digits < 8 {
            coefficient = coefficient * 10 + (ch - b'0') as u32;
        }
        digits += 1;
    }
    if digits == 0 {
        return Err("invalid GRBL decimal syntax".into());
    }
    let power = places_before - digits.min(8) as i32;
    let mut value = coefficient as f32;
    if value != 0. {
        if power < 0 {
            for _ in 0..(-power / 2) {
                value *= 0.01_f32;
            }
            if power % 2 != 0 {
                value *= 0.1_f32;
            }
        } else {
            for _ in 0..power {
                value *= 10_f32;
            }
        }
    }
    if negative {
        value = -value;
    }
    if !value.is_finite() {
        return Err("GRBL number overflows binary32".into());
    }
    Ok(value as f64)
}

/// Work coordinate of an axis word: GRBL scales the read value to mm and
/// adds the work offset in binary32 machine space; the work coordinate is
/// that machine value minus the offset.
pub fn axis_mm(value: f64, unit: f64, origin_mm: f64) -> f64 {
    let origin = origin_mm as f32;
    let mm = (value as f32) * (unit as f32);
    (mm + origin) as f64 - origin as f64
}

/// Feed in mm/min: the read value scaled in binary32.
pub fn feed_mm_min(value: f64, unit: f64) -> f64 {
    ((value as f32) * (unit as f32)) as f64
}

/// Work coordinate of the initial machine position: both values in binary32.
pub fn start_mm(machine_mm: f64, origin_mm: f64) -> f64 {
    machine_mm as f32 as f64 - origin_mm as f32 as f64
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn eight_digit_reader_and_binary32_scaling() {
        assert_eq!(grbl_number("10").unwrap(), 10.);
        // Digits past the eighth are ignored.
        assert_eq!(
            grbl_number("1.234567891").unwrap(),
            grbl_number("1.2345678").unwrap()
        );
        assert_eq!(grbl_number("-0.5").unwrap(), -0.5);
        assert!(grbl_number("1e3").is_err() && grbl_number("").is_err());
        // Binary32, not the decimal value.
        assert_ne!(grbl_number("10.123456").unwrap(), 10.123456);
        assert_eq!(axis_mm(10., 1., 0.), 10.);
        assert_eq!(start_mm(1002., 1000.), 2.);
        assert_eq!(feed_mm_min(600., 1.), 600.);
    }
}
