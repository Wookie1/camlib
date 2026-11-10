//! Independent Carvera actual-word guard; imports no encoder code.
//! M6 calibrates TLO; G53 applies to the directly following G0/G1:
//! https://carvera-community.gitbook.io/docs/firmware/supported-commands
use super::*;
impl Decoder {
    pub(super) fn read_carvera(
        &mut self,
        c: &crate::gcode::carvera::Carvera,
        line: &str,
        context: Option<&Context>,
    ) -> Result<Vec<Event>, String> {
        if self.ended {
            return Err("Carvera code after M30".into());
        }
        if line.len() > 1024
            || line
                .bytes()
                .any(|b| !(b == b'\t' || (0x20..=0x7e).contains(&b)))
        {
            return Err("invalid Carvera physical block".into());
        }
        let normalized = line.split(';').next().unwrap().trim().to_ascii_uppercase();
        let line = normalized.as_str();
        if self.carvera_air_sync_pending {
            if line != "M7" && line != "M9" {
                return Err("Carvera M400 must immediately precede M7 or M9".into());
            }
            self.carvera_air_sync_pending = false;
            return self.read_base(line, context);
        }
        if line == "M7" || line == "M9" {
            return Err("Carvera air change requires immediately preceding M400".into());
        }
        if line == "M400" {
            self.carvera_air_sync_pending = true;
            return Ok(vec![]);
        }
        if line == "G53 G0 Z-3" {
            self.validate_initialized()?;
            let length = self
                .active
                .as_ref()
                .map_or(c.initial_gauge_length_mm, |t| t.gauge_length_mm);
            let z = self.position[2] + self.machine.work_to_machine.origin_mm[2] + length;
            // Half one output unit; both ports divide by the exact integer
            // power of ten, avoiding platform-dependent pow implementations.
            if (z + 3.).abs() > 0.5 / (10_u64.pow(self.machine.decimals) as f64) {
                return Err("Carvera G53 retract requires planned gauge-line machine Z=-3; plan the retract".into());
            }
            self.carvera_retracted = true;
            return Ok(vec![]);
        }
        if let Some(t) = line.strip_prefix('T').and_then(|s| s.strip_suffix(" M6"))
            && !t.is_empty()
            && !t.starts_with('0')
            && t.bytes().all(|b| b.is_ascii_digit())
        {
            if !self.carvera_retracted || self.rpm != 0. || self.coolant != Coolant::Off {
                return Err("Carvera M6 requires G53 retract and spindle/air off".into());
            }
            self.activate(
                t.parse()
                    .map_err(|_| "Carvera tool number exceeds integer range")?,
            )?;
            let events = vec![self.length(Some(f64::from(
                self.active.as_ref().unwrap().length_register,
            )))?];
            self.carvera_retracted = false;
            return Ok(events);
        }
        if line == "M30"
            && (!self.carvera_retracted || self.rpm != 0. || self.coolant != Coolant::Off)
        {
            return Err("Carvera M30 requires G53 retract and spindle/air off".into());
        }
        if line == "M30" {
            self.ended = true;
            return Ok(vec![]);
        }
        // No standalone tool selection, M6, or H activation. Parse actual
        // words so alternate spacing/leading zeros cannot bypass the guard.
        let body = line.split(';').next().unwrap();
        let bytes = body.as_bytes();
        for (i, b) in bytes.iter().enumerate() {
            if *b == b'H'
                || *b == b'T'
                || (*b == b'M' && {
                    let rest = body[i + 1..].trim_start();
                    let count = rest
                        .bytes()
                        .take_while(|b| b.is_ascii_digit() || b"+-.".contains(b))
                        .count();
                    rest[..count]
                        .parse::<f64>()
                        .ok()
                        .is_some_and(|v| [6., 7., 9., 30., 400.].contains(&v))
                })
            {
                return Err("Carvera requires T<n> M6 without H; air requires standalone M400 immediately before M7 or M9".into());
            }
        }
        let events = self.read_base(line, context)?;
        if events.iter().any(|e| matches!(e, Event::Motion { .. })) {
            self.carvera_retracted = false;
        }
        Ok(events)
    }
}
