//! Transactional packed stock-buffer checks, independent of a Python runtime.
//! Foreign adapters supply safe scalar access while retaining their buffer lease.

use std::cell::Cell;

pub trait HeightRead {
    fn scalar_len(&self) -> usize;
    fn read(&self, index: usize) -> f64;
}
pub trait HeightWrite: HeightRead {
    fn write(&mut self, index: usize, value: f64);
}

fn valid_grid(buffer: &impl HeightRead, bottom: f64, top: f64) -> Result<usize, String> {
    if !bottom.is_finite()
        || !top.is_finite()
        || bottom >= top
        || bottom.abs() > 1e6
        || top.abs() > 1e6
        || !buffer.scalar_len().is_multiple_of(2)
        || buffer.scalar_len() == 0
        || buffer.scalar_len() as u64 > 2 * u64::from(u32::MAX)
    {
        return Err("invalid bounded height buffer shape or stock limits".into());
    }
    Ok(buffer.scalar_len() / 2)
}
fn valid_pair(lo: f64, hi: f64, bottom: f64, top: f64) -> bool {
    lo.is_finite() && hi.is_finite() && bottom <= lo && lo <= hi && hi <= top
}
pub fn validate_heights(buffer: &impl HeightRead, bottom: f64, top: f64) -> Result<(), String> {
    let cells = valid_grid(buffer, bottom, top)?;
    for i in 0..cells {
        if !valid_pair(buffer.read(2 * i), buffer.read(2 * i + 1), bottom, top) {
            return Err("invalid checkpoint height bounds".into());
        }
    }
    Ok(())
}
pub fn validate_packed_delta(
    buffer: &impl HeightRead,
    raw: &[u8],
    offset: usize,
    count: usize,
    nx: usize,
    bottom: f64,
    top: f64,
) -> Result<Option<[usize; 4]>, String> {
    let cells = valid_grid(buffer, bottom, top)?;
    if nx == 0 || !cells.is_multiple_of(nx) || count > cells {
        return Err("invalid packed delta count/grid".into());
    }
    let end = offset
        .checked_add(count.checked_mul(36).ok_or("packed delta size overflow")?)
        .ok_or("packed delta offset overflow")?;
    let changes = raw
        .get(offset..end)
        .ok_or("packed delta lies outside immutable input buffer")?;
    let mut previous = None;
    let mut rect = None::<[usize; 4]>;
    for c in changes.as_chunks::<36>().0 {
        let index = u32::from_le_bytes(c[..4].try_into().unwrap()) as usize;
        let read = |i: usize| f64::from_le_bytes(c[i..i + 8].try_into().unwrap());
        let (before_lo, before_hi, after_lo, after_hi) = (read(4), read(12), read(20), read(28));
        if index >= cells || previous.is_some_and(|p| index <= p) {
            return Err("invalid delta index/order".into());
        }
        if !valid_pair(before_lo, before_hi, bottom, top)
            || !valid_pair(after_lo, after_hi, bottom, top)
            || after_lo > before_lo
            || after_hi > before_hi
            || buffer.read(2 * index) != before_lo
            || buffer.read(2 * index + 1) != before_hi
        {
            return Err("delta old state or monotone height bounds mismatch".into());
        }
        previous = Some(index);
        let (x, y) = (index % nx, index / nx);
        rect = Some(match rect {
            None => [x, y, x, y],
            Some(r) => [r[0].min(x), r[1].min(y), r[2].max(x), r[3].max(y)],
        });
    }
    Ok(rect)
}

pub fn apply_packed_delta(
    buffer: &mut impl HeightWrite,
    raw: &[u8],
    offset: usize,
    count: usize,
    nx: usize,
    bottom: f64,
    top: f64,
) -> Result<Option<[usize; 4]>, String> {
    let rect = validate_packed_delta(buffer, raw, offset, count, nx, bottom, top)?;
    // All validation, including a malformed late cell, precedes the first write.
    for c in raw[offset..offset + count * 36].as_chunks::<36>().0 {
        let i = u32::from_le_bytes(c[..4].try_into().unwrap()) as usize;
        buffer.write(2 * i, f64::from_le_bytes(c[20..28].try_into().unwrap()));
        buffer.write(2 * i + 1, f64::from_le_bytes(c[28..36].try_into().unwrap()));
    }
    Ok(rect)
}

/// Borrowed read-only overlay of a validated prefix on its exact move-start
/// stock. No full stock clone and no new geometry/sweep calculation.
pub struct DeltaOverlay<'a, T> {
    base: &'a T,
    cells: &'a [[u8; 36]],
    range: Option<(usize, usize)>,
    cached: Cell<(usize, Option<usize>)>,
}
impl<'a, T: HeightRead> DeltaOverlay<'a, T> {
    pub fn new(
        base: &'a T,
        raw: &'a [u8],
        delta: [usize; 2],
        nx: usize,
        limits: [f64; 2],
    ) -> Result<Self, String> {
        let [offset, count] = delta;
        validate_packed_delta(base, raw, offset, count, nx, limits[0], limits[1])?;
        let cells = raw[offset..offset + count * 36].as_chunks::<36>().0;
        let index = |c: &[u8; 36]| u32::from_le_bytes(c[..4].try_into().unwrap()) as usize;
        let range = cells
            .first()
            .zip(cells.last())
            .map(|(a, b)| (index(a), index(b)));
        Ok(Self {
            base,
            cells,
            range,
            cached: Cell::new((usize::MAX, None)),
        })
    }
}
impl<T: HeightRead> HeightRead for DeltaOverlay<'_, T> {
    fn scalar_len(&self) -> usize {
        self.base.scalar_len()
    }
    fn read(&self, index: usize) -> f64 {
        let cell = index / 2;
        if self.range.is_none_or(|(a, b)| cell < a || cell > b) {
            return self.base.read(index);
        }
        let (previous, found) = self.cached.get();
        let found = if previous == cell {
            found
        } else {
            let found = self
                .cells
                .binary_search_by_key(&(cell as u32), |c| {
                    u32::from_le_bytes(c[..4].try_into().unwrap())
                })
                .ok();
            self.cached.set((cell, found));
            found
        };
        match found {
            None => self.base.read(index),
            Some(i) => {
                let at = 20 + 8 * (index % 2);
                f64::from_le_bytes(self.cells[i][at..at + 8].try_into().unwrap())
            }
        }
    }
}

pub fn display_overlay_patch(
    base: &impl HeightRead,
    raw: &[u8],
    delta: [usize; 2],
    nx: usize,
    display: [usize; 2],
    dirty: Option<[usize; 4]>,
    limits: [f64; 2],
) -> Result<Vec<u8>, String> {
    let overlay = DeltaOverlay::new(base, raw, delta, nx, limits)?;
    display_patch(&overlay, nx, display, dirty, limits)
}

pub fn volume(
    buffer: &impl HeightRead,
    bottom: f64,
    top: f64,
    area: f64,
) -> Result<[f64; 4], String> {
    validate_heights(buffer, bottom, top)?;
    if !area.is_finite() || area <= 0. || area > 1e12 {
        return Err("invalid stock cell area".into());
    }
    let (mut lo, mut hi, mut lo_error, mut hi_error) = (0., 0., 0., 0.);
    for i in 0..buffer.scalar_len() / 2 {
        let lower = (buffer.read(2 * i) - bottom) * area - lo_error;
        let next = lo + lower;
        lo_error = (next - lo) - lower;
        lo = next;
        let upper = (buffer.read(2 * i + 1) - bottom) * area - hi_error;
        let next = hi + upper;
        hi_error = (next - hi) - upper;
        hi = next;
    }
    let total = (top - bottom) * area * (buffer.scalar_len() / 2) as f64;
    Ok([lo, hi, (total - hi).max(0.), (total - lo).max(0.)])
}

/// Display-only max-upper aggregation. Partition source rows/columns into balanced
/// integer blocks; round f32 upward so the displayed material never understates
/// the authoritative upper envelope. Packed output: (u32 index, f32 height).
/// The input is read-only; only visited cells are validated on a partial patch.
pub fn display_patch(
    buffer: &impl HeightRead,
    nx: usize,
    display: [usize; 2],
    dirty: Option<[usize; 4]>,
    limits: [f64; 2],
) -> Result<Vec<u8>, String> {
    let [bottom, top] = limits;
    let cells = valid_grid(buffer, bottom, top)?;
    if nx == 0 || !cells.is_multiple_of(nx) {
        return Err("invalid display source grid".into());
    }
    let ny = cells / nx;
    let [dx, dy] = display;
    if dx == 0 || dy == 0 || dx > nx || dy > ny || dx > 256 || dy > 256 {
        return Err("display dimensions must fit source and be in [1,256]".into());
    }
    let owner = |p: usize, source: usize, target: usize| ((p + 1) * target - 1) / source;
    let r = match dirty {
        None => [0, 0, dx - 1, dy - 1],
        Some([x0, y0, x1, y1]) => {
            if x0 > x1 || y0 > y1 || x1 >= nx || y1 >= ny {
                return Err("dirty rectangle lies outside source grid".into());
            }
            [
                owner(x0, nx, dx),
                owner(y0, ny, dy),
                owner(x1, nx, dx),
                owner(y1, ny, dy),
            ]
        }
    };
    let mut result = Vec::with_capacity((r[2] - r[0] + 1) * (r[3] - r[1] + 1) * 8);
    for y in r[1]..=r[3] {
        for x in r[0]..=r[2] {
            let mut upper = bottom;
            for sy in y * ny / dy..(y + 1) * ny / dy {
                for sx in x * nx / dx..(x + 1) * nx / dx {
                    let index = sy * nx + sx;
                    let (lo, hi) = (buffer.read(2 * index), buffer.read(2 * index + 1));
                    if !valid_pair(lo, hi, bottom, top) {
                        return Err("invalid source height in display patch".into());
                    }
                    upper = upper.max(hi);
                }
            }
            let mut height = upper as f32;
            if (height as f64) < upper {
                height = height.next_up();
            }
            result.extend_from_slice(&((y * dx + x) as u32).to_le_bytes());
            result.extend_from_slice(&height.to_le_bytes());
        }
    }
    Ok(result)
}

/// Fill a fresh, bounded writable mapping without a temporary full-size array.
pub fn initialise_heights(
    buffer: &mut impl HeightWrite,
    bottom: f64,
    top: f64,
) -> Result<(), String> {
    valid_grid(buffer, bottom, top)?;
    for i in 0..buffer.scalar_len() {
        buffer.write(i, top);
    }
    Ok(())
}
