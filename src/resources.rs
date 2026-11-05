//! Planning memory guard.
//!
//! camlib does not cap work with fixed constants; run time is bounded by
//! cancellation. Memory is different: a plan that does not fit in memory makes
//! the host swap until it is unresponsive. Large allocations therefore check an
//! estimate against the memory that is actually available:
//!
//! 1. a scoped per-thread budget ([`with_memory_budget`]), for tests and
//!    embedding applications;
//! 2. otherwise `CAMLIB_MEMORY_BUDGET_BYTES` (an explicit caller budget, also
//!    passed to the engine process);
//! 3. otherwise the smaller of the operating system's available memory
//!    (`MemAvailable` on Linux) and the cgroup v2 headroom (containers,
//!    `systemd-run -p MemoryMax=`). Where neither can be read, no limit applies.
use std::cell::Cell;

thread_local! {
    static SCOPED: Cell<Option<u64>> = const { Cell::new(None) };
}

/// Run `f` with an explicit memory budget for allocations checked on this thread.
pub fn with_memory_budget<R>(bytes: u64, f: impl FnOnce() -> R) -> R {
    let previous = SCOPED.with(|s| s.replace(Some(bytes)));
    struct Restore(Option<u64>);
    impl Drop for Restore {
        fn drop(&mut self) {
            SCOPED.with(|s| s.set(self.0));
        }
    }
    let _restore = Restore(previous);
    f()
}

fn os_available() -> Option<u64> {
    let text = std::fs::read_to_string("/proc/meminfo").ok()?;
    let line = text.lines().find(|l| l.starts_with("MemAvailable:"))?;
    let kib: u64 = line.split_whitespace().nth(1)?.parse().ok()?;
    kib.checked_mul(1024)
}

/// Smallest `memory.max - memory.current` over this process's cgroup v2
/// ancestry, so a container or `systemd-run -p MemoryMax=` limit is honoured.
fn cgroup_headroom() -> Option<u64> {
    let text = std::fs::read_to_string("/proc/self/cgroup").ok()?;
    let path = text.lines().find_map(|l| l.strip_prefix("0::"))?.trim();
    let parts: Vec<&str> = path.split('/').filter(|p| !p.is_empty()).collect();
    let mut best: Option<u64> = None;
    for depth in (0..=parts.len()).rev() {
        let mut base = std::path::PathBuf::from("/sys/fs/cgroup");
        base.extend(&parts[..depth]);
        let Ok(limit) = std::fs::read_to_string(base.join("memory.max")) else {
            continue;
        };
        let Ok(limit) = limit.trim().parse::<u64>() else {
            continue; // "max": no limit at this level
        };
        let Some(current) = std::fs::read_to_string(base.join("memory.current"))
            .ok()
            .and_then(|c| c.trim().parse::<u64>().ok())
        else {
            continue;
        };
        let headroom = limit.saturating_sub(current);
        best = Some(best.map_or(headroom, |b| b.min(headroom)));
    }
    best
}

/// The memory budget in effect for this thread, if any.
pub fn memory_budget() -> Option<u64> {
    if let Some(bytes) = SCOPED.with(Cell::get) {
        return Some(bytes);
    }
    if let Some(bytes) = std::env::var("CAMLIB_MEMORY_BUDGET_BYTES")
        .ok()
        .and_then(|v| v.trim().parse().ok())
    {
        return Some(bytes);
    }
    match (os_available(), cgroup_headroom()) {
        (Some(a), Some(b)) => Some(a.min(b)),
        (a, b) => a.or(b),
    }
}

/// Refuse an allocation whose estimated size exceeds the memory budget.
pub fn check_memory(bytes: u128, what: &str) -> Result<(), String> {
    match memory_budget() {
        Some(budget) if bytes > budget as u128 => Err(format!(
            "{what} needs about {:.1} GiB but only {:.1} GiB is available; use a coarser \
             resolution or larger step, split the job, or raise CAMLIB_MEMORY_BUDGET_BYTES",
            bytes as f64 / (1u64 << 30) as f64,
            budget as f64 / (1u64 << 30) as f64
        )),
        _ => Ok(()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn scoped_budget_applies_and_restores() {
        assert_eq!(SCOPED.with(Cell::get), None);
        with_memory_budget(1000, || {
            assert!(check_memory(999, "x").is_ok());
            assert!(check_memory(1001, "x").unwrap_err().contains("x needs"));
            with_memory_budget(10, || assert!(check_memory(11, "y").is_err()));
            assert!(check_memory(999, "x").is_ok());
        });
        // Outside a scope the budget is live free memory again (not a fixed value).
        assert_eq!(SCOPED.with(Cell::get), None);
    }
}
