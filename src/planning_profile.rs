//! Opt-in, per-thread diagnostic attribution. Never part of CAM reports/cache.
use serde::Serialize;
use std::{cell::RefCell, collections::BTreeMap, marker::PhantomData, rc::Rc, time::Instant};

#[derive(Clone, Default, Debug, Serialize)]
pub struct Measurement {
    pub calls: u64,
    pub inclusive_ns: u128,
    pub self_ns: u128,
}
struct Frame {
    name: &'static str,
    started: Instant,
    children: u128,
}
#[derive(Default)]
struct State {
    stack: Vec<Frame>,
    rows: BTreeMap<&'static str, Measurement>,
}
thread_local! {
    static ACTIVE: RefCell<Option<State>> = const { RefCell::new(None) };
}
struct Reset;
impl Drop for Reset {
    fn drop(&mut self) {
        ACTIVE.with(|p| *p.borrow_mut() = None);
    }
}
/// Capture this thread's bounded fixed-label spans. Nested captures are refused;
/// unwinding restores inactivity. Elapsed times include instrumentation and
/// scheduler delays; these are not production latency distributions.
pub fn capture<R>(f: impl FnOnce() -> R) -> (R, BTreeMap<&'static str, Measurement>) {
    ACTIVE.with(|p| {
        let mut p = p.borrow_mut();
        assert!(p.is_none(), "planning captures cannot nest");
        *p = Some(State::default());
    });
    let _reset = Reset;
    let result = f();
    let state = ACTIVE.with(|p| p.borrow_mut().take().unwrap());
    assert!(
        state.stack.is_empty(),
        "planning spans must end within capture"
    );
    (result, state.rows)
}
/// Owned snapshot of completed spans only. Open spans are deliberately absent;
/// later work cannot modify this prefix. Not a CAM/verification authority.
pub fn completed_measurements() -> Option<BTreeMap<&'static str, Measurement>> {
    ACTIVE.with(|p| p.borrow().as_ref().map(|s| s.rows.clone()))
}

pub struct Span {
    active: bool,
    _thread: PhantomData<Rc<()>>,
}
#[must_use]
pub fn span(name: &'static str) -> Span {
    let active = ACTIVE.with(|p| {
        let mut p = p.borrow_mut();
        let Some(p) = p.as_mut() else { return false };
        assert!(p.stack.len() < 64, "planning span depth exceeds 64");
        p.stack.push(Frame {
            name,
            started: Instant::now(),
            children: 0,
        });
        true
    });
    Span {
        active,
        _thread: PhantomData,
    }
}
impl Drop for Span {
    fn drop(&mut self) {
        if !self.active {
            return;
        }
        ACTIVE.with(|p| {
            let mut p = p.borrow_mut();
            let p = p.as_mut().expect("planning span escaped capture");
            let frame = p.stack.pop().expect("unbalanced planning span");
            let elapsed = frame.started.elapsed().as_nanos();
            if let Some(parent) = p.stack.last_mut() {
                parent.children += elapsed;
            }
            let row = p.rows.entry(frame.name).or_default();
            row.calls += 1;
            row.inclusive_ns += elapsed;
            row.self_ns += elapsed.saturating_sub(frame.children);
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn completed_prefix_stays_fixed_while_the_original_job_finishes() {
        assert!(completed_measurements().is_none());
        let (prefix, full) = capture(|| {
            let _open = span("open_job");
            {
                let _early = span("early");
            }
            let prefix = completed_measurements().unwrap();
            {
                let _late = span("late");
            }
            prefix
        });
        assert_eq!(prefix.len(), 1);
        assert_eq!(prefix["early"].calls, 1);
        assert_eq!(full["early"].calls, 1);
        assert_eq!(full["late"].calls, 1);
        assert_eq!(full["open_job"].calls, 1);
        assert!(completed_measurements().is_none());
    }
}
