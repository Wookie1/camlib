//! Bounded independent preflight jobs; no motion is emitted until all succeed.
use crate::stock::CancelSignal;
use std::{
    sync::{
        atomic::{AtomicBool, AtomicUsize, Ordering},
        mpsc,
    },
    time::Duration,
};

pub(crate) fn ordered<T: Send, F, A>(
    count: usize,
    workers: usize,
    cancel: &impl CancelSignal,
    job: F,
    mut accept: A,
) -> Result<Vec<T>, String>
where
    F: Fn(usize, &dyn CancelSignal) -> Result<T, String> + Sync,
    A: FnMut(&T) -> Result<(), String>,
{
    if workers <= 1 || count <= 1 {
        return (0..count)
            .map(|i| {
                if cancel.is_cancelled() {
                    return Err("roughing layer preparation cancelled".into());
                }
                let value = job(i, cancel)?;
                accept(&value)?;
                Ok(value)
            })
            .collect();
    }
    let workers = workers.min(count);
    let stop = AtomicBool::new(false);
    let next = AtomicUsize::new(0);
    std::thread::scope(|scope| {
        let (tx, rx) = mpsc::sync_channel(workers);
        let mut handles = Vec::with_capacity(workers);
        for _ in 0..workers {
            let (tx, job, stop, next) = (tx.clone(), &job, &stop, &next);
            handles.push(scope.spawn(move || {
                while !stop.load(Ordering::Relaxed) {
                    let i = next.fetch_add(1, Ordering::Relaxed);
                    if i >= count {
                        break;
                    }
                    if tx.send((i, job(i, stop))).is_err() {
                        break;
                    }
                }
            }));
        }
        drop(tx);
        let mut values: Vec<Option<T>> = std::iter::repeat_with(|| None).take(count).collect();
        let (mut received, mut failure) = (0, None);
        while received < count {
            if cancel.is_cancelled() {
                failure = Some("roughing layer preparation cancelled".into());
                break;
            }
            match rx.recv_timeout(Duration::from_millis(5)) {
                Ok((i, Ok(value))) => {
                    if let Err(e) = accept(&value) {
                        failure = Some(e);
                        break;
                    }
                    values[i] = Some(value);
                    received += 1;
                }
                Ok((_, Err(e))) => {
                    failure = Some(e);
                    break;
                }
                Err(mpsc::RecvTimeoutError::Timeout) => (),
                Err(mpsc::RecvTimeoutError::Disconnected) => {
                    failure = Some("roughing layer worker stopped before completion".into());
                    break;
                }
            }
        }
        stop.store(true, Ordering::Relaxed);
        drop(rx); // Release workers blocked in a bounded send before joining.
        for h in handles {
            if h.join().is_err() {
                failure.get_or_insert("roughing layer worker panicked".into());
            }
        }
        if let Some(e) = failure {
            Err(e)
        } else {
            Ok(values.into_iter().map(Option::unwrap).collect())
        }
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::{cell::Cell, sync::atomic::AtomicUsize};
    #[test]
    fn ordered_results_and_non_sync_cancellation_join_every_worker() {
        let v = ordered(32, 4, &|| false, |i, _| Ok(31 - i), |_| Ok(())).unwrap();
        assert_eq!(v, (0..32).rev().collect::<Vec<_>>());
        let entered = AtomicUsize::new(0);
        let ended = AtomicUsize::new(0);
        let polls = Cell::new(0);
        let cancel = || {
            polls.set(polls.get() + 1);
            entered.load(Ordering::SeqCst) > 0
        };
        let result = ordered(
            16,
            4,
            &cancel,
            |_, signal| {
                entered.fetch_add(1, Ordering::SeqCst);
                while !signal.is_cancelled() {
                    std::thread::yield_now();
                }
                ended.fetch_add(1, Ordering::SeqCst);
                Err::<usize, _>("worker cancelled".into())
            },
            |_| Ok(()),
        );
        assert!(result.unwrap_err().contains("cancelled"));
        assert_eq!(entered.load(Ordering::SeqCst), ended.load(Ordering::SeqCst));
    }
    #[test]
    fn aggregate_refusal_and_worker_panic_publish_no_partial_collection() {
        let mut total = 0;
        assert!(
            ordered(
                32,
                4,
                &|| false,
                |_, _| Ok(10usize),
                |n| {
                    total += n;
                    if total > 30 {
                        Err("total budget".into())
                    } else {
                        Ok(())
                    }
                }
            )
            .unwrap_err()
            .contains("budget")
        );
        assert!(
            ordered(
                8,
                2,
                &|| false,
                |i, _| {
                    if i == 0 {
                        panic!("worker test")
                    };
                    Ok(i)
                },
                |_| Ok(())
            )
            .unwrap_err()
            .contains("worker")
        );
    }
}
