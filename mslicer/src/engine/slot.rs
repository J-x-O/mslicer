//! A cache slot: one cached result, at most one running job, and the inputs
//! consumers want now.

use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering},
};

/// Set once the job holding it is superseded or no longer wanted. Jobs check
/// it between chunks of work (per layer, per support, per voxel slab) and
/// stop early once it is set.
#[derive(Clone, Debug, Default)]
pub struct CancelToken(Arc<AtomicBool>);

/// A job was cancelled.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Cancelled;

impl CancelToken {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn cancel(&self) {
        self.0.store(true, Ordering::Relaxed);
    }

    pub fn is_cancelled(&self) -> bool {
        self.0.load(Ordering::Relaxed)
    }

    /// `Err(Cancelled)` once cancelled, for `?` between chunks.
    pub fn check(&self) -> Result<(), Cancelled> {
        match self.is_cancelled() {
            true => Err(Cancelled),
            false => Ok(()),
        }
    }

    /// The flag itself, for code that takes an `&AtomicBool`.
    pub fn flag(&self) -> &AtomicBool {
        &self.0
    }

    pub fn shared(&self) -> Arc<AtomicBool> {
        self.0.clone()
    }
}

/// Permission to start the job for `key`, handed out by [`Slot::request`].
/// The job reports back with [`Slot::finish`] and the same key.
#[derive(Debug)]
pub struct Ticket<K> {
    pub key: K,
    pub token: CancelToken,
}

/// How a slot stands with respect to the inputs consumers want.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SlotState {
    /// The cached result is for the wanted inputs.
    Fresh,
    /// A job for the wanted inputs is running.
    Computing,
    /// Nothing is cached for the wanted inputs and nothing runs for them (a
    /// result for older inputs may still be shown).
    Stale,
    /// Nothing is wanted.
    Idle,
}

/// One cached result with the key (the inputs, or a hash of them) it was
/// computed from.
///
/// Work only starts when a consumer asks for it with [`Slot::request`].
/// Asking for other inputs than a running job's cancels that job (latest
/// wins); its result, if it arrives anyway, is discarded. At most one job runs
/// per slot: the newest request starts once the cancelled job has stopped, the
/// next time a consumer asks. Requests in between replace each other, so
/// nothing queues up.
pub struct Slot<K, T> {
    value: Option<(K, Arc<T>)>,
    running: Option<(K, CancelToken)>,
    wanted: Option<K>,
}

impl<K, T> Default for Slot<K, T> {
    fn default() -> Self {
        Self {
            value: None,
            running: None,
            wanted: None,
        }
    }
}

/// A clone shares the cached result but not the running job: that belongs to
/// the original.
impl<K: Clone, T> Clone for Slot<K, T> {
    fn clone(&self) -> Self {
        Self {
            value: self.value.clone(),
            running: None,
            wanted: None,
        }
    }
}

impl<K: Clone + PartialEq, T> Slot<K, T> {
    /// The cached result, if it is for `key`.
    pub fn get(&self, key: &K) -> Option<&Arc<T>> {
        (self.value.as_ref()).and_then(|(k, v)| (k == key).then_some(v))
    }

    /// The cached result, whatever it is for: something to show while the
    /// current one is computed.
    pub fn latest(&self) -> Option<&Arc<T>> {
        self.value.as_ref().map(|(_, v)| v)
    }

    #[cfg(test)]
    pub fn is_running(&self) -> bool {
        self.running.is_some()
    }

    pub fn running_key(&self) -> Option<&K> {
        self.running.as_ref().map(|(k, _)| k)
    }

    pub fn state(&self) -> SlotState {
        let Some(wanted) = &self.wanted else {
            return SlotState::Idle;
        };
        if self.get(wanted).is_some() {
            SlotState::Fresh
        } else if self.running_key() == Some(wanted) {
            SlotState::Computing
        } else {
            SlotState::Stale
        }
    }

    /// Marks `key` as what consumers want, without starting anything:
    /// cancels a running job for other inputs.
    pub fn supersede(&mut self, key: K) {
        if let Some((running, token)) = &self.running
            && *running != key
        {
            token.cancel();
        }
        self.wanted = Some(key);
    }

    /// A consumer wants the result for `key`. Returns a ticket to start the
    /// job if nothing is cached or running for it and no other job is still
    /// winding down.
    pub fn request(&mut self, key: K) -> Option<Ticket<K>> {
        self.supersede(key.clone());
        if self.get(&key).is_some() || self.running.is_some() {
            return None;
        }
        let token = CancelToken::new();
        self.running = Some((key.clone(), token.clone()));
        Some(Ticket { key, token })
    }

    /// Nothing is wanted any more: cancels a running job.
    pub fn withdraw(&mut self) {
        if let Some((_, token)) = &self.running {
            token.cancel();
        }
        self.wanted = None;
    }

    /// The job for `key` ended with `result` (`None` if it gave up). Keeps
    /// the result only if `key` is still what is wanted. Returns whether it
    /// was kept.
    pub fn finish(&mut self, key: &K, result: Option<T>) -> bool {
        if self.running_key() == Some(key) {
            self.running = None;
        }
        match result {
            Some(value) if self.wanted.as_ref() == Some(key) => {
                self.value = Some((key.clone(), Arc::new(value)));
                true
            }
            _ => false,
        }
    }

    /// Stores a result computed elsewhere (synchronously, or loaded).
    pub fn insert(&mut self, key: K, value: Arc<T>) {
        self.value = Some((key, value));
    }

    /// Drops the cached result and cancels a running job.
    pub fn clear(&mut self) {
        self.withdraw();
        self.value = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn work_starts_only_on_request_and_only_once() {
        let mut slot = Slot::<u32, &str>::default();
        assert_eq!(slot.state(), SlotState::Idle);
        let ticket = slot.request(1).expect("starts");
        assert_eq!(slot.state(), SlotState::Computing);
        // Asking again while it runs starts nothing.
        assert!(slot.request(1).is_none());
        assert!(slot.finish(&ticket.key, Some("one")));
        assert_eq!(slot.state(), SlotState::Fresh);
        assert!(slot.request(1).is_none(), "cached");
        assert_eq!(**slot.get(&1).unwrap(), "one");
    }

    #[test]
    fn a_newer_request_cancels_the_running_job_and_its_result_is_discarded() {
        let mut slot = Slot::<u32, &str>::default();
        let old = slot.request(1).unwrap();
        // Latest wins: the old job is told to stop, the new one waits for it.
        assert!(slot.request(2).is_none());
        assert!(old.token.is_cancelled());
        assert_eq!(slot.state(), SlotState::Stale);
        // Its result arrives anyway: dropped.
        assert!(!slot.finish(&old.key, Some("one")));
        assert!(slot.latest().is_none());
        // Requests in between replace each other; only the newest starts.
        assert!(slot.request(3).is_some_and(|t| t.key == 3));
    }

    #[test]
    fn going_back_to_the_running_inputs_keeps_the_job() {
        let mut slot = Slot::<u32, &str>::default();
        let job = slot.request(1).unwrap();
        slot.supersede(1);
        assert!(!job.token.is_cancelled());
    }

    #[test]
    fn the_last_result_stays_shown_while_the_next_is_computed() {
        let mut slot = Slot::<u32, &str>::default();
        let t = slot.request(1).unwrap();
        slot.finish(&t.key, Some("one"));
        let _ = slot.request(2).unwrap();
        assert!(slot.get(&2).is_none());
        assert_eq!(**slot.latest().unwrap(), "one");
    }

    #[test]
    fn withdrawing_cancels() {
        let mut slot = Slot::<u32, &str>::default();
        let t = slot.request(1).unwrap();
        slot.withdraw();
        assert!(t.token.is_cancelled());
        assert_eq!(slot.state(), SlotState::Idle);
        assert!(!slot.finish(&t.key, Some("one")));
    }

    #[test]
    fn clones_share_results_not_jobs() {
        let mut slot = Slot::<u32, &str>::default();
        let t = slot.request(1).unwrap();
        slot.finish(&t.key, Some("one"));
        let _running = slot.request(2).unwrap();
        let mut copy = slot.clone();
        assert!(copy.get(&1).is_some());
        assert!(!copy.is_running());
        assert!(copy.request(2).is_some(), "the copy runs its own job");
    }
}
