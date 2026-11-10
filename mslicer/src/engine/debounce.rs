//! Waiting for inputs to settle before asking for work.

use std::time::{Duration, Instant};

/// How long drag-value widgets must stay put before work is asked for.
pub const SETTLE: Duration = Duration::from_millis(300);

/// Tells when an input has stayed the same for a while, so dragging a value
/// asks for one run at the end and not one per step.
#[derive(Clone, Debug)]
pub struct Debounce<K> {
    seen: Option<(K, Instant)>,
    delay: Duration,
}

impl<K> Default for Debounce<K> {
    fn default() -> Self {
        Self {
            seen: None,
            delay: SETTLE,
        }
    }
}

impl<K: PartialEq> Debounce<K> {
    #[cfg(test)]
    pub fn new(delay: Duration) -> Self {
        Self { seen: None, delay }
    }

    /// Whether `key` has been seen unchanged for the delay, as of `now`. A
    /// different key restarts the wait.
    pub fn settled(&mut self, key: K, now: Instant) -> bool {
        match &self.seen {
            Some((seen, since)) if *seen == key => now - *since >= self.delay,
            _ => {
                self.seen = Some((key, now));
                false
            }
        }
    }

    /// Settles `key` at once, e.g. when a drag is released.
    #[cfg(test)]
    pub fn release(&mut self, key: K, now: Instant) {
        self.seen = Some((key, now - self.delay));
    }

    pub fn reset(&mut self) {
        self.seen = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn settles_after_the_delay_and_restarts_on_change() {
        let t0 = Instant::now();
        let ms = |n| t0 + Duration::from_millis(n);
        let mut d = Debounce::new(Duration::from_millis(300));
        assert!(!d.settled(1, ms(0)));
        assert!(!d.settled(1, ms(200)));
        assert!(!d.settled(2, ms(250)));
        assert!(!d.settled(2, ms(500)));
        assert!(d.settled(2, ms(550)));
        d.release(3, ms(560));
        assert!(d.settled(3, ms(560)));
    }
}
