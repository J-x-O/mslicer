//! Stable hashes of evaluation inputs.

use std::{hash::Hasher, sync::Arc};

use nalgebra::{Matrix4, Vector3};

/// A hash of the inputs a result was computed from.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub struct InputHash(pub u64);

/// FNV-1a (64 bit): stable across runs and Rust versions, so hashes can be
/// compared with ones from earlier runs.
#[derive(Clone, Copy, Debug)]
pub struct InputHasher(u64);

impl Default for InputHasher {
    fn default() -> Self {
        Self(0xcbf2_9ce4_8422_2325)
    }
}

impl Hasher for InputHasher {
    fn finish(&self) -> u64 {
        self.0
    }

    fn write(&mut self, bytes: &[u8]) {
        for b in bytes {
            self.0 ^= *b as u64;
            self.0 = self.0.wrapping_mul(0x0100_0000_01b3);
        }
    }
}

impl InputHasher {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn u64(&mut self, x: u64) -> &mut Self {
        self.write(&x.to_le_bytes());
        self
    }

    pub fn f32(&mut self, x: f32) -> &mut Self {
        self.write(&x.to_bits().to_le_bytes());
        self
    }

    pub fn vector(&mut self, v: &Vector3<f32>) -> &mut Self {
        v.iter().for_each(|x| {
            self.f32(*x);
        });
        self
    }

    pub fn matrix(&mut self, m: &Matrix4<f32>) -> &mut Self {
        m.iter().for_each(|x| {
            self.f32(*x);
        });
        self
    }

    /// The identity of shared data: what it points at, not what it holds.
    /// Only meaningful while the data is alive, which holds for inputs kept
    /// by the thing being hashed.
    pub fn arc<T: ?Sized>(&mut self, arc: &Arc<T>) -> &mut Self {
        self.u64(Arc::as_ptr(arc) as *const () as usize as u64)
    }

    pub fn hash(&mut self, value: &impl std::hash::Hash) -> &mut Self {
        value.hash(self);
        self
    }

    pub fn input_hash(&self) -> InputHash {
        InputHash(self.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stable_and_sensitive() {
        let a = InputHasher::new().u64(1).f32(2.0).input_hash();
        let b = InputHasher::new().u64(1).f32(2.0).input_hash();
        let c = InputHasher::new().u64(1).f32(2.5).input_hash();
        assert_eq!(a, b);
        assert_ne!(a, c);
        // Known value: FNV-1a of nothing.
        assert_eq!(InputHasher::new().input_hash().0, 0xcbf2_9ce4_8422_2325);
    }
}
