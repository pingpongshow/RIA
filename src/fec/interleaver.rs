//! Interleaver implementations for turbo codes
//!
//! Implements S-random interleavers for turbo coding

use std::collections::HashSet;

/// Interleaver type
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InterleaverType {
    /// S-random interleaver with given S parameter
    SRandom(usize),
    /// Block interleaver (rows x cols)
    Block(usize, usize),
    /// Identity (no interleaving)
    Identity,
}

/// Interleaver for turbo codes
pub struct Interleaver {
    permutation: Vec<usize>,
    inverse: Vec<usize>,
    size: usize,
    interleaver_type: InterleaverType,
}

impl Interleaver {
    /// Create a new interleaver of the given type and size
    pub fn new(interleaver_type: InterleaverType, size: usize) -> Self {
        let permutation = match interleaver_type {
            InterleaverType::SRandom(s) => Self::generate_s_random(size, s),
            InterleaverType::Block(rows, cols) => Self::generate_block(rows, cols),
            InterleaverType::Identity => (0..size).collect(),
        };

        let inverse = Self::compute_inverse(&permutation);

        Self {
            permutation,
            inverse,
            size,
            interleaver_type,
        }
    }

    /// Create standard 298-element S-random interleaver (for data)
    pub fn new_298() -> Self {
        Self::new(InterleaverType::SRandom(17), 298)
    }

    /// Create standard 94-element S-random interleaver (for short blocks)
    pub fn new_94() -> Self {
        Self::new(InterleaverType::SRandom(9), 94)
    }

    /// Generate S-random permutation
    /// S-random: |π(i) - π(j)| > S for all |i - j| <= S
    fn generate_s_random(size: usize, s: usize) -> Vec<usize> {
        let mut rng = SimpleRng::new(42); // Fixed seed for reproducibility
        let mut permutation = vec![0usize; size];
        let mut used = HashSet::new();
        let mut attempts = 0;
        const MAX_ATTEMPTS: usize = 1000000;

        for i in 0..size {
            loop {
                attempts += 1;
                if attempts > MAX_ATTEMPTS {
                    // Fall back to simple random permutation
                    return Self::generate_random_fallback(size, &mut rng);
                }

                let candidate = rng.next() % size;

                if used.contains(&candidate) {
                    continue;
                }

                // Check S-random property
                let mut valid = true;
                for j in i.saturating_sub(s)..i {
                    let diff = if candidate > permutation[j] {
                        candidate - permutation[j]
                    } else {
                        permutation[j] - candidate
                    };
                    if diff <= s {
                        valid = false;
                        break;
                    }
                }

                if valid {
                    permutation[i] = candidate;
                    used.insert(candidate);
                    break;
                }
            }
        }

        permutation
    }

    /// Fallback random permutation when S-random fails
    fn generate_random_fallback(size: usize, rng: &mut SimpleRng) -> Vec<usize> {
        let mut permutation: Vec<usize> = (0..size).collect();

        // Fisher-Yates shuffle
        for i in (1..size).rev() {
            let j = rng.next() % (i + 1);
            permutation.swap(i, j);
        }

        permutation
    }

    /// Generate block interleaver permutation
    fn generate_block(rows: usize, cols: usize) -> Vec<usize> {
        let size = rows * cols;
        let mut permutation = vec![0usize; size];

        for i in 0..size {
            let row = i / cols;
            let col = i % cols;
            permutation[i] = col * rows + row;
        }

        permutation
    }

    /// Compute inverse permutation
    fn compute_inverse(permutation: &[usize]) -> Vec<usize> {
        let mut inverse = vec![0usize; permutation.len()];
        for (i, &p) in permutation.iter().enumerate() {
            inverse[p] = i;
        }
        inverse
    }

    /// Interleave data
    pub fn interleave<T: Clone>(&self, data: &[T]) -> Vec<T> {
        assert_eq!(data.len(), self.size, "Data size must match interleaver size");

        let mut output = Vec::with_capacity(self.size);
        for &idx in &self.permutation {
            output.push(data[idx].clone());
        }
        output
    }

    /// Interleave in place (for mutable slice)
    pub fn interleave_inplace<T: Clone + Default>(&self, data: &mut [T]) {
        assert_eq!(data.len(), self.size);

        let original: Vec<T> = data.to_vec();
        for (i, &idx) in self.permutation.iter().enumerate() {
            data[i] = original[idx].clone();
        }
    }

    /// Deinterleave data
    pub fn deinterleave<T: Clone>(&self, data: &[T]) -> Vec<T> {
        assert_eq!(data.len(), self.size, "Data size must match interleaver size");

        let mut output = Vec::with_capacity(self.size);
        for &idx in &self.inverse {
            output.push(data[idx].clone());
        }
        output
    }

    /// Deinterleave in place
    pub fn deinterleave_inplace<T: Clone + Default>(&self, data: &mut [T]) {
        assert_eq!(data.len(), self.size);

        let original: Vec<T> = data.to_vec();
        for (i, &idx) in self.inverse.iter().enumerate() {
            data[i] = original[idx].clone();
        }
    }

    /// Get permutation table
    pub fn permutation(&self) -> &[usize] {
        &self.permutation
    }

    /// Get inverse permutation table
    pub fn inverse(&self) -> &[usize] {
        &self.inverse
    }

    /// Get interleaver size
    pub fn size(&self) -> usize {
        self.size
    }

    /// Get permuted index
    pub fn get_index(&self, i: usize) -> usize {
        self.permutation[i]
    }

    /// Get inverse (deinterleaved) index
    pub fn get_inverse_index(&self, i: usize) -> usize {
        self.inverse[i]
    }
}

/// Simple deterministic RNG for interleaver generation
struct SimpleRng {
    state: u64,
}

impl SimpleRng {
    fn new(seed: u64) -> Self {
        Self { state: seed }
    }

    fn next(&mut self) -> usize {
        // xorshift64
        self.state ^= self.state << 13;
        self.state ^= self.state >> 7;
        self.state ^= self.state << 17;
        self.state as usize
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_identity_interleaver() {
        let interleaver = Interleaver::new(InterleaverType::Identity, 10);
        let data: Vec<u8> = (0..10).collect();

        let interleaved = interleaver.interleave(&data);
        assert_eq!(interleaved, data);

        let deinterleaved = interleaver.deinterleave(&interleaved);
        assert_eq!(deinterleaved, data);
    }

    #[test]
    fn test_block_interleaver() {
        let interleaver = Interleaver::new(InterleaverType::Block(3, 4), 12);
        let data: Vec<u8> = (0..12).collect();

        let interleaved = interleaver.interleave(&data);
        let deinterleaved = interleaver.deinterleave(&interleaved);

        assert_eq!(deinterleaved, data);
    }

    #[test]
    fn test_s_random_interleaver() {
        let interleaver = Interleaver::new_298();
        let data: Vec<u8> = (0..=255).cycle().take(298).collect();

        let interleaved = interleaver.interleave(&data);
        let deinterleaved = interleaver.deinterleave(&interleaved);

        assert_eq!(deinterleaved, data);

        // Verify it actually changes the order
        assert_ne!(interleaved, data);
    }

    #[test]
    fn test_permutation_is_valid() {
        let interleaver = Interleaver::new_94();
        let perm = interleaver.permutation();

        // Check all indices are present exactly once
        let mut sorted: Vec<usize> = perm.to_vec();
        sorted.sort();
        let expected: Vec<usize> = (0..94).collect();
        assert_eq!(sorted, expected);
    }
}
