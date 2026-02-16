//! Convolutional encoder and Viterbi decoder
//!
//! Implements:
//! - K=5 convolutional code with G1=0x17, G2=0x23 (primary)
//! - K=7 NASA convolutional code with G1=0x6D, G2=0x4F (backup/fallback)

use super::Llr;

/// K=5 convolutional encoder
pub struct ConvolutionalEncoder {
    g1: u8,           // Generator polynomial 1
    g2: u8,           // Generator polynomial 2
    constraint: u8,   // Constraint length K
    state: u8,        // Current encoder state
}

impl ConvolutionalEncoder {
    /// Create encoder with default polynomials (G1=0x17, G2=0x23, K=5)
    pub fn new() -> Self {
        Self {
            g1: 0x17, // 10111 binary = 23 decimal
            g2: 0x23, // 100011 binary = 35 decimal
            constraint: 5,
            state: 0,
        }
    }

    /// Create encoder with custom polynomials
    pub fn with_polynomials(g1: u8, g2: u8, k: u8) -> Self {
        Self {
            g1,
            g2,
            constraint: k,
            state: 0,
        }
    }

    /// Encode a single bit, returns (output1, output2)
    pub fn encode_bit(&mut self, bit: u8) -> (u8, u8) {
        // Build full register: input bit at LSB, state shifted left by 1
        let full_reg = ((self.state as u16) << 1) | ((bit & 1) as u16);

        // Calculate outputs using generator polynomials
        let out1 = ((full_reg as u8) & self.g1).count_ones() as u8 & 1;
        let out2 = ((full_reg as u8) & self.g2).count_ones() as u8 & 1;

        // Update state: keep (K-1) bits, shift in new bit
        self.state = (full_reg as u8) & ((1 << (self.constraint - 1)) - 1);

        (out1, out2)
    }

    /// Encode a block of bits
    pub fn encode(&mut self, bits: &[u8]) -> Vec<u8> {
        let mut output = Vec::with_capacity(bits.len() * 2);

        for &bit in bits {
            let (o1, o2) = self.encode_bit(bit);
            output.push(o1);
            output.push(o2);
        }

        output
    }

    /// Encode with tail-biting (for turbo codes)
    pub fn encode_tail(&mut self, bits: &[u8]) -> Vec<u8> {
        let mut output = self.encode(bits);

        // Add tail bits to flush encoder
        for _ in 0..(self.constraint - 1) {
            let (o1, o2) = self.encode_bit(0);
            output.push(o1);
            output.push(o2);
        }

        output
    }

    /// Reset encoder state
    pub fn reset(&mut self) {
        self.state = 0;
    }

    /// Get current state
    pub fn state(&self) -> u8 {
        self.state
    }

    /// Set state (for turbo codes)
    pub fn set_state(&mut self, state: u8) {
        self.state = state & ((1 << (self.constraint - 1)) - 1);
    }
}

impl Default for ConvolutionalEncoder {
    fn default() -> Self {
        Self::new()
    }
}

/// Viterbi decoder for K=5 convolutional codes
pub struct ViterbiDecoder {
    g1: u8,
    g2: u8,
    constraint: u8,
    num_states: usize,

    // Path metrics and traceback
    path_metric: Vec<f32>,
    path_metric_new: Vec<f32>,
    // traceback[i][next_state] = (prev_state, input_bit)
    traceback: Vec<Vec<(usize, u8)>>,
}

impl ViterbiDecoder {
    /// Create decoder with default polynomials
    pub fn new() -> Self {
        Self::with_polynomials(0x17, 0x23, 5)
    }

    /// Create decoder with custom polynomials
    pub fn with_polynomials(g1: u8, g2: u8, k: u8) -> Self {
        let num_states = 1 << (k - 1);

        Self {
            g1,
            g2,
            constraint: k,
            num_states,
            path_metric: vec![f32::NEG_INFINITY; num_states],
            path_metric_new: vec![f32::NEG_INFINITY; num_states],
            traceback: Vec::new(),
        }
    }

    /// Decode soft-decision LLRs
    ///
    /// LLRs must come in pairs (rate 1/2 code). If odd length, last LLR is ignored.
    pub fn decode(&mut self, llrs: &[Llr]) -> Vec<u8> {
        // Handle odd-length input gracefully instead of panicking
        let effective_len = if llrs.len() % 2 != 0 {
            log::warn!("ViterbiDecoder: LLR length {} is odd, ignoring last LLR", llrs.len());
            llrs.len() - 1
        } else {
            llrs.len()
        };
        if effective_len == 0 {
            return Vec::new();
        }

        let num_symbols = effective_len / 2;
        self.traceback.clear();
        self.traceback.resize(num_symbols, vec![(0, 0); self.num_states]);

        // Initialize path metrics
        self.path_metric.fill(f32::NEG_INFINITY);
        self.path_metric[0] = 0.0; // Start from state 0

        // Forward pass
        for i in 0..num_symbols {
            let llr1 = llrs[i * 2];
            let llr2 = llrs[i * 2 + 1];

            self.path_metric_new.fill(f32::NEG_INFINITY);

            for state in 0..self.num_states {
                if self.path_metric[state] == f32::NEG_INFINITY {
                    continue;
                }

                // Try both input bits
                for input in 0..2usize {
                    // Calculate next state: shift state left, add input at LSB, keep K-1 bits
                    let next_state = ((state << 1) | input) & (self.num_states - 1);

                    // Calculate expected outputs using full register
                    let full_reg = ((state << 1) | input) as u8;
                    let exp1 = (full_reg & self.g1).count_ones() & 1;
                    let exp2 = (full_reg & self.g2).count_ones() & 1;

                    // Branch metric (correlation with expected)
                    let bm = if exp1 == 0 { llr1 } else { -llr1 }
                           + if exp2 == 0 { llr2 } else { -llr2 };

                    let new_metric = self.path_metric[state] + bm;

                    if new_metric > self.path_metric_new[next_state] {
                        self.path_metric_new[next_state] = new_metric;
                        self.traceback[i][next_state] = (state, input as u8);
                    }
                }
            }

            std::mem::swap(&mut self.path_metric, &mut self.path_metric_new);
        }

        // Traceback from best final state
        let mut best_state = 0;
        let mut best_metric = f32::NEG_INFINITY;
        for (state, &metric) in self.path_metric.iter().enumerate() {
            if metric > best_metric {
                best_metric = metric;
                best_state = state;
            }
        }

        let mut decoded = vec![0u8; num_symbols];
        let mut state = best_state;

        for i in (0..num_symbols).rev() {
            let (prev_state, input) = self.traceback[i][state];
            decoded[i] = input;
            state = prev_state;
        }

        decoded
    }

    /// Decode hard-decision bits
    pub fn decode_hard(&mut self, bits: &[u8]) -> Vec<u8> {
        // Convert to soft decisions
        let llrs: Vec<Llr> = bits.iter()
            .map(|&b| if b == 0 { 1.0 } else { -1.0 })
            .collect();
        self.decode(&llrs)
    }

    /// Reset decoder state
    pub fn reset(&mut self) {
        self.path_metric.fill(f32::NEG_INFINITY);
        self.traceback.clear();
    }
}

impl Default for ViterbiDecoder {
    fn default() -> Self {
        Self::new()
    }
}

/// K=7 NASA convolutional encoder (G1=0x6D, G2=0x4F)
/// Used as backup/fallback FEC for difficult channel conditions
pub struct K7ConvolutionalEncoder {
    state: u8,
}

impl K7ConvolutionalEncoder {
    /// NASA standard polynomials for K=7
    const G1: u8 = 0x6D; // 1101101 binary = 109 decimal
    const G2: u8 = 0x4F; // 1001111 binary = 79 decimal
    const K: u8 = 7;

    /// Create new K=7 encoder
    pub fn new() -> Self {
        Self { state: 0 }
    }

    /// Encode a single bit, returns (output1, output2)
    pub fn encode_bit(&mut self, bit: u8) -> (u8, u8) {
        // Full register: input bit at MSB position, state shifted
        let full_reg = ((bit & 1) << 6) | (self.state as u8);

        // Calculate outputs using generator polynomials
        let out1 = (full_reg & Self::G1).count_ones() as u8 & 1;
        let out2 = (full_reg & Self::G2).count_ones() as u8 & 1;

        // Update state: shift right, input goes to MSB of state (6 bits)
        self.state = (full_reg >> 1) & 0x3F;

        (out1, out2)
    }

    /// Encode a block of bits
    pub fn encode(&mut self, bits: &[u8]) -> Vec<u8> {
        let mut output = Vec::with_capacity(bits.len() * 2);

        for &bit in bits {
            let (o1, o2) = self.encode_bit(bit);
            output.push(o1);
            output.push(o2);
        }

        output
    }

    /// Encode with tail bits to flush encoder
    pub fn encode_tail(&mut self, bits: &[u8]) -> Vec<u8> {
        let mut output = self.encode(bits);

        // Add tail bits to flush encoder (K-1 = 6 bits)
        for _ in 0..(Self::K - 1) {
            let (o1, o2) = self.encode_bit(0);
            output.push(o1);
            output.push(o2);
        }

        output
    }

    /// Reset encoder state
    pub fn reset(&mut self) {
        self.state = 0;
    }

    /// Get current state
    pub fn state(&self) -> u8 {
        self.state
    }
}

impl Default for K7ConvolutionalEncoder {
    fn default() -> Self {
        Self::new()
    }
}

/// K=7 Viterbi decoder for NASA convolutional code
pub struct K7ViterbiDecoder {
    num_states: usize,
    path_metric: Vec<f32>,
    path_metric_new: Vec<f32>,
    traceback: Vec<Vec<(usize, u8)>>,
}

impl K7ViterbiDecoder {
    const G1: u8 = 0x6D;
    const G2: u8 = 0x4F;
    const K: u8 = 7;

    /// Create new K=7 decoder
    pub fn new() -> Self {
        let num_states = 1 << (Self::K - 1); // 64 states

        Self {
            num_states,
            path_metric: vec![f32::NEG_INFINITY; num_states],
            path_metric_new: vec![f32::NEG_INFINITY; num_states],
            traceback: Vec::new(),
        }
    }

    /// Decode soft-decision LLRs
    ///
    /// LLRs must come in pairs (rate 1/2 code). If odd length, last LLR is ignored.
    pub fn decode(&mut self, llrs: &[Llr]) -> Vec<u8> {
        // Handle odd-length input gracefully instead of panicking
        let effective_len = if llrs.len() % 2 != 0 {
            log::warn!("K7ViterbiDecoder: LLR length {} is odd, ignoring last LLR", llrs.len());
            llrs.len() - 1
        } else {
            llrs.len()
        };
        if effective_len == 0 {
            return Vec::new();
        }

        let num_symbols = effective_len / 2;
        self.traceback.clear();
        self.traceback.resize(num_symbols, vec![(0, 0); self.num_states]);

        // Initialize path metrics
        self.path_metric.fill(f32::NEG_INFINITY);
        self.path_metric[0] = 0.0;

        // Forward pass
        for i in 0..num_symbols {
            let llr1 = llrs[i * 2];
            let llr2 = llrs[i * 2 + 1];

            self.path_metric_new.fill(f32::NEG_INFINITY);

            for state in 0..self.num_states {
                if self.path_metric[state] == f32::NEG_INFINITY {
                    continue;
                }

                // Try both input bits
                for input in 0..2usize {
                    // Full register: input at MSB, state at lower bits
                    let full_reg = ((input << 6) | state) as u8;

                    // Expected outputs
                    let exp1 = (full_reg & Self::G1).count_ones() & 1;
                    let exp2 = (full_reg & Self::G2).count_ones() & 1;

                    // Next state: shift right
                    let next_state = (full_reg >> 1) as usize & (self.num_states - 1);

                    // Branch metric
                    let bm = if exp1 == 0 { llr1 } else { -llr1 }
                           + if exp2 == 0 { llr2 } else { -llr2 };

                    let new_metric = self.path_metric[state] + bm;

                    if new_metric > self.path_metric_new[next_state] {
                        self.path_metric_new[next_state] = new_metric;
                        self.traceback[i][next_state] = (state, input as u8);
                    }
                }
            }

            std::mem::swap(&mut self.path_metric, &mut self.path_metric_new);
        }

        // Traceback from best final state
        let mut best_state = 0;
        let mut best_metric = f32::NEG_INFINITY;
        for (state, &metric) in self.path_metric.iter().enumerate() {
            if metric > best_metric {
                best_metric = metric;
                best_state = state;
            }
        }

        let mut decoded = vec![0u8; num_symbols];
        let mut state = best_state;

        for i in (0..num_symbols).rev() {
            let (prev_state, input) = self.traceback[i][state];
            decoded[i] = input;
            state = prev_state;
        }

        decoded
    }

    /// Decode hard-decision bits
    pub fn decode_hard(&mut self, bits: &[u8]) -> Vec<u8> {
        let llrs: Vec<Llr> = bits.iter()
            .map(|&b| if b == 0 { 1.0 } else { -1.0 })
            .collect();
        self.decode(&llrs)
    }

    /// Reset decoder state
    pub fn reset(&mut self) {
        self.path_metric.fill(f32::NEG_INFINITY);
        self.traceback.clear();
    }
}

impl Default for K7ViterbiDecoder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_encoder_basic() {
        let mut encoder = ConvolutionalEncoder::new();

        // Encode a simple pattern
        let bits = vec![1, 0, 1, 1, 0];
        let encoded = encoder.encode(&bits);

        assert_eq!(encoded.len(), bits.len() * 2);
    }

    #[test]
    fn test_round_trip() {
        let mut encoder = ConvolutionalEncoder::new();
        let mut decoder = ViterbiDecoder::new();

        let original = vec![1, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 0];
        let encoded = encoder.encode_tail(&original);

        // Convert to soft decisions
        let llrs: Vec<Llr> = encoded.iter()
            .map(|&b| if b == 0 { 5.0 } else { -5.0 })
            .collect();

        let decoded = decoder.decode(&llrs);

        // Check that original bits match (excluding tail)
        for i in 0..original.len() {
            assert_eq!(decoded[i], original[i], "Mismatch at position {}", i);
        }
    }

    #[test]
    fn test_k7_encoder_basic() {
        let mut encoder = K7ConvolutionalEncoder::new();

        let bits = vec![1, 0, 1, 1, 0];
        let encoded = encoder.encode(&bits);

        assert_eq!(encoded.len(), bits.len() * 2);
    }

    #[test]
    fn test_k7_round_trip() {
        let mut encoder = K7ConvolutionalEncoder::new();
        let mut decoder = K7ViterbiDecoder::new();

        let original = vec![1, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1];
        let encoded = encoder.encode_tail(&original);

        // Convert to soft decisions
        let llrs: Vec<Llr> = encoded.iter()
            .map(|&b| if b == 0 { 5.0 } else { -5.0 })
            .collect();

        let decoded = decoder.decode(&llrs);

        // Check that original bits match (excluding tail)
        for i in 0..original.len() {
            assert_eq!(decoded[i], original[i], "K7 mismatch at position {}", i);
        }
    }

    #[test]
    fn test_k7_with_noise() {
        let mut encoder = K7ConvolutionalEncoder::new();
        let mut decoder = K7ViterbiDecoder::new();

        let original: Vec<u8> = (0..50).map(|i| ((i * 7) % 2) as u8).collect();
        let encoded = encoder.encode_tail(&original);

        // Add noise - flip some bits
        let noisy: Vec<Llr> = encoded.iter().enumerate()
            .map(|(i, &b)| {
                let base = if b == 0 { 3.0 } else { -3.0 };
                // Add noise
                base + ((i as f32 * 0.7).sin() * 1.5)
            })
            .collect();

        let decoded = decoder.decode(&noisy);

        // Count errors
        let errors: usize = original.iter()
            .zip(decoded.iter())
            .filter(|(&o, &d)| o != d)
            .count();

        // K=7 should handle moderate noise well
        assert!(errors < 5, "Too many errors: {}", errors);
    }
}
