//! Turbo encoder and decoder
//!
//! Implements parallel concatenated convolutional codes (PCCC) turbo coding
//! with iterative BCJR/MAP decoding.

use super::{ConvolutionalEncoder, Llr, CodeRate};
use super::interleaver::{Interleaver, InterleaverType};

/// Turbo encoder using two constituent convolutional encoders
pub struct TurboEncoder {
    encoder1: ConvolutionalEncoder,
    encoder2: ConvolutionalEncoder,
    interleaver: Interleaver,
    rate: CodeRate,
}

impl TurboEncoder {
    /// Create turbo encoder with given block size
    pub fn new(block_size: usize) -> Self {
        let s = (block_size as f32).sqrt() as usize;
        Self {
            encoder1: ConvolutionalEncoder::new(),
            encoder2: ConvolutionalEncoder::new(),
            interleaver: Interleaver::new(InterleaverType::SRandom(s.max(5)), block_size),
            rate: CodeRate::Rate1_2,
        }
    }

    /// Create turbo encoder with given block size and rate
    pub fn new_with_rate(block_size: usize, rate: CodeRate) -> Self {
        let s = (block_size as f32).sqrt() as usize;
        Self {
            encoder1: ConvolutionalEncoder::new(),
            encoder2: ConvolutionalEncoder::new(),
            interleaver: Interleaver::new(InterleaverType::SRandom(s.max(5)), block_size),
            rate,
        }
    }

    /// Create turbo encoder with standard 298-element interleaver
    pub fn new_298() -> Self {
        Self {
            encoder1: ConvolutionalEncoder::new(),
            encoder2: ConvolutionalEncoder::new(),
            interleaver: Interleaver::new_298(),
            rate: CodeRate::Rate1_2,
        }
    }

    /// Create turbo encoder with standard 94-element interleaver
    pub fn new_94() -> Self {
        Self {
            encoder1: ConvolutionalEncoder::new(),
            encoder2: ConvolutionalEncoder::new(),
            interleaver: Interleaver::new_94(),
            rate: CodeRate::Rate1_2,
        }
    }

    /// Set puncturing (rate 1/2 if true, rate 1/3 if false) - legacy API
    pub fn set_puncture(&mut self, puncture: bool) {
        self.rate = if puncture { CodeRate::Rate1_2 } else { CodeRate::Rate1_3 };
    }

    /// Set code rate
    pub fn set_rate(&mut self, rate: CodeRate) {
        self.rate = rate;
    }

    /// Get current code rate
    pub fn rate(&self) -> CodeRate {
        self.rate
    }

    /// Encode a block of data
    /// Returns systematic bits + parity from encoder 1 + parity from encoder 2
    pub fn encode(&mut self, data: &[u8]) -> Vec<u8> {
        assert_eq!(data.len(), self.interleaver.size(),
            "Data size must match interleaver size");

        self.encoder1.reset();
        self.encoder2.reset();

        // First encoder: systematic data
        let mut parity1 = Vec::with_capacity(data.len());
        for &bit in data {
            let (_, p1) = self.encoder1.encode_bit(bit);
            parity1.push(p1);
        }

        // Tail bits for encoder 1
        let mut tail1 = Vec::new();
        for _ in 0..4 {
            let (s, p) = self.encoder1.encode_bit(0);
            tail1.push(s);
            tail1.push(p);
        }

        // Interleave for second encoder
        let interleaved = self.interleaver.interleave(data);

        // Second encoder: interleaved data
        let mut parity2 = Vec::with_capacity(data.len());
        for &bit in &interleaved {
            let (_, p2) = self.encoder2.encode_bit(bit);
            parity2.push(p2);
        }

        // Tail bits for encoder 2
        let mut tail2 = Vec::new();
        for _ in 0..4 {
            let (s, p) = self.encoder2.encode_bit(0);
            tail2.push(s);
            tail2.push(p);
        }

        // Combine outputs based on rate
        let mut output = Vec::new();

        match self.rate {
            CodeRate::Rate1_3 => {
                // Rate 1/3: systematic + both parities (no puncturing)
                for i in 0..data.len() {
                    output.push(data[i]);
                    output.push(parity1[i]);
                    output.push(parity2[i]);
                }
            }
            CodeRate::Rate1_2 => {
                // Rate 1/2: systematic + alternating parity
                for i in 0..data.len() {
                    output.push(data[i]);
                    if i % 2 == 0 {
                        output.push(parity1[i]);
                    } else {
                        output.push(parity2[i]);
                    }
                }
            }
            CodeRate::Rate2_3 => {
                // Rate 2/3: systematic + parity every 2nd position
                // Pattern over 4 bits: sys, par, sys, sys, par, sys = 6 output for 4 input
                for i in 0..data.len() {
                    output.push(data[i]);
                    if i % 2 == 0 {
                        // Keep parity for even positions only
                        if (i / 2) % 2 == 0 {
                            output.push(parity1[i]);
                        } else {
                            output.push(parity2[i]);
                        }
                    }
                }
            }
            CodeRate::Rate3_4 => {
                // Rate 3/4: systematic + parity every 3rd position
                // Pattern: sys, par, sys, sys, sys, par, ... = 4 output for 3 input
                for i in 0..data.len() {
                    output.push(data[i]);
                    if i % 3 == 0 {
                        if (i / 3) % 2 == 0 {
                            output.push(parity1[i]);
                        } else {
                            output.push(parity2[i]);
                        }
                    }
                }
            }
            CodeRate::Rate5_6 => {
                // Rate 5/6: systematic + parity every 5th position
                // Pattern: sys, par, sys, sys, sys, sys, sys, par, ... = 6 output for 5 input
                for i in 0..data.len() {
                    output.push(data[i]);
                    if i % 5 == 0 {
                        if (i / 5) % 2 == 0 {
                            output.push(parity1[i]);
                        } else {
                            output.push(parity2[i]);
                        }
                    }
                }
            }
        }

        // Append tail bits
        output.extend_from_slice(&tail1);
        output.extend_from_slice(&tail2);

        output
    }

    /// Get the block size (interleaver size)
    pub fn block_size(&self) -> usize {
        self.interleaver.size()
    }
}

/// BCJR/MAP decoder state
struct BcjrState {
    num_states: usize,
    alpha: Vec<Vec<f32>>,
    beta: Vec<Vec<f32>>,
    gamma: Vec<Vec<[f32; 2]>>,
}

impl BcjrState {
    fn new(num_states: usize, length: usize) -> Self {
        Self {
            num_states,
            alpha: vec![vec![f32::NEG_INFINITY; num_states]; length + 1],
            beta: vec![vec![f32::NEG_INFINITY; num_states]; length + 1],
            gamma: vec![vec![[0.0f32; 2]; num_states]; length],
        }
    }

    fn reset(&mut self, length: usize) {
        for a in &mut self.alpha {
            a.fill(f32::NEG_INFINITY);
        }
        for b in &mut self.beta {
            b.fill(f32::NEG_INFINITY);
        }
        self.gamma.resize(length, vec![[0.0f32; 2]; self.num_states]);
    }
}

/// Turbo decoder using iterative BCJR algorithm
pub struct TurboDecoder {
    g1: u8,
    g2: u8,
    constraint: u8,
    num_states: usize,
    interleaver: Interleaver,
    rate: CodeRate,
    iterations: usize,

    // Decoder state
    bcjr1: BcjrState,
    bcjr2: BcjrState,

    // Extrinsic information
    extrinsic1: Vec<Llr>,
    extrinsic2: Vec<Llr>,
}

impl TurboDecoder {
    /// Create turbo decoder with given block size
    pub fn new(block_size: usize) -> Self {
        let s = (block_size as f32).sqrt() as usize;
        let num_states = 16; // K=5 -> 2^4 states

        Self {
            g1: 0x17,
            g2: 0x23,
            constraint: 5,
            num_states,
            interleaver: Interleaver::new(InterleaverType::SRandom(s.max(5)), block_size),
            rate: CodeRate::Rate1_2,
            iterations: 8,
            bcjr1: BcjrState::new(num_states, block_size),
            bcjr2: BcjrState::new(num_states, block_size),
            extrinsic1: vec![0.0; block_size],
            extrinsic2: vec![0.0; block_size],
        }
    }

    /// Create turbo decoder with given block size and rate
    pub fn new_with_rate(block_size: usize, rate: CodeRate) -> Self {
        let s = (block_size as f32).sqrt() as usize;
        let num_states = 16;

        Self {
            g1: 0x17,
            g2: 0x23,
            constraint: 5,
            num_states,
            interleaver: Interleaver::new(InterleaverType::SRandom(s.max(5)), block_size),
            rate,
            iterations: 8,
            bcjr1: BcjrState::new(num_states, block_size),
            bcjr2: BcjrState::new(num_states, block_size),
            extrinsic1: vec![0.0; block_size],
            extrinsic2: vec![0.0; block_size],
        }
    }

    /// Create decoder with standard 298-element interleaver
    pub fn new_298() -> Self {
        Self::new_with_interleaver(Interleaver::new_298())
    }

    /// Create decoder with standard 94-element interleaver
    pub fn new_94() -> Self {
        Self::new_with_interleaver(Interleaver::new_94())
    }

    fn new_with_interleaver(interleaver: Interleaver) -> Self {
        let block_size = interleaver.size();
        let num_states = 16;

        Self {
            g1: 0x17,
            g2: 0x23,
            constraint: 5,
            num_states,
            interleaver,
            rate: CodeRate::Rate1_2,
            iterations: 8,
            bcjr1: BcjrState::new(num_states, block_size),
            bcjr2: BcjrState::new(num_states, block_size),
            extrinsic1: vec![0.0; block_size],
            extrinsic2: vec![0.0; block_size],
        }
    }

    /// Set number of iterations
    pub fn set_iterations(&mut self, iterations: usize) {
        self.iterations = iterations.max(1).min(20);
    }

    /// Set puncturing mode - legacy API
    pub fn set_punctured(&mut self, punctured: bool) {
        self.rate = if punctured { CodeRate::Rate1_2 } else { CodeRate::Rate1_3 };
    }

    /// Set code rate
    pub fn set_rate(&mut self, rate: CodeRate) {
        self.rate = rate;
    }

    /// Get current code rate
    pub fn rate(&self) -> CodeRate {
        self.rate
    }

    /// Decode received LLRs
    pub fn decode(&mut self, llrs: &[Llr]) -> Vec<u8> {
        let block_size = self.interleaver.size();

        // Separate systematic and parity LLRs
        let (sys, parity1, parity2) = self.demux_llrs(llrs, block_size);

        // Initialize extrinsic information
        self.extrinsic1.fill(0.0);
        self.extrinsic2.fill(0.0);

        // Extract parameters needed for BCJR
        let g2 = self.g2;
        let constraint = self.constraint;
        let num_states = self.num_states;

        // Iterative decoding
        for _ in 0..self.iterations {
            // Decoder 1
            let llr_in1: Vec<Llr> = sys.iter()
                .zip(self.extrinsic2.iter())
                .map(|(&s, &e)| s + e)
                .collect();

            bcjr_decode_impl(g2, constraint, num_states, &llr_in1, &parity1, &mut self.bcjr1, &mut self.extrinsic1);

            // Interleave extrinsic for decoder 2
            let ext1_int = self.interleaver.interleave(&self.extrinsic1);
            let sys_int = self.interleaver.interleave(&sys);

            // Decoder 2
            let llr_in2: Vec<Llr> = sys_int.iter()
                .zip(ext1_int.iter())
                .map(|(&s, &e)| s + e)
                .collect();

            let mut ext2_int = vec![0.0; block_size];
            bcjr_decode_impl(g2, constraint, num_states, &llr_in2, &parity2, &mut self.bcjr2, &mut ext2_int);

            // Deinterleave extrinsic for next iteration
            self.extrinsic2 = self.interleaver.deinterleave(&ext2_int);
        }

        // Final hard decision
        // Positive LLR means bit 0 is more likely, negative means bit 1
        sys.iter()
            .zip(self.extrinsic1.iter())
            .zip(self.extrinsic2.iter())
            .map(|((&s, &e1), &e2)| if s + e1 + e2 >= 0.0 { 0 } else { 1 })
            .collect()
    }

    /// Demultiplex received LLRs into systematic and parity streams
    fn demux_llrs(&self, llrs: &[Llr], block_size: usize) -> (Vec<Llr>, Vec<Llr>, Vec<Llr>) {
        let mut sys = vec![0.0; block_size];
        let mut parity1 = vec![0.0; block_size];
        let mut parity2 = vec![0.0; block_size];

        match self.rate {
            CodeRate::Rate1_3 => {
                // Rate 1/3: systematic + both parities
                for i in 0..block_size {
                    let idx = i * 3;
                    if idx + 2 < llrs.len() {
                        sys[i] = llrs[idx];
                        parity1[i] = llrs[idx + 1];
                        parity2[i] = llrs[idx + 2];
                    }
                }
            }
            CodeRate::Rate1_2 => {
                // Rate 1/2: systematic + alternating parity
                for i in 0..block_size {
                    let idx = i * 2;
                    if idx + 1 < llrs.len() {
                        sys[i] = llrs[idx];
                        if i % 2 == 0 {
                            parity1[i] = llrs[idx + 1];
                            parity2[i] = 0.0; // Punctured
                        } else {
                            parity1[i] = 0.0; // Punctured
                            parity2[i] = llrs[idx + 1];
                        }
                    }
                }
            }
            CodeRate::Rate2_3 => {
                // Rate 2/3: systematic + parity every 2nd position
                // Encoder pattern: sys always, parity at even i positions only
                let mut llr_idx = 0;
                for i in 0..block_size {
                    if llr_idx < llrs.len() {
                        sys[i] = llrs[llr_idx];
                        llr_idx += 1;
                    }
                    if i % 2 == 0 {
                        // Parity present at even positions
                        if llr_idx < llrs.len() {
                            if (i / 2) % 2 == 0 {
                                parity1[i] = llrs[llr_idx];
                                parity2[i] = 0.0; // Punctured
                            } else {
                                parity1[i] = 0.0; // Punctured
                                parity2[i] = llrs[llr_idx];
                            }
                            llr_idx += 1;
                        }
                    } else {
                        // No parity at odd positions - both punctured
                        parity1[i] = 0.0;
                        parity2[i] = 0.0;
                    }
                }
            }
            CodeRate::Rate3_4 => {
                // Rate 3/4: systematic + parity every 3rd position
                // Encoder pattern: sys always, parity at i % 3 == 0 positions only
                let mut llr_idx = 0;
                for i in 0..block_size {
                    if llr_idx < llrs.len() {
                        sys[i] = llrs[llr_idx];
                        llr_idx += 1;
                    }
                    if i % 3 == 0 {
                        // Parity present at positions divisible by 3
                        if llr_idx < llrs.len() {
                            if (i / 3) % 2 == 0 {
                                parity1[i] = llrs[llr_idx];
                                parity2[i] = 0.0; // Punctured
                            } else {
                                parity1[i] = 0.0; // Punctured
                                parity2[i] = llrs[llr_idx];
                            }
                            llr_idx += 1;
                        }
                    } else {
                        // No parity at non-3rd positions - both punctured
                        parity1[i] = 0.0;
                        parity2[i] = 0.0;
                    }
                }
            }
            CodeRate::Rate5_6 => {
                // Rate 5/6: systematic + parity every 5th position
                // Encoder pattern: sys always, parity at i % 5 == 0 positions only
                let mut llr_idx = 0;
                for i in 0..block_size {
                    if llr_idx < llrs.len() {
                        sys[i] = llrs[llr_idx];
                        llr_idx += 1;
                    }
                    if i % 5 == 0 {
                        // Parity present at positions divisible by 5
                        if llr_idx < llrs.len() {
                            if (i / 5) % 2 == 0 {
                                parity1[i] = llrs[llr_idx];
                                parity2[i] = 0.0; // Punctured
                            } else {
                                parity1[i] = 0.0; // Punctured
                                parity2[i] = llrs[llr_idx];
                            }
                            llr_idx += 1;
                        }
                    } else {
                        // No parity at non-5th positions - both punctured
                        parity1[i] = 0.0;
                        parity2[i] = 0.0;
                    }
                }
            }
        }

        (sys, parity1, parity2)
    }

    /// Get block size
    pub fn block_size(&self) -> usize {
        self.interleaver.size()
    }
}

/// BCJR/MAP decoding for one constituent decoder (free function to avoid borrow issues)
fn bcjr_decode_impl(
    g2: u8,
    _constraint: u8,
    num_states: usize,
    sys: &[Llr],
    parity: &[Llr],
    state: &mut BcjrState,
    extrinsic: &mut [Llr],
) {
    let len = sys.len();
    state.reset(len);

    // Calculate branch metrics (gamma)
    // State transition: next_state = ((state << 1) | input) & (num_states - 1)
    for k in 0..len {
        for s in 0..num_states {
            for input in 0..2 {
                // Calculate expected parity: full_reg = (state << 1) | input
                let full_reg = ((s << 1) | input) as u8;
                let exp_parity = (full_reg & g2).count_ones() & 1;

                // Branch metric for this (state, input) pair
                let bm_sys = if input == 0 { sys[k] } else { -sys[k] };
                let bm_par = if exp_parity == 0 { parity[k] } else { -parity[k] };

                state.gamma[k][s][input] = (bm_sys + bm_par) / 2.0;
            }
        }
    }

    // Forward recursion (alpha)
    // alpha[k][s] = probability of being in state s at time k
    state.alpha[0][0] = 0.0; // Start in state 0
    for k in 0..len {
        for s in 0..num_states {
            if state.alpha[k][s] == f32::NEG_INFINITY {
                continue;
            }
            for input in 0..2 {
                let next_state = ((s << 1) | input) & (num_states - 1);
                let metric = state.alpha[k][s] + state.gamma[k][s][input];
                state.alpha[k + 1][next_state] = log_sum_exp(state.alpha[k + 1][next_state], metric);
            }
        }

        // Normalize to prevent overflow
        let max_alpha = state.alpha[k + 1].iter().cloned().fold(f32::NEG_INFINITY, f32::max);
        if max_alpha.is_finite() {
            for s in 0..num_states {
                state.alpha[k + 1][s] -= max_alpha;
            }
        }
    }

    // Backward recursion (beta)
    // beta[k][s] = probability of ending in terminal state from state s at time k
    // All terminal states allowed (no termination constraint for now)
    for s in 0..num_states {
        state.beta[len][s] = 0.0;
    }
    for k in (0..len).rev() {
        for s in 0..num_states {
            for input in 0..2 {
                let next_state = ((s << 1) | input) & (num_states - 1);
                let metric = state.beta[k + 1][next_state] + state.gamma[k][s][input];
                state.beta[k][s] = log_sum_exp(state.beta[k][s], metric);
            }
        }

        // Normalize
        let max_beta = state.beta[k].iter().cloned().fold(f32::NEG_INFINITY, f32::max);
        if max_beta.is_finite() {
            for s in 0..num_states {
                state.beta[k][s] -= max_beta;
            }
        }
    }

    // Calculate LLRs and extrinsic information
    for k in 0..len {
        let mut llr0 = f32::NEG_INFINITY;
        let mut llr1 = f32::NEG_INFINITY;

        for s in 0..num_states {
            // Input 0: transition from state s with input 0
            let next_state0 = ((s << 1) | 0) & (num_states - 1);
            let metric0 = state.alpha[k][s] + state.gamma[k][s][0] + state.beta[k + 1][next_state0];
            llr0 = log_sum_exp(llr0, metric0);

            // Input 1: transition from state s with input 1
            let next_state1 = ((s << 1) | 1) & (num_states - 1);
            let metric1 = state.alpha[k][s] + state.gamma[k][s][1] + state.beta[k + 1][next_state1];
            llr1 = log_sum_exp(llr1, metric1);
        }

        // Extrinsic = posterior - prior (prior = a priori from systematic)
        extrinsic[k] = llr0 - llr1 - sys[k];
    }
}

/// Log-sum-exp for combining probabilities in log domain
#[inline]
fn log_sum_exp(a: f32, b: f32) -> f32 {
    if a == f32::NEG_INFINITY {
        return b;
    }
    if b == f32::NEG_INFINITY {
        return a;
    }

    let max = a.max(b);
    let min = a.min(b);

    max + (1.0 + (min - max).exp()).ln()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_turbo_round_trip() {
        let block_size = 94;
        let mut encoder = TurboEncoder::new_94();
        let mut decoder = TurboDecoder::new_94();

        // Random data
        let data: Vec<u8> = (0..block_size).map(|i| (i % 2) as u8).collect();

        // Encode
        let encoded = encoder.encode(&data);

        // Convert to LLRs (clean channel)
        let llrs: Vec<Llr> = encoded.iter()
            .map(|&b| if b == 0 { 5.0 } else { -5.0 })
            .collect();

        // Decode
        let decoded = decoder.decode(&llrs);

        // Verify
        for i in 0..block_size {
            assert_eq!(decoded[i], data[i], "Mismatch at position {}", i);
        }
    }

    #[test]
    fn test_turbo_with_noise() {
        let block_size = 94;
        let mut encoder = TurboEncoder::new_94();
        let mut decoder = TurboDecoder::new_94();
        decoder.set_iterations(8);

        let data: Vec<u8> = (0..block_size).map(|i| ((i * 7) % 2) as u8).collect();
        let encoded = encoder.encode(&data);

        // Add some noise to LLRs
        let llrs: Vec<Llr> = encoded.iter().enumerate()
            .map(|(i, &b)| {
                let base = if b == 0 { 3.0 } else { -3.0 };
                // Add deterministic "noise"
                base + (((i * 17) % 7) as f32 - 3.0) * 0.3
            })
            .collect();

        let decoded = decoder.decode(&llrs);

        // Count errors
        let errors: usize = decoded.iter()
            .zip(data.iter())
            .filter(|(&d, &o)| d != o)
            .count();

        // Should have very few errors with turbo coding
        assert!(errors < block_size / 10, "Too many errors: {}", errors);
    }

    #[test]
    fn test_turbo_rate_2_3() {
        // Mode 11 uses 2688 bits with rate 2/3
        let block_size = 2688;
        let mut encoder = TurboEncoder::new_with_rate(block_size, CodeRate::Rate2_3);
        let mut decoder = TurboDecoder::new_with_rate(block_size, CodeRate::Rate2_3);
        decoder.set_iterations(8);

        // Pseudo-random data pattern
        let data: Vec<u8> = (0..block_size).map(|i| ((i * 7 + 13) % 2) as u8).collect();
        let encoded = encoder.encode(&data);

        // Expected output size: block_size + block_size/2 (sys + parity at half rate) + tail
        let expected_data_bits = block_size + block_size / 2;  // 2688 + 1344 = 4032
        let tail_bits = 16;  // 8 tail bits from each encoder
        eprintln!("Rate 2/3: {} input bits -> {} encoded bits (expected ~{})",
            block_size, encoded.len(), expected_data_bits + tail_bits);

        // Convert to LLRs (clean channel)
        let llrs: Vec<Llr> = encoded.iter()
            .map(|&b| if b == 0 { 5.0 } else { -5.0 })
            .collect();

        let decoded = decoder.decode(&llrs);

        // Count errors
        let errors: usize = decoded.iter()
            .zip(data.iter())
            .filter(|(&d, &o)| d != o)
            .count();

        assert_eq!(errors, 0, "Rate 2/3 clean channel should have 0 errors, got {}", errors);
    }

    #[test]
    fn test_turbo_rate_3_4() {
        // Mode 12 uses 3024 bits with rate 3/4
        let block_size = 3024;
        let mut encoder = TurboEncoder::new_with_rate(block_size, CodeRate::Rate3_4);
        let mut decoder = TurboDecoder::new_with_rate(block_size, CodeRate::Rate3_4);
        decoder.set_iterations(8);

        // Pseudo-random data pattern
        let data: Vec<u8> = (0..block_size).map(|i| ((i * 11 + 5) % 2) as u8).collect();
        let encoded = encoder.encode(&data);

        // Expected output size: block_size + block_size/3 (sys + parity at 1/3 rate) + tail
        let expected_data_bits = block_size + block_size / 3;  // 3024 + 1008 = 4032
        let tail_bits = 16;
        eprintln!("Rate 3/4: {} input bits -> {} encoded bits (expected ~{})",
            block_size, encoded.len(), expected_data_bits + tail_bits);

        // Convert to LLRs (clean channel)
        let llrs: Vec<Llr> = encoded.iter()
            .map(|&b| if b == 0 { 5.0 } else { -5.0 })
            .collect();

        let decoded = decoder.decode(&llrs);

        // Count errors
        let errors: usize = decoded.iter()
            .zip(data.iter())
            .filter(|(&d, &o)| d != o)
            .count();

        assert_eq!(errors, 0, "Rate 3/4 clean channel should have 0 errors, got {}", errors);
    }

    #[test]
    fn test_rate_output_size() {
        // Verify encoder output sizes for each rate
        let block_size = 100;

        // Rate 1/3: 3 output bits per input bit
        let mut enc_1_3 = TurboEncoder::new_with_rate(block_size, CodeRate::Rate1_3);
        let data: Vec<u8> = (0..block_size).map(|i| (i % 2) as u8).collect();
        let encoded_1_3 = enc_1_3.encode(&data);
        assert_eq!(encoded_1_3.len(), 3 * block_size + 16, "Rate 1/3 output size");

        // Rate 1/2: 2 output bits per input bit
        let mut enc_1_2 = TurboEncoder::new_with_rate(block_size, CodeRate::Rate1_2);
        let encoded_1_2 = enc_1_2.encode(&data);
        assert_eq!(encoded_1_2.len(), 2 * block_size + 16, "Rate 1/2 output size");

        // Rate 2/3: 1.5 output bits per input bit (sys + parity every other)
        let mut enc_2_3 = TurboEncoder::new_with_rate(block_size, CodeRate::Rate2_3);
        let encoded_2_3 = enc_2_3.encode(&data);
        let expected_2_3 = block_size + block_size / 2 + 16;  // sys + half parity + tail
        assert_eq!(encoded_2_3.len(), expected_2_3, "Rate 2/3 output size");

        // Rate 3/4: 4/3 output bits per input bit (sys + parity every third)
        let mut enc_3_4 = TurboEncoder::new_with_rate(block_size, CodeRate::Rate3_4);
        let encoded_3_4 = enc_3_4.encode(&data);
        let expected_3_4 = block_size + block_size / 3 + (if block_size % 3 != 0 { 1 } else { 0 }) + 16;  // sys + third parity + tail
        assert_eq!(encoded_3_4.len(), expected_3_4, "Rate 3/4 output size");

        // Rate 5/6: 6/5 output bits per input bit (sys + parity every fifth)
        let mut enc_5_6 = TurboEncoder::new_with_rate(block_size, CodeRate::Rate5_6);
        let encoded_5_6 = enc_5_6.encode(&data);
        let expected_5_6 = block_size + block_size / 5 + 16;  // sys + fifth parity + tail
        assert_eq!(encoded_5_6.len(), expected_5_6, "Rate 5/6 output size");
    }

    #[test]
    fn test_turbo_rate_5_6() {
        // Mode 16-17 uses rate 5/6 for high throughput
        // Use a block size that's divisible by 5 for clean testing
        let block_size = 500;
        let mut encoder = TurboEncoder::new_with_rate(block_size, CodeRate::Rate5_6);
        let mut decoder = TurboDecoder::new_with_rate(block_size, CodeRate::Rate5_6);
        decoder.set_iterations(10);  // More iterations for higher rate

        // Pseudo-random data pattern
        let data: Vec<u8> = (0..block_size).map(|i| ((i * 13 + 7) % 2) as u8).collect();
        let encoded = encoder.encode(&data);

        // Expected output size: block_size + block_size/5 + tail
        let expected_data_bits = block_size + block_size / 5;  // 500 + 100 = 600
        let tail_bits = 16;
        eprintln!("Rate 5/6: {} input bits -> {} encoded bits (expected ~{})",
            block_size, encoded.len(), expected_data_bits + tail_bits);

        // Convert to LLRs (clean channel with moderate confidence)
        let llrs: Vec<Llr> = encoded.iter()
            .map(|&b| if b == 0 { 4.0 } else { -4.0 })
            .collect();

        let decoded = decoder.decode(&llrs);

        // Count errors
        let errors: usize = decoded.iter()
            .zip(data.iter())
            .filter(|(&d, &o)| d != o)
            .count();

        assert_eq!(errors, 0, "Rate 5/6 clean channel should have 0 errors, got {}", errors);
    }

    #[test]
    fn test_turbo_rate_5_6_with_noise() {
        // Test rate 5/6 with some noise - it should still work but may have some errors
        let block_size = 500;
        let mut encoder = TurboEncoder::new_with_rate(block_size, CodeRate::Rate5_6);
        let mut decoder = TurboDecoder::new_with_rate(block_size, CodeRate::Rate5_6);
        decoder.set_iterations(12);  // More iterations for noisy channel

        let data: Vec<u8> = (0..block_size).map(|i| ((i * 17 + 3) % 2) as u8).collect();
        let encoded = encoder.encode(&data);

        // Add deterministic "noise" to LLRs
        let llrs: Vec<Llr> = encoded.iter().enumerate()
            .map(|(i, &b)| {
                let base = if b == 0 { 2.5 } else { -2.5 };
                // Add deterministic "noise"
                base + (((i * 23) % 11) as f32 - 5.0) * 0.2
            })
            .collect();

        let decoded = decoder.decode(&llrs);

        // Count errors - rate 5/6 with noise should still have good performance
        let errors: usize = decoded.iter()
            .zip(data.iter())
            .filter(|(&d, &o)| d != o)
            .count();

        // Rate 5/6 has less error correction, so allow more errors with noise
        assert!(errors < block_size / 5, "Rate 5/6 with noise had too many errors: {}", errors);
    }
}
