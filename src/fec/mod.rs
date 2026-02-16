//! Forward Error Correction (FEC) module
//!
//! Provides turbo coding, convolutional coding, interleaving, and compression
//! for reliable data transmission over noisy channels.

mod convolutional;
mod turbo;
mod interleaver;
mod huffman;
pub mod crc;

#[allow(unused_imports)]
pub use convolutional::{ConvolutionalEncoder, ViterbiDecoder, K7ConvolutionalEncoder, K7ViterbiDecoder};
pub use turbo::{TurboEncoder, TurboDecoder};
#[allow(unused_imports)]
pub use interleaver::{Interleaver, InterleaverType};
#[allow(unused_imports)]
pub use crc::{Crc16, Crc32, crc16, crc32, callsign_hash, session_seed, generate_random_seed};
#[allow(unused_imports)]
pub use huffman::{HuffmanEncoder, HuffmanDecoder, HuffmanError, compression_ratio};

/// FEC code rate
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CodeRate {
    Rate1_2,  // 1/2
    Rate1_3,  // 1/3
    Rate2_3,  // 2/3
    Rate3_4,  // 3/4
    Rate5_6,  // 5/6 (higher throughput, less error correction)
}

impl CodeRate {
    pub fn ratio(&self) -> f32 {
        match self {
            CodeRate::Rate1_2 => 0.5,
            CodeRate::Rate1_3 => 1.0 / 3.0,
            CodeRate::Rate2_3 => 2.0 / 3.0,
            CodeRate::Rate3_4 => 0.75,
            CodeRate::Rate5_6 => 5.0 / 6.0,
        }
    }

    /// Calculate output bits for given input bits
    pub fn output_bits(&self, input_bits: usize) -> usize {
        match self {
            CodeRate::Rate1_2 => input_bits * 2,
            CodeRate::Rate1_3 => input_bits * 3,
            CodeRate::Rate2_3 => (input_bits * 3 + 1) / 2,
            CodeRate::Rate3_4 => (input_bits * 4 + 2) / 3,
            CodeRate::Rate5_6 => (input_bits * 6 + 4) / 5,
        }
    }

    /// Get code rate for a given speed level
    /// Modes 1-10: rate 1/2
    /// Mode 11: rate 2/3
    /// Mode 12: rate 3/4
    /// Mode 13: rate 2/3 (8PSK)
    /// Mode 14-15: rate 3/4
    /// Modes 16-17: rate 5/6 (32QAM high throughput)
    pub fn for_mode(level: u8) -> CodeRate {
        match level {
            1..=10 => CodeRate::Rate1_2,
            11 => CodeRate::Rate2_3,
            12 => CodeRate::Rate3_4,
            13 => CodeRate::Rate2_3,    // 8PSK 2/3
            14 => CodeRate::Rate3_4,    // 8PSK 3/4
            15 => CodeRate::Rate3_4,    // 16QAM 3/4
            16..=17 => CodeRate::Rate5_6, // 32QAM 5/6
            _ => CodeRate::Rate1_2,
        }
    }
}

/// Soft decision log-likelihood ratio
pub type Llr = f32;

/// Convert hard bits to soft LLRs
pub fn bits_to_llr(bits: &[u8], snr_db: f32) -> Vec<Llr> {
    let amplitude = 10.0_f32.powf(snr_db / 20.0);
    bits.iter()
        .map(|&b| if b == 0 { amplitude } else { -amplitude })
        .collect()
}

/// Convert soft LLRs to hard bits
pub fn llr_to_bits(llrs: &[Llr]) -> Vec<u8> {
    llrs.iter().map(|&l| if l > 0.0 { 0 } else { 1 }).collect()
}
