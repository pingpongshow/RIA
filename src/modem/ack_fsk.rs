//! 48-FSK ACK modulation
//!
//! Implements dual parallel 48-FSK streams for ACK frames:
//! - 31 symbols per stream @ 37.5 baud
//! - 842ms total duration
//! - Session-unique scrambling via VB6 PRNG

use std::f32::consts::PI;
use crate::dsp::{ComplexSample, Fft, wrap_phase};

/// ACK frame types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum AckType {
    /// First ACK after CONNECT
    Ack0 = 0,
    /// Data received OK, continue
    Ack1 = 1,
    /// Increase speed level
    SpeedUp = 2,
    /// Decrease speed level
    SpeedDown = 3,
    /// Request retransmission
    Nack = 4,
    /// End of transmission turn
    Break = 5,
    /// Keep-alive
    Idle = 6,
    /// Reserved
    Reserved = 7,
}

impl From<u8> for AckType {
    fn from(v: u8) -> Self {
        match v {
            0 => AckType::Ack0,
            1 => AckType::Ack1,
            2 => AckType::SpeedUp,
            3 => AckType::SpeedDown,
            4 => AckType::Nack,
            5 => AckType::Break,
            6 => AckType::Idle,
            _ => AckType::Reserved,
        }
    }
}

/// Linear Congruential Generator (PRNG)
/// Used for session-unique ACK scrambling
#[derive(Debug, Clone)]
pub struct Vb6Prng {
    state: u32,
}

impl Vb6Prng {
    /// Create new PRNG with seed
    pub fn new(seed: u32) -> Self {
        let mut prng = Self { state: 0 };
        prng.randomize(seed);
        prng
    }

    /// Seed the PRNG (VB6 Randomize function)
    pub fn randomize(&mut self, seed: u32) {
        // VB6 uses lower 24 bits of seed
        self.state = seed & 0xFFFFFF;
    }

    /// Get next random value 0.0 to <1.0 (VB6 Rnd function)
    pub fn rnd(&mut self) -> f32 {
        // VB6 LCG: state = (state * 0x43FD43FD + 0xC39EC3) AND 0xFFFFFF
        self.state = ((self.state as u64 * 0x43FD43FD + 0xC39EC3) & 0xFFFFFF) as u32;
        self.state as f32 / 16777216.0
    }

    /// Get random byte 0-255
    pub fn rnd_byte(&mut self) -> u8 {
        (self.rnd() * 256.0) as u8
    }
}

/// ACK base patterns
/// Format: interleaved [stream1, stream2, stream1, stream2, ...] for first 8 pairs
const ACK_BASE_PATTERNS: [[u8; 16]; 8] = [
    [7, 10, 5, 30, 7, 24, 19, 2, 25, 22, 27, 14, 23, 26, 19, 4],     // ACK0
    [25, 8, 25, 26, 11, 6, 27, 16, 1, 32, 31, 26, 9, 6, 33, 16],     // ACK1
    [21, 20, 11, 28, 25, 12, 5, 22, 29, 16, 23, 12, 31, 28, 7, 20],  // SpeedUp
    [3, 28, 25, 16, 5, 0, 17, 30, 13, 10, 1, 8, 21, 32, 7, 10],      // SpeedDown
    [21, 16, 3, 12, 23, 0, 1, 18, 9, 18, 7, 22, 15, 4, 13, 22],      // NACK
    [29, 16, 25, 14, 17, 18, 7, 16, 13, 28, 25, 4, 31, 6, 7, 4],     // Break
    [29, 32, 11, 26, 5, 4, 23, 18, 25, 28, 11, 16, 17, 6, 19, 6],    // Idle
    [23, 10, 23, 30, 15, 14, 11, 2, 25, 20, 33, 26, 9, 16, 13, 24],  // Reserved
];

/// Role offset tables for session-unique scrambling
const MASTER_OFFSETS: [[u16; 2]; 2] = [[50, 278], [850, 1078]];
const SLAVE_OFFSETS: [[u16; 2]; 2] = [[278, 50], [1078, 850]];

/// Output amplitude for ACK FSK (per-tone, dual-tone sum gives ~0.2 peak)
const ACK_FSK_AMPLITUDE: f32 = 0.1;

/// 48-FSK ACK modulator
pub struct AckFskModulator {
    sample_rate: f32,
    num_tones: usize,
    symbol_rate: f32,
    samples_per_symbol: usize,
    fft_size: usize,
    bin_spacing: f32,
    base_bin: usize,
    phase: [f32; 2], // Phase for each stream
}

impl AckFskModulator {
    /// Create new ACK FSK modulator
    pub fn new(sample_rate: f32) -> Self {
        let fft_size = 1024;
        // 31 symbols in 842ms at 48000 Hz
        // 842ms = 40416 samples, 40416 / 31 ≈ 1304 samples per symbol
        let samples_per_symbol = ((sample_rate * 0.842) / 31.0).round() as usize;
        let symbol_rate = sample_rate / samples_per_symbol as f32;
        let bin_spacing = sample_rate / fft_size as f32; // 46.875 Hz

        Self {
            sample_rate,
            num_tones: 48,
            symbol_rate,
            samples_per_symbol,
            fft_size,
            bin_spacing,
            base_bin: 17, // Start at bin 17 (~796 Hz)
            phase: [0.0, 0.0],
        }
    }

    /// Get frequency for tone index
    fn tone_frequency(&self, tone: u8) -> f32 {
        let bin = self.base_bin + (tone as usize).min(47);
        bin as f32 * self.bin_spacing
    }

    /// Generate ACK frame samples
    pub fn modulate(&mut self, ack_type: AckType, session_seed: u32, is_master: bool) -> Vec<f32> {
        let symbols = self.generate_symbols(ack_type, session_seed, is_master);
        self.modulate_symbols(&symbols)
    }

    /// Generate scrambled symbol pairs for ACK
    fn generate_symbols(&self, ack_type: AckType, session_seed: u32, is_master: bool) -> Vec<(u8, u8)> {
        let base = &ACK_BASE_PATTERNS[ack_type as usize];
        let offsets = if is_master { &MASTER_OFFSETS } else { &SLAVE_OFFSETS };
        let mut prng = Vb6Prng::new(session_seed);

        let mut symbols = Vec::with_capacity(31);

        // Generate 31 symbol pairs
        for i in 0..31 {
            let base_idx = (i * 2) % 16;
            let base_s1 = base[base_idx];
            let base_s2 = base[base_idx + 1];

            // Apply session-unique scrambling
            let rnd_byte = prng.rnd_byte();
            let set_idx = if prng.rnd() > 0.5 { 1 } else { 0 };
            let offset_idx = if prng.rnd() > 0.5 { 1 } else { 0 };

            let offset = offsets[set_idx][offset_idx];

            // Scramble tones
            let s1 = ((base_s1 as u16 + 10 + rnd_byte as u16 + offset) % 48) as u8;
            let s2 = ((base_s2 as u16 + 10 + rnd_byte as u16 + offset) % 48) as u8;

            symbols.push((s1, s2));
        }

        symbols
    }

    /// Modulate symbol pairs into audio samples
    fn modulate_symbols(&mut self, symbols: &[(u8, u8)]) -> Vec<f32> {
        let total_samples = symbols.len() * self.samples_per_symbol;
        let mut output = Vec::with_capacity(total_samples);

        for &(tone1, tone2) in symbols {
            let freq1 = self.tone_frequency(tone1);
            let freq2 = self.tone_frequency(tone2);

            let phase_inc1 = 2.0 * PI * freq1 / self.sample_rate;
            let phase_inc2 = 2.0 * PI * freq2 / self.sample_rate;

            for _ in 0..self.samples_per_symbol {
                // Sum both tones (dual-stream) with consistent amplitude
                let sample = ACK_FSK_AMPLITUDE * (self.phase[0].sin() + self.phase[1].sin());
                output.push(sample);

                // Update phases using proper modulo arithmetic
                self.phase[0] = wrap_phase(self.phase[0] + phase_inc1);
                self.phase[1] = wrap_phase(self.phase[1] + phase_inc2);
            }
        }

        output
    }

    /// Reset modulator state
    pub fn reset(&mut self) {
        self.phase = [0.0, 0.0];
    }

    /// Get samples per ACK frame
    pub fn samples_per_frame(&self) -> usize {
        31 * self.samples_per_symbol
    }

    /// Get frame duration in milliseconds
    pub fn frame_duration_ms(&self) -> f32 {
        31.0 * 1000.0 / self.symbol_rate // ~842ms
    }
}

/// 48-FSK ACK demodulator
pub struct AckFskDemodulator {
    sample_rate: f32,
    num_tones: usize,
    symbol_rate: f32,
    samples_per_symbol: usize,
    fft_size: usize,
    bin_spacing: f32,
    base_bin: usize,
    fft: Fft,
    tone_bins: Vec<usize>,
}

impl AckFskDemodulator {
    /// Create new ACK FSK demodulator
    pub fn new(sample_rate: f32) -> Self {
        let fft_size = 1024;
        let symbol_rate = 37.5;
        let samples_per_symbol = (sample_rate / symbol_rate) as usize;
        let bin_spacing = sample_rate / fft_size as f32;
        let base_bin = 17;

        let fft = Fft::new(samples_per_symbol);

        // Pre-calculate FFT bins for all 48 tones
        let tone_bins: Vec<usize> = (0..48)
            .map(|t| {
                let freq = (base_bin + t) as f32 * bin_spacing;
                ((freq * samples_per_symbol as f32 / sample_rate).round() as usize)
                    .min(samples_per_symbol / 2 - 1)
            })
            .collect();

        Self {
            sample_rate,
            num_tones: 48,
            symbol_rate,
            samples_per_symbol,
            fft_size,
            bin_spacing,
            base_bin,
            fft,
            tone_bins,
        }
    }

    /// Demodulate ACK frame samples
    pub fn demodulate(&mut self, samples: &[f32], session_seed: u32, is_master: bool) -> Option<AckType> {
        if samples.len() < 31 * self.samples_per_symbol {
            return None;
        }

        // Demodulate all symbol pairs
        let mut detected_pairs = Vec::with_capacity(31);
        for i in 0..31 {
            let start = i * self.samples_per_symbol;
            let (t1, t2, confidence) = self.demodulate_symbol(&samples[start..]);
            if confidence < 0.3 {
                return None; // Poor signal
            }
            detected_pairs.push((t1, t2));
        }

        // Try to match against each ACK type
        self.match_ack_type(&detected_pairs, session_seed, is_master)
    }

    /// Demodulate single symbol, returns (tone1, tone2, confidence)
    fn demodulate_symbol(&mut self, samples: &[f32]) -> (u8, u8, f32) {
        let complex: Vec<ComplexSample> = samples[..self.samples_per_symbol]
            .iter()
            .map(|&s| ComplexSample::new(s, 0.0))
            .collect();

        let spectrum = self.fft.forward_new(&complex);

        // Get magnitude for each tone
        let mut magnitudes: Vec<(usize, f32)> = self.tone_bins
            .iter()
            .enumerate()
            .map(|(tone, &bin)| {
                let mag = if bin < spectrum.len() {
                    spectrum[bin].norm()
                } else {
                    0.0
                };
                (tone, mag)
            })
            .collect();

        // Sort by magnitude descending (NaN-safe comparison)
        magnitudes.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));

        // Top two tones should be from different streams
        let tone1 = magnitudes[0].0 as u8;
        let tone2 = magnitudes[1].0 as u8;
        let total_mag = magnitudes[0].1 + magnitudes[1].1;
        let noise_floor: f32 = magnitudes[4..].iter().map(|(_, m)| m).sum::<f32>() / (magnitudes.len() - 4) as f32;

        let confidence = if noise_floor > 0.0 {
            (total_mag / (2.0 * noise_floor + 0.001)).min(1.0)
        } else {
            1.0
        };

        (tone1, tone2, confidence)
    }

    /// Match detected symbols against expected ACK patterns
    fn match_ack_type(&self, detected: &[(u8, u8)], session_seed: u32, is_master: bool) -> Option<AckType> {
        let offsets = if is_master { &SLAVE_OFFSETS } else { &MASTER_OFFSETS }; // Receiver uses opposite role
        let prng = Vb6Prng::new(session_seed);

        let mut best_type = AckType::Ack0;
        let mut best_matches = 0;

        for ack_idx in 0..8 {
            let base = &ACK_BASE_PATTERNS[ack_idx];
            let mut prng_copy = prng.clone();
            let mut matches = 0;

            for i in 0..31 {
                let base_idx = (i * 2) % 16;
                let base_s1 = base[base_idx];
                let base_s2 = base[base_idx + 1];

                let rnd_byte = prng_copy.rnd_byte();
                let set_idx = if prng_copy.rnd() > 0.5 { 1 } else { 0 };
                let offset_idx = if prng_copy.rnd() > 0.5 { 1 } else { 0 };
                let offset = offsets[set_idx][offset_idx];

                let exp_s1 = ((base_s1 as u16 + 10 + rnd_byte as u16 + offset) % 48) as u8;
                let exp_s2 = ((base_s2 as u16 + 10 + rnd_byte as u16 + offset) % 48) as u8;

                let (det_s1, det_s2) = detected[i];

                // Allow tones in either order (dual stream)
                if (det_s1 == exp_s1 && det_s2 == exp_s2) || (det_s1 == exp_s2 && det_s2 == exp_s1) {
                    matches += 2;
                } else if det_s1 == exp_s1 || det_s1 == exp_s2 || det_s2 == exp_s1 || det_s2 == exp_s2 {
                    matches += 1;
                }
            }

            if matches > best_matches {
                best_matches = matches;
                best_type = AckType::from(ack_idx as u8);
            }
        }

        // Require at least 70% match
        if best_matches >= 43 { // 31 symbols * 2 tones * 0.7
            Some(best_type)
        } else {
            None
        }
    }

    /// Get samples needed per frame
    pub fn samples_per_frame(&self) -> usize {
        31 * self.samples_per_symbol
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_vb6_prng() {
        let mut prng = Vb6Prng::new(12345);
        let v1 = prng.rnd();
        let v2 = prng.rnd();

        assert!(v1 >= 0.0 && v1 < 1.0);
        assert!(v2 >= 0.0 && v2 < 1.0);
        assert_ne!(v1, v2);
    }

    #[test]
    fn test_ack_modulator_frame_length() {
        let mod_ = AckFskModulator::new(48000.0);
        let duration = mod_.frame_duration_ms();

        // Should be approximately 842ms
        assert!((duration - 842.0).abs() < 10.0);
    }

    #[test]
    fn test_ack_round_trip() {
        let mut modulator = AckFskModulator::new(48000.0);
        let mut demodulator = AckFskDemodulator::new(48000.0);

        let session_seed = 0x12345678;

        for ack_type in [AckType::Ack0, AckType::Ack1, AckType::Nack, AckType::Break] {
            modulator.reset();
            let samples = modulator.modulate(ack_type, session_seed, true);
            let detected = demodulator.demodulate(&samples, session_seed, false);

            assert_eq!(detected, Some(ack_type), "Failed for {:?}", ack_type);
        }
    }

    #[test]
    fn test_ack_types() {
        assert_eq!(AckType::from(0), AckType::Ack0);
        assert_eq!(AckType::from(4), AckType::Nack);
        assert_eq!(AckType::from(99), AckType::Reserved);
    }
}
