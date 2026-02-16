//! MFSK modulation for headers and control frames
//!
//! Implements 16-tone MFSK for robust header transmission

use std::f32::consts::PI;
use crate::dsp::{ComplexSample, Fft, wrap_phase};

/// Output amplitude for MFSK (matches OFDM data levels)
const MFSK_AMPLITUDE: f32 = 0.2;

/// MFSK modulator for header transmission
pub struct MfskModulator {
    num_tones: usize,
    tone_spacing: f32,
    center_freq: f32,
    sample_rate: f32,
    symbol_duration: f32,
    samples_per_symbol: usize,
    phase: f32,
}

impl MfskModulator {
    /// Create 16-tone MFSK modulator
    pub fn new(sample_rate: f32, center_freq: f32) -> Self {
        Self::with_tones(16, sample_rate, center_freq, 46.875)
    }

    /// Create MFSK modulator with custom parameters
    pub fn with_tones(num_tones: usize, sample_rate: f32, center_freq: f32, tone_spacing: f32) -> Self {
        let symbol_duration = 1.0 / tone_spacing;
        // Validate and clamp to prevent invalid samples_per_symbol
        let samples_per_symbol = (sample_rate * symbol_duration).max(1.0) as usize;

        Self {
            num_tones,
            tone_spacing,
            center_freq,
            sample_rate,
            symbol_duration,
            samples_per_symbol,
            phase: 0.0,
        }
    }

    /// Get frequency for tone index
    /// Tones are placed exactly on FFT bin centers for optimal detection
    fn tone_frequency(&self, tone: usize) -> f32 {
        // Calculate the center bin (integer)
        let center_bin = (self.center_freq * self.samples_per_symbol as f32 / self.sample_rate).round();
        // Calculate offset from center (tones span from -half to +half)
        let half_tones = self.num_tones as f32 / 2.0;
        let tone_offset = tone as f32 - half_tones + 0.5;
        // Round to integer bin, then convert to frequency
        let target_bin = (center_bin + tone_offset).round();
        target_bin * self.sample_rate / self.samples_per_symbol as f32
    }

    /// Modulate a single symbol (tone index)
    pub fn modulate_symbol(&mut self, tone: usize) -> Vec<f32> {
        let freq = self.tone_frequency(tone);
        let phase_inc = 2.0 * PI * freq / self.sample_rate;

        let mut samples = Vec::with_capacity(self.samples_per_symbol);

        for _ in 0..self.samples_per_symbol {
            samples.push(MFSK_AMPLITUDE * self.phase.sin());
            self.phase = wrap_phase(self.phase + phase_inc);
        }

        samples
    }

    /// Modulate a sequence of symbols
    pub fn modulate(&mut self, symbols: &[u8]) -> Vec<f32> {
        let mut output = Vec::with_capacity(symbols.len() * self.samples_per_symbol);

        for &symbol in symbols {
            let tone = (symbol as usize) % self.num_tones;
            output.extend(self.modulate_symbol(tone));
        }

        output
    }

    /// Modulate bits (4 bits per symbol for 16-MFSK, 5 bits for 32-MFSK)
    /// Pads input to multiple of bits_per_symbol with zeros for clean symbol alignment
    pub fn modulate_bits(&mut self, bits: &[u8]) -> Vec<f32> {
        let bits_per_symbol = (self.num_tones as f32).log2() as usize;

        // Pad bits to multiple of bits_per_symbol for clean alignment
        let remainder = bits.len() % bits_per_symbol;
        let padded_bits: Vec<u8> = if remainder == 0 {
            bits.to_vec()
        } else {
            let padding_needed = bits_per_symbol - remainder;
            let mut padded = bits.to_vec();
            padded.extend(std::iter::repeat(0u8).take(padding_needed));
            padded
        };

        let mut symbols = Vec::new();

        for chunk in padded_bits.chunks(bits_per_symbol) {
            let mut symbol = 0u8;
            for (i, &bit) in chunk.iter().enumerate() {
                symbol |= (bit & 1) << (bits_per_symbol - 1 - i);
            }
            symbols.push(symbol);
        }

        self.modulate(&symbols)
    }

    /// Reset phase
    pub fn reset(&mut self) {
        self.phase = 0.0;
    }

    /// Get samples per symbol
    pub fn samples_per_symbol(&self) -> usize {
        self.samples_per_symbol
    }

    /// Get number of tones
    pub fn num_tones(&self) -> usize {
        self.num_tones
    }
}

/// MFSK demodulator
pub struct MfskDemodulator {
    num_tones: usize,
    tone_spacing: f32,
    center_freq: f32,
    sample_rate: f32,
    samples_per_symbol: usize,
    fft: Fft,
    tone_bins: Vec<usize>,
}

impl MfskDemodulator {
    /// Create 16-tone MFSK demodulator
    pub fn new(sample_rate: f32, center_freq: f32) -> Self {
        Self::with_tones(16, sample_rate, center_freq, 46.875)
    }

    /// Create MFSK demodulator with custom parameters
    pub fn with_tones(num_tones: usize, sample_rate: f32, center_freq: f32, tone_spacing: f32) -> Self {
        let symbol_duration = 1.0 / tone_spacing;
        let samples_per_symbol = (sample_rate * symbol_duration) as usize;

        // Use FFT size equal to samples per symbol
        let fft = Fft::new(samples_per_symbol);

        // Calculate FFT bin for each tone - must match modulator's tone_frequency()
        let mut tone_bins = Vec::with_capacity(num_tones);
        let center_bin = (center_freq * samples_per_symbol as f32 / sample_rate).round();
        let half_tones = num_tones as f32 / 2.0;
        for tone in 0..num_tones {
            let tone_offset = tone as f32 - half_tones + 0.5;
            let bin = (center_bin + tone_offset).round() as usize;
            tone_bins.push(bin.min(samples_per_symbol / 2));
        }

        Self {
            num_tones,
            tone_spacing,
            center_freq,
            sample_rate,
            samples_per_symbol,
            fft,
            tone_bins,
        }
    }

    /// Demodulate a single symbol
    ///
    /// Returns (0, 0.0) if samples is too short.
    pub fn demodulate_symbol(&mut self, samples: &[f32]) -> (u8, f32) {
        if samples.len() < self.samples_per_symbol {
            log::warn!("MFSK demodulate: samples too short ({} < {})",
                       samples.len(), self.samples_per_symbol);
            return (0, 0.0);
        }

        // Convert to complex and perform FFT
        let complex: Vec<ComplexSample> = samples[..self.samples_per_symbol]
            .iter()
            .map(|&s| ComplexSample::new(s, 0.0))
            .collect();

        let spectrum = self.fft.forward_new(&complex);

        // Find tone with maximum magnitude
        let mut max_mag = 0.0f32;
        let mut max_tone = 0usize;

        for (tone, &bin) in self.tone_bins.iter().enumerate() {
            if bin < spectrum.len() {
                let mag = spectrum[bin].norm();
                if mag > max_mag {
                    max_mag = mag;
                    max_tone = tone;
                }
            }
        }

        (max_tone as u8, max_mag)
    }

    /// Demodulate a sequence of symbols
    pub fn demodulate(&mut self, samples: &[f32]) -> Vec<u8> {
        let num_symbols = samples.len() / self.samples_per_symbol;
        let mut symbols = Vec::with_capacity(num_symbols);

        for i in 0..num_symbols {
            let start = i * self.samples_per_symbol;
            let (symbol, _) = self.demodulate_symbol(&samples[start..]);
            symbols.push(symbol);
        }

        symbols
    }

    /// Demodulate to bits
    pub fn demodulate_bits(&mut self, samples: &[f32]) -> Vec<u8> {
        let symbols = self.demodulate(samples);
        let bits_per_symbol = (self.num_tones as f32).log2() as usize;

        let mut bits = Vec::with_capacity(symbols.len() * bits_per_symbol);
        for symbol in symbols {
            for i in (0..bits_per_symbol).rev() {
                bits.push((symbol >> i) & 1);
            }
        }

        bits
    }

    /// Demodulate with soft outputs (magnitude for each tone)
    pub fn demodulate_soft(&mut self, samples: &[f32]) -> Vec<Vec<f32>> {
        let num_symbols = samples.len() / self.samples_per_symbol;
        let mut soft_outputs = Vec::with_capacity(num_symbols);

        for i in 0..num_symbols {
            let start = i * self.samples_per_symbol;
            let complex: Vec<ComplexSample> = samples[start..start + self.samples_per_symbol]
                .iter()
                .map(|&s| ComplexSample::new(s, 0.0))
                .collect();

            let spectrum = self.fft.forward_new(&complex);

            let magnitudes: Vec<f32> = self.tone_bins.iter()
                .map(|&bin| if bin < spectrum.len() { spectrum[bin].norm() } else { 0.0 })
                .collect();

            soft_outputs.push(magnitudes);
        }

        soft_outputs
    }

    /// Get samples per symbol
    pub fn samples_per_symbol(&self) -> usize {
        self.samples_per_symbol
    }

    /// Get number of tones
    pub fn num_tones(&self) -> usize {
        self.num_tones
    }

    /// Get symbol rate (tone spacing)
    pub fn symbol_rate(&self) -> f32 {
        self.tone_spacing
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_mfsk_round_trip() {
        let sample_rate = 48000.0;
        let center_freq = 1500.0;

        let mut modulator = MfskModulator::new(sample_rate, center_freq);
        let mut demodulator = MfskDemodulator::new(sample_rate, center_freq);

        // Test each tone
        for tone in 0..16u8 {
            modulator.reset();
            let samples = modulator.modulate(&[tone]);
            let recovered = demodulator.demodulate(&samples);
            assert_eq!(recovered[0], tone, "Failed for tone {}", tone);
        }
    }

    #[test]
    fn test_mfsk_bits() {
        let sample_rate = 48000.0;
        let center_freq = 1500.0;

        let mut modulator = MfskModulator::new(sample_rate, center_freq);
        let mut demodulator = MfskDemodulator::new(sample_rate, center_freq);

        let bits = vec![1, 0, 1, 1, 0, 0, 1, 0]; // Two symbols
        let samples = modulator.modulate_bits(&bits);
        let recovered = demodulator.demodulate_bits(&samples);

        assert_eq!(recovered, bits);
    }
}
