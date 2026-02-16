//! STANAG 4197-style 39-tone DQPSK modulation
//!
//! Implements a waveform inspired by NATO STANAG 4197 and MIL-STD-188-110 Appendix B.
//! Uses 39 orthogonal DQPSK-modulated subcarriers for robust HF transmission.
//!
//! Key parameters:
//! - 39 tones from 675 Hz to 2812.5 Hz (56.25 Hz spacing)
//! - Pilot tone at 393.75 Hz for Doppler tracking
//! - Symbol duration: 22.5 ms (1080 samples at 48 kHz)
//! - Cyclic prefix: ~4.7 ms (227 samples)
//! - DQPSK modulation (2 bits per symbol per tone)

use std::f32::consts::PI;
use crate::dsp::ComplexSample;
use crate::protocol::clamp_mode_for_bandwidth;
use crate::Bandwidth as ConfigBandwidth;
use super::{Symbol, Constellation, ConstellationType, Bandwidth};
use rustfft::{Fft, FftPlanner};
use std::sync::Arc;

/// Sample rate (Hz)
pub const SAMPLE_RATE: u32 = 48000;

/// Symbol duration in seconds
pub const SYMBOL_DURATION: f32 = 0.0225; // 22.5 ms

/// Samples per symbol (including cyclic prefix)
pub const SAMPLES_PER_SYMBOL: usize = 1080; // 48000 * 0.0225

/// Cyclic prefix samples (guard interval)
pub const CYCLIC_PREFIX_SAMPLES: usize = 227; // ~4.7 ms

/// Active symbol samples (without CP)
pub const ACTIVE_SYMBOL_SAMPLES: usize = SAMPLES_PER_SYMBOL - CYCLIC_PREFIX_SAMPLES; // 853

/// Number of data tones
pub const NUM_TONES: usize = 39;

/// Tone spacing in Hz
pub const TONE_SPACING: f32 = 56.25;

/// First data tone frequency
pub const FIRST_TONE_FREQ: f32 = 675.0;

/// Last data tone frequency
pub const LAST_TONE_FREQ: f32 = 2812.5;

/// Pilot/Doppler reference tone frequency
pub const PILOT_FREQ: f32 = 393.75;

/// Center frequency
pub const CENTER_FREQ: f32 = 1800.0;

/// Get frequency for tone index (0..38)
pub fn tone_frequency(index: usize) -> f32 {
    debug_assert!(index < NUM_TONES, "Tone index out of range");
    FIRST_TONE_FREQ + (index as f32) * TONE_SPACING
}

/// DQPSK phase states (Gray coded)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DqpskSymbol {
    S00 = 0, // 0 degrees
    S01 = 1, // 90 degrees
    S11 = 2, // 180 degrees
    S10 = 3, // 270 degrees
}

impl DqpskSymbol {
    /// Create from 2 bits (Gray coded)
    pub fn from_bits(b1: bool, b0: bool) -> Self {
        match (b1, b0) {
            (false, false) => DqpskSymbol::S00,
            (false, true) => DqpskSymbol::S01,
            (true, true) => DqpskSymbol::S11,
            (true, false) => DqpskSymbol::S10,
        }
    }

    /// Convert to bits (Gray coded)
    pub fn to_bits(&self) -> (bool, bool) {
        match self {
            DqpskSymbol::S00 => (false, false),
            DqpskSymbol::S01 => (false, true),
            DqpskSymbol::S11 => (true, true),
            DqpskSymbol::S10 => (true, false),
        }
    }

    /// Get phase change in radians
    pub fn phase_delta(&self) -> f32 {
        match self {
            DqpskSymbol::S00 => 0.0,
            DqpskSymbol::S01 => PI / 2.0,
            DqpskSymbol::S11 => PI,
            DqpskSymbol::S10 => 3.0 * PI / 2.0,
        }
    }

    /// Create from phase difference (nearest symbol)
    pub fn from_phase_delta(delta: f32) -> Self {
        // Normalize to 0..2pi
        let mut d = delta % (2.0 * PI);
        if d < 0.0 {
            d += 2.0 * PI;
        }

        // Quantize to nearest symbol
        if d < PI / 4.0 || d >= 7.0 * PI / 4.0 {
            DqpskSymbol::S00
        } else if d < 3.0 * PI / 4.0 {
            DqpskSymbol::S01
        } else if d < 5.0 * PI / 4.0 {
            DqpskSymbol::S11
        } else {
            DqpskSymbol::S10
        }
    }
}

/// STANAG configuration
#[derive(Debug, Clone)]
pub struct StanagConfig {
    /// Number of tones to use (up to 39)
    pub num_tones: usize,
    /// Number of carriers (alias for num_tones for OFDM compatibility)
    pub num_carriers: usize,
    /// FFT size for demodulation (ACTIVE_SYMBOL_SAMPLES)
    pub fft_size: usize,
    /// Cyclic prefix length
    pub cp_length: usize,
    /// Sample rate
    pub sample_rate: f32,
    /// Pilot tone enabled
    pub pilot_enabled: bool,
    /// Pilot amplitude relative to data tones
    pub pilot_amplitude: f32,
    /// Output amplitude scaling
    pub output_scale: f32,
}

impl StanagConfig {
    /// Create configuration for given bandwidth
    pub fn for_bandwidth(bandwidth: Bandwidth, sample_rate: f32) -> Self {
        let num_tones = match bandwidth {
            Bandwidth::Narrow => 11,   // Reduced tones for 500 Hz
            Bandwidth::Wide => 39,     // Full 39 tones for 2300 Hz
            Bandwidth::Ultra => 39,    // Full 39 tones for 2750 Hz
        };

        Self {
            num_tones,
            num_carriers: num_tones, // Alias for OFDM compatibility
            fft_size: ACTIVE_SYMBOL_SAMPLES,
            cp_length: CYCLIC_PREFIX_SAMPLES,
            sample_rate,
            pilot_enabled: true,
            pilot_amplitude: 0.5,
            output_scale: 0.2,
        }
    }

    /// Create configuration for given speed level
    pub fn for_speed_level(speed_level: u8, bandwidth: Bandwidth, sample_rate: f32) -> Self {
        // Convert modem Bandwidth to config Bandwidth
        let config_bw = match bandwidth {
            Bandwidth::Narrow => ConfigBandwidth::Hz500,
            Bandwidth::Wide => ConfigBandwidth::Hz2300,
            Bandwidth::Ultra => ConfigBandwidth::Hz2750,
        };

        // Clamp speed level to valid range
        let _clamped_level = clamp_mode_for_bandwidth(speed_level, config_bw);

        // For STANAG, tone count varies by bandwidth, not speed level
        Self::for_bandwidth(bandwidth, sample_rate)
    }

    /// Get symbol duration in seconds
    pub fn symbol_duration(&self) -> f32 {
        SYMBOL_DURATION
    }

    /// Get bits per symbol (2 bits per tone with DQPSK)
    pub fn bits_per_symbol(&self) -> usize {
        self.num_tones * 2
    }

    /// Get samples per symbol
    pub fn samples_per_symbol(&self) -> usize {
        SAMPLES_PER_SYMBOL
    }
}

/// STANAG 39-tone modulator
pub struct StanagModulator {
    config: StanagConfig,
    /// Current phases for each tone (for DQPSK differential encoding)
    current_phases: Vec<f32>,
    /// Pilot oscillator phase
    pilot_phase: f32,
    /// Constellation for soft demodulation compatibility
    constellation: Constellation,
}

impl StanagModulator {
    /// Create new STANAG modulator
    pub fn new(config: StanagConfig, constellation_type: ConstellationType) -> Self {
        let current_phases = vec![0.0; config.num_tones];

        Self {
            config,
            current_phases,
            pilot_phase: 0.0,
            constellation: Constellation::new(constellation_type),
        }
    }

    /// Create modulator for bandwidth with default constellation
    pub fn for_bandwidth(bandwidth: Bandwidth, sample_rate: f32) -> Self {
        let config = StanagConfig::for_bandwidth(bandwidth, sample_rate);
        Self::new(config, ConstellationType::Qpsk) // DQPSK is similar to QPSK
    }

    /// Set constellation type
    pub fn set_constellation(&mut self, constellation_type: ConstellationType) {
        self.constellation = Constellation::new(constellation_type);
    }

    /// Modulate bits to STANAG symbol
    ///
    /// Takes bits (2 per tone) and produces audio samples
    pub fn modulate_symbol(&mut self, bits: &[u8]) -> Vec<f32> {
        let bits_per_symbol = self.config.bits_per_symbol();

        if bits.len() < bits_per_symbol {
            log::debug!("STANAG modulate: not enough bits ({} < {}), padding with zeros",
                       bits.len(), bits_per_symbol);
        }

        // Convert bits to DQPSK symbols
        let mut dqpsk_symbols = Vec::with_capacity(self.config.num_tones);
        for i in 0..self.config.num_tones {
            let bit_idx = i * 2;
            let b1 = if bit_idx < bits.len() { bits[bit_idx] != 0 } else { false };
            let b0 = if bit_idx + 1 < bits.len() { bits[bit_idx + 1] != 0 } else { false };
            dqpsk_symbols.push(DqpskSymbol::from_bits(b1, b0));
        }

        // Apply differential phase shifts
        for (i, &sym) in dqpsk_symbols.iter().enumerate() {
            self.current_phases[i] += sym.phase_delta();
            // Keep phase in range
            while self.current_phases[i] >= 2.0 * PI {
                self.current_phases[i] -= 2.0 * PI;
            }
        }

        // Generate active symbol samples
        let mut active_symbol = vec![0.0f32; ACTIVE_SYMBOL_SAMPLES];
        let sample_rate = self.config.sample_rate;

        for n in 0..ACTIVE_SYMBOL_SAMPLES {
            let t = n as f32 / sample_rate;
            let mut sample = 0.0f32;

            // Sum all tones
            for i in 0..self.config.num_tones {
                let freq = tone_frequency(i);
                let phase = 2.0 * PI * freq * t + self.current_phases[i];
                sample += phase.cos();
            }

            // Add pilot tone if enabled
            if self.config.pilot_enabled {
                let pilot_phase = 2.0 * PI * PILOT_FREQ * t + self.pilot_phase;
                sample += self.config.pilot_amplitude * pilot_phase.cos();
            }

            // Normalize
            let norm_factor = if self.config.pilot_enabled {
                self.config.num_tones as f32 + self.config.pilot_amplitude
            } else {
                self.config.num_tones as f32
            };
            active_symbol[n] = (sample / norm_factor) * self.config.output_scale;
        }

        // Build output with cyclic prefix
        let mut output = Vec::with_capacity(SAMPLES_PER_SYMBOL);

        // Copy cyclic prefix (end of symbol to beginning)
        let cp_start = ACTIVE_SYMBOL_SAMPLES - CYCLIC_PREFIX_SAMPLES;
        for i in 0..CYCLIC_PREFIX_SAMPLES {
            output.push(active_symbol[cp_start + i]);
        }

        // Copy active symbol after prefix
        output.extend_from_slice(&active_symbol);

        // Update pilot phase for next symbol
        self.pilot_phase += 2.0 * PI * PILOT_FREQ * SYMBOL_DURATION;
        while self.pilot_phase >= 2.0 * PI {
            self.pilot_phase -= 2.0 * PI;
        }

        output
    }

    /// Modulate multiple symbols
    pub fn modulate(&mut self, bits: &[u8]) -> Vec<f32> {
        let bits_per_symbol = self.bits_per_symbol();
        let num_symbols = (bits.len() + bits_per_symbol - 1) / bits_per_symbol;

        let mut output = Vec::new();

        for i in 0..num_symbols {
            let start = i * bits_per_symbol;
            let end = (start + bits_per_symbol).min(bits.len());

            // Pad if necessary
            let mut symbol_bits = bits[start..end].to_vec();
            symbol_bits.resize(bits_per_symbol, 0);

            output.extend(self.modulate_symbol(&symbol_bits));
        }

        output
    }

    /// Get bits per STANAG symbol
    pub fn bits_per_symbol(&self) -> usize {
        self.config.bits_per_symbol()
    }

    /// Get samples per STANAG symbol
    pub fn samples_per_symbol(&self) -> usize {
        SAMPLES_PER_SYMBOL
    }

    /// Reset modulator state
    pub fn reset(&mut self) {
        self.current_phases.fill(0.0);
        self.pilot_phase = 0.0;
    }
}

/// STANAG 39-tone demodulator
pub struct StanagDemodulator {
    config: StanagConfig,
    /// FFT processor
    fft: Arc<dyn Fft<f32>>,
    /// FFT size
    fft_size: usize,
    /// Bin indices for each tone
    tone_bins: Vec<usize>,
    /// Previous symbol phases for differential detection
    prev_phases: Vec<f32>,
    /// Pilot bin index
    pilot_bin: usize,
    /// Constellation for soft demodulation
    constellation: Constellation,
    /// Last demodulated constellation points (for display)
    last_constellation_points: Vec<(f32, f32)>,
}

impl StanagDemodulator {
    /// Create new STANAG demodulator
    pub fn new(config: StanagConfig, constellation_type: ConstellationType) -> Self {
        let mut planner = FftPlanner::new();
        let fft_size = ACTIVE_SYMBOL_SAMPLES;
        let fft = planner.plan_fft_forward(fft_size);

        // Calculate FFT bin for each tone
        let bin_spacing = config.sample_rate / fft_size as f32;
        let mut tone_bins = Vec::with_capacity(config.num_tones);
        for i in 0..config.num_tones {
            let freq = tone_frequency(i);
            tone_bins.push((freq / bin_spacing).round() as usize);
        }

        // Pilot bin
        let pilot_bin = (PILOT_FREQ / bin_spacing).round() as usize;

        let prev_phases = vec![0.0; config.num_tones];

        Self {
            config,
            fft,
            fft_size,
            tone_bins,
            prev_phases,
            pilot_bin,
            constellation: Constellation::new(constellation_type),
            last_constellation_points: Vec::new(),
        }
    }

    /// Create demodulator for bandwidth
    pub fn for_bandwidth(bandwidth: Bandwidth, sample_rate: f32) -> Self {
        let config = StanagConfig::for_bandwidth(bandwidth, sample_rate);
        Self::new(config, ConstellationType::Qpsk)
    }

    /// Set constellation type
    pub fn set_constellation(&mut self, constellation_type: ConstellationType) {
        self.constellation = Constellation::new(constellation_type);
    }

    /// Demodulate STANAG symbol to bits
    pub fn demodulate_symbol(&mut self, samples: &[f32]) -> Vec<u8> {
        if samples.len() < SAMPLES_PER_SYMBOL {
            log::warn!("STANAG demodulate: samples too short ({} < {})",
                       samples.len(), SAMPLES_PER_SYMBOL);
            return Vec::new();
        }

        // Skip cyclic prefix and take active symbol
        let active_start = CYCLIC_PREFIX_SAMPLES;
        let active_samples = &samples[active_start..active_start + ACTIVE_SYMBOL_SAMPLES];

        // Perform FFT
        let mut fft_input: Vec<rustfft::num_complex::Complex<f32>> = active_samples
            .iter()
            .map(|&s| rustfft::num_complex::Complex::new(s, 0.0))
            .collect();

        // Zero-pad if needed
        fft_input.resize(self.fft_size, rustfft::num_complex::Complex::new(0.0, 0.0));

        self.fft.process(&mut fft_input);

        // Extract DQPSK symbols using differential detection
        let mut bits = Vec::with_capacity(self.config.num_tones * 2);
        self.last_constellation_points.clear();

        for i in 0..self.config.num_tones {
            let bin = self.tone_bins[i];
            if bin < fft_input.len() {
                let current_phase = fft_input[bin].arg();
                let phase_diff = current_phase - self.prev_phases[i];

                // Decode DQPSK symbol from phase difference
                let symbol = DqpskSymbol::from_phase_delta(phase_diff);
                let (b1, b0) = symbol.to_bits();
                bits.push(if b1 { 1 } else { 0 });
                bits.push(if b0 { 1 } else { 0 });

                // Store constellation point for display
                let cos_phase = phase_diff.cos();
                let sin_phase = phase_diff.sin();
                self.last_constellation_points.push((cos_phase, sin_phase));

                // Store phase for next symbol
                self.prev_phases[i] = current_phase;
            } else {
                bits.push(0);
                bits.push(0);
            }
        }

        bits
    }

    /// Demodulate symbol with soft outputs (LLRs)
    pub fn demodulate_symbol_soft(&mut self, samples: &[f32], noise_var: f32) -> Vec<f32> {
        if samples.len() < SAMPLES_PER_SYMBOL {
            log::warn!("STANAG demodulate_soft: samples too short ({} < {})",
                       samples.len(), SAMPLES_PER_SYMBOL);
            return Vec::new();
        }

        // Skip cyclic prefix and take active symbol
        let active_start = CYCLIC_PREFIX_SAMPLES;
        let active_samples = &samples[active_start..active_start + ACTIVE_SYMBOL_SAMPLES];

        // Perform FFT
        let mut fft_input: Vec<rustfft::num_complex::Complex<f32>> = active_samples
            .iter()
            .map(|&s| rustfft::num_complex::Complex::new(s, 0.0))
            .collect();

        fft_input.resize(self.fft_size, rustfft::num_complex::Complex::new(0.0, 0.0));
        self.fft.process(&mut fft_input);

        let mut llrs = Vec::with_capacity(self.config.num_tones * 2);
        self.last_constellation_points.clear();

        // Clamp noise variance
        let noise_var = noise_var.clamp(0.01, 10.0);

        for i in 0..self.config.num_tones {
            let bin = self.tone_bins[i];
            if bin < fft_input.len() {
                let current_phase = fft_input[bin].arg();
                let phase_diff = current_phase - self.prev_phases[i];

                // Convert phase difference to I/Q point on unit circle
                let cos_phase = phase_diff.cos();
                let sin_phase = phase_diff.sin();

                // Store constellation point
                self.last_constellation_points.push((cos_phase, sin_phase));

                // DQPSK soft demapping with points on axes:
                // 00 → (1, 0), 01 → (0, 1), 11 → (-1, 0), 10 → (0, -1)
                //
                // For bit b1 (MSB):
                //   b1=0: points 00(1,0), 01(0,1)  - right side and top
                //   b1=1: points 10(0,-1), 11(-1,0) - left side and bottom
                // For bit b0 (LSB):
                //   b0=0: points 00(1,0), 10(0,-1)
                //   b0=1: points 01(0,1), 11(-1,0)
                //
                // Distance squared to each point:
                let d_00 = (cos_phase - 1.0).powi(2) + sin_phase.powi(2);      // (1, 0)
                let d_01 = cos_phase.powi(2) + (sin_phase - 1.0).powi(2);      // (0, 1)
                let d_11 = (cos_phase + 1.0).powi(2) + sin_phase.powi(2);      // (-1, 0)
                let d_10 = cos_phase.powi(2) + (sin_phase + 1.0).powi(2);      // (0, -1)

                // LLR for b1: min_d(b1=1) - min_d(b1=0)
                let min_d_b1_0 = d_00.min(d_01);  // b1=0: 00 or 01
                let min_d_b1_1 = d_10.min(d_11);  // b1=1: 10 or 11
                let llr_b1 = (min_d_b1_1 - min_d_b1_0) / (2.0 * noise_var);

                // LLR for b0: min_d(b0=1) - min_d(b0=0)
                let min_d_b0_0 = d_00.min(d_10);  // b0=0: 00 or 10
                let min_d_b0_1 = d_01.min(d_11);  // b0=1: 01 or 11
                let llr_b0 = (min_d_b0_1 - min_d_b0_0) / (2.0 * noise_var);

                // Clamp LLRs
                llrs.push(llr_b1.clamp(-100.0, 100.0));
                llrs.push(llr_b0.clamp(-100.0, 100.0));

                // Store phase for next symbol
                self.prev_phases[i] = current_phase;
            } else {
                llrs.push(0.0);
                llrs.push(0.0);
            }
        }

        llrs
    }

    /// Demodulate multiple symbols
    pub fn demodulate(&mut self, samples: &[f32]) -> Vec<u8> {
        let num_symbols = samples.len() / SAMPLES_PER_SYMBOL;

        let mut bits = Vec::new();

        for i in 0..num_symbols {
            let start = i * SAMPLES_PER_SYMBOL;
            bits.extend(self.demodulate_symbol(&samples[start..]));
        }

        bits
    }

    /// Get bits per STANAG symbol
    pub fn bits_per_symbol(&self) -> usize {
        self.config.bits_per_symbol()
    }

    /// Get samples per STANAG symbol
    pub fn samples_per_symbol(&self) -> usize {
        SAMPLES_PER_SYMBOL
    }

    /// Get last demodulated constellation points (I/Q symbols)
    pub fn constellation_points(&self) -> &[(f32, f32)] {
        &self.last_constellation_points
    }

    /// Reset demodulator state
    pub fn reset(&mut self) {
        self.prev_phases.fill(0.0);
        self.last_constellation_points.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_stanag_config() {
        let config = StanagConfig::for_bandwidth(Bandwidth::Wide, 48000.0);
        assert_eq!(config.num_tones, 39);
        assert_eq!(config.bits_per_symbol(), 78); // 39 tones * 2 bits
    }

    #[test]
    fn test_dqpsk_symbols() {
        // Test Gray coding roundtrip
        for b1 in [false, true] {
            for b0 in [false, true] {
                let sym = DqpskSymbol::from_bits(b1, b0);
                let (rb1, rb0) = sym.to_bits();
                assert_eq!((b1, b0), (rb1, rb0));
            }
        }
    }

    #[test]
    fn test_dqpsk_phase() {
        assert!((DqpskSymbol::S00.phase_delta() - 0.0).abs() < 0.01);
        assert!((DqpskSymbol::S01.phase_delta() - PI / 2.0).abs() < 0.01);
        assert!((DqpskSymbol::S11.phase_delta() - PI).abs() < 0.01);
        assert!((DqpskSymbol::S10.phase_delta() - 3.0 * PI / 2.0).abs() < 0.01);
    }

    #[test]
    fn test_tone_frequencies() {
        assert!((tone_frequency(0) - 675.0).abs() < 0.01);
        assert!((tone_frequency(20) - 1800.0).abs() < 0.01);
        assert!((tone_frequency(38) - 2812.5).abs() < 0.01);
    }

    #[test]
    fn test_stanag_round_trip() {
        let config = StanagConfig::for_bandwidth(Bandwidth::Wide, 48000.0);
        let mut modulator = StanagModulator::new(config.clone(), ConstellationType::Qpsk);
        let mut demodulator = StanagDemodulator::new(config, ConstellationType::Qpsk);

        let bits_per_symbol = modulator.bits_per_symbol();
        let bits: Vec<u8> = (0..bits_per_symbol).map(|i| (i % 2) as u8).collect();

        // First symbol establishes reference phase
        let ref_bits: Vec<u8> = vec![0; bits_per_symbol];
        let ref_samples = modulator.modulate_symbol(&ref_bits);
        let _ = demodulator.demodulate_symbol(&ref_samples);

        // Second symbol carries actual data
        let samples = modulator.modulate_symbol(&bits);
        let recovered = demodulator.demodulate_symbol(&samples);

        // Check most bits match (DQPSK may have some edge effects)
        let matches: usize = bits.iter()
            .zip(recovered.iter())
            .filter(|(&a, &b)| a == b)
            .count();

        assert!(matches > bits_per_symbol * 8 / 10,
                "Too few matching bits: {}/{}", matches, bits_per_symbol);
    }

    #[test]
    fn test_bandwidth_tones() {
        let narrow = StanagConfig::for_bandwidth(Bandwidth::Narrow, 48000.0);
        let wide = StanagConfig::for_bandwidth(Bandwidth::Wide, 48000.0);

        assert_eq!(narrow.num_tones, 11);
        assert_eq!(wide.num_tones, 39);
    }
}
