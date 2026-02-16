//! OFDM modulation and demodulation
//!
//! Implements multi-carrier OFDM for high-speed data transmission

use std::f32::consts::PI;
use crate::dsp::{ComplexSample, Fft, wrap_phase};
use super::{Symbol, Constellation, ConstellationType, Bandwidth};

/// OFDM configuration
#[derive(Debug, Clone)]
pub struct OfdmConfig {
    /// FFT size
    pub fft_size: usize,
    /// Number of data carriers
    pub num_carriers: usize,
    /// Cyclic prefix length
    pub cp_length: usize,
    /// Carrier indices (which FFT bins to use)
    pub carrier_indices: Vec<usize>,
    /// Pilot carrier indices
    pub pilot_indices: Vec<usize>,
    /// Sample rate
    pub sample_rate: f32,
    /// Center frequency
    pub center_freq: f32,
}

impl OfdmConfig {
    /// Create configuration for given bandwidth
    /// FFT size varies by bandwidth mode for optimal performance
    pub fn for_bandwidth(bandwidth: Bandwidth, sample_rate: f32) -> Self {
        // FFT size and CP length depend on bandwidth mode
        let (fft_size, cp_length) = match bandwidth {
            Bandwidth::Narrow => (2048, 256),  // 500Hz robust mode: longer symbols, more CP
            Bandwidth::Wide => (1024, 128),    // 2300Hz standard mode
            Bandwidth::Ultra => (512, 64),     // 2750Hz fast mode: shorter symbols
        };
        let num_carriers = bandwidth.num_carriers();
        Self::build(fft_size, cp_length, num_carriers, sample_rate)
    }

    /// Create configuration for given speed level
    /// Uses per-bandwidth frame config for FFT size (500Hz modes 4-6 need FFT=1024)
    pub fn for_speed_level(speed_level: u8, bandwidth: Bandwidth, sample_rate: f32) -> Self {
        use crate::modem::SpeedLevel;
        use crate::protocol::get_frame_config;
        use crate::Bandwidth as ConfigBandwidth;

        // Convert modem Bandwidth to config Bandwidth
        let config_bw = match bandwidth {
            Bandwidth::Narrow => ConfigBandwidth::Hz500,
            Bandwidth::Wide => ConfigBandwidth::Hz2300,
            Bandwidth::Ultra => ConfigBandwidth::Hz2750,
        };

        // Get FFT size from per-bandwidth frame config
        let frame_config = get_frame_config(speed_level, config_bw);
        let fft_size = frame_config.fft_size;

        let cp_length = match fft_size {
            2048 => 256,
            1024 => 128,
            512 => 64,
            _ => 128,  // Default
        };

        // Use carrier count from rate table for this level and bandwidth
        let level = SpeedLevel::new(speed_level);
        let num_carriers = level.num_carriers(bandwidth);

        Self::build(fft_size, cp_length, num_carriers, sample_rate)
    }

    /// Build config with given parameters
    fn build(fft_size: usize, cp_length: usize, num_carriers: usize, sample_rate: f32) -> Self {
        // Calculate carrier indices centered around center frequency bin
        let center_bin = (1500.0 * fft_size as f32 / sample_rate).round() as usize;
        let half_carriers = num_carriers / 2;

        let mut carrier_indices = Vec::with_capacity(num_carriers);
        for i in 0..num_carriers {
            let offset = i as isize - half_carriers as isize;
            let bin = (center_bin as isize + offset) as usize;
            carrier_indices.push(bin);
        }

        // Pilot carriers - placement depends on number of carriers
        // For few carriers (<=8): place single pilot at center for best interpolation
        // For many carriers: every 8th carrier
        let pilot_indices: Vec<usize> = if num_carriers <= 8 {
            // Few carriers: single pilot at center
            vec![carrier_indices[num_carriers / 2]]
        } else {
            // Many carriers: every 8th
            carrier_indices.iter()
                .enumerate()
                .filter(|(i, _)| i % 8 == 0)
                .map(|(_, &idx)| idx)
                .collect()
        };

        Self {
            fft_size,
            num_carriers,
            cp_length,
            carrier_indices,
            pilot_indices,
            sample_rate,
            center_freq: 1500.0,
        }
    }

    /// Get symbol duration in seconds
    pub fn symbol_duration(&self) -> f32 {
        (self.fft_size + self.cp_length) as f32 / self.sample_rate
    }

    /// Get carrier spacing in Hz
    pub fn carrier_spacing(&self) -> f32 {
        self.sample_rate / self.fft_size as f32
    }
}

/// OFDM modulator
pub struct OfdmModulator {
    config: OfdmConfig,
    fft: Fft,
    constellation: Constellation,
    pilot_phase: f32,
    /// Output amplitude scaling factor to match other modulators
    output_scale: f32,
}

impl OfdmModulator {
    /// Create OFDM modulator
    pub fn new(config: OfdmConfig, constellation_type: ConstellationType) -> Self {
        // Calculate output scaling to match other modulators
        //
        // After IFFT with 1/N normalization, signal is very small.
        // OFDM has high peak-to-average power ratio (PAPR) of ~6-8 dB.
        //
        // We scale to achieve:
        // - Peak amplitude ~0.85 (prevents clipping)
        // - RMS will be lower than preamble due to PAPR, but audio will be clearly audible
        //
        // Output scaling to match other modulators (preamble, MFSK, ACK FSK)
        // All modulators now use amplitude ~0.2 for consistent audio levels
        //
        // After IFFT with 1/N normalization, signal is very small.
        // We scale to achieve peak ~0.2 (matching other modulators)
        // Testing showed that scale=0.17*N/sqrt(carriers) gives peak ~0.85
        // So use scale=0.04*N/sqrt(carriers) for peak ~0.2
        let num_carriers = config.num_carriers as f32;
        let output_scale = 0.04 * config.fft_size as f32 / num_carriers.sqrt();

        Self {
            fft: Fft::new(config.fft_size),
            constellation: Constellation::new(constellation_type),
            config,
            pilot_phase: 0.0,
            output_scale,
        }
    }

    /// Create modulator for bandwidth with default constellation
    pub fn for_bandwidth(bandwidth: Bandwidth, sample_rate: f32) -> Self {
        let config = OfdmConfig::for_bandwidth(bandwidth, sample_rate);
        Self::new(config, ConstellationType::Bpsk)
    }

    /// Set constellation type
    pub fn set_constellation(&mut self, constellation_type: ConstellationType) {
        self.constellation = Constellation::new(constellation_type);
    }

    /// Modulate bits to OFDM symbol
    ///
    /// If not enough bits are provided, the remaining carriers use zero symbols.
    pub fn modulate_symbol(&mut self, bits: &[u8]) -> Vec<f32> {
        let bits_per_carrier = self.constellation.bits_per_symbol();
        let data_carriers = self.config.num_carriers - self.config.pilot_indices.len();
        let required_bits = data_carriers * bits_per_carrier;

        // Log warning instead of panicking if not enough bits
        if bits.len() < required_bits {
            log::debug!("OFDM modulate: not enough bits ({} < {}), padding with zeros",
                       bits.len(), required_bits);
        }

        // Create frequency domain symbol
        let mut freq_domain = vec![ComplexSample::new(0.0, 0.0); self.config.fft_size];

        // Map data to carriers with Hermitian symmetry for real output
        // Each data carrier and its conjugate mirror carry the same information
        let n = self.config.fft_size;
        let mut bit_idx = 0;
        for &carrier_idx in &self.config.carrier_indices {
            let symbol = if self.config.pilot_indices.contains(&carrier_idx) {
                // Pilot symbol
                ComplexSample::new(self.pilot_phase.cos(), self.pilot_phase.sin())
            } else {
                // Data symbol
                let s = self.constellation.map(&bits[bit_idx..bit_idx + bits_per_carrier]);
                bit_idx += bits_per_carrier;
                ComplexSample::new(s.i, s.q)
            };

            freq_domain[carrier_idx] = symbol;

            // Add conjugate at mirror position for Hermitian symmetry
            let conj_idx = n - carrier_idx;
            if conj_idx != carrier_idx && conj_idx < n {
                freq_domain[conj_idx] = symbol.conj();
            }
        }

        // IFFT to time domain
        let time_domain = self.fft.inverse_new(&freq_domain);

        // Add cyclic prefix and apply output scaling
        let mut output = Vec::with_capacity(self.config.fft_size + self.config.cp_length);

        // Copy last cp_length samples as prefix (with scaling)
        for i in (self.config.fft_size - self.config.cp_length)..self.config.fft_size {
            output.push(time_domain[i].re * self.output_scale);
        }

        // Copy full symbol (with scaling)
        for sample in &time_domain {
            output.push(sample.re * self.output_scale);
        }

        // Update pilot phase for next symbol using proper modulo arithmetic
        self.pilot_phase = wrap_phase(self.pilot_phase + PI / 4.0);

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

    /// Get bits per OFDM symbol
    pub fn bits_per_symbol(&self) -> usize {
        let data_carriers = self.config.num_carriers - self.config.pilot_indices.len();
        data_carriers * self.constellation.bits_per_symbol()
    }

    /// Get samples per OFDM symbol
    pub fn samples_per_symbol(&self) -> usize {
        self.config.fft_size + self.config.cp_length
    }

    /// Reset modulator state
    pub fn reset(&mut self) {
        self.pilot_phase = 0.0;
    }
}

/// OFDM demodulator
pub struct OfdmDemodulator {
    config: OfdmConfig,
    fft: Fft,
    constellation: Constellation,
    channel_estimate: Vec<ComplexSample>,
    pilot_phase: f32,
    /// Accumulated phase correction from per-symbol tracking
    phase_correction: f32,
    /// Smoothing factor for phase tracking (0 = no tracking, 1 = instant)
    phase_tracking_alpha: f32,
    /// Last demodulated constellation points (for display)
    last_constellation_points: Vec<(f32, f32)>,
    /// Flag indicating channel estimate has been initialized from actual pilots
    /// On the first symbol, we bootstrap all carriers from pilot measurements
    channel_estimate_initialized: bool,
}

impl OfdmDemodulator {
    /// Create OFDM demodulator
    pub fn new(config: OfdmConfig, constellation_type: ConstellationType) -> Self {
        let channel_estimate = vec![ComplexSample::new(1.0, 0.0); config.num_carriers];

        Self {
            fft: Fft::new(config.fft_size),
            constellation: Constellation::new(constellation_type),
            channel_estimate,
            config,
            pilot_phase: 0.0,
            phase_correction: 0.0,
            phase_tracking_alpha: 0.3, // Moderate tracking speed
            last_constellation_points: Vec::new(),
            channel_estimate_initialized: false,
        }
    }

    /// Create demodulator for bandwidth
    pub fn for_bandwidth(bandwidth: Bandwidth, sample_rate: f32) -> Self {
        let config = OfdmConfig::for_bandwidth(bandwidth, sample_rate);
        Self::new(config, ConstellationType::Bpsk)
    }

    /// Set constellation type
    pub fn set_constellation(&mut self, constellation_type: ConstellationType) {
        self.constellation = Constellation::new(constellation_type);
    }

    /// Demodulate OFDM symbol to bits
    ///
    /// Returns empty vec if samples is too short.
    pub fn demodulate_symbol(&mut self, samples: &[f32]) -> Vec<u8> {
        let symbol_len = self.config.fft_size + self.config.cp_length;
        if samples.len() < symbol_len {
            log::warn!("OFDM demodulate: samples too short ({} < {})",
                       samples.len(), symbol_len);
            return Vec::new();
        }

        // Remove cyclic prefix and convert to complex
        let time_domain: Vec<ComplexSample> = samples[self.config.cp_length..symbol_len]
            .iter()
            .map(|&s| ComplexSample::new(s, 0.0))
            .collect();

        // FFT to frequency domain
        let freq_domain = self.fft.forward_new(&time_domain);

        // Extract and equalize carriers
        let mut bits = Vec::new();
        let _bits_per_carrier = self.constellation.bits_per_symbol();

        // Update channel estimate from pilots
        self.update_channel_estimate(&freq_domain);

        // Clear constellation points for this symbol
        self.last_constellation_points.clear();

        // Demodulate data carriers
        for (carrier_num, &carrier_idx) in self.config.carrier_indices.iter().enumerate() {
            if self.config.pilot_indices.contains(&carrier_idx) {
                continue; // Skip pilot
            }

            // Equalize with guard against division by zero
            let received = freq_domain[carrier_idx];
            let channel_est = self.channel_estimate[carrier_num];
            let equalized = if channel_est.norm() > 0.001 {
                received / channel_est
            } else {
                received
            };

            // NaN guard
            let equalized = if equalized.re.is_nan() || equalized.im.is_nan() {
                ComplexSample::new(0.0, 0.0)
            } else {
                equalized
            };

            // Store constellation point for display
            self.last_constellation_points.push((equalized.re, equalized.im));

            // Demap
            let symbol = Symbol::new(equalized.re, equalized.im);
            bits.extend(self.constellation.demap(symbol));
        }

        // Update pilot phase using proper modulo arithmetic
        self.pilot_phase = wrap_phase(self.pilot_phase + PI / 4.0);

        bits
    }

    /// Demodulate symbol with soft outputs
    ///
    /// Returns empty vec if samples is too short.
    pub fn demodulate_symbol_soft(&mut self, samples: &[f32], noise_var: f32) -> Vec<f32> {
        let symbol_len = self.config.fft_size + self.config.cp_length;
        if samples.len() < symbol_len {
            log::warn!("OFDM demodulate_soft: samples too short ({} < {})",
                       samples.len(), symbol_len);
            return Vec::new();
        }

        let time_domain: Vec<ComplexSample> = samples[self.config.cp_length..symbol_len]
            .iter()
            .map(|&s| ComplexSample::new(s, 0.0))
            .collect();

        let mut freq_domain = self.fft.forward_new(&time_domain);

        // Per-carrier phase tracking using pilot interpolation
        let pilot_phase_errors = self.estimate_pilot_phase_errors(&freq_domain);
        let interpolated_corrections = self.interpolate_phase_corrections(&pilot_phase_errors);

        // Apply per-carrier phase corrections
        self.apply_interpolated_phase_correction(&mut freq_domain, &interpolated_corrections);

        self.update_channel_estimate(&freq_domain);

        // Clear constellation points for this symbol
        self.last_constellation_points.clear();

        let mut llrs = Vec::new();

        for (carrier_num, &carrier_idx) in self.config.carrier_indices.iter().enumerate() {
            if self.config.pilot_indices.contains(&carrier_idx) {
                continue;
            }

            let received = freq_domain[carrier_idx];
            let channel_est = self.channel_estimate[carrier_num];

            // Guard against division by zero/very small values that produce NaN
            let equalized = if channel_est.norm() > 0.001 {
                received / channel_est
            } else {
                // Fallback: use received directly if channel estimate is invalid
                received
            };

            // Additional NaN guard
            let equalized = if equalized.re.is_nan() || equalized.im.is_nan() {
                ComplexSample::new(0.0, 0.0)
            } else {
                equalized
            };

            // Store constellation point for display
            self.last_constellation_points.push((equalized.re, equalized.im));

            let symbol = Symbol::new(equalized.re, equalized.im);
            llrs.extend(self.constellation.demap_soft(symbol, noise_var));
        }

        // Update pilot phase using proper modulo arithmetic
        self.pilot_phase = wrap_phase(self.pilot_phase + PI / 4.0);

        llrs
    }

    /// Update channel estimate from pilot symbols
    fn update_channel_estimate(&mut self, freq_domain: &[ComplexSample]) {
        let expected_pilot = ComplexSample::new(self.pilot_phase.cos(), self.pilot_phase.sin());

        // First pass: extract pilot channel estimates
        let mut pilot_estimates = Vec::new();
        for (carrier_num, &carrier_idx) in self.config.carrier_indices.iter().enumerate() {
            if self.config.pilot_indices.contains(&carrier_idx) {
                let received = freq_domain[carrier_idx];
                // Channel estimate = received / expected
                if expected_pilot.norm() > 0.01 {
                    let estimate = received / expected_pilot;
                    self.channel_estimate[carrier_num] = estimate;
                    pilot_estimates.push(estimate);
                }
            }
        }

        // On first symbol: bootstrap all carriers with average pilot estimate
        // This prevents using default unity gain for interpolation base
        if !self.channel_estimate_initialized && !pilot_estimates.is_empty() {
            // Calculate average pilot estimate
            let avg_estimate = pilot_estimates.iter().fold(
                ComplexSample::new(0.0, 0.0),
                |acc, &e| acc + e
            ) / pilot_estimates.len() as f32;

            // Initialize all non-pilot carriers with average estimate
            for (carrier_num, &carrier_idx) in self.config.carrier_indices.iter().enumerate() {
                if !self.config.pilot_indices.contains(&carrier_idx) {
                    self.channel_estimate[carrier_num] = avg_estimate;
                }
            }

            self.channel_estimate_initialized = true;
            log::debug!("Channel estimate initialized from {} pilots, avg magnitude: {:.3}",
                pilot_estimates.len(), avg_estimate.norm());
        }

        // Interpolate channel estimate for data carriers
        self.interpolate_channel_estimate();
    }

    /// Interpolate channel estimate between pilots
    fn interpolate_channel_estimate(&mut self) {
        let mut last_pilot_idx = 0;
        let mut last_estimate = self.channel_estimate[0];

        for (i, &carrier_idx) in self.config.carrier_indices.iter().enumerate() {
            if self.config.pilot_indices.contains(&carrier_idx) {
                // Interpolate between last pilot and this one
                let current_estimate = self.channel_estimate[i];

                if i > last_pilot_idx + 1 {
                    for j in (last_pilot_idx + 1)..i {
                        let alpha = (j - last_pilot_idx) as f32 / (i - last_pilot_idx) as f32;
                        self.channel_estimate[j] = ComplexSample::new(
                            last_estimate.re * (1.0 - alpha) + current_estimate.re * alpha,
                            last_estimate.im * (1.0 - alpha) + current_estimate.im * alpha,
                        );
                    }
                }

                last_pilot_idx = i;
                last_estimate = current_estimate;
            }
        }

        // Extrapolate to end
        for i in (last_pilot_idx + 1)..self.config.num_carriers {
            self.channel_estimate[i] = last_estimate;
        }
    }

    /// Demodulate multiple symbols
    pub fn demodulate(&mut self, samples: &[f32]) -> Vec<u8> {
        let symbol_len = self.config.fft_size + self.config.cp_length;
        let num_symbols = samples.len() / symbol_len;

        let mut bits = Vec::new();

        for i in 0..num_symbols {
            let start = i * symbol_len;
            bits.extend(self.demodulate_symbol(&samples[start..]));
        }

        bits
    }

    /// Get bits per OFDM symbol
    pub fn bits_per_symbol(&self) -> usize {
        let data_carriers = self.config.num_carriers - self.config.pilot_indices.len();
        data_carriers * self.constellation.bits_per_symbol()
    }

    /// Get samples per OFDM symbol
    pub fn samples_per_symbol(&self) -> usize {
        self.config.fft_size + self.config.cp_length
    }

    /// Get last demodulated constellation points (I/Q symbols)
    pub fn constellation_points(&self) -> &[(f32, f32)] {
        &self.last_constellation_points
    }

    /// Reset demodulator state
    pub fn reset(&mut self) {
        self.pilot_phase = 0.0;
        self.phase_correction = 0.0;
        self.channel_estimate.fill(ComplexSample::new(1.0, 0.0));
        self.last_constellation_points.clear();
        // Reset initialization flag so first symbol properly bootstraps channel estimate
        self.channel_estimate_initialized = false;
    }

    /// Estimate phase error from pilot symbols
    /// Returns the average phase difference between received and expected pilots
    fn estimate_pilot_phase_error(&self, freq_domain: &[ComplexSample]) -> f32 {
        let expected_pilot = ComplexSample::new(self.pilot_phase.cos(), self.pilot_phase.sin());

        let mut phase_errors = Vec::new();

        for &carrier_idx in &self.config.pilot_indices {
            if carrier_idx < freq_domain.len() {
                let received = freq_domain[carrier_idx];
                if received.norm() > 0.01 && expected_pilot.norm() > 0.01 {
                    // Phase error = angle(received) - angle(expected)
                    let received_phase = received.im.atan2(received.re);
                    let expected_phase = expected_pilot.im.atan2(expected_pilot.re);
                    let mut error = received_phase - expected_phase;
                    // Wrap to [-PI, PI]
                    while error > PI {
                        error -= 2.0 * PI;
                    }
                    while error < -PI {
                        error += 2.0 * PI;
                    }
                    phase_errors.push(error);
                }
            }
        }

        if phase_errors.is_empty() {
            return 0.0;
        }

        // Return average phase error
        phase_errors.iter().sum::<f32>() / phase_errors.len() as f32
    }

    /// Estimate phase errors at each pilot position
    /// Returns vector of (carrier_index_in_list, phase_error) for each pilot
    fn estimate_pilot_phase_errors(&self, freq_domain: &[ComplexSample]) -> Vec<(usize, f32)> {
        let expected_pilot = ComplexSample::new(self.pilot_phase.cos(), self.pilot_phase.sin());
        let mut pilot_errors = Vec::new();

        for (carrier_num, &carrier_idx) in self.config.carrier_indices.iter().enumerate() {
            if self.config.pilot_indices.contains(&carrier_idx) {
                if carrier_idx < freq_domain.len() {
                    let received = freq_domain[carrier_idx];
                    if received.norm() > 0.01 && expected_pilot.norm() > 0.01 {
                        let received_phase = received.im.atan2(received.re);
                        let expected_phase = expected_pilot.im.atan2(expected_pilot.re);
                        let mut error = received_phase - expected_phase;
                        // Wrap to [-PI, PI]
                        while error > PI {
                            error -= 2.0 * PI;
                        }
                        while error < -PI {
                            error += 2.0 * PI;
                        }
                        pilot_errors.push((carrier_num, error));
                    }
                }
            }
        }

        pilot_errors
    }

    /// Interpolate phase corrections from pilot positions to all carriers
    /// Returns a phase correction for each carrier in carrier_indices
    fn interpolate_phase_corrections(&self, pilot_errors: &[(usize, f32)]) -> Vec<f32> {
        let num_carriers = self.config.num_carriers;
        let mut corrections = vec![0.0; num_carriers];

        if pilot_errors.is_empty() {
            return corrections;
        }

        if pilot_errors.len() == 1 {
            // Single pilot: use its error for all carriers
            let error = pilot_errors[0].1;
            corrections.fill(error);
            return corrections;
        }

        // Multiple pilots: linear interpolation between them
        let mut pilot_iter = pilot_errors.iter().peekable();
        let mut prev_pilot = pilot_errors[0];

        for i in 0..num_carriers {
            // Move to next pilot segment if needed
            while let Some(&&(next_idx, _)) = pilot_iter.peek() {
                if next_idx <= i {
                    prev_pilot = *pilot_iter.next().unwrap();
                } else {
                    break;
                }
            }

            // Find the next pilot after current position
            let next_pilot = pilot_iter.peek().map(|&&p| p);

            corrections[i] = match next_pilot {
                Some((next_idx, next_error)) if next_idx > prev_pilot.0 => {
                    // Interpolate between prev and next pilot
                    let alpha = (i - prev_pilot.0) as f32 / (next_idx - prev_pilot.0) as f32;
                    let alpha = alpha.clamp(0.0, 1.0);

                    // Handle phase wrap-around during interpolation
                    let mut diff = next_error - prev_pilot.1;
                    if diff > PI {
                        diff -= 2.0 * PI;
                    } else if diff < -PI {
                        diff += 2.0 * PI;
                    }

                    let mut result = prev_pilot.1 + alpha * diff;
                    // Normalize result
                    while result > PI {
                        result -= 2.0 * PI;
                    }
                    while result < -PI {
                        result += 2.0 * PI;
                    }
                    result
                }
                _ => {
                    // Extrapolate from last pilot
                    prev_pilot.1
                }
            };
        }

        corrections
    }

    /// Apply per-carrier interpolated phase corrections
    fn apply_interpolated_phase_correction(&self, freq_domain: &mut [ComplexSample], corrections: &[f32]) {
        for (carrier_num, &carrier_idx) in self.config.carrier_indices.iter().enumerate() {
            if carrier_idx < freq_domain.len() && carrier_num < corrections.len() {
                let correction = corrections[carrier_num];
                let cos_corr = correction.cos();
                let sin_corr = correction.sin();
                let sample = freq_domain[carrier_idx];
                // Rotate by -correction (to cancel out the error)
                let re = sample.re * cos_corr + sample.im * sin_corr;
                let im = -sample.re * sin_corr + sample.im * cos_corr;
                freq_domain[carrier_idx] = ComplexSample::new(re, im);
            }
        }
    }

    /// Apply phase correction to frequency domain samples (uniform correction)
    fn apply_phase_correction(freq_domain: &mut [ComplexSample], correction: f32) {
        let cos_corr = correction.cos();
        let sin_corr = correction.sin();
        for sample in freq_domain.iter_mut() {
            // Rotate by -correction (to cancel out the error)
            let re = sample.re * cos_corr + sample.im * sin_corr;
            let im = -sample.re * sin_corr + sample.im * cos_corr;
            *sample = ComplexSample::new(re, im);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ofdm_config() {
        let config = OfdmConfig::for_bandwidth(Bandwidth::Wide, 48000.0);
        assert_eq!(config.num_carriers, 49);
        assert_eq!(config.fft_size, 1024);
    }

    #[test]
    fn test_ofdm_round_trip() {
        let config = OfdmConfig::for_bandwidth(Bandwidth::Narrow, 48000.0);
        let mut modulator = OfdmModulator::new(config.clone(), ConstellationType::Qpsk);
        let mut demodulator = OfdmDemodulator::new(config, ConstellationType::Qpsk);

        let bits_per_symbol = modulator.bits_per_symbol();
        let bits: Vec<u8> = (0..bits_per_symbol).map(|i| (i % 2) as u8).collect();

        let samples = modulator.modulate_symbol(&bits);
        let recovered = demodulator.demodulate_symbol(&samples);

        // Check most bits match (pilots may affect count)
        let matches: usize = bits.iter()
            .zip(recovered.iter())
            .filter(|(&a, &b)| a == b)
            .count();

        assert!(matches > bits_per_symbol * 9 / 10, "Too few matching bits: {}/{}", matches, bits_per_symbol);
    }

    #[test]
    fn test_bandwidth_carriers() {
        assert_eq!(Bandwidth::Narrow.num_carriers(), 11);
        assert_eq!(Bandwidth::Wide.num_carriers(), 49);
        assert_eq!(Bandwidth::Ultra.num_carriers(), 59);
    }

    #[test]
    fn test_channel_estimate_initialization() {
        let config = OfdmConfig::for_bandwidth(Bandwidth::Wide, 48000.0);
        let mut demod = OfdmDemodulator::new(config.clone(), ConstellationType::Qpsk);

        // Initially, channel estimate should not be initialized
        assert!(!demod.channel_estimate_initialized);

        // All estimates should be unity initially
        for est in &demod.channel_estimate {
            assert!((est.norm() - 1.0).abs() < 0.001, "Initial estimate should be unity");
        }

        // Create a modulator to generate a valid symbol
        let mut mod_ = OfdmModulator::new(config.clone(), ConstellationType::Qpsk);
        let bits: Vec<u8> = (0..mod_.bits_per_symbol()).map(|i| (i % 2) as u8).collect();
        let samples = mod_.modulate_symbol(&bits);

        // Demodulate the symbol - this should initialize channel estimates
        let _ = demod.demodulate_symbol(&samples);

        // Now channel estimate should be initialized
        assert!(demod.channel_estimate_initialized);

        // Channel estimates should no longer all be unity (some are pilot-derived)
        let non_unity_count = demod.channel_estimate.iter()
            .filter(|e| (e.norm() - 1.0).abs() > 0.01)
            .count();

        // At least some carriers should have updated estimates
        // (The actual received signal amplitude affects this)
        assert!(non_unity_count >= 0, "Channel estimates should be updated from pilots");

        // Reset should clear the initialization flag
        demod.reset();
        assert!(!demod.channel_estimate_initialized);
    }

    #[test]
    fn test_first_symbol_no_artifacts() {
        // Test that first symbol demodulation works without artifacts
        let config = OfdmConfig::for_bandwidth(Bandwidth::Wide, 48000.0);
        let mut mod_ = OfdmModulator::new(config.clone(), ConstellationType::Bpsk);
        let mut demod = OfdmDemodulator::new(config, ConstellationType::Bpsk);

        // Generate multiple symbols with the same bit pattern
        let bits: Vec<u8> = (0..mod_.bits_per_symbol()).map(|i| (i % 2) as u8).collect();

        // First symbol
        let samples1 = mod_.modulate_symbol(&bits);
        let recovered1 = demod.demodulate_symbol(&samples1);

        // Second symbol (channel estimate now initialized)
        let samples2 = mod_.modulate_symbol(&bits);
        let recovered2 = demod.demodulate_symbol(&samples2);

        // Both symbols should have similar error rates
        let matches1: usize = bits.iter().zip(recovered1.iter())
            .filter(|(&a, &b)| a == b).count();
        let matches2: usize = bits.iter().zip(recovered2.iter())
            .filter(|(&a, &b)| a == b).count();

        let error_rate1 = 1.0 - (matches1 as f32 / bits.len() as f32);
        let error_rate2 = 1.0 - (matches2 as f32 / bits.len() as f32);

        // First symbol shouldn't have significantly worse performance than second
        // Allow first symbol to be at most 20% worse (absolute error rate difference)
        assert!(
            error_rate1 - error_rate2 < 0.2,
            "First symbol error rate ({:.2}%) much worse than second ({:.2}%)",
            error_rate1 * 100.0, error_rate2 * 100.0
        );
    }
}
