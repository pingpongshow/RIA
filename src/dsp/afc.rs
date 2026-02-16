//! Automatic Frequency Control (AFC)
//!
//! Provides frequency offset estimation and correction for:
//! - OFDM signals using cyclic prefix correlation
//! - Single-tone signals using phase tracking
//! - General signals using autocorrelation

use super::ComplexSample;
use std::f32::consts::PI;

/// Wrap phase to [-PI, PI] range using proper modulo arithmetic
/// This avoids floating-point error accumulation from repeated if/else wrapping
#[inline]
pub fn wrap_phase(phase: f32) -> f32 {
    (phase + PI).rem_euclid(2.0 * PI) - PI
}

/// Wrap phase to [0, 2*PI] range using proper modulo arithmetic
#[inline]
pub fn wrap_phase_positive(phase: f32) -> f32 {
    phase.rem_euclid(2.0 * PI)
}

/// AFC processor for tracking and correcting frequency offset
pub struct Afc {
    sample_rate: f32,
    center_freq: f32,
    offset: f32,
    max_offset: f32,
    tracking_bandwidth: f32,
    alpha: f32, // Smoothing factor
    enabled: bool,

    // Phase accumulator for frequency shift
    phase: f32,
    phase_increment: f32,
}

impl Afc {
    /// Create a new AFC centered at the given frequency
    pub fn new(sample_rate: f32, center_freq: f32) -> Self {
        Self {
            sample_rate,
            center_freq,
            offset: 0.0,
            max_offset: 250.0, // +/- 250 Hz max tracking (covers ±200 Hz requirement)
            tracking_bandwidth: 100.0,
            alpha: 0.1,
            enabled: true,
            phase: 0.0,
            phase_increment: 0.0,
        }
    }

    /// Set maximum tracking offset in Hz
    pub fn set_max_offset(&mut self, hz: f32) {
        self.max_offset = hz.abs();
    }

    /// Set tracking bandwidth in Hz
    pub fn set_tracking_bandwidth(&mut self, hz: f32) {
        self.tracking_bandwidth = hz.abs();
    }

    /// Set smoothing factor (0.0 to 1.0, higher = faster tracking)
    pub fn set_alpha(&mut self, alpha: f32) {
        self.alpha = alpha.clamp(0.01, 1.0);
    }

    /// Enable or disable AFC
    pub fn set_enabled(&mut self, enabled: bool) {
        self.enabled = enabled;
        if !enabled {
            self.offset = 0.0;
            self.phase_increment = 0.0;
        }
    }

    /// Get current frequency offset in Hz
    pub fn offset(&self) -> f32 {
        self.offset
    }

    /// Get corrected center frequency
    pub fn corrected_freq(&self) -> f32 {
        self.center_freq + self.offset
    }

    /// Update AFC with detected frequency error
    pub fn update(&mut self, freq_error: f32) {
        if !self.enabled {
            return;
        }

        // Clamp to tracking bandwidth
        let clamped_error = freq_error.clamp(-self.tracking_bandwidth, self.tracking_bandwidth);

        // Smooth update
        self.offset = self.alpha * clamped_error + (1.0 - self.alpha) * self.offset;

        // Clamp to max offset
        self.offset = self.offset.clamp(-self.max_offset, self.max_offset);

        // Update phase increment for frequency correction
        self.phase_increment = 2.0 * PI * self.offset / self.sample_rate;
    }

    /// Apply frequency correction to a sample
    pub fn correct(&mut self, sample: ComplexSample) -> ComplexSample {
        if !self.enabled || self.offset.abs() < 0.1 {
            return sample;
        }

        // Generate correction phasor (negative frequency shift)
        let correction = ComplexSample::new(self.phase.cos(), -self.phase.sin());

        // Update phase using proper modulo arithmetic to avoid floating-point error accumulation
        self.phase = wrap_phase(self.phase + self.phase_increment);

        sample * correction
    }

    /// Apply frequency correction to a block of samples
    pub fn correct_block(&mut self, samples: &mut [ComplexSample]) {
        for sample in samples.iter_mut() {
            *sample = self.correct(*sample);
        }
    }

    /// Estimate frequency offset using OFDM cyclic prefix correlation
    ///
    /// This method exploits the fact that the cyclic prefix is a copy of the
    /// last `cp_length` samples of the OFDM symbol. The correlation between
    /// the CP and its source (at lag = fft_size) gives the frequency offset.
    ///
    /// **Important**: This method has an unambiguous range of:
    /// `±sample_rate / (2 * fft_size)` (e.g., ±23.4 Hz for 48kHz/1024)
    ///
    /// For larger offsets, use `estimate_tone_offset` on the preamble first
    /// for coarse acquisition, then use this for fine tracking.
    ///
    /// # Arguments
    /// * `samples` - Complex baseband samples containing OFDM symbols
    /// * `fft_size` - OFDM FFT size (e.g., 1024)
    /// * `cp_length` - Cyclic prefix length (e.g., 128)
    ///
    /// # Returns
    /// Estimated frequency offset in Hz (accurate within unambiguous range)
    pub fn estimate_ofdm_offset(
        &self,
        samples: &[ComplexSample],
        fft_size: usize,
        cp_length: usize,
    ) -> f32 {
        let symbol_length = fft_size + cp_length;

        if samples.len() < symbol_length {
            return 0.0;
        }

        // Maximum unambiguous offset for this FFT size
        let max_unambiguous = self.sample_rate / (2.0 * fft_size as f32);

        // Accumulate correlation across all available symbols
        let mut total_correlation = ComplexSample::new(0.0, 0.0);
        let mut num_symbols = 0;

        let mut pos = 0;
        while pos + symbol_length <= samples.len() {
            // Correlate CP (first cp_length samples) with end of symbol
            // CP is at: pos .. pos + cp_length
            // Source is at: pos + fft_size .. pos + fft_size + cp_length
            let mut symbol_corr = ComplexSample::new(0.0, 0.0);

            for i in 0..cp_length {
                if pos + fft_size + i >= samples.len() {
                    break;
                }
                let cp_sample = samples[pos + i];
                let src_sample = samples[pos + fft_size + i];
                // Conjugate multiply: cp * conj(src)
                // The phase accumulation is: -2π * Δf * fft_size / fs
                symbol_corr += cp_sample * src_sample.conj();
            }

            total_correlation += symbol_corr;
            num_symbols += 1;
            pos += symbol_length;
        }

        if num_symbols == 0 {
            return 0.0;
        }

        // Average correlation
        let avg_correlation = total_correlation / num_symbols as f32;

        // Phase of correlation gives frequency offset
        // Phase = -2π * Δf * fft_size / fs
        // Therefore: Δf = -phase * fs / (2π * fft_size)
        let phase = avg_correlation.arg();
        let freq_offset = -phase * self.sample_rate / (2.0 * PI * fft_size as f32);

        // Clamp to unambiguous range (phase wrapping limit)
        freq_offset.clamp(-max_unambiguous, max_unambiguous)
    }

    /// Get the maximum unambiguous offset for OFDM CP-based AFC
    ///
    /// Offsets larger than this will wrap due to phase ambiguity.
    /// Use `estimate_tone_offset` for coarse acquisition first.
    pub fn max_ofdm_offset(&self, fft_size: usize) -> f32 {
        self.sample_rate / (2.0 * fft_size as f32)
    }

    /// Estimate frequency offset from a single tone signal
    ///
    /// Uses phase difference between consecutive samples to estimate frequency.
    /// Best for preamble tones or pilot signals.
    ///
    /// # Arguments
    /// * `samples` - Complex baseband samples of a single tone
    /// * `expected_freq` - Expected tone frequency in Hz
    ///
    /// # Returns
    /// Estimated frequency offset in Hz (actual_freq - expected_freq)
    pub fn estimate_tone_offset(
        &self,
        samples: &[ComplexSample],
        expected_freq: f32,
    ) -> f32 {
        if samples.len() < 2 {
            return 0.0;
        }

        // Compute average phase increment between consecutive samples
        let mut phase_sum = 0.0f32;
        let mut count = 0;

        for i in 1..samples.len() {
            // Phase difference: arg(s[i] * conj(s[i-1]))
            let diff = samples[i] * samples[i - 1].conj();
            let phase_diff = diff.arg();
            phase_sum += phase_diff;
            count += 1;
        }

        if count == 0 {
            return 0.0;
        }

        let avg_phase_inc = phase_sum / count as f32;

        // Convert phase increment to frequency
        // freq = phase_increment * sample_rate / (2 * PI)
        let detected_freq = avg_phase_inc * self.sample_rate / (2.0 * PI);

        // Return offset from expected
        let offset = detected_freq - expected_freq;
        offset.clamp(-self.max_offset, self.max_offset)
    }

    /// Estimate frequency offset using autocorrelation at a specific lag
    ///
    /// General-purpose method that works for various signal types.
    /// For best results, the lag should match a known repetition period.
    ///
    /// # Arguments
    /// * `samples` - Complex baseband samples
    /// * `lag` - Autocorrelation lag in samples
    ///
    /// # Returns
    /// Estimated frequency offset in Hz
    pub fn estimate_offset(&self, samples: &[ComplexSample], lag: usize) -> f32 {
        if samples.len() < lag + 1 || lag == 0 {
            return 0.0;
        }

        // Compute autocorrelation at the specified lag
        let mut correlation = ComplexSample::new(0.0, 0.0);

        for i in 0..(samples.len() - lag) {
            // R(lag) = sum of s[i] * conj(s[i + lag])
            correlation += samples[i] * samples[i + lag].conj();
        }

        // Phase of autocorrelation gives frequency offset
        // For a signal with frequency offset Δf:
        // s[n+L] = s[n] * exp(j * 2π * Δf * L / fs)
        // So arg(R(L)) = 2π * Δf * L / fs
        // Therefore: Δf = arg(R(L)) * fs / (2π * L)
        let phase = correlation.arg();
        let freq_offset = phase * self.sample_rate / (2.0 * PI * lag as f32);

        freq_offset.clamp(-self.max_offset, self.max_offset)
    }

    /// Estimate frequency offset with fine resolution using multiple lags
    ///
    /// Uses weighted average of estimates from multiple lags to improve accuracy.
    /// Longer lags give finer resolution but are more sensitive to noise.
    ///
    /// # Arguments
    /// * `samples` - Complex baseband samples
    /// * `min_lag` - Minimum lag to use
    /// * `max_lag` - Maximum lag to use
    ///
    /// # Returns
    /// Estimated frequency offset in Hz
    pub fn estimate_offset_fine(
        &self,
        samples: &[ComplexSample],
        min_lag: usize,
        max_lag: usize,
    ) -> f32 {
        if samples.len() < max_lag + 1 || min_lag >= max_lag {
            return 0.0;
        }

        let mut weighted_sum = 0.0f32;
        let mut weight_total = 0.0f32;

        // Use multiple lags and weight by correlation magnitude
        for lag in min_lag..=max_lag {
            if lag == 0 || samples.len() <= lag {
                continue;
            }

            let mut correlation = ComplexSample::new(0.0, 0.0);
            for i in 0..(samples.len() - lag) {
                correlation += samples[i] * samples[i + lag].conj();
            }

            // Magnitude indicates reliability of this estimate
            let magnitude = correlation.norm();
            if magnitude < 1e-10 {
                continue;
            }

            let phase = correlation.arg();
            let freq_offset = phase * self.sample_rate / (2.0 * PI * lag as f32);

            // Weight by correlation magnitude and lag (longer lags = finer resolution)
            let weight = magnitude * (lag as f32).sqrt();
            weighted_sum += freq_offset * weight;
            weight_total += weight;
        }

        if weight_total < 1e-10 {
            return 0.0;
        }

        let estimate = weighted_sum / weight_total;
        estimate.clamp(-self.max_offset, self.max_offset)
    }

    /// Reset AFC state
    pub fn reset(&mut self) {
        self.offset = 0.0;
        self.phase = 0.0;
        self.phase_increment = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn generate_complex_tone(freq: f32, sample_rate: f32, num_samples: usize) -> Vec<ComplexSample> {
        (0..num_samples)
            .map(|i| {
                let t = i as f32 / sample_rate;
                let phase = 2.0 * PI * freq * t;
                ComplexSample::new(phase.cos(), phase.sin())
            })
            .collect()
    }

    #[test]
    fn test_afc_tracking() {
        let mut afc = Afc::new(48000.0, 1500.0);

        // Simulate frequency error
        afc.update(50.0);
        assert!(afc.offset() > 0.0);

        // Continue updating
        for _ in 0..100 {
            afc.update(50.0);
        }

        // Should converge near 50 Hz
        assert!((afc.offset() - 50.0).abs() < 5.0);
    }

    #[test]
    fn test_tone_offset_estimation() {
        let afc = Afc::new(48000.0, 1500.0);

        // Test various offsets
        for offset in [-50.0, -25.0, 0.0, 25.0, 50.0] {
            let tone = generate_complex_tone(1500.0 + offset, 48000.0, 4800);
            let estimated = afc.estimate_tone_offset(&tone, 1500.0);

            let error = (estimated - offset).abs();
            assert!(
                error < 2.0,
                "Tone offset {}: estimated {}, error {}",
                offset,
                estimated,
                error
            );
        }
    }

    #[test]
    fn test_wrap_phase() {
        // Test that wrap_phase keeps values in [-PI, PI]
        let test_cases = [
            (0.0, 0.0),
            (PI, PI),                    // Edge case: exactly PI
            (-PI, -PI),                  // Edge case: exactly -PI
            (PI + 0.1, -PI + 0.1),       // Just over PI
            (-PI - 0.1, PI - 0.1),       // Just under -PI
            (3.0 * PI, -PI),             // Multiple wraps positive
            (-3.0 * PI, -PI),            // Multiple wraps negative
            (2.0 * PI, 0.0),             // Exactly 2*PI
            (-2.0 * PI, 0.0),            // Exactly -2*PI
            (100.0 * PI, 0.0),           // Many wraps (tests numerical stability)
        ];

        for (input, expected) in test_cases {
            let result = wrap_phase(input);
            let diff = (result - expected).abs();
            assert!(
                diff < 1e-5 || (result.abs() - PI.abs()).abs() < 1e-5,
                "wrap_phase({}) = {}, expected {} (diff={})",
                input, result, expected, diff
            );
            // Always check range
            assert!(
                result >= -PI && result <= PI,
                "wrap_phase({}) = {} is outside [-PI, PI]",
                input, result
            );
        }
    }

    #[test]
    fn test_wrap_phase_accumulation() {
        // Test that repeated wrapping doesn't accumulate errors
        let mut phase = 0.0f32;
        let increment = 0.1f32;

        // Simulate 10 million phase updates (like running for hours)
        for _ in 0..10_000_000 {
            phase = wrap_phase(phase + increment);
        }

        // Phase should still be in valid range
        assert!(
            phase >= -PI && phase <= PI,
            "After 10M iterations, phase {} is outside [-PI, PI]",
            phase
        );
    }

    #[test]
    fn test_ofdm_offset_estimation() {
        let afc = Afc::new(48000.0, 1500.0);
        let fft_size = 1024;
        let cp_length = 128;

        // Maximum unambiguous offset: 48000 / (2 * 1024) ≈ 23.4 Hz
        // Test with offsets within this range
        for offset in [-20.0, -10.0, 0.0, 10.0, 20.0] {
            let mut samples = Vec::new();

            // Generate multiple OFDM symbols with CP structure
            for sym in 0..4 {
                // Generate symbol data (random-ish phases)
                let symbol_data: Vec<ComplexSample> = (0..fft_size)
                    .map(|i| {
                        let phase = (i as f32 * 0.1 + sym as f32 * 0.5) % (2.0 * PI);
                        ComplexSample::new(phase.cos(), phase.sin())
                    })
                    .collect();

                // Add cyclic prefix (copy of last cp_length samples)
                for i in (fft_size - cp_length)..fft_size {
                    samples.push(symbol_data[i]);
                }
                // Add symbol data
                samples.extend(&symbol_data);
            }

            // Apply frequency offset
            let offset_samples: Vec<ComplexSample> = samples
                .iter()
                .enumerate()
                .map(|(i, &s)| {
                    let t = i as f32 / 48000.0;
                    let phase = 2.0 * PI * offset * t;
                    s * ComplexSample::new(phase.cos(), phase.sin())
                })
                .collect();

            let estimated = afc.estimate_ofdm_offset(&offset_samples, fft_size, cp_length);

            let error = (estimated - offset).abs();
            assert!(
                error < 2.0,
                "OFDM offset {}: estimated {}, error {}",
                offset,
                estimated,
                error
            );
        }
    }
}
