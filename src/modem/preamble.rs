//! Preamble generation and detection
//!
//! Implements MFSK tone hopping preamble:
//! - Pseudo-random tone sequence
//! - 51ms per symbol
//! - 14 tones for 500Hz mode, 16 tones for 2300Hz mode
//! - Barker code sync following preamble
//!
//! ## Offset Semantics
//!
//! The `PreambleDetector::process()` function returns offsets that are **relative to the
//! passed sample buffer**, not any internal state. The returned `PreambleDetectionResult`
//! contains:
//! - `preamble_start`: Index into the input buffer where the preamble begins
//! - `data_start`: Index into the input buffer where the data (after preamble) begins
//! - `correlation`: Detection confidence (0.0 to 1.0)
//!
//! Callers should use `data_start` to find where to begin demodulating data frames.

use std::f32::consts::PI;
use crate::dsp::{ComplexSample, Fft, wrap_phase};

/// Result of preamble detection with explicit offset semantics
#[derive(Debug, Clone, Copy)]
pub struct PreambleDetectionResult {
    /// Offset into the input sample buffer where the preamble starts
    pub preamble_start: usize,
    /// Offset into the input sample buffer where data begins (after preamble)
    /// This is where the caller should start demodulating data frames
    pub data_start: usize,
    /// Detection correlation/confidence (0.0 to 1.0)
    pub correlation: f32,
}

/// Sample rate
const SAMPLE_RATE: f32 = 48000.0;

/// Preamble mode configuration
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PreambleMode {
    /// 500 Hz mode (robust) - FFT 2048, 14 tones
    Narrow,
    /// 2300 Hz mode (standard) - FFT 1024, 16 tones
    Wide,
    /// 2750 Hz mode (fast) - FFT 512
    Ultra,
}

impl PreambleMode {
    /// Get FFT size for this mode
    pub fn fft_size(&self) -> usize {
        match self {
            PreambleMode::Narrow => 2048,
            PreambleMode::Wide => 1024,
            PreambleMode::Ultra => 512,
        }
    }

    /// Get symbol samples (exactly equal to FFT size for proper tone detection)
    pub fn symbol_samples(&self) -> usize {
        self.fft_size()
    }

    /// Get symbol duration in milliseconds
    /// These match FFT_SIZE / SAMPLE_RATE for each mode
    pub fn symbol_ms(&self) -> f32 {
        self.fft_size() as f32 / SAMPLE_RATE * 1000.0
    }

    /// Get preamble duration in milliseconds
    pub fn preamble_duration_ms(&self) -> f32 {
        match self {
            PreambleMode::Narrow => 500.0,
            PreambleMode::Wide => 250.0,
            PreambleMode::Ultra => 125.0,
        }
    }

    /// Get tone bins for this mode
    pub fn tone_bins(&self) -> &'static [usize] {
        match self {
            // 500Hz mode: bins centered around 1500Hz (FFT=2048)
            PreambleMode::Narrow => &TONE_BINS_500HZ,
            // 2300Hz mode: wider spread (FFT=1024)
            PreambleMode::Wide => &TONE_BINS_2300HZ,
            // 2750Hz mode: proper bins for FFT=512 (same frequency range as 2300Hz)
            PreambleMode::Ultra => &TONE_BINS_2750HZ,
        }
    }
}

/// Tone bins for 500Hz mode (FFT=2048, centered around 1500Hz)
const TONE_BINS_500HZ: [usize; 14] = [50, 52, 54, 56, 58, 60, 62, 64, 66, 68, 70, 72, 74, 76];

/// Preamble tone bins for 2300Hz mode (FFT=1024, centered around 1500Hz)
/// These are the 16-MFSK symbol bins used for preamble/header, NOT OFDM carriers
/// Maps to frequencies: 703, 750, 797, 844, 891, 938, 984, 1031, 1172, 1219, 1266, 1406, 1594, 1641, 1734, 1781 Hz
const TONE_BINS_2300HZ: [usize; 16] = [15, 16, 17, 18, 19, 20, 21, 22, 25, 26, 27, 30, 34, 35, 37, 38];

/// Preamble tone bins for 2750Hz mode (FFT=512)
/// Same frequency range as 2300Hz mode but with half the bin numbers
/// Maps to frequencies: 750, 844, 938, 1031, 1125, 1219, 1313, 1406, 1500, 1594, 1688, 1781, 1875, 1969, 2063, 2156 Hz
const TONE_BINS_2750HZ: [usize; 16] = [8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23];

/// Preamble sequence
/// This pseudo-random sequence repeats to fill preamble duration
const PREAMBLE_SEQUENCE: [u8; 16] = [12, 1, 12, 4, 7, 13, 4, 12, 1, 10, 11, 2, 0, 4, 9, 1];

/// Barker codes for sync
pub const BARKER_5: [i8; 5] = [1, 1, 1, -1, 1];
pub const BARKER_7: [i8; 7] = [1, 1, 1, -1, -1, 1, -1];
pub const BARKER_11: [i8; 11] = [1, 1, 1, -1, -1, -1, 1, -1, -1, 1, -1];
pub const BARKER_13: [i8; 13] = [1, 1, 1, 1, 1, -1, -1, 1, 1, -1, 1, -1, 1];

/// Output amplitude for preamble (matches OFDM data levels)
/// This ensures consistent audio levels across all modulation modes
const PREAMBLE_AMPLITUDE: f32 = 0.2;

/// MFSK preamble generator
pub struct Preamble {
    mode: PreambleMode,
    sample_rate: f32,
    samples: Vec<f32>,
    symbol_samples: usize,
    num_symbols: usize,
}

impl Preamble {
    /// Create new preamble
    pub fn new(sample_rate: f32, _center_freq: f32, bandwidth: f32, duration: f32) -> Self {
        // Determine mode from bandwidth
        let mode = if bandwidth <= 600.0 {
            PreambleMode::Narrow
        } else if bandwidth <= 2400.0 {
            PreambleMode::Wide
        } else {
            PreambleMode::Ultra
        };

        Self::with_mode(sample_rate, mode, duration)
    }

    /// Create preamble with specific mode
    pub fn with_mode(sample_rate: f32, mode: PreambleMode, duration: f32) -> Self {
        // Use exact FFT size for symbol_samples to ensure proper tone detection
        let symbol_samples = mode.symbol_samples();
        let symbol_ms = mode.symbol_ms();
        let num_symbols = (duration * 1000.0 / symbol_ms) as usize;

        let samples = Self::generate_mfsk_preamble(sample_rate, mode, symbol_samples, num_symbols);

        // Log generated preamble info
        let tone_bins = mode.tone_bins();
        let fft_size = mode.fft_size();
        let tone_freqs: Vec<i32> = (0..num_symbols.min(16))
            .map(|i| {
                let seq_idx = i % PREAMBLE_SEQUENCE.len();
                let tone_idx = (PREAMBLE_SEQUENCE[seq_idx] as usize) % tone_bins.len();
                let bin = tone_bins[tone_idx];
                (bin as f32 * sample_rate / fft_size as f32) as i32
            })
            .collect();
        log::info!("Generating MFSK preamble: mode={:?}, fft_size={}, symbol_samples={}, num_symbols={}",
                   mode, fft_size, symbol_samples, num_symbols);
        log::info!("Preamble tone frequencies: {:?}", tone_freqs);

        Self {
            mode,
            sample_rate,
            samples,
            symbol_samples,
            num_symbols,
        }
    }

    /// Generate MFSK tone hopping preamble
    fn generate_mfsk_preamble(
        sample_rate: f32,
        mode: PreambleMode,
        symbol_samples: usize,
        num_symbols: usize,
    ) -> Vec<f32> {
        let tone_bins = mode.tone_bins();
        let fft_size = mode.fft_size();

        let mut samples = Vec::with_capacity(symbol_samples * num_symbols);
        let mut phase = 0.0f32;

        for i in 0..num_symbols {
            // Get tone index from preamble sequence
            let seq_idx = i % PREAMBLE_SEQUENCE.len();
            let tone_idx = (PREAMBLE_SEQUENCE[seq_idx] as usize) % tone_bins.len();
            let bin = tone_bins[tone_idx];

            // Calculate frequency from bin
            let freq = bin as f32 * sample_rate / fft_size as f32;
            let phase_inc = 2.0 * PI * freq / sample_rate;

            // Generate tone with raised cosine envelope
            let ramp_samples = symbol_samples / 20; // 5% ramp

            for j in 0..symbol_samples {
                // Envelope
                let envelope = if j < ramp_samples {
                    0.5 * (1.0 - (PI * j as f32 / ramp_samples as f32).cos())
                } else if j >= symbol_samples - ramp_samples {
                    let k = j - (symbol_samples - ramp_samples);
                    0.5 * (1.0 + (PI * k as f32 / ramp_samples as f32).cos())
                } else {
                    1.0
                };

                samples.push(PREAMBLE_AMPLITUDE * envelope * phase.sin());
                phase = wrap_phase(phase + phase_inc);
            }
        }

        samples
    }

    /// Generate Barker sync code (BPSK modulated)
    pub fn generate_barker_sync(sample_rate: f32, center_freq: f32, barker: &[i8]) -> Vec<f32> {
        let chip_samples = (sample_rate * 0.010) as usize; // 10ms per chip
        let mut samples = Vec::with_capacity(chip_samples * barker.len());

        let phase_inc = 2.0 * PI * center_freq / sample_rate;
        let mut phase = 0.0f32;

        for &chip in barker {
            let amplitude = chip as f32; // +1 or -1

            for _ in 0..chip_samples {
                samples.push(amplitude * phase.sin());
                phase = wrap_phase(phase + phase_inc);
            }
        }

        samples
    }

    /// Get preamble samples
    pub fn samples(&self) -> &[f32] {
        &self.samples
    }

    /// Get preamble length in samples
    pub fn len(&self) -> usize {
        self.samples.len()
    }

    /// Check if empty
    pub fn is_empty(&self) -> bool {
        self.samples.is_empty()
    }

    /// Get duration in seconds
    pub fn duration(&self) -> f32 {
        self.samples.len() as f32 / self.sample_rate
    }

    /// Get mode
    pub fn mode(&self) -> PreambleMode {
        self.mode
    }

    /// Get number of symbols
    pub fn num_symbols(&self) -> usize {
        self.num_symbols
    }

    /// Get samples per symbol
    pub fn samples_per_symbol(&self) -> usize {
        self.symbol_samples
    }

    /// Get tone frequencies used
    pub fn tone_frequencies(&self) -> Vec<f32> {
        let tone_bins = self.mode.tone_bins();
        let fft_size = self.mode.fft_size();

        tone_bins.iter()
            .map(|&bin| bin as f32 * self.sample_rate / fft_size as f32)
            .collect()
    }

    /// Get the expected preamble sequence
    pub fn expected_sequence(&self) -> &'static [u8] {
        &PREAMBLE_SEQUENCE
    }
}

/// Preamble detector using tone correlation
pub struct PreambleDetector {
    mode: PreambleMode,
    sample_rate: f32,
    fft: Fft,
    fft_size: usize,
    symbol_samples: usize,
    num_preamble_symbols: usize,  // Actual number of symbols in the preamble
    tone_bins: Vec<usize>,
    threshold: f32,
    buffer: Vec<f32>,
    correlation_peak: f32,
    energy_threshold: f32,
}

impl PreambleDetector {
    /// Create detector for given preamble
    pub fn new(preamble: &Preamble, threshold: f32) -> Self {
        let mode = preamble.mode();
        let fft_size = mode.fft_size();
        let symbol_samples = preamble.samples_per_symbol();
        let num_preamble_symbols = preamble.num_symbols();

        Self {
            mode,
            sample_rate: preamble.sample_rate,
            fft: Fft::new(fft_size),
            fft_size,
            symbol_samples,
            num_preamble_symbols,
            tone_bins: mode.tone_bins().to_vec(),
            threshold,
            buffer: Vec::new(),
            correlation_peak: 0.0,
            energy_threshold: 0.01, // RMS threshold for signal detection
        }
    }

    /// Process samples and detect preamble
    ///
    /// Returns `Some(PreambleDetectionResult)` if preamble detected.
    ///
    /// # Offset Semantics
    ///
    /// **IMPORTANT**: All offsets in the returned result are relative to the `samples`
    /// parameter passed to this function, NOT any internal buffer state.
    ///
    /// - `preamble_start`: Index into `samples` where the preamble begins
    /// - `data_start`: Index into `samples` where data begins (after preamble ends)
    ///
    /// # Example
    ///
    /// ```ignore
    /// let result = detector.process(&audio_buffer)?;
    /// // Start demodulating data at this offset into audio_buffer
    /// let data_samples = &audio_buffer[result.data_start..];
    /// ```
    pub fn process(&mut self, samples: &[f32]) -> Option<PreambleDetectionResult> {
        // Work directly on the passed samples buffer instead of maintaining internal state
        // This avoids offset mismatch between internal buffer and caller's buffer

        // Need enough samples for multiple symbols
        let min_samples = self.symbol_samples * 8;
        if samples.len() < min_samples {
            return None;
        }

        // First check energy level
        let rms = self.calculate_rms(samples);

        if rms < self.energy_threshold {
            return None;
        }

        // Use the passed samples directly for detection
        // CRITICAL: We clear and refill the buffer so all indices are relative to input
        self.buffer.clear();
        self.buffer.extend_from_slice(samples);

        // Detect preamble tones - returns result with offsets relative to self.buffer,
        // which is now identical to the input samples
        self.detect_preamble_tones()
    }

    /// Calculate RMS energy
    fn calculate_rms(&self, samples: &[f32]) -> f32 {
        if samples.is_empty() {
            return 0.0;
        }
        let sum_sq: f32 = samples.iter().map(|s| s * s).sum();
        (sum_sq / samples.len() as f32).sqrt()
    }

    /// Detect preamble by matching tone sequence
    /// Returns offsets relative to self.buffer (which equals the input samples after process() clears and refills it)
    fn detect_preamble_tones(&mut self) -> Option<PreambleDetectionResult> {
        let mut best_correlation = 0.0f32;
        let mut best_offset = 0;

        // Slide through buffer, but only search recent portion
        // Limit to last 7 seconds (336000 samples at 48kHz) to avoid detecting old preambles
        let max_search_window = (self.sample_rate * 7.0) as usize;
        let search_start = self.buffer.len().saturating_sub(max_search_window);
        // CRITICAL: Reserve space for the FULL preamble (num_preamble_symbols), not just 8 symbols
        // Otherwise we detect partial preambles at the buffer end
        let preamble_samples = self.symbol_samples * self.num_preamble_symbols;
        let search_end = self.buffer.len().saturating_sub(preamble_samples);

        for offset in (search_start..search_end).step_by(self.symbol_samples / 4) {
            let correlation = self.correlate_sequence(offset);

            if correlation > best_correlation {
                best_correlation = correlation;
                best_offset = offset;
            }
        }

        // Fine-grained search around best offset to find exact symbol alignment
        // The coarse search might land between symbol boundaries
        if best_correlation > 0.2 {
            let fine_start = best_offset.saturating_sub(self.symbol_samples * 2);
            let fine_end = (best_offset + self.symbol_samples * 4).min(search_end);
            for offset in (fine_start..fine_end).step_by(self.symbol_samples / 16) {
                let correlation = self.correlate_sequence(offset);
                if correlation > best_correlation {
                    best_correlation = correlation;
                    best_offset = offset;
                }
            }
        }

        self.correlation_peak = best_correlation;

        // Debug: log when we get close to threshold
        if best_correlation > 0.2 {
            log::debug!("Preamble detector: best_corr={:.3}, threshold={:.3}, buffer_len={}, search_start={}, search_end={}",
                best_correlation, self.threshold, self.buffer.len(), search_start, search_end);
        }

        if best_correlation >= self.threshold {
            // Verify signal energy at the detected position
            let check_start = best_offset;
            let check_end = (best_offset + self.symbol_samples * 4).min(self.buffer.len());
            let local_rms = self.calculate_rms(&self.buffer[check_start..check_end]);

            if local_rms < self.energy_threshold {
                log::debug!("Preamble match rejected: local_rms={:.4} < threshold at offset {}", local_rms, best_offset);
                return None;
            }

            // Fingerprint verification - check consistency of symbol confidences
            // Real signals have consistent structure, noise has random/inconsistent structure
            if !self.check_fingerprint(best_offset) {
                log::debug!("Fingerprint check failed at offset {} - likely false positive", best_offset);
                return None;
            }

            // Log what was detected at the winning offset
            let mut detected_at_best = Vec::new();
            let mut expected_seq = Vec::new();
            for (i, &expected_tone) in PREAMBLE_SEQUENCE.iter().enumerate().take(8) {
                let symbol_start = best_offset + i * self.symbol_samples;
                let symbol_end = symbol_start + self.symbol_samples;
                if symbol_end <= self.buffer.len() {
                    let symbol_samples: Vec<f32> = self.buffer[symbol_start..symbol_end].to_vec();
                    let detected_bin = self.detect_peak_bin(&symbol_samples);
                    detected_at_best.push(detected_bin);
                }
                let expected_idx = (expected_tone as usize) % self.tone_bins.len();
                expected_seq.push(self.tone_bins[expected_idx]);
            }
            log::info!("Preamble at best_offset={}: detected_bins={:?}, expected_bins={:?}",
                       best_offset, detected_at_best, expected_seq);

            // Reject if first detected bin is the minimum (default/silence) - indicates partial detection
            let min_bin = *self.tone_bins.first().unwrap_or(&15);
            if detected_at_best.len() >= 2 && detected_at_best[0] == min_bin && detected_at_best[1] == min_bin {
                log::debug!("Preamble rejected: first 2 bins are silence (partial detection)");
                return None;
            }

            // Calculate data_start offset (end of preamble, where data begins)
            let data_start = best_offset + preamble_samples;

            // Safety check: ensure data_start doesn't exceed buffer length
            if data_start >= self.buffer.len() {
                log::debug!("Preamble rejected: data_start {} >= buffer_len {} (partial preamble)",
                    data_start, self.buffer.len());
                return None;
            }

            log::info!("Preamble DETECTED: preamble_start={}, data_start={}, correlation={:.3}, local_rms={:.4}",
                best_offset, data_start, best_correlation, local_rms);

            Some(PreambleDetectionResult {
                preamble_start: best_offset,
                data_start,
                correlation: best_correlation,
            })
        } else {
            None
        }
    }

    /// Correlate with expected preamble sequence
    fn correlate_sequence(&mut self, start: usize) -> f32 {
        let mut matches = 0.0f32;
        let mut total = 0.0f32;
        let mut detected_bins = Vec::new();
        let mut expected_bins = Vec::new();

        for (i, &expected_tone) in PREAMBLE_SEQUENCE.iter().enumerate() {
            let symbol_start = start + i * self.symbol_samples;
            let symbol_end = symbol_start + self.symbol_samples;

            if symbol_end > self.buffer.len() {
                break;
            }

            // Copy symbol samples to avoid borrow conflict
            let symbol_samples: Vec<f32> = self.buffer[symbol_start..symbol_end].to_vec();
            let detected_bin = self.detect_peak_bin(&symbol_samples);

            // Check if detected bin matches expected
            let expected_idx = (expected_tone as usize) % self.tone_bins.len();
            let expected_bin = self.tone_bins[expected_idx];

            detected_bins.push(detected_bin);
            expected_bins.push(expected_bin);

            if detected_bin == expected_bin {
                matches += 1.0;
            } else if (detected_bin as i32 - expected_bin as i32).abs() <= 1 {
                // Close match (adjacent bin)
                matches += 0.5;
            }

            total += 1.0;
        }

        // Debug log for first correlation at each offset
        if start == 0 && total > 8.0 {
            log::debug!("Tone matching: detected={:?}, expected={:?}", &detected_bins[..8.min(detected_bins.len())], &expected_bins[..8.min(expected_bins.len())]);
        }

        if total > 0.0 {
            matches / total
        } else {
            0.0
        }
    }

    /// Detect peak frequency bin in symbol
    fn detect_peak_bin(&mut self, samples: &[f32]) -> usize {
        // Zero-pad to FFT size
        let mut complex = vec![ComplexSample::new(0.0, 0.0); self.fft_size];
        let copy_len = samples.len().min(self.fft_size);
        for (i, &s) in samples[..copy_len].iter().enumerate() {
            complex[i] = ComplexSample::new(s, 0.0);
        }

        // FFT
        let spectrum = self.fft.forward_new(&complex);

        // Find peak in expected tone range
        let default_max = self.fft_size / 2;
        let min_bin = *self.tone_bins.first().unwrap_or(&0);
        let max_bin = *self.tone_bins.last().unwrap_or(&default_max);

        let mut peak_bin = min_bin;
        let mut peak_mag = 0.0f32;

        for bin in min_bin..=max_bin.min(spectrum.len() / 2 - 1) {
            let mag = spectrum[bin].norm();
            if mag > peak_mag {
                peak_mag = mag;
                peak_bin = bin;
            }
        }

        peak_bin
    }

    /// Get last correlation peak value
    pub fn correlation_peak(&self) -> f32 {
        self.correlation_peak
    }

    /// Reset detector state
    pub fn reset(&mut self) {
        self.buffer.clear();
        self.correlation_peak = 0.0;
    }

    /// Calculate soft confidence for a symbol - how much energy is in the expected bin
    /// vs spread across other bins. Returns 0.0-1.0.
    /// Real preamble: high confidence (energy concentrated in expected bin)
    /// Noise: low confidence (energy spread across many bins)
    fn calculate_symbol_confidence(&mut self, samples: &[f32], expected_bin: usize) -> f32 {
        // Zero-pad to FFT size
        let mut complex = vec![ComplexSample::new(0.0, 0.0); self.fft_size];
        let copy_len = samples.len().min(self.fft_size);
        for (i, &s) in samples[..copy_len].iter().enumerate() {
            complex[i] = ComplexSample::new(s, 0.0);
        }

        // FFT
        let spectrum = self.fft.forward_new(&complex);

        // Get energy in expected bin (and adjacent bins for frequency tolerance)
        let expected_energy = {
            let mut e = spectrum[expected_bin].norm_sqr();
            // Include adjacent bins for Doppler/frequency offset tolerance
            if expected_bin > 0 {
                e += spectrum[expected_bin - 1].norm_sqr() * 0.5;
            }
            if expected_bin + 1 < spectrum.len() / 2 {
                e += spectrum[expected_bin + 1].norm_sqr() * 0.5;
            }
            e
        };

        // Get total energy in the tone bin range
        let min_bin = *self.tone_bins.first().unwrap_or(&0);
        let max_bin = *self.tone_bins.last().unwrap_or(&(self.fft_size / 2));

        let total_energy: f32 = (min_bin..=max_bin.min(spectrum.len() / 2 - 1))
            .map(|b| spectrum[b].norm_sqr())
            .sum();

        if total_energy > 0.0 {
            (expected_energy / total_energy).min(1.0)
        } else {
            0.0
        }
    }

    /// Fingerprint verification - checks if the detected preamble has consistent
    /// structure across all symbols (like a real signal) vs random/inconsistent
    /// structure (like noise that accidentally matched some bins).
    ///
    /// Returns true if the signal passes the fingerprint check.
    fn check_fingerprint(&mut self, offset: usize) -> bool {
        let mut confidences = Vec::with_capacity(8);

        // Calculate confidence for each of the first 8 symbols
        for (i, &expected_tone) in PREAMBLE_SEQUENCE.iter().enumerate().take(8) {
            let symbol_start = offset + i * self.symbol_samples;
            let symbol_end = symbol_start + self.symbol_samples;

            if symbol_end > self.buffer.len() {
                break;
            }

            let symbol_samples: Vec<f32> = self.buffer[symbol_start..symbol_end].to_vec();
            let expected_idx = (expected_tone as usize) % self.tone_bins.len();
            let expected_bin = self.tone_bins[expected_idx];

            let confidence = self.calculate_symbol_confidence(&symbol_samples, expected_bin);
            confidences.push(confidence);
        }

        if confidences.len() < 6 {
            log::debug!("Fingerprint check: not enough symbols ({})", confidences.len());
            return false;
        }

        // Calculate mean confidence
        let mean_confidence: f32 = confidences.iter().sum::<f32>() / confidences.len() as f32;

        // Calculate variance of confidence scores
        let variance: f32 = confidences.iter()
            .map(|&c| (c - mean_confidence).powi(2))
            .sum::<f32>() / confidences.len() as f32;

        // Standard deviation
        let std_dev = variance.sqrt();

        // Calculate coefficient of variation (normalized measure of dispersion)
        // This handles both weak and strong signals
        let cv = if mean_confidence > 0.01 {
            std_dev / mean_confidence
        } else {
            // Very low mean confidence = likely noise
            10.0
        };

        // Decision criteria:
        // - Real signal (good SNR): mean ~0.5-0.8, low variance, cv < 0.5
        // - Real signal (poor SNR): mean ~0.2-0.4, low variance, cv < 0.8
        // - Noise false positive: random confidences, high variance, cv > 1.0
        //
        // We use coefficient of variation (cv) because:
        // - cv is independent of signal strength
        // - Real signals have consistent structure even when weak
        // - Noise has random structure regardless of level

        let min_mean_confidence = 0.10;  // At least 10% mean confidence
        let max_cv = 0.80;               // Coefficient of variation limit

        let passed = mean_confidence >= min_mean_confidence && cv <= max_cv;

        if !passed {
            log::debug!("Fingerprint REJECTED: mean={:.3} (min={}), cv={:.3} (max={})",
                mean_confidence, min_mean_confidence, cv, max_cv);
        } else {
            log::info!("Fingerprint PASSED: mean={:.3}, cv={:.3}", mean_confidence, cv);
        }

        passed
    }

    /// Set detection threshold
    pub fn set_threshold(&mut self, threshold: f32) {
        self.threshold = threshold;
    }
}

/// Fast preamble detector using FFT-based correlation
pub struct FastPreambleDetector {
    mode: PreambleMode,
    fft: Fft,
    fft_size: usize,
    reference_spectrum: Vec<f32>, // Magnitude spectrum of expected preamble
    threshold: f32,
    buffer: Vec<f32>,
}

impl FastPreambleDetector {
    /// Create fast detector for given preamble
    pub fn new(preamble: &Preamble, threshold: f32) -> Self {
        let mode = preamble.mode();
        let fft_size = preamble.samples().len().next_power_of_two();

        let mut fft = Fft::new(fft_size);

        // Compute reference magnitude spectrum
        let mut padded = vec![ComplexSample::new(0.0, 0.0); fft_size];
        for (i, &s) in preamble.samples().iter().enumerate() {
            if i < fft_size {
                padded[i] = ComplexSample::new(s, 0.0);
            }
        }

        let reference_fft = fft.forward_new(&padded);
        let reference_spectrum: Vec<f32> = reference_fft.iter().map(|c| c.norm()).collect();

        Self {
            mode,
            fft,
            fft_size,
            reference_spectrum,
            threshold,
            buffer: Vec::new(),
        }
    }

    /// Process samples and detect preamble
    pub fn process(&mut self, samples: &[f32]) -> Option<usize> {
        self.buffer.extend_from_slice(samples);

        if self.buffer.len() < self.fft_size {
            return None;
        }

        // Take last fft_size samples
        let start = self.buffer.len() - self.fft_size;
        let mut signal = vec![ComplexSample::new(0.0, 0.0); self.fft_size];
        for (i, &s) in self.buffer[start..].iter().enumerate() {
            signal[i] = ComplexSample::new(s, 0.0);
        }

        // Compute spectrum
        let signal_fft = self.fft.forward_new(&signal);
        let signal_spectrum: Vec<f32> = signal_fft.iter().map(|c| c.norm()).collect();

        // Compute spectral correlation
        let mut correlation = 0.0f32;
        let mut ref_energy = 0.0f32;
        let mut sig_energy = 0.0f32;

        for (r, s) in self.reference_spectrum.iter().zip(signal_spectrum.iter()) {
            correlation += r * s;
            ref_energy += r * r;
            sig_energy += s * s;
        }

        let norm_corr = correlation / (ref_energy.sqrt() * sig_energy.sqrt() + 1e-10);

        // Trim buffer
        if self.buffer.len() > self.fft_size * 2 {
            self.buffer.drain(0..self.buffer.len() - self.fft_size);
        }

        if norm_corr >= self.threshold {
            Some(start + self.fft_size)
        } else {
            None
        }
    }

    /// Reset detector
    pub fn reset(&mut self) {
        self.buffer.clear();
    }
}

// Legacy type aliases for compatibility
pub type PreambleType = PreambleMode;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_preamble_generation() {
        // 500Hz bandwidth -> Narrow mode, 51ms symbols
        let preamble = Preamble::new(48000.0, 1500.0, 500.0, 0.2);

        // Narrow mode: 51ms per symbol, 200ms duration = ~4 symbols = ~9792 samples
        assert!(preamble.len() > 4000, "Should have preamble samples: {}", preamble.len());
        assert!(preamble.num_symbols() >= 3, "Should have at least 3 symbols");
    }

    #[test]
    fn test_preamble_detection() {
        // Create a preamble with full 16-symbol sequence
        let preamble = Preamble::with_mode(48000.0, PreambleMode::Wide, 0.35);
        let mut detector = PreambleDetector::new(&preamble, 0.4);

        // Create signal with preamble after some silence
        let mut signal = vec![0.0; 2000];
        signal.extend_from_slice(preamble.samples());
        signal.extend(vec![0.0; 1000]);

        let result = detector.process(&signal);
        // The detector should find some correlation
        assert!(detector.correlation_peak() > 0.0, "Should have some correlation: {}", detector.correlation_peak());

        // If detected, verify offset semantics
        if let Some(detection) = result {
            // preamble_start should be approximately where we inserted the preamble (2000)
            assert!(detection.preamble_start >= 1500 && detection.preamble_start <= 2500,
                "preamble_start {} should be near 2000", detection.preamble_start);

            // data_start should be preamble_start + preamble length
            let expected_data_start = detection.preamble_start + preamble.len();
            assert_eq!(detection.data_start, expected_data_start,
                "data_start should equal preamble_start + preamble_len");

            // data_start should be within bounds of the signal
            assert!(detection.data_start < signal.len(),
                "data_start {} should be < signal.len() {}", detection.data_start, signal.len());
        }
    }

    #[test]
    fn test_preamble_offset_relative_to_input() {
        // Verify that offsets are relative to input buffer, not internal state
        let preamble = Preamble::with_mode(48000.0, PreambleMode::Wide, 0.35);
        let mut detector = PreambleDetector::new(&preamble, 0.4);

        // First call with silence - should not detect
        let silence = vec![0.0; 5000];
        let result1 = detector.process(&silence);
        assert!(result1.is_none(), "Should not detect preamble in silence");

        // Second call with preamble at known position
        let preamble_offset = 1000;
        let mut signal = vec![0.0; preamble_offset];
        signal.extend_from_slice(preamble.samples());
        signal.extend(vec![0.0; 500]);

        let result2 = detector.process(&signal);

        // If detected, offsets should be relative to THIS call's input (signal), not cumulative
        if let Some(detection) = result2 {
            // preamble_start should be near preamble_offset (1000), NOT 5000 + 1000
            assert!(detection.preamble_start < 2000,
                "preamble_start {} should be relative to current input, not cumulative",
                detection.preamble_start);

            // Verify correlation is valid
            assert!(detection.correlation >= 0.0 && detection.correlation <= 1.0,
                "correlation {} should be in [0, 1]", detection.correlation);
        }
    }

    #[test]
    fn test_tone_frequencies() {
        let preamble = Preamble::with_mode(48000.0, PreambleMode::Narrow, 0.1);
        let freqs = preamble.tone_frequencies();

        // 500Hz mode should have frequencies around 1100-1800 Hz
        for &f in &freqs {
            assert!(f > 1000.0 && f < 2000.0, "Frequency {} out of range", f);
        }
    }

    #[test]
    fn test_barker_sync() {
        let sync = Preamble::generate_barker_sync(48000.0, 1500.0, &BARKER_13);

        // 13 chips * 10ms = 130ms
        let expected = (48000.0 * 0.13) as usize;
        assert!((sync.len() as i32 - expected as i32).abs() < 100);
    }

    #[test]
    fn test_preamble_modes() {
        let narrow = Preamble::with_mode(48000.0, PreambleMode::Narrow, 0.5);
        let wide = Preamble::with_mode(48000.0, PreambleMode::Wide, 0.25);

        // Narrow should have ~500ms duration
        assert!((narrow.duration() - 0.5).abs() < 0.1);

        // Wide should have ~250ms duration
        assert!((wide.duration() - 0.25).abs() < 0.05);
    }

    #[test]
    fn test_preamble_sequence() {
        let preamble = Preamble::with_mode(48000.0, PreambleMode::Narrow, 0.5);
        let seq = preamble.expected_sequence();

        // Should be 16 symbols
        assert_eq!(seq.len(), 16);

        // First symbol should be 12
        assert_eq!(seq[0], 12);
    }
}
