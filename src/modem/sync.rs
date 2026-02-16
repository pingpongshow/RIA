//! Symbol timing and carrier synchronization
//!
//! Implements timing recovery and frequency offset correction

use std::f32::consts::PI;
use crate::dsp::{ComplexSample, wrap_phase};

/// Symbol timing recovery using Gardner algorithm
pub struct TimingRecovery {
    samples_per_symbol: f32,
    mu: f32,                    // Fractional sample offset
    mu_gain: f32,               // Loop gain
    omega: f32,                 // Samples per symbol (tracked)
    omega_gain: f32,            // Omega loop gain
    omega_mid: f32,             // Nominal samples per symbol
    omega_limit: f32,           // Max deviation from nominal
    last_sample: f32,
    last_last_sample: f32,
}

impl TimingRecovery {
    /// Create timing recovery for given samples per symbol
    pub fn new(samples_per_symbol: f32) -> Self {
        Self {
            samples_per_symbol,
            mu: 0.5,
            mu_gain: 0.05,
            omega: samples_per_symbol,
            omega_gain: 0.001,
            omega_mid: samples_per_symbol,
            omega_limit: samples_per_symbol * 0.1,
            last_sample: 0.0,
            last_last_sample: 0.0,
        }
    }

    /// Set loop gains
    pub fn set_gains(&mut self, mu_gain: f32, omega_gain: f32) {
        self.mu_gain = mu_gain;
        self.omega_gain = omega_gain;
    }

    /// Process samples and output synchronized symbols
    pub fn process(&mut self, samples: &[f32]) -> Vec<f32> {
        let mut output = Vec::new();
        let mut idx = 0.0f32;

        while (idx as usize) < samples.len().saturating_sub(2) {
            let i = idx as usize;
            let frac = idx - i as f32;

            // Linear interpolation
            let sample = samples[i] * (1.0 - frac) + samples[i + 1] * frac;

            // Gardner timing error detector
            let mid_idx = idx - self.omega / 2.0;
            let mid_sample = if mid_idx >= 0.0 && (mid_idx as usize) < samples.len() - 1 {
                let mi = mid_idx as usize;
                let mf = mid_idx - mi as f32;
                samples[mi] * (1.0 - mf) + samples[mi + 1] * mf
            } else {
                0.0
            };

            // Timing error
            let ted = mid_sample * (self.last_sample - sample);

            // Update loop
            self.mu += self.mu_gain * ted;
            self.omega += self.omega_gain * ted;

            // Constrain omega
            self.omega = self.omega.clamp(
                self.omega_mid - self.omega_limit,
                self.omega_mid + self.omega_limit,
            );

            // Constrain mu
            if self.mu >= 1.0 {
                self.mu -= 1.0;
                idx += 1.0;
            } else if self.mu < 0.0 {
                self.mu += 1.0;
                idx -= 1.0;
            }

            output.push(sample);

            self.last_last_sample = self.last_sample;
            self.last_sample = sample;

            idx += self.omega;
        }

        output
    }

    /// Get current fractional offset
    pub fn mu(&self) -> f32 {
        self.mu
    }

    /// Get current samples per symbol estimate
    pub fn omega(&self) -> f32 {
        self.omega
    }

    /// Reset state
    pub fn reset(&mut self) {
        self.mu = 0.5;
        self.omega = self.omega_mid;
        self.last_sample = 0.0;
        self.last_last_sample = 0.0;
    }
}

/// Complex symbol synchronization
pub struct SymbolSync {
    timing: TimingRecovery,
    phase: f32,
    frequency: f32,
    alpha: f32,           // Phase loop gain
    beta: f32,            // Frequency loop gain
    last_symbol: ComplexSample,
}

impl SymbolSync {
    /// Create symbol synchronizer
    pub fn new(samples_per_symbol: f32) -> Self {
        Self {
            timing: TimingRecovery::new(samples_per_symbol),
            phase: 0.0,
            frequency: 0.0,
            alpha: 0.1,
            beta: 0.01,
            last_symbol: ComplexSample::new(0.0, 0.0),
        }
    }

    /// Set loop bandwidths
    pub fn set_loop_bandwidth(&mut self, bw: f32) {
        // Convert bandwidth to gains
        let damp = 0.707; // Critical damping
        let omega_n = 2.0 * PI * bw;

        self.alpha = 2.0 * damp * omega_n;
        self.beta = omega_n * omega_n;
    }

    /// Process complex samples
    pub fn process(&mut self, samples: &[ComplexSample]) -> Vec<ComplexSample> {
        // First do timing recovery on magnitude
        let magnitudes: Vec<f32> = samples.iter().map(|s| s.norm()).collect();
        let _timing_out = self.timing.process(&magnitudes);

        // Process with carrier recovery
        let mut output = Vec::new();
        let mut idx = 0.0f32;

        while (idx as usize) < samples.len().saturating_sub(2) {
            let i = idx as usize;
            let frac = idx - i as f32;

            // Interpolate complex sample
            let sample = ComplexSample::new(
                samples[i].re * (1.0 - frac) + samples[i + 1].re * frac,
                samples[i].im * (1.0 - frac) + samples[i + 1].im * frac,
            );

            // Apply phase correction
            let correction = ComplexSample::new(self.phase.cos(), -self.phase.sin());
            let corrected = sample * correction;

            // Phase error detector (decision-directed)
            let decision = self.make_decision(corrected);
            let phase_error = (corrected * decision.conj()).arg();

            // Update phase and frequency
            self.frequency += self.beta * phase_error;
            self.phase = wrap_phase(self.phase + self.alpha * phase_error + self.frequency);

            output.push(corrected);
            self.last_symbol = corrected;

            idx += self.timing.omega;
        }

        output
    }

    /// Make hard decision for phase error detector
    fn make_decision(&self, sample: ComplexSample) -> ComplexSample {
        // QPSK decision regions
        let re = if sample.re >= 0.0 { 1.0 } else { -1.0 };
        let im = if sample.im >= 0.0 { 1.0 } else { -1.0 };
        ComplexSample::new(re, im) * 0.7071
    }

    /// Get current phase estimate
    pub fn phase(&self) -> f32 {
        self.phase
    }

    /// Get current frequency offset estimate
    pub fn frequency(&self) -> f32 {
        self.frequency
    }

    /// Reset synchronizer
    pub fn reset(&mut self) {
        self.timing.reset();
        self.phase = 0.0;
        self.frequency = 0.0;
        self.last_symbol = ComplexSample::new(0.0, 0.0);
    }
}

/// Frame synchronization using correlation
pub struct FrameSync {
    sync_pattern: Vec<ComplexSample>,
    threshold: f32,
    buffer: Vec<ComplexSample>,
}

impl FrameSync {
    /// Create frame synchronizer with given sync pattern
    pub fn new(sync_pattern: Vec<ComplexSample>, threshold: f32) -> Self {
        Self {
            sync_pattern,
            threshold,
            buffer: Vec::new(),
        }
    }

    /// Create with BPSK sync pattern
    pub fn with_bpsk_pattern(pattern: &[i8], threshold: f32) -> Self {
        let sync_pattern: Vec<ComplexSample> = pattern.iter()
            .map(|&b| ComplexSample::new(b as f32, 0.0))
            .collect();

        Self::new(sync_pattern, threshold)
    }

    /// Process samples and detect sync
    /// Returns Some(offset) if sync detected
    pub fn process(&mut self, samples: &[ComplexSample]) -> Option<usize> {
        self.buffer.extend_from_slice(samples);

        if self.buffer.len() < self.sync_pattern.len() * 2 {
            return None;
        }

        let pattern_len = self.sync_pattern.len();
        let search_len = self.buffer.len() - pattern_len;

        let mut max_corr = 0.0f32;
        let mut max_idx = 0;

        for i in 0..search_len {
            let mut corr = ComplexSample::new(0.0, 0.0);
            let mut energy = 0.0f32;

            for j in 0..pattern_len {
                corr += self.buffer[i + j] * self.sync_pattern[j].conj();
                energy += self.buffer[i + j].norm_sqr();
            }

            let norm_corr = corr.norm() / (energy.sqrt() + 1e-10);

            if norm_corr > max_corr {
                max_corr = norm_corr;
                max_idx = i;
            }
        }

        // Trim buffer
        if self.buffer.len() > pattern_len * 3 {
            self.buffer.drain(0..self.buffer.len() - pattern_len * 2);
        }

        if max_corr >= self.threshold {
            Some(max_idx + pattern_len)
        } else {
            None
        }
    }

    /// Reset synchronizer
    pub fn reset(&mut self) {
        self.buffer.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_timing_recovery() {
        let mut tr = TimingRecovery::new(10.0);

        // Generate simple test signal
        let samples: Vec<f32> = (0..200)
            .map(|i| ((i as f32 * PI / 5.0).sin()))
            .collect();

        let output = tr.process(&samples);
        assert!(!output.is_empty());
    }

    #[test]
    fn test_frame_sync() {
        let pattern = vec![1, 1, 1, -1, -1, -1, 1, -1];
        let mut sync = FrameSync::with_bpsk_pattern(&pattern, 0.7);

        // Create test signal with sync pattern
        let mut signal: Vec<ComplexSample> = (0..50)
            .map(|_| ComplexSample::new(0.1, 0.0))
            .collect();

        for &p in &pattern {
            signal.push(ComplexSample::new(p as f32, 0.0));
        }

        signal.extend((0..50).map(|_| ComplexSample::new(0.1, 0.0)));

        let result = sync.process(&signal);
        assert!(result.is_some());
    }
}
