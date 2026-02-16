//! Automatic Gain Control (AGC)

/// Maximum gain change per sample in dB (prevents instability)
const MAX_GAIN_CHANGE_DB_PER_SAMPLE: f32 = 0.1;

/// AGC processor for maintaining consistent signal levels
pub struct Agc {
    target_level: f32,
    attack_rate: f32,
    decay_rate: f32,
    min_gain: f32,
    max_gain: f32,
    current_gain: f32,
    frozen: bool,
    saved_attack_rate: f32,
    saved_decay_rate: f32,
    /// Maximum gain change ratio per sample (derived from dB limit)
    max_change_ratio: f32,
}

impl Agc {
    /// Create a new AGC with default settings
    pub fn new() -> Self {
        // 0.1 dB per sample = 10^(0.1/20) ≈ 1.0116 ratio
        let max_change_ratio = 10.0_f32.powf(MAX_GAIN_CHANGE_DB_PER_SAMPLE / 20.0);
        Self {
            target_level: 0.5,
            attack_rate: 0.01,
            decay_rate: 0.0001,
            min_gain: 0.001,
            max_gain: 1000.0,
            current_gain: 1.0,
            frozen: false,
            saved_attack_rate: 0.01,
            saved_decay_rate: 0.0001,
            max_change_ratio,
        }
    }

    /// Set target output level (0.0 to 1.0)
    pub fn set_target(&mut self, level: f32) {
        self.target_level = level.clamp(0.01, 1.0);
    }

    /// Set attack rate (how fast gain decreases for loud signals)
    pub fn set_attack(&mut self, rate: f32) {
        self.attack_rate = rate.clamp(0.0001, 0.1);
    }

    /// Set decay rate (how fast gain increases for quiet signals)
    pub fn set_decay(&mut self, rate: f32) {
        self.decay_rate = rate.clamp(0.00001, 0.01);
    }

    /// Process a single sample
    pub fn process(&mut self, sample: f32) -> f32 {
        let output = sample * self.current_gain;
        let abs_output = output.abs();

        // Calculate desired gain adjustment
        let gain_factor = if abs_output > self.target_level {
            // Signal too loud - decrease gain quickly
            1.0 - self.attack_rate
        } else {
            // Signal too quiet - increase gain slowly
            1.0 + self.decay_rate
        };

        // Rate-limit the gain change to prevent instability
        // Clamp factor to [1/max_ratio, max_ratio] range
        let limited_factor = gain_factor.clamp(
            1.0 / self.max_change_ratio,
            self.max_change_ratio,
        );

        self.current_gain *= limited_factor;

        // Clamp gain to absolute limits
        self.current_gain = self.current_gain.clamp(self.min_gain, self.max_gain);

        output.clamp(-1.0, 1.0)
    }

    /// Process a block of samples
    pub fn process_block(&mut self, samples: &mut [f32]) {
        for sample in samples.iter_mut() {
            *sample = self.process(*sample);
        }
    }

    /// Get current gain value
    pub fn gain(&self) -> f32 {
        self.current_gain
    }

    /// Get current gain in dB
    pub fn gain_db(&self) -> f32 {
        20.0 * self.current_gain.log10()
    }

    /// Reset AGC state
    pub fn reset(&mut self) {
        self.current_gain = 1.0;
    }

    /// Freeze gain at current value (stops gain adjustments)
    pub fn freeze(&mut self) {
        if !self.frozen {
            self.saved_attack_rate = self.attack_rate;
            self.saved_decay_rate = self.decay_rate;
            self.attack_rate = 0.0;
            self.decay_rate = 0.0;
            self.frozen = true;
        }
    }

    /// Unfreeze and resume normal gain adjustments
    pub fn unfreeze(&mut self) {
        if self.frozen {
            self.attack_rate = self.saved_attack_rate;
            self.decay_rate = self.saved_decay_rate;
            self.frozen = false;
        }
    }

    /// Check if AGC is frozen
    pub fn is_frozen(&self) -> bool {
        self.frozen
    }
}

impl Default for Agc {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_agc_loud_signal() {
        let mut agc = Agc::new();
        agc.set_target(0.5);

        // Process loud signal
        for _ in 0..1000 {
            let _ = agc.process(0.9);
        }

        // Gain should have decreased
        assert!(agc.gain() < 1.0);
    }

    #[test]
    fn test_agc_quiet_signal() {
        let mut agc = Agc::new();
        agc.set_target(0.5);

        // Process quiet signal
        for _ in 0..10000 {
            let _ = agc.process(0.01);
        }

        // Gain should have increased
        assert!(agc.gain() > 1.0);
    }

    #[test]
    fn test_agc_stability_rate_limited() {
        let mut agc = Agc::new();
        agc.set_target(0.5);

        // Set extreme attack rate that would cause instability without rate limiting
        agc.set_attack(0.1); // 10% per sample - would halve gain in ~7 samples

        let initial_gain = agc.gain();

        // Process a few loud samples
        for _ in 0..10 {
            let _ = agc.process(0.9);
        }

        // With rate limiting at 0.1 dB/sample, max change in 10 samples is 1 dB
        // 1 dB = factor of ~0.89, so gain should be >= initial * 0.89
        let min_expected = initial_gain * 0.89;
        assert!(
            agc.gain() >= min_expected,
            "Gain changed too fast: {} < {}",
            agc.gain(),
            min_expected
        );
    }

    #[test]
    fn test_agc_no_oscillation() {
        let mut agc = Agc::new();
        agc.set_target(0.5);

        // Alternating loud/quiet samples (adversarial pattern)
        let mut gains = Vec::new();
        for i in 0..1000 {
            let sample = if i % 2 == 0 { 0.9 } else { 0.1 };
            let _ = agc.process(sample);
            gains.push(agc.gain());
        }

        // Check that gain doesn't oscillate wildly
        // Compute variance of gain changes
        let mut max_change_ratio = 1.0_f32;
        for i in 1..gains.len() {
            let ratio = (gains[i] / gains[i - 1]).max(gains[i - 1] / gains[i]);
            max_change_ratio = max_change_ratio.max(ratio);
        }

        // Max change should be bounded by rate limit (~1.012 for 0.1 dB)
        assert!(
            max_change_ratio < 1.02,
            "Gain oscillating too much: max ratio {}",
            max_change_ratio
        );
    }
}
