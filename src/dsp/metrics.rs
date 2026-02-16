//! Signal metrics and measurements

/// Minimum power level for dB calculations (avoids -infinity)
/// Corresponds to approximately -120 dB
const MIN_POWER_FOR_DB: f32 = 1e-12;

/// Minimum RMS level for dB calculations (avoids -infinity)
/// Corresponds to approximately -120 dB
const MIN_RMS_FOR_DB: f32 = 1e-6;

/// Signal quality metrics
#[derive(Debug, Clone, Default)]
pub struct SignalMetrics {
    pub snr_db: f32,
    pub signal_power: f32,
    pub noise_power: f32,
    pub peak_level: f32,
    pub rms_level: f32,
    pub crest_factor: f32,
}

/// Calculate RMS level of samples
/// Treats NaN and infinite values as 0.0, returns 0.0 for empty input
pub fn calculate_rms(samples: &[f32]) -> f32 {
    if samples.is_empty() {
        return 0.0;
    }

    let sum_sq: f32 = samples.iter()
        .map(|s| {
            let sq = s * s;
            // Treat NaN and infinity as 0.0 to avoid propagation
            if sq.is_finite() { sq } else { 0.0 }
        })
        .sum();

    let mean_sq = sum_sq / samples.len() as f32;
    let rms = mean_sq.sqrt();

    // Final guard against non-finite result
    if rms.is_finite() { rms } else { 0.0 }
}

/// Calculate peak level of samples
/// Handles NaN and infinite values by treating them as 0.0 (skipping them)
pub fn calculate_peak(samples: &[f32]) -> f32 {
    samples.iter()
        .map(|s| {
            let abs = s.abs();
            // Treat NaN and infinity as 0.0 to avoid propagation
            if abs.is_finite() { abs } else { 0.0 }
        })
        .fold(0.0_f32, f32::max)
}

/// Calculate SNR from signal and noise power
/// Returns 0.0 for invalid inputs, clamps result to reasonable dB range
pub fn calculate_snr(signal_power: f32, noise_power: f32) -> f32 {
    // Check for invalid inputs (including NaN)
    if noise_power < MIN_POWER_FOR_DB || signal_power < MIN_POWER_FOR_DB
        || noise_power.is_nan() || signal_power.is_nan()
    {
        return 0.0;
    }

    let ratio = signal_power / noise_power;
    // Guard against ratio being 0, NaN, or infinity
    if ratio <= 0.0 || !ratio.is_finite() {
        return 0.0;
    }

    let snr = 10.0 * ratio.log10();
    // Clamp to reasonable range (-60 to +60 dB)
    snr.clamp(-60.0, 60.0)
}

/// Calculate SNR from samples using noise floor estimation
/// Returns 0.0 for invalid inputs, uses minimum RMS to avoid -infinity
pub fn estimate_snr(samples: &[f32], noise_floor_db: f32) -> f32 {
    let rms = calculate_rms(samples);

    // Guard against zero, very small, or NaN RMS values
    if rms < MIN_RMS_FOR_DB || rms.is_nan() || !noise_floor_db.is_finite() {
        return 0.0;
    }

    let signal_db = 20.0 * rms.log10();
    let snr = signal_db - noise_floor_db;

    // Return 0 if result is not finite
    if snr.is_finite() { snr } else { 0.0 }
}

/// Estimate noise floor from quiet period
/// Uses minimum RMS to avoid -infinity, returns -120 dB as minimum
pub fn estimate_noise_floor(samples: &[f32]) -> f32 {
    let rms = calculate_rms(samples);

    // Guard against zero, very small, or NaN RMS values
    if rms < MIN_RMS_FOR_DB || rms.is_nan() {
        return -120.0; // Minimum noise floor (very quiet)
    }

    let db = 20.0 * rms.log10();
    // Ensure finite result, clamp to reasonable range
    if db.is_finite() {
        db.clamp(-120.0, 0.0)
    } else {
        -120.0
    }
}

/// Calculate all signal metrics
pub fn calculate_metrics(samples: &[f32], noise_floor_db: f32) -> SignalMetrics {
    let rms = calculate_rms(samples);
    let peak = calculate_peak(samples);

    let signal_power = rms * rms;
    let noise_power = 10.0_f32.powf(noise_floor_db / 10.0);
    let snr = calculate_snr(signal_power, noise_power);

    let crest_factor = if rms > 0.0 { peak / rms } else { 0.0 };

    SignalMetrics {
        snr_db: snr,
        signal_power,
        noise_power,
        peak_level: peak,
        rms_level: rms,
        crest_factor,
    }
}

/// EVM (Error Vector Magnitude) calculator for constellation analysis
pub struct EvmCalculator {
    sum_error_sq: f64,
    sum_ref_sq: f64,
    count: usize,
}

impl EvmCalculator {
    pub fn new() -> Self {
        Self {
            sum_error_sq: 0.0,
            sum_ref_sq: 0.0,
            count: 0,
        }
    }

    /// Add a measurement
    pub fn add(&mut self, received: (f32, f32), ideal: (f32, f32)) {
        let error_i = received.0 - ideal.0;
        let error_q = received.1 - ideal.1;
        let error_sq = (error_i * error_i + error_q * error_q) as f64;
        let ref_sq = (ideal.0 * ideal.0 + ideal.1 * ideal.1) as f64;

        self.sum_error_sq += error_sq;
        self.sum_ref_sq += ref_sq;
        self.count += 1;
    }

    /// Get EVM as percentage
    pub fn evm_percent(&self) -> f32 {
        if self.count == 0 || self.sum_ref_sq == 0.0 {
            return 0.0;
        }
        (100.0 * (self.sum_error_sq / self.sum_ref_sq).sqrt()) as f32
    }

    /// Get EVM in dB
    pub fn evm_db(&self) -> f32 {
        let percent = self.evm_percent();
        if percent > 0.0 {
            20.0 * (percent / 100.0).log10()
        } else {
            -60.0
        }
    }

    /// Reset calculator
    pub fn reset(&mut self) {
        self.sum_error_sq = 0.0;
        self.sum_ref_sq = 0.0;
        self.count = 0;
    }
}

impl Default for EvmCalculator {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_rms() {
        let samples = vec![1.0, -1.0, 1.0, -1.0];
        let rms = calculate_rms(&samples);
        assert!((rms - 1.0).abs() < 0.001);
    }

    #[test]
    fn test_rms_empty() {
        let samples: Vec<f32> = vec![];
        let rms = calculate_rms(&samples);
        assert_eq!(rms, 0.0);
    }

    #[test]
    fn test_rms_with_nan() {
        let samples = vec![1.0, f32::NAN, 1.0, -1.0];
        let rms = calculate_rms(&samples);
        // NaN treated as 0.0, so sum_sq = 1 + 0 + 1 + 1 = 3, mean = 0.75, sqrt(0.75) ≈ 0.866
        assert!(rms.is_finite(), "RMS should be finite even with NaN input");
        assert!(rms > 0.0, "RMS should be positive");
    }

    #[test]
    fn test_rms_all_nan() {
        let samples = vec![f32::NAN, f32::NAN, f32::NAN];
        let rms = calculate_rms(&samples);
        assert_eq!(rms, 0.0, "RMS of all NaN should be 0.0");
    }

    #[test]
    fn test_snr() {
        let snr = calculate_snr(100.0, 1.0);
        assert!((snr - 20.0).abs() < 0.1); // 100:1 = 20 dB
    }

    #[test]
    fn test_snr_zero_noise() {
        let snr = calculate_snr(100.0, 0.0);
        assert_eq!(snr, 0.0, "SNR with zero noise should be 0.0");
    }

    #[test]
    fn test_snr_zero_signal() {
        let snr = calculate_snr(0.0, 1.0);
        assert_eq!(snr, 0.0, "SNR with zero signal should be 0.0");
    }

    #[test]
    fn test_snr_negative_power() {
        let snr = calculate_snr(-1.0, 1.0);
        assert_eq!(snr, 0.0, "SNR with negative power should be 0.0");
    }

    #[test]
    fn test_snr_nan_input() {
        assert_eq!(calculate_snr(f32::NAN, 1.0), 0.0, "SNR with NaN signal should be 0.0");
        assert_eq!(calculate_snr(1.0, f32::NAN), 0.0, "SNR with NaN noise should be 0.0");
    }

    #[test]
    fn test_snr_very_small_values() {
        let snr = calculate_snr(1e-15, 1e-15);
        assert_eq!(snr, 0.0, "SNR with very small values should be 0.0");
    }

    #[test]
    fn test_peak() {
        let samples = vec![0.5, -0.8, 0.3, -0.2];
        let peak = calculate_peak(&samples);
        assert!((peak - 0.8).abs() < 0.001);
    }

    #[test]
    fn test_peak_with_nan() {
        let samples = vec![0.5, f32::NAN, -0.8, 0.3];
        let peak = calculate_peak(&samples);
        assert!(peak.is_finite(), "Peak should be finite even with NaN input");
        assert!((peak - 0.8).abs() < 0.001, "Peak should ignore NaN");
    }

    #[test]
    fn test_peak_all_nan() {
        let samples = vec![f32::NAN, f32::NAN];
        let peak = calculate_peak(&samples);
        assert_eq!(peak, 0.0, "Peak of all NaN should be 0.0");
    }

    #[test]
    fn test_peak_empty() {
        let samples: Vec<f32> = vec![];
        let peak = calculate_peak(&samples);
        assert_eq!(peak, 0.0);
    }

    #[test]
    fn test_estimate_snr_zero_rms() {
        let samples = vec![0.0, 0.0, 0.0];
        let snr = estimate_snr(&samples, -60.0);
        assert_eq!(snr, 0.0, "SNR estimate with zero RMS should be 0.0");
    }

    #[test]
    fn test_estimate_snr_nan_noise_floor() {
        let samples = vec![1.0, 1.0, 1.0];
        let snr = estimate_snr(&samples, f32::NAN);
        assert_eq!(snr, 0.0, "SNR estimate with NaN noise floor should be 0.0");
    }

    #[test]
    fn test_noise_floor_zero_input() {
        let samples = vec![0.0, 0.0, 0.0];
        let floor = estimate_noise_floor(&samples);
        assert_eq!(floor, -120.0, "Noise floor of silence should be -120 dB");
    }

    #[test]
    fn test_noise_floor_normal() {
        let samples = vec![0.1, -0.1, 0.1, -0.1];
        let floor = estimate_noise_floor(&samples);
        assert!(floor.is_finite(), "Noise floor should be finite");
        assert!(floor < 0.0, "Noise floor should be negative dB");
    }

    #[test]
    fn test_all_results_finite() {
        // Test that all functions return finite values for various edge cases
        let edge_cases: Vec<Vec<f32>> = vec![
            vec![],
            vec![0.0],
            vec![f32::NAN],
            vec![f32::INFINITY],
            vec![f32::NEG_INFINITY],
            vec![1e-40, 1e-40],
            vec![1e38, 1e38],
        ];

        for samples in &edge_cases {
            let rms = calculate_rms(samples);
            let peak = calculate_peak(samples);
            let floor = estimate_noise_floor(samples);
            let snr = estimate_snr(samples, -60.0);

            assert!(rms.is_finite(), "RMS should be finite for {:?}", samples);
            assert!(peak.is_finite(), "Peak should be finite for {:?}", samples);
            assert!(floor.is_finite(), "Noise floor should be finite for {:?}", samples);
            assert!(snr.is_finite(), "SNR should be finite for {:?}", samples);
        }
    }
}
