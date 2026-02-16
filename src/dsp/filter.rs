//! Digital filters (FIR and IIR)

use std::f32::consts::PI;

/// Filter type
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FilterType {
    LowPass,
    HighPass,
    BandPass,
    BandStop,
}

/// Generic filter trait
pub trait Filter {
    fn process(&mut self, sample: f32) -> f32;
    fn process_block(&mut self, samples: &mut [f32]) {
        for sample in samples.iter_mut() {
            *sample = self.process(*sample);
        }
    }
    fn reset(&mut self);
}

/// FIR (Finite Impulse Response) filter
pub struct FirFilter {
    coefficients: Vec<f32>,
    delay_line: Vec<f32>,
    delay_index: usize,
}

impl FirFilter {
    /// Create a new FIR filter with given coefficients
    pub fn new(coefficients: Vec<f32>) -> Self {
        let len = coefficients.len();
        Self {
            coefficients,
            delay_line: vec![0.0; len],
            delay_index: 0,
        }
    }

    /// Design a low-pass FIR filter using windowed sinc method
    pub fn design_lowpass(cutoff: f32, sample_rate: f32, num_taps: usize) -> Self {
        let normalized_cutoff = cutoff / sample_rate;
        let mut coefficients = vec![0.0; num_taps];
        let m = (num_taps - 1) as f32 / 2.0;

        for i in 0..num_taps {
            let n = i as f32 - m;
            if n.abs() < 1e-10 {
                coefficients[i] = 2.0 * normalized_cutoff;
            } else {
                coefficients[i] = (2.0 * PI * normalized_cutoff * n).sin() / (PI * n);
            }

            // Apply Hamming window
            let window = 0.54 - 0.46 * (2.0 * PI * i as f32 / (num_taps - 1) as f32).cos();
            coefficients[i] *= window;
        }

        // Normalize
        let sum: f32 = coefficients.iter().sum();
        for c in coefficients.iter_mut() {
            *c /= sum;
        }

        Self::new(coefficients)
    }

    /// Design a high-pass FIR filter
    pub fn design_highpass(cutoff: f32, sample_rate: f32, num_taps: usize) -> Self {
        let mut filter = Self::design_lowpass(cutoff, sample_rate, num_taps);

        // Spectral inversion
        let m = (num_taps - 1) / 2;
        for (i, c) in filter.coefficients.iter_mut().enumerate() {
            *c = -*c;
            if i == m {
                *c += 1.0;
            }
        }

        filter
    }

    /// Design a band-pass FIR filter
    pub fn design_bandpass(low: f32, high: f32, sample_rate: f32, num_taps: usize) -> Self {
        let lp = Self::design_lowpass(high, sample_rate, num_taps);
        let hp = Self::design_highpass(low, sample_rate, num_taps);

        // Convolve the two filters
        let mut coefficients = vec![0.0; num_taps];
        for i in 0..num_taps {
            for j in 0..num_taps {
                if i + j < num_taps {
                    coefficients[i + j] += lp.coefficients[i] * hp.coefficients[j];
                }
            }
        }

        // Simple alternative: combine low-pass and high-pass
        let mut filter = lp;
        for (i, c) in filter.coefficients.iter_mut().enumerate() {
            *c += hp.coefficients[i];
        }

        filter
    }

    /// Get filter order (number of taps)
    pub fn order(&self) -> usize {
        self.coefficients.len()
    }
}

impl Filter for FirFilter {
    fn process(&mut self, sample: f32) -> f32 {
        // Add sample to delay line
        self.delay_line[self.delay_index] = sample;

        // Compute output
        let mut output = 0.0;
        let mut j = self.delay_index;

        for coef in &self.coefficients {
            output += coef * self.delay_line[j];
            if j == 0 {
                j = self.delay_line.len() - 1;
            } else {
                j -= 1;
            }
        }

        // Update delay index
        self.delay_index = (self.delay_index + 1) % self.delay_line.len();

        output
    }

    fn reset(&mut self) {
        self.delay_line.fill(0.0);
        self.delay_index = 0;
    }
}

/// IIR (Infinite Impulse Response) biquad filter
pub struct IirFilter {
    // Coefficients
    b0: f32,
    b1: f32,
    b2: f32,
    a1: f32,
    a2: f32,

    // State
    x1: f32,
    x2: f32,
    y1: f32,
    y2: f32,
}

impl IirFilter {
    /// Create a new IIR filter with given coefficients
    pub fn new(b0: f32, b1: f32, b2: f32, a1: f32, a2: f32) -> Self {
        Self {
            b0, b1, b2, a1, a2,
            x1: 0.0, x2: 0.0,
            y1: 0.0, y2: 0.0,
        }
    }

    /// Design a 2nd-order low-pass Butterworth filter
    pub fn design_lowpass(cutoff: f32, sample_rate: f32) -> Self {
        let omega = 2.0 * PI * cutoff / sample_rate;
        let cos_omega = omega.cos();
        let sin_omega = omega.sin();
        let alpha = sin_omega / (2.0 * 0.7071); // Q = 0.7071 for Butterworth

        let b0 = (1.0 - cos_omega) / 2.0;
        let b1 = 1.0 - cos_omega;
        let b2 = (1.0 - cos_omega) / 2.0;
        let a0 = 1.0 + alpha;
        let a1 = -2.0 * cos_omega;
        let a2 = 1.0 - alpha;

        Self::new(b0 / a0, b1 / a0, b2 / a0, a1 / a0, a2 / a0)
    }

    /// Design a 2nd-order high-pass Butterworth filter
    pub fn design_highpass(cutoff: f32, sample_rate: f32) -> Self {
        let omega = 2.0 * PI * cutoff / sample_rate;
        let cos_omega = omega.cos();
        let sin_omega = omega.sin();
        let alpha = sin_omega / (2.0 * 0.7071);

        let b0 = (1.0 + cos_omega) / 2.0;
        let b1 = -(1.0 + cos_omega);
        let b2 = (1.0 + cos_omega) / 2.0;
        let a0 = 1.0 + alpha;
        let a1 = -2.0 * cos_omega;
        let a2 = 1.0 - alpha;

        Self::new(b0 / a0, b1 / a0, b2 / a0, a1 / a0, a2 / a0)
    }

    /// Design a 2nd-order band-pass filter
    pub fn design_bandpass(center: f32, bandwidth: f32, sample_rate: f32) -> Self {
        let omega = 2.0 * PI * center / sample_rate;
        let cos_omega = omega.cos();
        let sin_omega = omega.sin();
        let alpha = sin_omega * (bandwidth / sample_rate / 2.0).sinh();

        let b0 = alpha;
        let b1 = 0.0;
        let b2 = -alpha;
        let a0 = 1.0 + alpha;
        let a1 = -2.0 * cos_omega;
        let a2 = 1.0 - alpha;

        Self::new(b0 / a0, b1 / a0, b2 / a0, a1 / a0, a2 / a0)
    }

    /// Design a notch (band-reject) filter
    pub fn design_notch(center: f32, bandwidth: f32, sample_rate: f32) -> Self {
        let omega = 2.0 * PI * center / sample_rate;
        let cos_omega = omega.cos();
        let sin_omega = omega.sin();
        let alpha = sin_omega * (bandwidth / sample_rate / 2.0).sinh();

        let b0 = 1.0;
        let b1 = -2.0 * cos_omega;
        let b2 = 1.0;
        let a0 = 1.0 + alpha;
        let a1 = -2.0 * cos_omega;
        let a2 = 1.0 - alpha;

        Self::new(b0 / a0, b1 / a0, b2 / a0, a1 / a0, a2 / a0)
    }
}

impl Filter for IirFilter {
    fn process(&mut self, sample: f32) -> f32 {
        let output = self.b0 * sample
            + self.b1 * self.x1
            + self.b2 * self.x2
            - self.a1 * self.y1
            - self.a2 * self.y2;

        self.x2 = self.x1;
        self.x1 = sample;
        self.y2 = self.y1;
        self.y1 = output;

        output
    }

    fn reset(&mut self) {
        self.x1 = 0.0;
        self.x2 = 0.0;
        self.y1 = 0.0;
        self.y2 = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_fir_lowpass() {
        let mut filter = FirFilter::design_lowpass(1000.0, 48000.0, 31);

        // DC should pass through
        for _ in 0..100 {
            filter.process(1.0);
        }
        let output = filter.process(1.0);
        assert!((output - 1.0).abs() < 0.1);
    }

    #[test]
    fn test_iir_lowpass() {
        let mut filter = IirFilter::design_lowpass(1000.0, 48000.0);

        // DC should pass through
        for _ in 0..100 {
            filter.process(1.0);
        }
        let output = filter.process(1.0);
        assert!((output - 1.0).abs() < 0.01);
    }
}
