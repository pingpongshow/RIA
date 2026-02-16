//! Window functions for spectral analysis

use std::f32::consts::PI;

/// Window function type
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WindowType {
    Rectangular,
    Hann,
    Hamming,
    Blackman,
    BlackmanHarris,
    Kaiser(u32), // Parameter beta * 10
}

/// Window function generator
pub struct Window {
    coefficients: Vec<f32>,
    window_type: WindowType,
}

impl Window {
    /// Create a new window of the given type and size
    pub fn new(window_type: WindowType, size: usize) -> Self {
        let coefficients = match window_type {
            WindowType::Rectangular => vec![1.0; size],
            WindowType::Hann => Self::hann(size),
            WindowType::Hamming => Self::hamming(size),
            WindowType::Blackman => Self::blackman(size),
            WindowType::BlackmanHarris => Self::blackman_harris(size),
            WindowType::Kaiser(beta) => Self::kaiser(size, beta as f32 / 10.0),
        };

        Self {
            coefficients,
            window_type,
        }
    }

    fn hann(size: usize) -> Vec<f32> {
        (0..size)
            .map(|i| 0.5 * (1.0 - (2.0 * PI * i as f32 / (size - 1) as f32).cos()))
            .collect()
    }

    fn hamming(size: usize) -> Vec<f32> {
        (0..size)
            .map(|i| 0.54 - 0.46 * (2.0 * PI * i as f32 / (size - 1) as f32).cos())
            .collect()
    }

    fn blackman(size: usize) -> Vec<f32> {
        (0..size)
            .map(|i| {
                let n = i as f32 / (size - 1) as f32;
                0.42 - 0.5 * (2.0 * PI * n).cos() + 0.08 * (4.0 * PI * n).cos()
            })
            .collect()
    }

    fn blackman_harris(size: usize) -> Vec<f32> {
        (0..size)
            .map(|i| {
                let n = i as f32 / (size - 1) as f32;
                0.35875
                    - 0.48829 * (2.0 * PI * n).cos()
                    + 0.14128 * (4.0 * PI * n).cos()
                    - 0.01168 * (6.0 * PI * n).cos()
            })
            .collect()
    }

    fn kaiser(size: usize, beta: f32) -> Vec<f32> {
        let i0_beta = Self::bessel_i0(beta);
        (0..size)
            .map(|i| {
                let n = 2.0 * i as f32 / (size - 1) as f32 - 1.0;
                let arg = beta * (1.0 - n * n).sqrt();
                Self::bessel_i0(arg) / i0_beta
            })
            .collect()
    }

    /// Modified Bessel function of the first kind, order 0
    fn bessel_i0(x: f32) -> f32 {
        let mut sum = 1.0f32;
        let mut term = 1.0f32;
        let x_sq = x * x / 4.0;

        for k in 1..50 {
            term *= x_sq / (k * k) as f32;
            sum += term;
            if term < 1e-10 {
                break;
            }
        }

        sum
    }

    /// Apply window to samples
    pub fn apply(&self, samples: &mut [f32]) {
        assert_eq!(samples.len(), self.coefficients.len());
        for (sample, &coef) in samples.iter_mut().zip(self.coefficients.iter()) {
            *sample *= coef;
        }
    }

    /// Apply window and return new vector
    pub fn apply_new(&self, samples: &[f32]) -> Vec<f32> {
        assert_eq!(samples.len(), self.coefficients.len());
        samples
            .iter()
            .zip(self.coefficients.iter())
            .map(|(&s, &c)| s * c)
            .collect()
    }

    /// Get window coefficients
    pub fn coefficients(&self) -> &[f32] {
        &self.coefficients
    }

    /// Get window size
    pub fn size(&self) -> usize {
        self.coefficients.len()
    }

    /// Get coherent gain (sum of coefficients / size)
    pub fn coherent_gain(&self) -> f32 {
        self.coefficients.iter().sum::<f32>() / self.coefficients.len() as f32
    }

    /// Get noise bandwidth (relative to rectangular)
    pub fn noise_bandwidth(&self) -> f32 {
        let sum_sq: f32 = self.coefficients.iter().map(|c| c * c).sum();
        let sum: f32 = self.coefficients.iter().sum();
        self.coefficients.len() as f32 * sum_sq / (sum * sum)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_window_symmetry() {
        let window = Window::new(WindowType::Hann, 64);
        let coefs = window.coefficients();

        for i in 0..32 {
            assert!((coefs[i] - coefs[63 - i]).abs() < 1e-6);
        }
    }

    #[test]
    fn test_window_endpoints() {
        // Hann window should be zero at endpoints
        let window = Window::new(WindowType::Hann, 64);
        assert!(window.coefficients()[0].abs() < 1e-6);
        assert!(window.coefficients()[63].abs() < 1e-6);
    }
}
