//! FFT (Fast Fourier Transform) wrapper

use super::ComplexSample;
use num_complex::Complex;
use rustfft::{Fft as RustFft, FftPlanner};
use std::sync::Arc;

/// FFT direction
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FftDirection {
    Forward,
    Inverse,
}

/// FFT processor
pub struct Fft {
    size: usize,
    forward: Arc<dyn RustFft<f32>>,
    inverse: Arc<dyn RustFft<f32>>,
    scratch: Vec<ComplexSample>,
}

impl Fft {
    /// Create a new FFT processor with the given size
    pub fn new(size: usize) -> Self {
        let mut planner = FftPlanner::new();
        let forward = planner.plan_fft_forward(size);
        let inverse = planner.plan_fft_inverse(size);

        let scratch_len = forward.get_inplace_scratch_len()
            .max(inverse.get_inplace_scratch_len());

        Self {
            size,
            forward,
            inverse,
            scratch: vec![Complex::new(0.0, 0.0); scratch_len],
        }
    }

    /// Get FFT size
    pub fn size(&self) -> usize {
        self.size
    }

    /// Perform forward FFT in-place
    ///
    /// # Panics
    /// Panics in debug mode if data length doesn't match FFT size.
    /// In release mode, size mismatch will produce incorrect results.
    pub fn forward(&mut self, data: &mut [ComplexSample]) {
        debug_assert_eq!(data.len(), self.size, "Data length must match FFT size");
        if data.len() != self.size {
            log::error!("FFT size mismatch: expected {}, got {}", self.size, data.len());
            return;
        }
        self.forward.process_with_scratch(data, &mut self.scratch);
    }

    /// Perform inverse FFT in-place
    ///
    /// # Panics
    /// Panics in debug mode if data length doesn't match FFT size.
    /// In release mode, size mismatch will produce incorrect results.
    pub fn inverse(&mut self, data: &mut [ComplexSample]) {
        debug_assert_eq!(data.len(), self.size, "Data length must match FFT size");
        if data.len() != self.size {
            log::error!("IFFT size mismatch: expected {}, got {}", self.size, data.len());
            return;
        }
        self.inverse.process_with_scratch(data, &mut self.scratch);

        // Normalize
        let scale = 1.0 / self.size as f32;
        for sample in data.iter_mut() {
            *sample *= scale;
        }
    }

    /// Perform FFT and return new buffer
    pub fn forward_new(&mut self, data: &[ComplexSample]) -> Vec<ComplexSample> {
        let mut result = data.to_vec();
        self.forward(&mut result);
        result
    }

    /// Perform IFFT and return new buffer
    pub fn inverse_new(&mut self, data: &[ComplexSample]) -> Vec<ComplexSample> {
        let mut result = data.to_vec();
        self.inverse(&mut result);
        result
    }

    /// Compute power spectrum from real samples
    ///
    /// If input length doesn't match FFT size, pads with zeros or truncates.
    pub fn power_spectrum(&mut self, samples: &[f32]) -> Vec<f32> {
        // Pad or truncate to match FFT size
        let mut complex: Vec<ComplexSample> = if samples.len() >= self.size {
            samples[..self.size]
                .iter()
                .map(|&s| Complex::new(s, 0.0))
                .collect()
        } else {
            let mut v: Vec<ComplexSample> = samples
                .iter()
                .map(|&s| Complex::new(s, 0.0))
                .collect();
            v.resize(self.size, Complex::new(0.0, 0.0));
            v
        };

        // Forward FFT
        self.forward(&mut complex);

        // Return magnitude squared (power)
        complex.iter().map(|c| c.norm_sqr()).collect()
    }

    /// Compute magnitude spectrum from real samples
    pub fn magnitude_spectrum(&mut self, samples: &[f32]) -> Vec<f32> {
        self.power_spectrum(samples)
            .iter()
            .map(|&p| p.sqrt())
            .collect()
    }

    /// Compute phase spectrum from real samples
    pub fn phase_spectrum(&mut self, samples: &[f32]) -> Vec<f32> {
        assert_eq!(samples.len(), self.size);

        let mut complex: Vec<ComplexSample> = samples
            .iter()
            .map(|&s| Complex::new(s, 0.0))
            .collect();

        self.forward(&mut complex);

        complex.iter().map(|c| c.arg()).collect()
    }

    /// Find peak bin in power spectrum
    pub fn find_peak(&mut self, samples: &[f32]) -> (usize, f32) {
        let power = self.power_spectrum(samples);

        // Only look at first half (positive frequencies)
        let half = power.len() / 2;
        let (peak_bin, &peak_power) = power[..half]
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal))
            .unwrap_or((0, &0.0));

        (peak_bin, peak_power.sqrt())
    }

    /// Bin to frequency conversion
    pub fn bin_to_freq(&self, bin: usize, sample_rate: f32) -> f32 {
        bin as f32 * sample_rate / self.size as f32
    }

    /// Frequency to bin conversion
    pub fn freq_to_bin(&self, freq: f32, sample_rate: f32) -> usize {
        ((freq * self.size as f32) / sample_rate).round() as usize
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::PI;

    #[test]
    fn test_fft_inverse() {
        let mut fft = Fft::new(64);
        let original: Vec<ComplexSample> = (0..64)
            .map(|i| Complex::new((2.0 * PI * i as f32 / 64.0).sin(), 0.0))
            .collect();

        let mut data = original.clone();
        fft.forward(&mut data);
        fft.inverse(&mut data);

        for (a, b) in original.iter().zip(data.iter()) {
            assert!((a.re - b.re).abs() < 1e-5);
            assert!((a.im - b.im).abs() < 1e-5);
        }
    }

    #[test]
    fn test_find_peak() {
        let mut fft = Fft::new(256);
        let sample_rate = 48000.0;
        let freq = 1500.0;

        // Generate sine wave at 1500 Hz
        let samples: Vec<f32> = (0..256)
            .map(|i| (2.0 * PI * freq * i as f32 / sample_rate).sin())
            .collect();

        let (peak_bin, _) = fft.find_peak(&samples);
        let detected_freq = fft.bin_to_freq(peak_bin, sample_rate);

        assert!((detected_freq - freq).abs() < sample_rate / 256.0);
    }
}
