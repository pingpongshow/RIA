//! Discrete Affine Fourier Transform (DAFT)
//!
//! DAFT is a generalization of the DFT with two chirp parameters (c1, c2)
//! that enable full delay-Doppler diversity for doubly-dispersive channels.
//!
//! The forward DAFT is defined as:
//!   X[k] = (1/sqrt(N)) * sum(n=0 to N-1) x[n] * exp(-j*2*pi*(c1*n^2 + k*n + c2*k^2)/N)
//!
//! Special cases:
//!   - c1=0, c2=0: Standard DFT (OFDM mode)
//!   - c1!=0, c2=0: Pre-chirped DFT
//!   - c1=0, c2!=0: Post-chirped DFT
//!   - c1!=0, c2!=0: Full AFDM with maximum diversity

use super::ComplexSample;
use num_complex::Complex;
use rustfft::{Fft as RustFft, FftPlanner};
use std::f32::consts::PI;
use std::sync::Arc;

/// Chirp parameters for DAFT
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ChirpParams {
    /// Pre-chirp parameter (applied before FFT)
    pub c1: f32,
    /// Post-chirp parameter (applied after FFT)
    pub c2: f32,
}

impl ChirpParams {
    /// Create new chirp parameters
    pub fn new(c1: f32, c2: f32) -> Self {
        Self { c1, c2 }
    }

    /// OFDM-compatible mode (no chirp)
    pub fn ofdm_mode() -> Self {
        Self { c1: 0.0, c2: 0.0 }
    }

    /// Default AFDM parameters for good delay-Doppler diversity
    /// c1=0.5, c2=0.3 provides balanced delay-Doppler resilience
    pub fn default_afdm() -> Self {
        Self { c1: 0.5, c2: 0.3 }
    }

    /// Robust mode with stronger chirp for high Doppler
    pub fn robust() -> Self {
        Self { c1: 0.7, c2: 0.5 }
    }

    /// Check if in OFDM-compatible mode
    pub fn is_ofdm_mode(&self) -> bool {
        self.c1.abs() < 1e-6 && self.c2.abs() < 1e-6
    }
}

impl Default for ChirpParams {
    fn default() -> Self {
        Self::default_afdm()
    }
}

/// Discrete Affine Fourier Transform implementation
pub struct Daft {
    size: usize,
    chirp: ChirpParams,
    /// Pre-chirp factors: exp(-j*2*pi*c1*n^2/N)
    pre_chirp: Vec<ComplexSample>,
    /// Post-chirp factors: exp(-j*2*pi*c2*k^2/N)
    post_chirp: Vec<ComplexSample>,
    /// Inverse pre-chirp: exp(+j*2*pi*c1*n^2/N)
    inv_pre_chirp: Vec<ComplexSample>,
    /// Inverse post-chirp: exp(+j*2*pi*c2*k^2/N)
    inv_post_chirp: Vec<ComplexSample>,
    /// FFT plan for forward transform
    fft_forward: Arc<dyn RustFft<f32>>,
    /// FFT plan for inverse transform
    fft_inverse: Arc<dyn RustFft<f32>>,
    /// Scratch buffer
    scratch: Vec<ComplexSample>,
    /// Normalization factor (1/sqrt(N))
    norm: f32,
}

impl Daft {
    /// Create a new DAFT instance with given size and chirp parameters
    pub fn new(size: usize, chirp: ChirpParams) -> Self {
        assert!(size.is_power_of_two(), "Size must be power of 2");

        let n_f32 = size as f32;

        // Precompute chirp factors
        let mut pre_chirp = Vec::with_capacity(size);
        let mut post_chirp = Vec::with_capacity(size);
        let mut inv_pre_chirp = Vec::with_capacity(size);
        let mut inv_post_chirp = Vec::with_capacity(size);

        for i in 0..size {
            let i_f32 = i as f32;

            // Pre-chirp: exp(-j*2*pi*c1*n^2/N)
            let phase_pre = -2.0 * PI * chirp.c1 * i_f32 * i_f32 / n_f32;
            pre_chirp.push(Complex::new(phase_pre.cos(), phase_pre.sin()));
            inv_pre_chirp.push(Complex::new(phase_pre.cos(), -phase_pre.sin()));

            // Post-chirp: exp(-j*2*pi*c2*k^2/N)
            let phase_post = -2.0 * PI * chirp.c2 * i_f32 * i_f32 / n_f32;
            post_chirp.push(Complex::new(phase_post.cos(), phase_post.sin()));
            inv_post_chirp.push(Complex::new(phase_post.cos(), -phase_post.sin()));
        }

        // Create FFT plans
        let mut planner = FftPlanner::new();
        let fft_forward = planner.plan_fft_forward(size);
        let fft_inverse = planner.plan_fft_inverse(size);

        let scratch_len = fft_forward.get_inplace_scratch_len()
            .max(fft_inverse.get_inplace_scratch_len());

        let norm = 1.0 / n_f32.sqrt();

        Self {
            size,
            chirp,
            pre_chirp,
            post_chirp,
            inv_pre_chirp,
            inv_post_chirp,
            fft_forward,
            fft_inverse,
            scratch: vec![Complex::new(0.0, 0.0); scratch_len],
            norm,
        }
    }

    /// Create DAFT with OFDM-compatible mode (no chirp)
    pub fn ofdm_mode(size: usize) -> Self {
        Self::new(size, ChirpParams::ofdm_mode())
    }

    /// Get transform size
    pub fn size(&self) -> usize {
        self.size
    }

    /// Get chirp parameters
    pub fn chirp_params(&self) -> ChirpParams {
        self.chirp
    }

    /// Check if in OFDM-compatible mode
    pub fn is_ofdm_mode(&self) -> bool {
        self.chirp.is_ofdm_mode()
    }

    /// Forward DAFT: Time domain -> DAFT domain
    ///
    /// X[k] = (1/sqrt(N)) * sum(n) x[n] * exp(-j*2*pi*(c1*n^2 + k*n + c2*k^2)/N)
    pub fn forward(&mut self, data: &mut [ComplexSample]) {
        debug_assert_eq!(data.len(), self.size, "Data length must match DAFT size");
        if data.len() != self.size {
            log::error!("DAFT size mismatch: expected {}, got {}", self.size, data.len());
            return;
        }

        // Step 1: Apply pre-chirp
        for (sample, chirp) in data.iter_mut().zip(self.pre_chirp.iter()) {
            *sample *= chirp;
        }

        // Step 2: Apply FFT
        self.fft_forward.process_with_scratch(data, &mut self.scratch);

        // Step 3: Apply post-chirp and normalize
        for (sample, chirp) in data.iter_mut().zip(self.post_chirp.iter()) {
            *sample = *sample * chirp * self.norm;
        }
    }

    /// Inverse DAFT: DAFT domain -> Time domain
    ///
    /// x[n] = (1/sqrt(N)) * sum(k) X[k] * exp(+j*2*pi*(c1*n^2 + k*n + c2*k^2)/N)
    pub fn inverse(&mut self, data: &mut [ComplexSample]) {
        debug_assert_eq!(data.len(), self.size, "Data length must match DAFT size");
        if data.len() != self.size {
            log::error!("IDAFT size mismatch: expected {}, got {}", self.size, data.len());
            return;
        }

        // Step 1: Apply inverse post-chirp
        for (sample, chirp) in data.iter_mut().zip(self.inv_post_chirp.iter()) {
            *sample *= chirp;
        }

        // Step 2: Apply IFFT
        self.fft_inverse.process_with_scratch(data, &mut self.scratch);

        // Step 3: Apply inverse pre-chirp and normalize
        // Note: rustfft IFFT doesn't normalize, so we divide by N, then multiply by sqrt(N) for DAFT
        // Combined: 1/N * sqrt(N) = 1/sqrt(N)
        let scale = self.norm;
        for (sample, chirp) in data.iter_mut().zip(self.inv_pre_chirp.iter()) {
            *sample = *sample * chirp * scale;
        }
    }

    /// Forward DAFT returning new buffer
    pub fn forward_new(&mut self, data: &[ComplexSample]) -> Vec<ComplexSample> {
        let mut result = data.to_vec();
        self.forward(&mut result);
        result
    }

    /// Inverse DAFT returning new buffer
    pub fn inverse_new(&mut self, data: &[ComplexSample]) -> Vec<ComplexSample> {
        let mut result = data.to_vec();
        self.inverse(&mut result);
        result
    }

    /// Update chirp parameters (requires recomputing chirp factors)
    pub fn set_chirp_params(&mut self, params: ChirpParams) {
        if params == self.chirp {
            return;
        }

        self.chirp = params;
        let n_f32 = self.size as f32;

        for i in 0..self.size {
            let i_f32 = i as f32;

            let phase_pre = -2.0 * PI * params.c1 * i_f32 * i_f32 / n_f32;
            self.pre_chirp[i] = Complex::new(phase_pre.cos(), phase_pre.sin());
            self.inv_pre_chirp[i] = Complex::new(phase_pre.cos(), -phase_pre.sin());

            let phase_post = -2.0 * PI * params.c2 * i_f32 * i_f32 / n_f32;
            self.post_chirp[i] = Complex::new(phase_post.cos(), phase_post.sin());
            self.inv_post_chirp[i] = Complex::new(phase_post.cos(), -phase_post.sin());
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn complex_close(a: ComplexSample, b: ComplexSample, tol: f32) -> bool {
        (a.re - b.re).abs() < tol && (a.im - b.im).abs() < tol
    }

    #[test]
    fn test_daft_roundtrip() {
        let mut daft = Daft::new(64, ChirpParams::new(0.5, 0.3));

        let original: Vec<ComplexSample> = (0..64)
            .map(|i| Complex::new((i as f32 * 0.1).sin(), (i as f32 * 0.2).cos()))
            .collect();

        let freq = daft.forward_new(&original);
        let recovered = daft.inverse_new(&freq);

        for (a, b) in original.iter().zip(recovered.iter()) {
            assert!(complex_close(*a, *b, 1e-4), "Mismatch: {:?} vs {:?}", a, b);
        }
    }

    #[test]
    fn test_daft_ofdm_mode() {
        let mut daft = Daft::ofdm_mode(64);

        assert!(daft.is_ofdm_mode());

        // Impulse at position 0
        let mut input = vec![Complex::new(0.0, 0.0); 64];
        input[0] = Complex::new(1.0, 0.0);

        let freq = daft.forward_new(&input);
        let expected_mag = 1.0 / (64.0_f32).sqrt();

        for x in freq.iter() {
            assert!((x.norm() - expected_mag).abs() < 1e-4);
        }
    }

    #[test]
    fn test_parseval_theorem() {
        // Energy should be preserved through DAFT
        let mut daft = Daft::new(64, ChirpParams::new(0.7, 0.4));

        let input: Vec<ComplexSample> = (0..64)
            .map(|i| Complex::new((i as f32 * 0.3).sin(), (i as f32 * 0.15).cos()))
            .collect();

        let output = daft.forward_new(&input);

        let energy_in: f32 = input.iter().map(|x| x.norm_sqr()).sum();
        let energy_out: f32 = output.iter().map(|x| x.norm_sqr()).sum();

        assert!(
            (energy_in - energy_out).abs() < 1e-3,
            "Energy not preserved: {} vs {}",
            energy_in,
            energy_out
        );
    }

    #[test]
    fn test_different_chirp_params() {
        let params_list = [
            ChirpParams::new(0.0, 0.0),   // OFDM
            ChirpParams::new(1.0, 0.0),   // Pre-chirp only
            ChirpParams::new(0.0, 1.0),   // Post-chirp only
            ChirpParams::new(0.5, 0.5),   // Symmetric
            ChirpParams::new(2.0, 1.5),   // Asymmetric
        ];

        for params in params_list.iter() {
            let mut daft = Daft::new(64, *params);

            let input: Vec<ComplexSample> = (0..64)
                .map(|_| Complex::new(1.0, 0.0))
                .collect();

            let freq = daft.forward_new(&input);
            let recovered = daft.inverse_new(&freq);

            for (a, b) in input.iter().zip(recovered.iter()) {
                assert!(
                    complex_close(*a, *b, 1e-4),
                    "Failed for c1={}, c2={}",
                    params.c1,
                    params.c2
                );
            }
        }
    }
}
