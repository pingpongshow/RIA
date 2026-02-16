//! Digital Signal Processing module
//!
//! Provides DAFT (Discrete Affine Fourier Transform), FFT, filtering, AGC, AFC,
//! and signal analysis functions.

mod daft;
mod fft;
mod filter;
mod agc;
mod afc;
mod metrics;
mod window;

pub use daft::{Daft, ChirpParams};
#[allow(unused_imports)]
pub use fft::{Fft, FftDirection};
#[allow(unused_imports)]
pub use filter::{Filter, FilterType, FirFilter, IirFilter};
#[allow(unused_imports)]
pub use agc::Agc;
pub use afc::{Afc, wrap_phase, wrap_phase_positive};
#[allow(unused_imports)]
pub use metrics::{SignalMetrics, calculate_snr, calculate_rms, calculate_metrics};
#[allow(unused_imports)]
pub use window::{Window, WindowType};

use num_complex::Complex;

/// Complex sample type
pub type ComplexSample = Complex<f32>;

/// Convert real samples to complex
pub fn real_to_complex(samples: &[f32]) -> Vec<ComplexSample> {
    samples.iter().map(|&s| Complex::new(s, 0.0)).collect()
}

/// Convert complex samples to real (take real part)
pub fn complex_to_real(samples: &[ComplexSample]) -> Vec<f32> {
    samples.iter().map(|c| c.re).collect()
}

/// Calculate magnitude of complex samples
pub fn magnitude(samples: &[ComplexSample]) -> Vec<f32> {
    samples.iter().map(|c| c.norm()).collect()
}

/// Calculate phase of complex samples
pub fn phase(samples: &[ComplexSample]) -> Vec<f32> {
    samples.iter().map(|c| c.arg()).collect()
}

/// Calculate power spectrum (magnitude squared)
pub fn power_spectrum(samples: &[ComplexSample]) -> Vec<f32> {
    samples.iter().map(|c| c.norm_sqr()).collect()
}

/// Convert linear to decibels
pub fn lin_to_db(value: f32) -> f32 {
    if value > 0.0 {
        20.0 * value.log10()
    } else {
        -120.0 // Floor
    }
}

/// Convert decibels to linear
pub fn db_to_lin(db: f32) -> f32 {
    10.0_f32.powf(db / 20.0)
}
