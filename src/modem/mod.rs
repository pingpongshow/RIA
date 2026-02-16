//! Modem module - modulation and demodulation
//!
//! Implements AFDM (Affine Frequency Division Multiplexing) data modulation,
//! MFSK header modulation, and 48-FSK ACK frames.
//!
//! AFDM uses DAFT (Discrete Affine Fourier Transform) instead of FFT for
//! superior performance in doubly-dispersive channels.

mod preamble;
mod mfsk;
mod afdm;
mod constellation;
mod sync;
mod ack_fsk;

// Public API exports (some may not be used internally but are part of the public interface)
#[allow(unused_imports)]
pub use preamble::{Preamble, PreambleDetector, PreambleMode, FastPreambleDetector};
#[allow(unused_imports)]
pub use preamble::{BARKER_5, BARKER_7, BARKER_11, BARKER_13};
#[allow(unused_imports)]
pub use mfsk::{MfskModulator, MfskDemodulator};
pub use afdm::{AfdmModulator, AfdmDemodulator, AfdmConfig};
pub use constellation::{Constellation, ConstellationType};
#[allow(unused_imports)]
pub use sync::{SymbolSync, TimingRecovery};
#[allow(unused_imports)]
pub use ack_fsk::{AckFskModulator, AckFskDemodulator, AckType, Vb6Prng};

#[allow(unused_imports)]
use crate::dsp::ComplexSample;
use crate::protocol::{FftSize, ModulationType, RateInfo, all_rate_info, get_frame_config, max_mode_for_bandwidth, clamp_mode_for_bandwidth};
use crate::Bandwidth as ConfigBandwidth;

/// Bandwidth mode
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Bandwidth {
    /// 500 Hz bandwidth (up to 11 carriers)
    Narrow,
    /// 2300 Hz bandwidth (up to 49 carriers)
    Wide,
    /// 2750 Hz bandwidth (up to 59 carriers)
    Ultra,
}

impl Bandwidth {
    /// Get maximum number of AFDM data carriers for this bandwidth
    pub fn max_carriers(&self) -> usize {
        match self {
            Bandwidth::Narrow => 11,
            Bandwidth::Wide => 49,
            Bandwidth::Ultra => 59,
        }
    }

    /// Get number of carriers for a specific speed level
    pub fn num_carriers_for_level(&self, level: u8) -> usize {
        if let Some(info) = all_rate_info().get((level.clamp(2, 17) - 1) as usize) {
            match self {
                Bandwidth::Narrow => info.carriers_500,
                Bandwidth::Wide => info.carriers,
                Bandwidth::Ultra => info.carriers_2750,
            }
        } else {
            self.max_carriers()
        }
    }

    /// Get maximum number of AFDM data carriers for this bandwidth
    /// Use num_carriers_for_level() to get level-specific carrier counts
    pub fn num_carriers(&self) -> usize {
        self.max_carriers()
    }

    /// Get total bandwidth in Hz
    pub fn bandwidth_hz(&self) -> f32 {
        match self {
            Bandwidth::Narrow => 500.0,
            Bandwidth::Wide => 2300.0,
            Bandwidth::Ultra => 2750.0,
        }
    }

    /// Get carrier spacing for a given FFT size (at 48kHz sample rate)
    pub fn carrier_spacing_for_fft(&self, fft_size: FftSize) -> f32 {
        fft_size.carrier_spacing()
    }

    /// Get default carrier spacing in Hz (FFT=1024)
    pub fn carrier_spacing(&self) -> f32 {
        46.875 // 48000/1024
    }

    /// Get symbol rate for a given FFT size
    pub fn symbol_rate_for_fft(&self, fft_size: FftSize) -> f32 {
        fft_size.carrier_spacing()
    }

    /// Get default symbol rate (symbols per second)
    pub fn symbol_rate(&self) -> f32 {
        self.carrier_spacing()
    }

    /// Get data rate for a specific speed level
    pub fn data_rate_bps(&self, level: u8) -> f32 {
        if let Some(info) = all_rate_info().get((level.clamp(2, 17) - 1) as usize) {
            match self {
                Bandwidth::Narrow => info.bps_500,
                Bandwidth::Wide => info.bps,
                Bandwidth::Ultra => info.bps_2750,
            }
        } else {
            0.0
        }
    }
}

/// Speed level (1-17) determines modulation and coding
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SpeedLevel(u8);

impl SpeedLevel {
    pub fn new(level: u8) -> Self {
        // Mode 1 disabled: 32-tone FSK needs soft-decision decoding for reliable operation
        Self(level.clamp(2, 17))
    }

    pub fn level(&self) -> u8 {
        self.0
    }

    /// Get rate info for this speed level
    pub fn rate_info(&self) -> &'static RateInfo {
        &all_rate_info()[(self.0 - 1) as usize]
    }

    /// Get constellation type for this speed level (uses global RATE_TABLE)
    /// For per-bandwidth modulation, use constellation_for_bandwidth() instead
    pub fn constellation(&self) -> ConstellationType {
        match self.rate_info().modulation {
            ModulationType::Fsk | ModulationType::Bpsk => ConstellationType::Bpsk,
            ModulationType::Qpsk => ConstellationType::Qpsk,
            ModulationType::Psk8 => ConstellationType::Psk8,
            ModulationType::Qam16 => ConstellationType::Qam16,
            ModulationType::Qam32 => ConstellationType::Qam32,
            ModulationType::Qam64 => ConstellationType::Qam64,
        }
    }

    /// Get constellation type for this speed level and bandwidth
    /// Uses per-bandwidth frame config which may have different modulation than RATE_TABLE
    pub fn constellation_for_bandwidth(&self, bandwidth: ConfigBandwidth) -> ConstellationType {
        let frame_config = get_frame_config(self.0, bandwidth);
        match frame_config.modulation {
            ModulationType::Fsk | ModulationType::Bpsk => ConstellationType::Bpsk,
            ModulationType::Qpsk => ConstellationType::Qpsk,
            ModulationType::Psk8 => ConstellationType::Psk8,
            ModulationType::Qam16 => ConstellationType::Qam16,
            ModulationType::Qam32 => ConstellationType::Qam32,
            ModulationType::Qam64 => ConstellationType::Qam64,
        }
    }

    /// Create a SpeedLevel clamped to valid range for the given bandwidth
    /// 500Hz: modes 1-13 only
    pub fn new_for_bandwidth(level: u8, bandwidth: ConfigBandwidth) -> Self {
        let clamped = clamp_mode_for_bandwidth(level, bandwidth);
        // Mode 1 disabled for all bandwidths
        Self(clamped.clamp(2, max_mode_for_bandwidth(bandwidth)))
    }

    /// Get modulation type
    pub fn modulation(&self) -> ModulationType {
        self.rate_info().modulation
    }

    /// Get bits per symbol for the modulation
    pub fn bits_per_symbol(&self) -> usize {
        self.rate_info().modulation.bits_per_symbol()
    }

    /// Get number of carriers for this speed level and bandwidth
    pub fn num_carriers(&self, bandwidth: Bandwidth) -> usize {
        bandwidth.num_carriers_for_level(self.0)
    }

    /// Get FFT size for this speed level
    pub fn fft_size(&self) -> FftSize {
        self.rate_info().fft_size
    }

    /// Get symbol rate for this speed level
    pub fn symbol_rate(&self) -> f32 {
        self.rate_info().symbol_rate
    }

    /// Get data rate in bits per second for given bandwidth
    pub fn data_rate_bps(&self, bandwidth: Bandwidth) -> f32 {
        bandwidth.data_rate_bps(self.0)
    }

    /// Get code rate
    pub fn code_rate(&self) -> f32 {
        self.rate_info().code_rate
    }

    /// Get minimum SNR required for this speed level (dB)
    pub fn min_snr_db(&self) -> f32 {
        self.rate_info().min_snr
    }

    /// Check if this speed level uses FSK modulation (modes 1-4)
    pub fn is_fsk_mode(&self) -> bool {
        self.0 <= 4
    }

    /// Get FSK parameters for this mode (num_tones, symbol_rate)
    /// Returns None if not an FSK mode
    pub fn fsk_params(&self) -> Option<(usize, f32)> {
        match self.0 {
            1 => Some((32, 23.0)),  // 32 tones, 23 sym/s
            2 => Some((16, 47.0)),  // 16 tones, 47 sym/s
            3 => Some((16, 47.0)),  // 16 tones, 47 sym/s
            4 => Some((16, 94.0)),  // 16 tones, 94 sym/s
            _ => None,
        }
    }
}

impl Default for SpeedLevel {
    fn default() -> Self {
        Self(9) // Middle speed level
    }
}

/// Modulated symbol
#[derive(Debug, Clone, Copy)]
pub struct Symbol {
    pub i: f32,
    pub q: f32,
}

impl Symbol {
    pub fn new(i: f32, q: f32) -> Self {
        Self { i, q }
    }

    pub fn from_complex(c: ComplexSample) -> Self {
        Self { i: c.re, q: c.im }
    }

    pub fn to_complex(&self) -> ComplexSample {
        ComplexSample::new(self.i, self.q)
    }

    pub fn magnitude(&self) -> f32 {
        (self.i * self.i + self.q * self.q).sqrt()
    }

    pub fn phase(&self) -> f32 {
        self.q.atan2(self.i)
    }
}

impl From<ComplexSample> for Symbol {
    fn from(c: ComplexSample) -> Self {
        Self::from_complex(c)
    }
}

impl From<Symbol> for ComplexSample {
    fn from(s: Symbol) -> Self {
        s.to_complex()
    }
}
