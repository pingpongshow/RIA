//! RIA - STANAG 4197-style 39-tone HF Modem Library
//!
//! This library provides the core components for building a STANAG 4197-style
//! 39-tone DQPSK HF modem with LDPC FEC coding and ARQ protocol support.

// Allow dead code for library items that are part of the public API
// but not currently used internally
#![allow(dead_code)]

pub mod audio;
pub mod dsp;
pub mod fec;
pub mod interface;
pub mod modem;
pub mod protocol;

#[cfg(feature = "gui")]
pub mod gui;

// Re-export commonly used types
pub use audio::{AudioEngine, RingBuffer as AudioBuffer, PttController};
pub use dsp::{Fft, Filter, Agc, Afc, ComplexSample};
pub use fec::{TurboEncoder, TurboDecoder, Interleaver, ConvolutionalEncoder, ViterbiDecoder};
pub use fec::{K7ConvolutionalEncoder, K7ViterbiDecoder, HuffmanEncoder, HuffmanDecoder};
pub use modem::{StanagModulator, StanagDemodulator, MfskModulator, MfskDemodulator, Preamble, PreambleDetector};
pub use modem::{AckFskModulator, AckFskDemodulator, AckType};
pub use protocol::{Frame, FrameType, Session, SessionState, ArqController, ArqState, RateAdapter};
pub use protocol::{Encryptor, EncryptionConfig};
pub use interface::{TcpServer, TcpConfig, Command, Response, KissFrame, KissFramer};

/// RIA protocol version
pub const PROTOCOL_VERSION: u8 = 1;

/// Sample rate used by the modem
pub const SAMPLE_RATE: u32 = 48000;

/// FFT sizes for different modes
pub const FFT_SIZE_SMALL: usize = 512;
pub const FFT_SIZE_MEDIUM: usize = 1024;
pub const FFT_SIZE_LARGE: usize = 2048;

/// Center frequency offset from USB dial (Hz)
pub const CENTER_FREQUENCY: f32 = 1500.0;

/// Bandwidth modes
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
pub enum Bandwidth {
    Hz500,
    Hz2300,
    Hz2750,
}

impl Bandwidth {
    pub fn carrier_count(&self) -> usize {
        match self {
            Bandwidth::Hz500 => 11,
            Bandwidth::Hz2300 => 49,
            Bandwidth::Hz2750 => 59,
        }
    }

    pub fn hz(&self) -> u32 {
        match self {
            Bandwidth::Hz500 => 500,
            Bandwidth::Hz2300 => 2300,
            Bandwidth::Hz2750 => 2750,
        }
    }
}

/// Configuration for the modem
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
pub struct Config {
    pub callsign: String,
    pub sample_rate: u32,
    pub bandwidth: Bandwidth,
    pub speed_level: u8,
    pub tcp_cmd_port: u16,
    pub tcp_data_port: u16,
    pub audio_input: Option<String>,
    pub audio_output: Option<String>,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            callsign: "N0CALL".to_string(),
            sample_rate: SAMPLE_RATE,
            bandwidth: Bandwidth::Hz2300,
            speed_level: 9,
            tcp_cmd_port: 8300,
            tcp_data_port: 8301,
            audio_input: None,
            audio_output: None,
        }
    }
}

/// Prelude module for convenient imports
pub mod prelude {
    pub use crate::audio::AudioEngine;
    pub use crate::dsp::{Fft, Filter, Agc, Afc};
    pub use crate::fec::{TurboEncoder, TurboDecoder};
    pub use crate::modem::{StanagModulator, StanagDemodulator};
    pub use crate::protocol::{Frame, FrameType, Session, ArqController};
    pub use crate::interface::{TcpServer, Command, Response};
    pub use crate::{Bandwidth, Config, SAMPLE_RATE, CENTER_FREQUENCY};
}
