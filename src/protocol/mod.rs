//! Protocol module - ARQ and session management
//!
//! Implements the RIA protocol with selective repeat ARQ,
//! rate adaptation, session management, and optional encryption.

mod frame;
mod arq;
mod session;
mod rate;
mod encryption;

#[allow(unused_imports)]
pub use frame::{Frame, FrameType, FrameHeader, FrameBuilder};
#[allow(unused_imports)]
pub use arq::{ArqController, ArqState, ArqConfig, ArqStats};
pub use session::{Session, SessionState, SessionConfig, SessionEvent};
#[allow(unused_imports)]
pub use rate::{RateAdapter, RateInfo, RateAdapterConfig, FftSize, ModulationType, all_rate_info, FrameConfig, get_frame_config, max_mode_for_bandwidth, clamp_mode_for_bandwidth, RateNegotiator, NegotiationState, ModeThreshold, MODE_THRESHOLDS, calculate_proposed_mode, get_mode_threshold};
#[allow(unused_imports)]
pub use encryption::{Encryptor, EncryptionConfig, EncryptionError};

/// Protocol version
pub const PROTOCOL_VERSION: u8 = 1;

/// Maximum frame payload size
pub const MAX_PAYLOAD_SIZE: usize = 512;

/// Maximum retransmission attempts
pub const MAX_RETRIES: usize = 10;

/// Default timeout in milliseconds
pub const DEFAULT_TIMEOUT_MS: u32 = 4000;

/// Timer values for protocol timeouts
#[derive(Debug, Clone, Copy)]
pub struct TimerConfig {
    /// Timeout for WINLINK mode (ms)
    pub winlink_timeout: u32,
    /// Timeout for P2P mode (ms)
    pub p2p_timeout: u32,
    /// Overhead time (ms)
    pub overhead: u32,
    /// ACK wait time (ms)
    pub ack_wait: u32,
}

impl Default for TimerConfig {
    fn default() -> Self {
        Self {
            winlink_timeout: 15000, // 15s for FSK mode 1 compatibility
            p2p_timeout: 15000,    // 15s for FSK mode 1 compatibility
            overhead: 300,
            ack_wait: 500,
        }
    }
}

/// Connection mode
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConnectionMode {
    /// Point-to-point connection
    P2P,
    /// Winlink gateway connection
    Winlink,
}

impl ConnectionMode {
    /// Get appropriate timeout for this mode
    pub fn timeout(&self, config: &TimerConfig) -> u32 {
        match self {
            ConnectionMode::P2P => config.p2p_timeout,
            ConnectionMode::Winlink => config.winlink_timeout,
        }
    }
}

/// Protocol error types
#[derive(Debug, Clone)]
pub enum ProtocolError {
    /// CRC check failed
    CrcError,
    /// Frame too short
    FrameTooShort,
    /// Invalid frame type
    InvalidFrameType,
    /// Session not established
    NoSession,
    /// Timeout waiting for response
    Timeout,
    /// Maximum retries exceeded
    MaxRetries,
    /// Invalid sequence number
    InvalidSequence,
    /// Protocol version mismatch
    VersionMismatch,
    /// TX queue is full
    QueueFull,
    /// Encryption/decryption error
    EncryptionError(String),
}

impl std::fmt::Display for ProtocolError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ProtocolError::CrcError => write!(f, "CRC check failed"),
            ProtocolError::FrameTooShort => write!(f, "Frame too short"),
            ProtocolError::InvalidFrameType => write!(f, "Invalid frame type"),
            ProtocolError::NoSession => write!(f, "No active session"),
            ProtocolError::Timeout => write!(f, "Timeout waiting for response"),
            ProtocolError::MaxRetries => write!(f, "Maximum retries exceeded"),
            ProtocolError::InvalidSequence => write!(f, "Invalid sequence number"),
            ProtocolError::VersionMismatch => write!(f, "Protocol version mismatch"),
            ProtocolError::QueueFull => write!(f, "TX queue is full"),
            ProtocolError::EncryptionError(msg) => write!(f, "Encryption error: {}", msg),
        }
    }
}

impl std::error::Error for ProtocolError {}
