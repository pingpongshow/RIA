//! Audio I/O module
//!
//! Handles audio device enumeration, input/output streams, and ring buffers
//! for real-time audio processing.

mod device;
mod buffer;
mod ptt;

#[allow(unused_imports)]
pub use device::{AudioEngine, AudioDevice};
pub use buffer::RingBuffer;
pub use ptt::{PttController, PttMethod};

/// Audio sample type (32-bit float, mono)
pub type Sample = f32;

/// Audio callback for processing received samples
pub type RxCallback = Box<dyn Fn(&[Sample]) + Send + Sync>;

/// Audio callback for generating transmit samples
pub type TxCallback = Box<dyn Fn(&mut [Sample]) + Send + Sync>;

/// Audio stream configuration
#[derive(Debug, Clone)]
pub struct AudioConfig {
    pub sample_rate: u32,
    pub buffer_size: usize,
    pub channels: u16,
}

impl Default for AudioConfig {
    fn default() -> Self {
        Self {
            sample_rate: 48000,
            buffer_size: 1024,
            channels: 1,
        }
    }
}

/// Audio statistics
#[derive(Debug, Clone, Default)]
pub struct AudioStats {
    pub rx_level_db: f32,
    pub tx_level_db: f32,
    pub rx_overruns: u64,
    pub tx_underruns: u64,
    pub latency_ms: f32,
}
