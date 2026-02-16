//! GUI module - User interface components
//!
//! Implements the RIA modem graphical interface using egui

mod app;
mod waterfall;
mod constellation;
mod meters;
mod settings;

pub use app::RiaApp;
pub use waterfall::Waterfall;
#[allow(unused_imports)]
pub use constellation::{ConstellationDisplay, ConstellationMode};
#[allow(unused_imports)]
pub use meters::{SnrMeter, VuMeter, CpuMeter, AnalogGauge, StatusIndicators, BitrateGraph};
#[allow(unused_imports)]
pub use settings::{SettingsPanel, SettingsEvent};

/// GUI color palette
pub mod colors {
    use eframe::egui::Color32;

    pub const BACKGROUND: Color32 = Color32::from_rgb(32, 32, 32);
    pub const PANEL: Color32 = Color32::from_rgb(48, 48, 48);
    pub const TEXT: Color32 = Color32::from_rgb(220, 220, 220);
    pub const TEXT_DIM: Color32 = Color32::from_rgb(150, 150, 150);
    pub const ACCENT: Color32 = Color32::from_rgb(100, 180, 255);
    pub const SUCCESS: Color32 = Color32::from_rgb(100, 200, 100);
    pub const WARNING: Color32 = Color32::from_rgb(255, 200, 100);
    pub const ERROR: Color32 = Color32::from_rgb(255, 100, 100);

    // Waterfall colors (cold to hot)
    pub const WATERFALL_LOW: Color32 = Color32::from_rgb(0, 0, 50);
    pub const WATERFALL_MID: Color32 = Color32::from_rgb(0, 100, 200);
    pub const WATERFALL_HIGH: Color32 = Color32::from_rgb(255, 255, 0);
    pub const WATERFALL_MAX: Color32 = Color32::from_rgb(255, 100, 100);

    // Status indicators
    pub const TX_ACTIVE: Color32 = Color32::from_rgb(255, 100, 100);
    pub const RX_ACTIVE: Color32 = Color32::from_rgb(100, 255, 100);
    pub const IDLE: Color32 = Color32::from_rgb(100, 100, 100);
    pub const BUSY: Color32 = Color32::from_rgb(255, 200, 0);
}

/// Application state shared with GUI
#[derive(Debug, Clone)]
pub struct GuiState {
    // Connection state
    pub connected: bool,
    pub local_call: String,
    pub remote_call: String,
    pub connection_time: f32,

    // Modem state
    pub tx_active: bool,
    pub rx_active: bool,
    pub busy: bool,
    pub listen: bool,
    pub bandwidth: String,
    pub tcp_connected: bool,  // TCP client connected (cmd or data)

    // Protocol status indicators
    pub status_data: bool,    // DATA: Sending/receiving data frames
    pub status_ack: bool,     // ACK: ACK frame sent/received
    pub status_idle: bool,    // IDLE: No activity
    pub status_req: bool,     // REQ: Connection request
    pub status_start: bool,   // START: Session starting
    pub status_nack: bool,    // NACK: Negative acknowledgement
    pub status_break: bool,   // BREAK: Link break detected
    pub status_qrt: bool,     // QRT: Quit request

    // Signal quality
    pub snr_db: f32,
    pub signal_level: f32,
    pub noise_level: f32,
    pub afc_offset: f32,

    // Performance
    pub cpu_percent: f32,
    pub speed_level: u8,
    pub bitrate: f32,

    // Buffers
    pub tx_buffer_bytes: usize,
    pub rx_buffer_bytes: usize,

    // Counters
    pub frames_sent: u64,
    pub frames_received: u64,
    pub bytes_sent: u64,
    pub bytes_received: u64,

    // Constellation display points
    pub constellation_points: Vec<(f32, f32)>,

    // Waterfall spectrum data (dB values)
    pub spectrum_data: Vec<f32>,
}

impl Default for GuiState {
    fn default() -> Self {
        Self {
            connected: false,
            local_call: String::new(),
            remote_call: String::new(),
            connection_time: 0.0,
            tx_active: false,
            rx_active: false,
            busy: false,
            listen: true,
            bandwidth: "2300".to_string(),
            tcp_connected: false,
            // Protocol status indicators - default to IDLE
            status_data: false,
            status_ack: false,
            status_idle: true,
            status_req: false,
            status_start: false,
            status_nack: false,
            status_break: false,
            status_qrt: false,
            // Signal quality
            snr_db: 0.0,
            signal_level: -60.0,
            noise_level: -60.0,
            afc_offset: 0.0,
            cpu_percent: 0.0,
            speed_level: 9,
            bitrate: 0.0,
            tx_buffer_bytes: 0,
            rx_buffer_bytes: 0,
            frames_sent: 0,
            frames_received: 0,
            bytes_sent: 0,
            bytes_received: 0,
            constellation_points: Vec::new(),
            spectrum_data: Vec::new(),
        }
    }
}

/// GUI event for communication with modem core
#[derive(Debug, Clone)]
pub enum GuiEvent {
    /// Connect to remote station
    Connect { callsign: String },
    /// Disconnect current session
    Disconnect,
    /// Change bandwidth mode
    SetBandwidth { mode: String },
    /// Toggle listen mode
    SetListen { enabled: bool },
    /// Set local callsign
    SetMyCall { callsign: String },
    /// Start tune mode
    StartTune,
    /// Stop tune mode
    StopTune,
    /// Change speed level
    SetSpeedLevel { level: u8 },
    /// Open settings
    OpenSettings,
    /// Close application
    Quit,
    /// Change audio input device
    SetAudioInput { device: String },
    /// Change audio output device
    SetAudioOutput { device: String },
    /// Change audio channel (0=L, 1=R, 2=L+R)
    SetAudioChannel { channel: usize },
    /// Change drive level (0.0 to 1.0)
    SetDriveLevel { level: f32 },
    /// Update TCP ports
    SetTcpPorts { command: u16, data: u16, kiss: u16 },
    /// Set 500 Hz accept mode
    SetAccept500Hz { enabled: bool },
    /// Set KISS interface mode
    SetKissInterface { enabled: bool },
    /// Set CW ID mode
    SetCwId { enabled: bool },
    /// Set retry count
    SetRetries { count: u8 },
}
