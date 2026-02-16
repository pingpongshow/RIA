//! RIA - High-performance OFDM HF Modem
//!
//! A cross-platform amateur radio modem supporting multiple modulation modes,
//! turbo FEC coding, and ARQ protocol for reliable HF communications.

// Allow dead code for items that are part of the API but not yet fully integrated
#![allow(dead_code)]

use anyhow::Result;
use log::{info, error, warn, debug};
use std::sync::{Arc, Mutex};
use std::time::Instant;

mod audio;
mod dsp;
mod fec;
mod gui;
mod interface;
mod modem;
mod protocol;

use audio::{AudioEngine, PttController, PttMethod as AudioPttMethod};
use protocol::{Session, SessionConfig, SessionState, ConnectionMode, Frame, SessionEvent, get_frame_config, clamp_mode_for_bandwidth, EncryptionConfig};
use interface::{TcpServer, TcpConfig, Command, Response};
use gui::{RiaApp, GuiState, GuiEvent};
use modem::{
    OfdmModulator, OfdmDemodulator, OfdmConfig, Bandwidth as ModemBandwidth,
    SpeedLevel, Preamble, PreambleDetector, PreambleMode,
    MfskModulator, MfskDemodulator,
};
use dsp::{SignalMetrics, Afc, Fft, real_to_complex, complex_to_real, lin_to_db, wrap_phase_positive};
use fec::{TurboEncoder, TurboDecoder};

/// Application configuration
#[derive(Debug, Clone)]
pub struct Config {
    pub callsign: String,
    pub tcp_command_port: u16,
    pub tcp_data_port: u16,
    pub audio_input: Option<String>,
    pub audio_output: Option<String>,
    pub sample_rate: u32,
    pub bandwidth: Bandwidth,
    pub default_speed_level: u8,
    pub ptt_method: PttMethod,
    /// Encryption passphrase (empty = disabled)
    pub encryption_passphrase: String,
    /// Preamble detection threshold (0.0-1.0, default 0.50)
    pub preamble_threshold: f32,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            callsign: "N0CALL".to_string(),
            tcp_command_port: 8300,
            tcp_data_port: 8301,
            audio_input: None,
            audio_output: None,
            sample_rate: 48000,
            bandwidth: Bandwidth::Hz2300,
            default_speed_level: 9,
            ptt_method: PttMethod::Vox,
            encryption_passphrase: String::new(),
            preamble_threshold: 0.50,
        }
    }
}

/// Bandwidth modes
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
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

/// PTT control methods
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PttMethod {
    Vox,
    Rts,
    Dtr,
    None,
}

/// Shared application state
pub struct AppState {
    pub config: Config,
    pub session_state: SessionState,
    pub tx_active: bool,
    pub rx_active: bool,
    pub connected: bool,
    pub remote_callsign: Option<String>,
    pub snr_db: f32,
    pub cpu_usage: f32,
    pub vu_level: f32,
    pub afc_offset_hz: f32,
    pub current_speed_level: u8,
    pub busy: bool,
    pub listen: bool,
    pub tcp_connected: bool,  // TCP client connected (cmd or data)
    pub tx_buffer_bytes: usize,
    pub rx_buffer_bytes: usize,
    pub frames_sent: u64,
    pub frames_received: u64,
    pub bytes_sent: u64,
    pub bytes_received: u64,
    // Additional state for commands
    pub compression_enabled: bool,
    pub aux_callsigns: Vec<String>,
    pub chat_mode: bool,
    pub winlink_mode: bool,
    pub tune_mode: bool,
    pub auto_tune: bool,
    pub drive_level: f32,
    pub cwid_callsign: Option<String>,
    pub cwid_enabled: bool,
    // Bitrate and timing
    pub bitrate_bps: f32,
    pub connection_start: Option<std::time::Instant>,
    pub connection_time_secs: f32,
    // Data queues for TCP <-> modem communication
    pub tx_data_queue: Vec<u8>,    // Data from TCP to send over the air
    pub rx_data_queue: Vec<u8>,    // Data received over the air to send to TCP

    // Audio device change requests (set by GUI, cleared by modem thread)
    pub pending_audio_input: Option<String>,
    pub pending_audio_output: Option<String>,
}

impl Default for AppState {
    fn default() -> Self {
        Self {
            config: Config::default(),
            session_state: SessionState::Disconnected,
            tx_active: false,
            rx_active: false,
            connected: false,
            remote_callsign: None,
            snr_db: 0.0,
            cpu_usage: 0.0,
            vu_level: -60.0,
            afc_offset_hz: 0.0,
            current_speed_level: 9,
            busy: false,
            listen: true,
            tcp_connected: false,
            tx_buffer_bytes: 0,
            rx_buffer_bytes: 0,
            frames_sent: 0,
            frames_received: 0,
            bytes_sent: 0,
            bytes_received: 0,
            // Additional state defaults
            compression_enabled: false,
            aux_callsigns: Vec::new(),
            chat_mode: false,
            winlink_mode: false,
            tune_mode: false,
            auto_tune: true,
            drive_level: 0.8,
            cwid_callsign: None,
            cwid_enabled: false,
            // Bitrate and timing
            bitrate_bps: 0.0,
            connection_start: None,
            connection_time_secs: 0.0,
            // Data queues
            tx_data_queue: Vec::new(),
            rx_data_queue: Vec::new(),
            // Audio device change requests
            pending_audio_input: None,
            pending_audio_output: None,
        }
    }
}

fn main() -> Result<()> {
    // Initialize logging
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info"))
        .format_timestamp_millis()
        .init();

    info!("RIA OFDM HF Modem v{}", env!("CARGO_PKG_VERSION"));
    info!("Starting up...");

    // Load configuration
    let config = load_config().unwrap_or_else(|e| {
        info!("Using default config: {}", e);
        Config::default()
    });

    // Create shared state
    // Clamp speed level for bandwidth (500Hz max is mode 13)
    let app_state = Arc::new(Mutex::new(AppState {
        config: config.clone(),
        current_speed_level: clamp_mode_for_bandwidth(config.default_speed_level, config.bandwidth),
        ..Default::default()
    }));

    // Create GUI state with correct bandwidth from config
    let gui_state = Arc::new(Mutex::new({
        let mut state = GuiState::default();
        state.bandwidth = match config.bandwidth {
            Bandwidth::Hz500 => "500".to_string(),
            Bandwidth::Hz2300 => "2300".to_string(),
            Bandwidth::Hz2750 => "2750".to_string(),
        };
        state
    }));
    let gui_events = Arc::new(Mutex::new(Vec::<GuiEvent>::new()));

    // Start runtime
    let rt = tokio::runtime::Runtime::new()?;

    // Clone references for async tasks
    let app_state_tcp = app_state.clone();
    let app_state_modem = app_state.clone();
    let gui_state_update = gui_state.clone();
    let gui_state_modem = gui_state.clone();
    let gui_events_process = gui_events.clone();
    let config_modem = config.clone();

    // Spawn modem processing on a separate thread (AudioEngine is not Send)
    std::thread::spawn(move || {
        run_modem_processing(config_modem, app_state_modem, gui_state_modem);
    });

    // Spawn TCP server task
    rt.spawn(async move {
        let tcp_config = TcpConfig {
            cmd_port: config.tcp_command_port,
            data_port: config.tcp_data_port,
            bind_addr: "127.0.0.1".to_string(),
        };

        let mut server = TcpServer::new(tcp_config);

        if let Err(e) = server.start() {
            error!("Failed to start TCP server: {}", e);
            return;
        }

        info!("TCP server listening on ports {} (cmd) and {} (data)",
              config.tcp_command_port, config.tcp_data_port);

        loop {
            // Accept new connections
            let _ = server.accept();

            // Update TCP connection status in GUI
            let tcp_connected = server.has_cmd_clients() || server.has_data_client();
            if let Ok(mut state) = app_state_tcp.lock() {
                state.tcp_connected = tcp_connected;
            }

            // Process commands
            for (client_idx, cmd) in server.read_commands() {
                let response = handle_command(&cmd, &app_state_tcp);
                // For Close responses, only send to the originating client
                if matches!(response, Response::Close) {
                    server.send_response_to(client_idx, &response);
                } else {
                    server.send_response(&response);
                }
            }

            // Read TX data from TCP data client and queue for transmission
            if let Some(data) = server.read_data() {
                if !data.is_empty() {
                    if let Ok(mut state) = app_state_tcp.lock() {
                        state.tx_data_queue.extend_from_slice(&data);
                        debug!("Queued {} bytes from TCP for transmission", data.len());
                    }
                }
            }

            // Send RX data from queue to TCP data client
            {
                let mut data_to_send = Vec::new();
                if let Ok(mut state) = app_state_tcp.lock() {
                    if !state.rx_data_queue.is_empty() {
                        data_to_send = std::mem::take(&mut state.rx_data_queue);
                    }
                }
                if !data_to_send.is_empty() {
                    if let Err(e) = server.send_data(&data_to_send) {
                        warn!("Failed to send data to TCP client: {}", e);
                        // Re-queue the data to prevent data loss
                        if let Ok(mut state) = app_state_tcp.lock() {
                            // Prepend to preserve order (new data may have arrived)
                            let mut combined = data_to_send;
                            combined.extend(std::mem::take(&mut state.rx_data_queue));
                            state.rx_data_queue = combined;
                        }
                    } else {
                        debug!("Sent {} bytes to TCP data client", data_to_send.len());
                    }
                }
            }

            // Update connection time
            if let Ok(mut state) = app_state_tcp.lock() {
                if state.connected {
                    if let Some(start) = state.connection_start {
                        state.connection_time_secs = start.elapsed().as_secs_f32();
                    }
                } else {
                    state.connection_time_secs = 0.0;
                }
            }

            // Update GUI state from app state
            // Copy data from app_state first to avoid nested lock acquisition
            let gui_update = if let Ok(state) = app_state_tcp.lock() {
                Some((
                    state.connected,
                    state.config.callsign.clone(),
                    state.remote_callsign.clone().unwrap_or_default(),
                    state.tx_active,
                    state.rx_active,
                    state.busy,
                    state.listen,
                    state.tcp_connected,
                    state.snr_db,
                    state.vu_level,
                    state.afc_offset_hz,
                    state.cpu_usage,
                    state.current_speed_level,
                    state.tx_buffer_bytes,
                    state.rx_buffer_bytes,
                    state.frames_sent,
                    state.frames_received,
                    state.bytes_sent,
                    state.bytes_received,
                    state.bitrate_bps,
                    state.connection_time_secs,
                ))
            } else {
                None
            };
            // Now update gui_state with app_state lock released
            if let Some((connected, local_call, remote_call, tx_active, rx_active, busy, listen,
                         tcp_connected, snr_db, signal_level, afc_offset, cpu_percent, speed_level,
                         tx_buffer_bytes, rx_buffer_bytes, frames_sent, frames_received,
                         bytes_sent, bytes_received, bitrate, connection_time)) = gui_update {
                if let Ok(mut gui) = gui_state_update.lock() {
                    gui.connected = connected;
                    gui.local_call = local_call;
                    gui.remote_call = remote_call;
                    gui.tx_active = tx_active;
                    gui.rx_active = rx_active;
                    gui.busy = busy;
                    gui.listen = listen;
                    gui.tcp_connected = tcp_connected;
                    gui.snr_db = snr_db;
                    gui.signal_level = signal_level;
                    gui.afc_offset = afc_offset;
                    gui.cpu_percent = cpu_percent;
                    gui.speed_level = speed_level;
                    gui.tx_buffer_bytes = tx_buffer_bytes;
                    gui.rx_buffer_bytes = rx_buffer_bytes;
                    gui.frames_sent = frames_sent;
                    gui.frames_received = frames_received;
                    gui.bytes_sent = bytes_sent;
                    gui.bytes_received = bytes_received;
                    gui.bitrate = bitrate;
                    gui.connection_time = connection_time;
                }
            }

            // Process GUI events
            if let Ok(mut events) = gui_events_process.lock() {
                for event in events.drain(..) {
                    handle_gui_event(event, &app_state_tcp);
                }
            }

            tokio::time::sleep(tokio::time::Duration::from_millis(10)).await;
        }
    });

    // Run GUI
    info!("Starting GUI...");
    let instance_id = std::env::var("RIA_INSTANCE").unwrap_or_else(|_| "0".to_string());
    let window_title = if instance_id == "0" {
        format!("RIA - OFDM HF Modem [{}]", config.callsign)
    } else {
        format!("RIA - OFDM HF Modem [{}] (Instance {})", config.callsign, instance_id)
    };
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([900.0, 650.0])
            .with_min_inner_size([700.0, 500.0])
            .with_title(&window_title),
        ..Default::default()
    };

    eframe::run_native(
        "RIA - OFDM HF Modem",
        options,
        Box::new(move |cc| {
            Ok(Box::new(RiaApp::new(cc, gui_state.clone(), gui_events.clone())))
        }),
    ).map_err(|e| anyhow::anyhow!("GUI error: {}", e))?;

    info!("Shutting down...");
    Ok(())
}

/// Handle a command from TCP interface
fn handle_command(cmd: &Command, state: &Arc<Mutex<AppState>>) -> Response {
    match cmd {
        Command::Connect { callsign } => {
            if let Ok(mut s) = state.lock() {
                if s.connected {
                    return Response::Error("Already connected".to_string());
                }
                // Validate callsign
                let callsign = callsign.to_uppercase();
                if callsign.is_empty() || callsign.len() > 10 {
                    return Response::Error("Invalid callsign".to_string());
                }

                // Set connection state to initiating
                s.remote_callsign = Some(callsign.clone());
                s.session_state = SessionState::Connecting;
                s.busy = true;

                info!("Initiating connection to {}", callsign);

                // Return PENDING - the connection will complete asynchronously
                // The modem processing loop will handle preamble transmission
                Response::Pending
            } else {
                Response::Error("State lock failed".to_string())
            }
        }
        Command::Disconnect => {
            if let Ok(mut s) = state.lock() {
                if s.connected {
                    s.session_state = SessionState::Disconnecting;
                    info!("Initiating disconnection from {:?}", s.remote_callsign);
                }
                s.connected = false;
                s.remote_callsign = None;
                s.busy = false;
                Response::Disconnected
            } else {
                Response::Error("State lock failed".to_string())
            }
        }
        Command::Abort => {
            if let Ok(mut s) = state.lock() {
                if s.session_state == SessionState::Connecting {
                    s.session_state = SessionState::Disconnected;
                    s.remote_callsign = None;
                    s.busy = false;
                    info!("Connection attempt aborted");
                    Response::Ok
                } else {
                    Response::Error("No connection in progress".to_string())
                }
            } else {
                Response::Error("State lock failed".to_string())
            }
        }
        Command::Bandwidth { mode } => {
            if let Ok(mut s) = state.lock() {
                s.config.bandwidth = match mode.as_str() {
                    "500" => Bandwidth::Hz500,
                    "2750" => Bandwidth::Hz2750,
                    _ => Bandwidth::Hz2300,
                };
                info!("Bandwidth set to {} Hz", s.config.bandwidth.hz());
                Response::Ok
            } else {
                Response::Error("State lock failed".to_string())
            }
        }
        Command::BwMode { hz } => {
            if let Ok(mut s) = state.lock() {
                s.config.bandwidth = match hz {
                    500 => Bandwidth::Hz500,
                    2750 => Bandwidth::Hz2750,
                    _ => Bandwidth::Hz2300,
                };
                info!("Bandwidth set to {} Hz", s.config.bandwidth.hz());
                Response::Ok
            } else {
                Response::Error("State lock failed".to_string())
            }
        }
        Command::Compression { enabled } => {
            if let Ok(mut s) = state.lock() {
                s.compression_enabled = *enabled;
                info!("Compression: {}", if *enabled { "ON" } else { "OFF" });
                Response::Ok
            } else {
                Response::Error("State lock failed".to_string())
            }
        }
        Command::MyCall { callsign } => {
            if let Ok(mut s) = state.lock() {
                let callsign = callsign.to_uppercase();
                if callsign.is_empty() || callsign.len() > 10 {
                    return Response::Error("Invalid callsign".to_string());
                }
                s.config.callsign = callsign.clone();
                info!("Callsign set to: {}", callsign);
                Response::Ok
            } else {
                Response::Error("State lock failed".to_string())
            }
        }
        Command::MyAux { callsigns } => {
            if let Ok(mut s) = state.lock() {
                // Validate and normalize callsigns
                let valid_callsigns: Vec<String> = callsigns
                    .iter()
                    .map(|c| c.to_uppercase())
                    .filter(|c| !c.is_empty() && c.len() <= 10)
                    .collect();
                info!("Auxiliary callsigns set: {:?}", valid_callsigns);
                s.aux_callsigns = valid_callsigns;
                Response::Ok
            } else {
                Response::Error("State lock failed".to_string())
            }
        }
        Command::ChatMode { enabled } => {
            if let Ok(mut s) = state.lock() {
                s.chat_mode = *enabled;
                info!("Chat mode: {}", if *enabled { "ON" } else { "OFF" });
                Response::Ok
            } else {
                Response::Error("State lock failed".to_string())
            }
        }
        Command::Listen { enabled } => {
            if let Ok(mut s) = state.lock() {
                s.listen = *enabled;
                info!("Listen mode: {}", if *enabled { "ON" } else { "OFF" });
                Response::Ok
            } else {
                Response::Error("State lock failed".to_string())
            }
        }
        Command::Version => {
            Response::Version {
                version: env!("CARGO_PKG_VERSION").to_string(),
            }
        }
        Command::Codec => {
            // Return codec information
            if let Ok(s) = state.lock() {
                let codec_info = format!(
                    "RIA OFDM {} Hz, Speed Level {}, Turbo FEC",
                    s.config.bandwidth.hz(),
                    s.current_speed_level
                );
                Response::Custom(format!("CODEC {}", codec_info))
            } else {
                Response::Custom("CODEC RIA OFDM".to_string())
            }
        }
        Command::PttState => {
            if let Ok(s) = state.lock() {
                Response::Ptt { active: s.tx_active }
            } else {
                Response::Ptt { active: false }
            }
        }
        Command::WinlinkSession { enabled } => {
            if let Ok(mut s) = state.lock() {
                s.winlink_mode = *enabled;
                info!("Winlink mode: {}", if *enabled { "ON" } else { "OFF" });
                Response::Ok
            } else {
                Response::Error("State lock failed".to_string())
            }
        }
        Command::Tune { enabled } => {
            if let Ok(mut s) = state.lock() {
                s.tune_mode = *enabled;
                if *enabled {
                    info!("Tune mode started");
                } else {
                    info!("Tune mode stopped");
                }
                Response::Ok
            } else {
                Response::Error("State lock failed".to_string())
            }
        }
        Command::AutoTune { enabled } => {
            if let Ok(mut s) = state.lock() {
                s.auto_tune = *enabled;
                info!("Auto tune: {}", if *enabled { "ON" } else { "OFF" });
                Response::Ok
            } else {
                Response::Error("State lock failed".to_string())
            }
        }
        Command::CwId { callsign } => {
            if let Ok(mut s) = state.lock() {
                match callsign {
                    Some(call) => {
                        let call = call.to_uppercase();
                        if call.is_empty() {
                            s.cwid_enabled = false;
                            s.cwid_callsign = None;
                            info!("CW ID disabled");
                        } else {
                            s.cwid_enabled = true;
                            s.cwid_callsign = Some(call.clone());
                            info!("CW ID set to: {}", call);
                        }
                        Response::Ok
                    }
                    None => {
                        // Query current CW ID
                        if let Some(ref call) = s.cwid_callsign {
                            Response::Custom(format!("CWID {}", call))
                        } else {
                            Response::Custom("CWID OFF".to_string())
                        }
                    }
                }
            } else {
                Response::Error("State lock failed".to_string())
            }
        }
        Command::Busy => {
            if let Ok(s) = state.lock() {
                Response::Busy { detected: s.busy }
            } else {
                Response::Busy { detected: false }
            }
        }
        Command::Buffer => {
            if let Ok(s) = state.lock() {
                Response::Buffer { bytes: s.tx_buffer_bytes }
            } else {
                Response::Buffer { bytes: 0 }
            }
        }
        Command::Close => {
            info!("TCP command connection close requested");
            Response::Close
        }
        Command::Unknown(cmd_str) => {
            warn!("Unknown command received: {}", cmd_str);
            Response::Error(format!("Unknown command: {}", cmd_str))
        }
    }
}

/// Handle GUI events
fn handle_gui_event(event: GuiEvent, state: &Arc<Mutex<AppState>>) {
    match event {
        GuiEvent::Connect { callsign } => {
            if let Ok(mut s) = state.lock() {
                s.remote_callsign = Some(callsign);
                info!("Connect request initiated");
            }
        }
        GuiEvent::Disconnect => {
            if let Ok(mut s) = state.lock() {
                s.connected = false;
                s.remote_callsign = None;
                info!("Disconnect requested");
            }
        }
        GuiEvent::SetMyCall { callsign } => {
            if let Ok(mut s) = state.lock() {
                s.config.callsign = callsign.clone();
                info!("Callsign set to: {}", callsign);
            }
        }
        GuiEvent::SetListen { enabled } => {
            if let Ok(mut s) = state.lock() {
                s.listen = enabled;
                info!("Listen mode: {}", enabled);
            }
        }
        GuiEvent::SetBandwidth { mode } => {
            if let Ok(mut s) = state.lock() {
                s.config.bandwidth = match mode.as_str() {
                    "500" => Bandwidth::Hz500,
                    "2750" => Bandwidth::Hz2750,
                    _ => Bandwidth::Hz2300,
                };
                info!("Bandwidth set to: {} Hz", mode);
            }
        }
        GuiEvent::SetSpeedLevel { level } => {
            if let Ok(mut s) = state.lock() {
                // Clamp speed level for bandwidth (500Hz max is mode 13)
                s.current_speed_level = clamp_mode_for_bandwidth(level, s.config.bandwidth);
                info!("Speed level set to: {}", s.current_speed_level);
            }
        }
        GuiEvent::Quit => {
            info!("Quit requested");
            std::process::exit(0);
        }
        GuiEvent::StartTune => {
            if let Ok(mut s) = state.lock() {
                s.tune_mode = true;
                info!("Tune mode started");
            }
        }
        GuiEvent::StopTune => {
            if let Ok(mut s) = state.lock() {
                s.tune_mode = false;
                info!("Tune mode stopped");
            }
        }
        GuiEvent::OpenSettings => {
            info!("Settings dialog requested");
            // Settings dialog is handled in the GUI itself
        }
        GuiEvent::SetAudioInput { device } => {
            info!("Audio input device change requested: {}", device);
            if let Ok(mut s) = state.lock() {
                s.config.audio_input = if device == "Default" { None } else { Some(device.clone()) };
                s.pending_audio_input = Some(device);
            }
        }
        GuiEvent::SetAudioOutput { device } => {
            info!("Audio output device change requested: {}", device);
            if let Ok(mut s) = state.lock() {
                s.config.audio_output = if device == "Default" { None } else { Some(device.clone()) };
                s.pending_audio_output = Some(device);
            }
        }
        GuiEvent::SetAudioChannel { channel } => {
            info!("Audio channel changed to: {}", match channel {
                0 => "Left",
                1 => "Right",
                _ => "L+R",
            });
            // Channel selection is applied in audio processing
        }
        GuiEvent::SetDriveLevel { level } => {
            if let Ok(mut s) = state.lock() {
                s.drive_level = level;
                debug!("Drive level changed to: {:.2}", level);
            }
        }
        GuiEvent::SetTcpPorts { command, data, kiss } => {
            info!("TCP ports changed: cmd={}, data={}, kiss={}", command, data, kiss);
            if let Ok(mut s) = state.lock() {
                s.config.tcp_command_port = command;
                s.config.tcp_data_port = data;
                // Note: Changing ports requires restart to take effect
            }
        }
        GuiEvent::SetAccept500Hz { enabled } => {
            info!("Accept 500 Hz connections: {}", enabled);
            // This affects connection acceptance logic
        }
        GuiEvent::SetKissInterface { enabled } => {
            info!("KISS interface: {}", enabled);
            // This enables/disables the KISS TNC interface
        }
        GuiEvent::SetCwId { enabled } => {
            if let Ok(mut s) = state.lock() {
                s.cwid_enabled = enabled;
                info!("CW ID: {}", if enabled { "enabled" } else { "disabled" });
            }
        }
        GuiEvent::SetRetries { count } => {
            info!("Retries set to: {}", count);
            // This affects ARQ retry count
        }
    }
}

/// Load configuration from file
/// Supports RIA_CONFIG environment variable to override config path
/// Supports RIA_INSTANCE=N to offset ports by N*10 (e.g. N=1 -> ports 8310/8311)
fn load_config() -> Result<Config> {
    let config_path = if let Ok(path) = std::env::var("RIA_CONFIG") {
        std::path::PathBuf::from(path)
    } else {
        dirs::config_dir()
            .unwrap_or_else(|| std::path::PathBuf::from("."))
            .join("ria")
            .join("config.toml")
    };

    info!("Loading config from: {:?}", config_path);

    let mut config = if config_path.exists() {
        let content = std::fs::read_to_string(&config_path)?;
        parse_config(&content)?
    } else {
        info!("Config file not found, using defaults");
        Config::default()
    };

    // Support RIA_INSTANCE=N to offset ports
    if let Ok(instance_str) = std::env::var("RIA_INSTANCE") {
        if let Ok(instance) = instance_str.parse::<u16>() {
            let port_offset = instance * 10;
            config.tcp_command_port = 8300 + port_offset;
            config.tcp_data_port = 8301 + port_offset;
            info!("Instance {}: ports adjusted to {}/{}", instance, config.tcp_command_port, config.tcp_data_port);
        }
    }

    Ok(config)
}

/// Parse TOML configuration content
fn parse_config(content: &str) -> Result<Config> {
    let mut config = Config::default();

    for line in content.lines() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') || line.starts_with('[') {
            continue;
        }

        if let Some((key, value)) = line.split_once('=') {
            let key = key.trim();
            let value = value.trim().trim_matches('"').trim_matches('\'');

            match key {
                "callsign" => config.callsign = value.to_uppercase(),
                "tcp_command_port" => {
                    if let Ok(port) = value.parse() {
                        config.tcp_command_port = port;
                    }
                }
                "tcp_data_port" => {
                    if let Ok(port) = value.parse() {
                        config.tcp_data_port = port;
                    }
                }
                "audio_input" => {
                    if !value.is_empty() && value.to_lowercase() != "default" {
                        config.audio_input = Some(value.to_string());
                    }
                }
                "audio_output" => {
                    if !value.is_empty() && value.to_lowercase() != "default" {
                        config.audio_output = Some(value.to_string());
                    }
                }
                "sample_rate" => {
                    if let Ok(rate) = value.parse() {
                        config.sample_rate = rate;
                    }
                }
                "bandwidth" => {
                    config.bandwidth = match value {
                        "500" => Bandwidth::Hz500,
                        "2750" => Bandwidth::Hz2750,
                        _ => Bandwidth::Hz2300,
                    };
                }
                "default_speed_level" => {
                    if let Ok(level) = value.parse::<u8>() {
                        // Mode 1 disabled: needs soft-decision FSK decoding
                        config.default_speed_level = level.clamp(2, 17);
                    }
                }
                "ptt_method" => {
                    config.ptt_method = match value.to_lowercase().as_str() {
                        "vox" => PttMethod::Vox,
                        "rts" => PttMethod::Rts,
                        "dtr" => PttMethod::Dtr,
                        "none" => PttMethod::None,
                        _ => PttMethod::Vox,
                    };
                }
                "encryption_passphrase" | "passphrase" => {
                    config.encryption_passphrase = value.to_string();
                }
                "preamble_threshold" => {
                    if let Ok(threshold) = value.parse::<f32>() {
                        config.preamble_threshold = threshold.clamp(0.1, 0.9);
                    }
                }
                _ => {}
            }
        }
    }

    let enc_status = if config.encryption_passphrase.is_empty() { "disabled" } else { "enabled" };
    info!("Loaded config: callsign={}, bandwidth={} Hz, encryption={}",
          config.callsign, config.bandwidth.hz(), enc_status);
    Ok(config)
}

/// Convert main bandwidth to modem bandwidth
fn to_modem_bandwidth(bw: Bandwidth) -> ModemBandwidth {
    match bw {
        Bandwidth::Hz500 => ModemBandwidth::Narrow,
        Bandwidth::Hz2300 => ModemBandwidth::Wide,
        Bandwidth::Hz2750 => ModemBandwidth::Ultra,
    }
}

/// Convert main PttMethod to audio PttMethod
fn to_audio_ptt_method(method: PttMethod) -> AudioPttMethod {
    match method {
        PttMethod::Vox => AudioPttMethod::Vox,
        PttMethod::Rts => AudioPttMethod::Rts,
        PttMethod::Dtr => AudioPttMethod::Dtr,
        PttMethod::None => AudioPttMethod::None,
    }
}

/// Main modem processing loop (runs on dedicated thread)
fn run_modem_processing(
    config: Config,
    app_state: Arc<Mutex<AppState>>,
    gui_state: Arc<Mutex<GuiState>>,
) {
    info!("Starting modem processing...");

    // Initialize audio engine
    let mut audio_engine = match AudioEngine::new(&config) {
        Ok(engine) => engine,
        Err(e) => {
            error!("Failed to create audio engine: {}", e);
            return;
        }
    };

    if let Err(e) = audio_engine.start() {
        error!("Failed to start audio engine: {}", e);
        return;
    }

    info!("Audio engine started at {} Hz", config.sample_rate);

    // Initialize PTT controller
    let mut ptt = PttController::new(to_audio_ptt_method(config.ptt_method));

    // Initialize modem components
    // Use for_speed_level() to get correct FFT size for each mode
    // Modes 10-17 require FFT=1024 regardless of bandwidth setting
    let sample_rate = config.sample_rate as f32;
    let modem_bw = to_modem_bandwidth(config.bandwidth);
    let mut ofdm_config = OfdmConfig::for_speed_level(config.default_speed_level, modem_bw, sample_rate);

    let mut modulator = OfdmModulator::new(
        OfdmConfig::for_speed_level(config.default_speed_level, modem_bw, sample_rate),
        SpeedLevel::new_for_bandwidth(config.default_speed_level, config.bandwidth).constellation_for_bandwidth(config.bandwidth)
    );
    let mut demodulator = OfdmDemodulator::new(
        OfdmConfig::for_speed_level(config.default_speed_level, modem_bw, sample_rate),
        SpeedLevel::new_for_bandwidth(config.default_speed_level, config.bandwidth).constellation_for_bandwidth(config.bandwidth)
    );

    // FSK modulator/demodulator for modes 1-4
    let center_freq = 1500.0; // Center of HF passband
    let speed_level_obj = SpeedLevel::new(config.default_speed_level);
    let (mut fsk_modulator, mut fsk_demodulator) = if let Some((num_tones, symbol_rate)) = speed_level_obj.fsk_params() {
        info!("FSK mode: {} tones, {} sym/s", num_tones, symbol_rate);
        (
            Some(MfskModulator::with_tones(num_tones, sample_rate, center_freq, symbol_rate)),
            Some(MfskDemodulator::with_tones(num_tones, sample_rate, center_freq, symbol_rate)),
        )
    } else {
        (None, None)
    };
    let mut is_fsk_mode = speed_level_obj.is_fsk_mode();

    // Initialize preamble generator and detector
    // Use preamble mode based on FFT size from speed level, not bandwidth
    // Mode 17 uses FFT=1024 (Wide preamble) even with 2750Hz bandwidth
    use crate::protocol::FftSize;
    let preamble_mode = match speed_level_obj.fft_size() {
        FftSize::Fft2048 => PreambleMode::Narrow,
        FftSize::Fft1024 => PreambleMode::Wide,
        FftSize::Fft512 => PreambleMode::Ultra,
    };
    let mut preamble = Preamble::with_mode(sample_rate, preamble_mode, 0.35);
    let mut preamble_detector = PreambleDetector::new(&preamble, config.preamble_threshold);

    // Initialize FEC codec with block size from per-mode frame configuration
    // Each mode has its own FEC block size, symbol count, and payload size
    let mut current_frame_config = get_frame_config(config.default_speed_level, config.bandwidth);

    // Initialize protocol session with frame-duration-based ARQ timeout
    // This ensures slow modes like mode 1 (5.2s frames) have adequate time for ACKs
    use protocol::ArqConfig;
    let encryption_enabled = !config.encryption_passphrase.is_empty();
    let mut session = Session::new(SessionConfig {
        local_call: config.callsign.clone(),
        mode: ConnectionMode::P2P,
        speed_level: config.default_speed_level,
        arq: ArqConfig::for_frame_duration(current_frame_config.frame_duration_ms),
        encryption: EncryptionConfig::new(&config.encryption_passphrase, encryption_enabled),
        ..Default::default()
    });
    let mut current_code_rate = current_frame_config.code_rate;
    info!("Mode {}: {} symbols, {} bit FEC, {} byte payload, rate {:?}",
        current_frame_config.level, current_frame_config.symbols,
        current_frame_config.fec_block_bits, current_frame_config.max_payload_bytes,
        current_code_rate);
    let mut turbo_encoder = TurboEncoder::new_with_rate(current_frame_config.fec_block_bits, current_code_rate);
    let mut turbo_decoder = TurboDecoder::new_with_rate(current_frame_config.fec_block_bits, current_code_rate);
    turbo_decoder.set_iterations(8);

    // Signal metrics
    let mut metrics = SignalMetrics::default();
    let noise_floor_db = -60.0f32;

    // Initialize AFC (Automatic Frequency Control)
    // Center frequency is 1500 Hz (middle of HF passband)
    let mut afc = Afc::new(sample_rate, 1500.0);
    afc.set_max_offset(200.0);        // ±200 Hz max tracking
    afc.set_tracking_bandwidth(100.0); // 100 Hz tracking bandwidth
    afc.set_alpha(0.1);               // Smoothing factor
    let ofdm_symbol_period = ofdm_config.fft_size; // Symbol period for autocorrelation

    // FFT for spectrum display (512-point for waterfall)
    let spectrum_fft_size = 512;
    let mut spectrum_fft = Fft::new(spectrum_fft_size);
    let mut spectrum_data: Vec<f32> = vec![-60.0; spectrum_fft_size / 2];

    // Modem state
    let mut rx_buffer: Vec<f32> = Vec::with_capacity(config.sample_rate as usize);
    let mut tx_buffer: Vec<f32> = Vec::new();
    let mut pending_preamble_offset: Option<usize> = None; // Track detected but not yet processed preamble
    let mut pending_preamble_time: Option<Instant> = None; // When preamble was detected
    let pending_preamble_timeout = std::time::Duration::from_secs(10); // Max wait time for data (FSK mode 1 takes ~6s)
    let mut prev_listen_enabled = false; // Track listen state changes
    #[allow(unused_assignments)]
    let mut tune_mode = false;
    #[allow(unused_assignments)]
    let mut drive_level = 0.8f32;
    let mut last_cpu_check = Instant::now();
    let mut _cpu_samples = 0u64;
    let mut cpu_time = std::time::Duration::ZERO;

    // Tune mode phase accumulators (for continuous phase)
    let mut tune_phase1 = 0.0f32;
    let mut tune_phase2 = 0.0f32;
    let tune_freq1 = 1000.0f32; // 1000 Hz
    let tune_freq2 = 2000.0f32; // 2000 Hz
    let tune_phase_inc1 = 2.0 * std::f32::consts::PI * tune_freq1 / sample_rate;
    let tune_phase_inc2 = 2.0 * std::f32::consts::PI * tune_freq2 / sample_rate;

    // Status indicator state (with timeout for transient indicators)
    let mut status_start_until: Option<Instant> = None;
    let mut status_ack_until: Option<Instant> = None;
    let mut status_nack_until: Option<Instant> = None;
    let mut status_break_until: Option<Instant> = None;
    let indicator_display_duration = std::time::Duration::from_millis(500);

    // Bitrate calculation
    let mut last_bitrate_calc = Instant::now();
    let mut last_bytes_sent: u64 = 0;
    let mut last_bytes_received: u64 = 0;

    // Constellation points for GUI display
    let mut latest_constellation_points: Vec<(f32, f32)> = Vec::new();

    // Device switch settling period (ignore audio for brief period after switch)
    let mut device_switch_settling_until: Option<Instant> = None;
    let device_settling_duration = std::time::Duration::from_millis(200);

    // Track TX state for PTT transitions
    let mut was_transmitting = false;
    let mut tx_samples_sent: usize = 0; // Track samples sent this TX session for proper echo removal
    // TX cooldown: after TX ends, wait before processing RX (to let own signal fade)
    // Using large value to test if guard time is the issue
    let mut tx_cooldown_until: Option<Instant> = None;
    let mut tx_complete_time: Option<Instant> = None; // Track when TX finished for frame gap
    let tx_cooldown_duration = std::time::Duration::from_millis(150); // Short cooldown to clear own echo but receive response
    let mut last_data_frame_time: Option<Instant> = None; // Track when last data frame was sent for rate limiting

    // RX channel for receiving audio
    let rx_channel = audio_engine.rx_channel();
    let tx_channel = audio_engine.tx_channel();

    info!("Modem processing started");

    loop {
        let loop_start = Instant::now();

        // Check if we're in device settling period
        let is_settling = device_switch_settling_until
            .map(|t| Instant::now() < t)
            .unwrap_or(false);

        // Check if we're in TX cooldown - also skip buffering audio during cooldown
        // to prevent receiving our own echo via BlackHole loopback
        let in_tx_cooldown_early = tx_cooldown_until.map(|t| Instant::now() < t).unwrap_or(false);
        let is_transmitting_early = !tx_buffer.is_empty();

        // Read audio samples from RX channel
        while let Ok(samples) = rx_channel.try_recv() {
            // Skip audio processing during settling period, TX, or cooldown
            // This prevents receiving our own echo via BlackHole loopback
            if is_settling || is_transmitting_early || in_tx_cooldown_early {
                continue;
            }

            rx_buffer.extend_from_slice(&samples);

            // Update signal metrics periodically
            if rx_buffer.len() >= 1024 {
                let recent_samples = &rx_buffer[rx_buffer.len().saturating_sub(1024)..];
                metrics = dsp::calculate_metrics(recent_samples, noise_floor_db);
            }

            // Calculate spectrum for waterfall display
            if rx_buffer.len() >= spectrum_fft_size {
                let fft_samples = &rx_buffer[rx_buffer.len() - spectrum_fft_size..];
                let complex_samples: Vec<dsp::ComplexSample> = fft_samples
                    .iter()
                    .map(|&s| dsp::ComplexSample::new(s, 0.0))
                    .collect();
                let fft_result = spectrum_fft.forward_new(&complex_samples);

                // Convert to dB magnitude (only positive frequencies)
                spectrum_data = fft_result[0..spectrum_fft_size / 2]
                    .iter()
                    .map(|c| lin_to_db(c.norm() / spectrum_fft_size as f32))
                    .collect();
            }
        }

        // Update audio stats
        let audio_stats = audio_engine.stats();

        // Get current app state and check for pending device changes
        let (current_session_state, listen_enabled, speed_level, connecting_call, current_tune_mode, current_drive_level, pending_input, pending_output) = {
            if let Ok(mut state) = app_state.lock() {
                let pending_in = state.pending_audio_input.take();
                let pending_out = state.pending_audio_output.take();
                (state.session_state, state.listen, state.current_speed_level,
                 state.remote_callsign.clone(), state.tune_mode, state.drive_level,
                 pending_in, pending_out)
            } else {
                (SessionState::Disconnected, true, 9, None, false, 0.8, None, None)
            }
        };
        tune_mode = current_tune_mode;
        drive_level = current_drive_level;

        // Update frame config if speed level changed
        if speed_level != current_frame_config.level {
            current_frame_config = get_frame_config(speed_level, config.bandwidth);
            current_code_rate = current_frame_config.code_rate;
            turbo_encoder = TurboEncoder::new_with_rate(current_frame_config.fec_block_bits, current_code_rate);
            turbo_decoder = TurboDecoder::new_with_rate(current_frame_config.fec_block_bits, current_code_rate);
            turbo_decoder.set_iterations(8);

            // Update FSK modulator/demodulator if mode changes
            // Use new_for_bandwidth to clamp speed level for 500Hz (max mode 13)
            let new_speed_level = SpeedLevel::new_for_bandwidth(speed_level, config.bandwidth);
            is_fsk_mode = new_speed_level.is_fsk_mode();
            if let Some((num_tones, symbol_rate)) = new_speed_level.fsk_params() {
                info!("FSK mode: {} tones, {} sym/s", num_tones, symbol_rate);
                fsk_modulator = Some(MfskModulator::with_tones(num_tones, sample_rate, center_freq, symbol_rate));
                fsk_demodulator = Some(MfskDemodulator::with_tones(num_tones, sample_rate, center_freq, symbol_rate));
            } else {
                fsk_modulator = None;
                fsk_demodulator = None;
            }

            // Recreate OFDM config, modulator, and demodulator for new mode
            // This ensures correct FFT size and carrier positions for modes 5-9 (FFT=512)
            // vs modes 10-17 (FFT=1024)
            // Use constellation_for_bandwidth to get per-bandwidth modulation
            ofdm_config = OfdmConfig::for_speed_level(speed_level, modem_bw, sample_rate);
            modulator = OfdmModulator::new(
                OfdmConfig::for_speed_level(speed_level, modem_bw, sample_rate),
                new_speed_level.constellation_for_bandwidth(config.bandwidth)
            );
            demodulator = OfdmDemodulator::new(
                OfdmConfig::for_speed_level(speed_level, modem_bw, sample_rate),
                new_speed_level.constellation_for_bandwidth(config.bandwidth)
            );
            info!("Recreated OFDM config/mod/demod for mode {} (FFT={}, carriers={})",
                speed_level, ofdm_config.fft_size, ofdm_config.num_carriers);

            // Update preamble and detector for new FFT size
            // Different modes use different preamble symbol lengths
            let new_preamble_mode = match new_speed_level.fft_size() {
                FftSize::Fft2048 => PreambleMode::Narrow,
                FftSize::Fft1024 => PreambleMode::Wide,
                FftSize::Fft512 => PreambleMode::Ultra,
            };
            preamble = Preamble::with_mode(sample_rate, new_preamble_mode, 0.35);
            preamble_detector = PreambleDetector::new(&preamble, config.preamble_threshold);
            info!("Recreated preamble for mode {} ({:?}, {} symbols)",
                speed_level, new_preamble_mode, preamble.num_symbols());

            // Update ARQ timeout based on new frame duration
            session.update_arq_config(ArqConfig::for_frame_duration(current_frame_config.frame_duration_ms));
            debug!("Updated ARQ timeout for {} ms frame duration", current_frame_config.frame_duration_ms);

            info!("Mode {}: {} symbols, {} bit FEC, {} byte payload, rate {:?}{}",
                current_frame_config.level, current_frame_config.symbols,
                current_frame_config.fec_block_bits, current_frame_config.max_payload_bytes,
                current_code_rate, if is_fsk_mode { " (FSK)" } else { "" });
        }

        // Handle audio device changes
        if let Some(ref device_name) = pending_input {
            info!("Processing pending input device change to: {}", device_name);
            let name = if device_name == "Default" { None } else { Some(device_name.as_str()) };
            match audio_engine.set_input_device(name) {
                Ok(()) => info!("Input device change successful"),
                Err(e) => error!("Failed to change input device: {}", e),
            }
            // Clear the rx_buffer in the modem to avoid stale samples
            rx_buffer.clear();
            // Drain any pending samples in the channel
            while rx_channel.try_recv().is_ok() {}
            // Reset spectrum data
            spectrum_data = vec![-60.0; spectrum_fft_size / 2];
            // Set settling period to ignore initial audio artifacts
            device_switch_settling_until = Some(Instant::now() + device_settling_duration);
            info!("Cleared RX buffers and channel, settling for {:?}", device_settling_duration);
        }
        if let Some(ref device_name) = pending_output {
            info!("Processing pending output device change to: {}", device_name);
            let name = if device_name == "Default" { None } else { Some(device_name.as_str()) };
            match audio_engine.set_output_device(name) {
                Ok(()) => info!("Output device change successful"),
                Err(e) => error!("Failed to change output device: {}", e),
            }
            // Clear the tx_buffer to avoid stale samples
            tx_buffer.clear();
            info!("Cleared TX buffer");
        }

        // Update speed level in modulator/demodulator (use bandwidth-aware constellation)
        let speed = SpeedLevel::new_for_bandwidth(speed_level, config.bandwidth);
        modulator.set_constellation(speed.constellation_for_bandwidth(config.bandwidth));
        demodulator.set_constellation(speed.constellation_for_bandwidth(config.bandwidth));

        // Process RX: preamble detection and demodulation
        // Skip RX processing when transmitting or during cooldown to avoid receiving own signal
        let is_transmitting = !tx_buffer.is_empty();
        let in_tx_cooldown = tx_cooldown_until.map(|t| Instant::now() < t).unwrap_or(false);

        // Clear RX buffer when listen is just enabled to avoid stale data
        if listen_enabled && !prev_listen_enabled {
            rx_buffer.clear();
            pending_preamble_offset = None;
            pending_preamble_time = None;
            preamble_detector.reset();
        }
        prev_listen_enabled = listen_enabled;

        if listen_enabled && !is_transmitting && !in_tx_cooldown && rx_buffer.len() >= ofdm_config.fft_size + ofdm_config.cp_length {
            // Check for preamble timeout
            if let (Some(pending_offset), Some(t)) = (pending_preamble_offset, pending_preamble_time) {
                if t.elapsed() > pending_preamble_timeout {
                    debug!("Pending preamble timed out, draining and clearing");
                    // Drain up to the preamble position to skip past bad data
                    let preamble_start = pending_offset.saturating_sub(16368); // preamble_samples
                    if preamble_start < rx_buffer.len() {
                        rx_buffer.drain(0..preamble_start.min(rx_buffer.len()));
                    }
                    pending_preamble_offset = None;
                    pending_preamble_time = None;
                }
            }

            // Check for preamble (only if we don't have one pending)
            let offset = if let Some(pending) = pending_preamble_offset {
                // Already have a detected preamble, wait for enough data
                Some(pending)
            } else if let Some(result) = preamble_detector.process(&rx_buffer) {
                debug!("Preamble detected: preamble_start={}, data_start={}, correlation={:.3}",
                    result.preamble_start, result.data_start, result.correlation);

                pending_preamble_offset = Some(result.data_start);
                pending_preamble_time = Some(Instant::now());
                Some(result.data_start)
            } else {
                None
            };

            if let Some(offset) = offset {

                // Reset demodulator state for new frame
                demodulator.reset();

                // Update busy state
                if let Ok(mut state) = app_state.lock() {
                    state.busy = true;
                    state.rx_active = true;
                }

                // Demodulate following data
                // For OFDM: wait for enough symbols per frame config; for FSK: we check inside the FSK block
                let min_ofdm_samples = (ofdm_config.fft_size + ofdm_config.cp_length) * current_frame_config.symbols;
                // For FSK mode, always enter the block and let FSK code check sample count
                let have_enough = if is_fsk_mode {
                    rx_buffer.len() > offset // Just need some data after preamble for FSK to check
                } else {
                    rx_buffer.len() >= offset + min_ofdm_samples
                };
                debug!("Buffer len={}, offset={}, need={}, is_fsk={}", rx_buffer.len(), offset, offset + min_ofdm_samples, is_fsk_mode);
                if have_enough {
                    // DEBUG: Record RX buffer AFTER we have enough data for full frame
                    static RX_RECORDED: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);
                    if !RX_RECORDED.load(std::sync::atomic::Ordering::Relaxed) {
                        RX_RECORDED.store(true, std::sync::atomic::Ordering::Relaxed);
                        eprintln!("[RX-RECORD] Recording {} samples (offset={}, need={}) to rx_buffer.wav",
                            rx_buffer.len(), offset, offset + min_ofdm_samples);
                        if let Ok(file) = std::fs::File::create("test_debug/rx_buffer.wav") {
                            use std::io::Write;
                            let mut writer = std::io::BufWriter::new(file);
                            let num_samples = rx_buffer.len();
                            let data_size = (num_samples * 2) as u32;
                            let file_size = 36 + data_size;
                            let _ = writer.write_all(b"RIFF");
                            let _ = writer.write_all(&file_size.to_le_bytes());
                            let _ = writer.write_all(b"WAVE");
                            let _ = writer.write_all(b"fmt ");
                            let _ = writer.write_all(&16u32.to_le_bytes());
                            let _ = writer.write_all(&1u16.to_le_bytes());
                            let _ = writer.write_all(&1u16.to_le_bytes());
                            let _ = writer.write_all(&48000u32.to_le_bytes());
                            let _ = writer.write_all(&96000u32.to_le_bytes());
                            let _ = writer.write_all(&2u16.to_le_bytes());
                            let _ = writer.write_all(&16u16.to_le_bytes());
                            let _ = writer.write_all(b"data");
                            let _ = writer.write_all(&data_size.to_le_bytes());
                            for &sample in rx_buffer.iter() {
                                let s16 = (sample * 32767.0).clamp(-32768.0, 32767.0) as i16;
                                let _ = writer.write_all(&s16.to_le_bytes());
                            }
                            eprintln!("[RX-RECORD] WAV file written: {} samples", num_samples);
                        }
                    }
                    let data_samples = &rx_buffer[offset..];
                    let symbol_len = ofdm_config.fft_size + ofdm_config.cp_length;
                    let num_symbols_raw = data_samples.len() / symbol_len;
                    // CRITICAL: Limit symbols to match per-mode TX frame structure
                    // Processing more symbols would advance pilot phase beyond frame boundary
                    let max_symbols = current_frame_config.symbols;
                    let num_symbols = num_symbols_raw.min(max_symbols);

                    // Calculate RMS of data section to verify signal is present
                    let data_rms: f32 = (data_samples.iter().take(symbol_len).map(|s| s * s).sum::<f32>() / symbol_len as f32).sqrt();
                    debug!("After preamble: {} data samples, symbol_len={}, {} symbols available, data_rms={:.4}",
                           data_samples.len(), symbol_len, num_symbols, data_rms);

                    // Skip decode if no signal in data section
                    if data_rms < 0.01 {
                        debug!("Skipping decode: data_rms too low ({:.4})", data_rms);
                        // Clear pending preamble - this was a false detection
                        pending_preamble_offset = None;
                        pending_preamble_time = None;
                    } else if num_symbols > 0 {
                        // FSK mode uses different demodulation path
                        if is_fsk_mode {
                            if let Some(ref mut fsk_demod) = fsk_demodulator {
                                // Calculate samples needed for full FSK frame
                                let block_size = turbo_decoder.block_size();
                                let encoded_bits_expected = current_code_rate.output_bits(block_size) + 16;
                                let bits_per_symbol = (fsk_demod.num_tones() as f32).log2() as usize;
                                let fsk_symbols_needed = (encoded_bits_expected + bits_per_symbol - 1) / bits_per_symbol;
                                let samples_per_fsk_symbol = (sample_rate / fsk_demod.symbol_rate()) as usize;
                                let fsk_samples_needed = fsk_symbols_needed * samples_per_fsk_symbol;

                                // Wait for enough samples before attempting decode
                                if data_samples.len() < fsk_samples_needed {
                                    // Don't clear pending_preamble - keep waiting
                                } else {
                                    // We have enough samples - attempt decode
                                    // FSK demodulation - convert hard bits to pseudo-soft bits
                                    let hard_bits = fsk_demod.demodulate_bits(data_samples);

                                    // Convert hard bits to soft bits for turbo decoder
                                    // Use +1.0 for bit 0, -1.0 for bit 1 (turbo decoder convention)
                                    let all_soft_bits: Vec<f32> = hard_bits.iter()
                                        .map(|&b| if b == 0 { 1.0 } else { -1.0 })
                                        .collect();

                                    if all_soft_bits.len() >= encoded_bits_expected {
                                        let decoded_bits = turbo_decoder.decode(&all_soft_bits[..encoded_bits_expected]);

                                        let decoded_bytes: Vec<u8> = decoded_bits.chunks(8)
                                            .map(|chunk| {
                                                chunk.iter().enumerate().fold(0u8, |acc, (i, &b)| {
                                                    acc | ((b & 1) << (7 - i))
                                                })
                                            })
                                            .collect();

                                        match Frame::decode(&decoded_bytes) {
                                            Ok(frame) => {
                                                info!("FSK Received frame type: {:?}", frame.frame_type());
                                                if let Err(e) = session.receive(frame) {
                                                    warn!("Session receive error: {}", e);
                                                }
                                            }
                                            Err(e) => {
                                                debug!("FSK frame decode failed: {}", e);
                                            }
                                        }
                                    }
                                    // Clear pending preamble and drain only the decoded frame
                                    // Don't clear entire buffer - there may be incoming frames
                                    pending_preamble_offset = None;
                                    pending_preamble_time = None;

                                    // Calculate FSK frame length to drain
                                    // Preamble + FSK data symbols
                                    let preamble_len = preamble.samples().len();
                                    let fsk_frame_len = fsk_samples_needed;
                                    let total_frame_len = preamble_len + fsk_frame_len;

                                    // Drain just past the decoded frame with 10% margin
                                    let drain_len = ((total_frame_len as f32 * 1.1) as usize).min(rx_buffer.len());
                                    rx_buffer.drain(0..drain_len);

                                    // Reset preamble detector for clean state
                                    preamble_detector.reset();
                                    debug!("FSK: Drained {} samples after decode (preamble {} + FSK {}), reset preamble detector",
                                        drain_len, preamble_len, fsk_frame_len);
                                }
                            }
                        } else {
                        // OFDM mode: Apply AFC and demodulate
                        let mut complex_samples = real_to_complex(data_samples);

                        // Estimate frequency offset using autocorrelation at symbol period
                        let freq_error = afc.estimate_offset(&complex_samples, ofdm_symbol_period);
                        afc.update(freq_error);

                        // Apply frequency correction to all samples
                        afc.correct_block(&mut complex_samples);

                        // Convert back to real for demodulation
                        let corrected_samples = complex_to_real(&complex_samples);

                        // Find optimal timing offset using cyclic prefix correlation
                        // CP is first cp_len samples, which are a copy of the last cp_len of the body
                        // Symbol structure: [CP (128)] [Body (1024)] where CP = Body[896..1024]
                        let cp_len = ofdm_config.cp_length;
                        let fft_size = ofdm_config.fft_size;
                        let search_range = symbol_len.min(corrected_samples.len().saturating_sub(symbol_len));
                        let mut best_offset = 0;
                        let mut best_corr = 0.0f32;

                        for offset in 0..search_range {
                            // Correlate CP (at offset) with end of symbol body (at offset + fft_size)
                            let mut corr = 0.0f32;
                            let mut energy_a = 0.0f32;
                            let mut energy_b = 0.0f32;
                            for i in 0..cp_len {
                                let a_idx = offset + i;
                                let b_idx = offset + fft_size + i; // fft_size samples after CP start = end of body
                                if b_idx < corrected_samples.len() {
                                    let a = corrected_samples[a_idx];
                                    let b = corrected_samples[b_idx];
                                    corr += a * b;
                                    energy_a += a * a;
                                    energy_b += b * b;
                                }
                            }
                            let norm_corr = if energy_a > 0.0 && energy_b > 0.0 {
                                corr / (energy_a.sqrt() * energy_b.sqrt())
                            } else { 0.0 };
                            if norm_corr > best_corr {
                                best_corr = norm_corr;
                                best_offset = offset;
                            }
                        }
                        debug!("CP timing: best_offset={}, correlation={:.3}", best_offset, best_corr);

                        // Only proceed if we have reasonable symbol timing (correlation > 0.5)
                        if best_corr < 0.5 {
                            debug!("Poor CP correlation ({:.3}), skipping decode", best_corr);
                            // Clear pending preamble - this was likely a false detection
                            pending_preamble_offset = None;
                            pending_preamble_time = None;
                        } else {

                        // Demodulate ALL OFDM symbols (not just one) starting from best offset
                        let mut all_soft_bits = Vec::new();
                        for sym_idx in 0..num_symbols {
                            let sym_start = best_offset + sym_idx * symbol_len;
                            let sym_end = sym_start + symbol_len;
                            if sym_end <= corrected_samples.len() {
                                let sym_samples = &corrected_samples[sym_start..sym_end];
                                let soft_bits = demodulator.demodulate_symbol_soft(sym_samples, 0.1);
                                all_soft_bits.extend(soft_bits);
                            }
                        }

                        // Get constellation points for GUI display
                        if !demodulator.constellation_points().is_empty() {
                            latest_constellation_points = demodulator.constellation_points().to_vec();
                            // Debug: log constellation spread
                            if latest_constellation_points.len() >= 4 {
                                let avg_re: f32 = latest_constellation_points.iter().map(|p| p.0.abs()).sum::<f32>() / latest_constellation_points.len() as f32;
                                let avg_im: f32 = latest_constellation_points.iter().map(|p| p.1.abs()).sum::<f32>() / latest_constellation_points.len() as f32;
                                debug!("Constellation: {} points, avg_mag=({:.3}, {:.3})", latest_constellation_points.len(), avg_re, avg_im);
                            }
                        }

                        // Decode with FEC if we have enough bits for a block
                        let block_size = turbo_decoder.block_size();

                        // Debug: check soft bit statistics
                        if !all_soft_bits.is_empty() {
                            let avg: f32 = all_soft_bits.iter().sum::<f32>() / all_soft_bits.len() as f32;
                            let max = all_soft_bits.iter().cloned().fold(f32::NEG_INFINITY, f32::max);
                            let min = all_soft_bits.iter().cloned().fold(f32::INFINITY, f32::min);
                            debug!("Soft bits: count={}, avg={:.3}, min={:.3}, max={:.3}",
                                   all_soft_bits.len(), avg, min, max);
                        }

                        // Calculate expected encoded bits based on code rate (plus 16 tail bits)
                        let encoded_bits_expected = current_code_rate.output_bits(block_size) + 16;
                        debug!("Demodulated {} soft bits from {} symbols, need {} for turbo decode (rate {:?})",
                               all_soft_bits.len(), num_symbols, encoded_bits_expected, current_code_rate);
                        if all_soft_bits.len() >= encoded_bits_expected {
                            // Decode turbo code (pass all received bits to decoder which will demux based on rate)
                            let decoded_bits = turbo_decoder.decode(&all_soft_bits[..encoded_bits_expected]);

                            let decoded_bytes: Vec<u8> = decoded_bits.chunks(8)
                                .map(|chunk| {
                                    chunk.iter().enumerate().fold(0u8, |acc, (i, &b)| {
                                        acc | ((b & 1) << (7 - i))
                                    })
                                })
                                .collect();

                            debug!("Turbo decoded {} bytes: {:02x?}", decoded_bytes.len(), &decoded_bytes);

                            // Parse as frame
                            match Frame::decode(&decoded_bytes) {
                                Ok(frame) => {
                                    info!("Received frame type: {:?}", frame.frame_type());

                                    // Process frame through session
                                    if let Err(e) = session.receive(frame) {
                                        warn!("Session receive error: {}", e);
                                    }

                                    // Update statistics
                                    if let Ok(mut state) = app_state.lock() {
                                        state.frames_received += 1;
                                        state.bytes_received = session.bytes_received();
                                    }

                                    // Clear entire RX buffer after successful decode
                                    // This ensures clean state for the next frame and prevents
                                    // old samples from interfering with timing sync
                                    let drained = rx_buffer.len();
                                    rx_buffer.clear();
                                    pending_preamble_offset = None;
                                    pending_preamble_time = None;
                                    // Also drain any pending audio in the channel
                                    while rx_channel.try_recv().is_ok() {}
                                    // Reset preamble detector for clean state on next frame
                                    preamble_detector.reset();
                                    debug!("Cleared {} samples after successful decode, reset preamble detector", drained);
                                }
                                Err(e) => {
                                    debug!("Frame decode failed: {:?}", e);
                                    // Clear pending preamble on failure and drain past the failed detection
                                    // to prevent re-detecting the same false preamble
                                    pending_preamble_offset = None;
                                    pending_preamble_time = None;
                                    // Drain full estimated frame size (preamble + 48 OFDM symbols) to skip corrupted frame
                                    let preamble_len = preamble.samples().len();
                                    let ofdm_frame_len = 48 * (ofdm_config.fft_size + ofdm_config.cp_length); // 48 OFDM symbols
                                    let full_frame_len = preamble_len + ofdm_frame_len;
                                    let drain_len = full_frame_len.min(rx_buffer.len());
                                    rx_buffer.drain(0..drain_len);
                                    debug!("Drained {} samples after failed decode (preamble {} + OFDM {})", drain_len, preamble_len, ofdm_frame_len);
                                    preamble_detector.reset();
                                }
                            }
                        }
                    } // end if best_corr >= 0.5
                    }
                } // end else (OFDM mode)
                }

                // Note: Buffer draining is now handled inside the successful decode path
                // We don't drain on failed decodes because the "detected preamble" might be
                // a false positive, and draining would remove the actual incoming signal
            }

            // Trim RX buffer if too large (8 seconds to allow for FSK mode 1 frames which are ~5.5s)
            let max_rx_buffer = config.sample_rate as usize * 8;
            if rx_buffer.len() > max_rx_buffer {
                let drain_count = rx_buffer.len() - max_rx_buffer / 2;
                rx_buffer.drain(0..drain_count);

                // Adjust pending_preamble_offset since we removed samples from the front
                if let Some(ref mut offset) = pending_preamble_offset {
                    if *offset >= drain_count {
                        *offset -= drain_count;
                    } else {
                        // Preamble was in the drained portion, invalidate it
                        debug!("Pending preamble invalidated by buffer trim (offset {} < drain {})", *offset, drain_count);
                        pending_preamble_offset = None;
                        pending_preamble_time = None;
                    }
                }
            }

            // Trim TX buffer if too large (prevent memory exhaustion)
            let max_tx_buffer = config.sample_rate as usize * 2;
            if tx_buffer.len() > max_tx_buffer {
                warn!("TX buffer overflow, trimming {} samples", tx_buffer.len() - max_tx_buffer / 2);
                tx_buffer.drain(0..tx_buffer.len() - max_tx_buffer / 2);
            }
        }

        // Process session events
        for event in session.get_events() {
            match event {
                SessionEvent::Connected { remote_call } => {
                    info!("Connected to {}", remote_call);
                    // Set START indicator for initial handshake completion
                    status_start_until = Some(Instant::now() + indicator_display_duration);
                    // Reset AFC for new connection to start fresh
                    afc.reset();
                    if let Ok(mut state) = app_state.lock() {
                        state.connected = true;
                        state.session_state = SessionState::Connected;
                        state.remote_callsign = Some(remote_call);
                        state.busy = false;
                        // Start connection timer
                        state.connection_start = Some(Instant::now());
                        state.connection_time_secs = 0.0;
                    }
                }
                SessionEvent::ConnectRequest { remote_call } => {
                    info!("Connection request from {}", remote_call);
                    // Auto-accept if in listen mode
                    if listen_enabled {
                        if let Some(ack_frame) = session.accept_pending() {
                            info!("Auto-accepting connection from {}", remote_call);
                            // Add preamble and modulated ACK to TX buffer
                            tx_buffer.extend_from_slice(preamble.samples());

                            let frame_bytes = ack_frame.encode();
                            debug!("TX ACK Frame: {} bytes", frame_bytes.len());
                            let bits: Vec<u8> = frame_bytes.iter()
                                .flat_map(|&b| (0..8).rev().map(move |i| (b >> i) & 1))
                                .collect();
                            let block_size = turbo_encoder.block_size();
                            let mut padded_bits = bits.clone();
                            padded_bits.resize(block_size, 0);
                            let encoded = turbo_encoder.encode(&padded_bits);
                            let modulated = if is_fsk_mode {
                                if let Some(ref mut fsk_mod) = fsk_modulator {
                                    fsk_mod.reset();
                                    fsk_mod.modulate_bits(&encoded)
                                } else {
                                    modulator.reset();
                                    modulator.modulate(&encoded)
                                }
                            } else {
                                modulator.reset();
                                modulator.modulate(&encoded)
                            };
                            tx_buffer.extend_from_slice(&modulated);

                            if let Ok(mut state) = app_state.lock() {
                                state.frames_sent += 1;
                                state.session_state = SessionState::Connected;
                                state.remote_callsign = Some(remote_call);
                                state.connected = true;
                            }
                        }
                    }
                }
                SessionEvent::Disconnected { reason } => {
                    info!("Disconnected: {}", reason);
                    // Set BREAK indicator if disconnection was unexpected
                    if reason.contains("Timeout") || reason.contains("broken") {
                        status_break_until = Some(Instant::now() + indicator_display_duration * 2);
                    }
                    // Reset AFC on disconnect
                    afc.reset();
                    if let Ok(mut state) = app_state.lock() {
                        state.connected = false;
                        state.session_state = SessionState::Disconnected;
                        state.remote_callsign = None;
                        state.busy = false;
                        // Clear connection timer
                        state.connection_start = None;
                        state.connection_time_secs = 0.0;
                    }
                }
                SessionEvent::DataReceived { data } => {
                    debug!("Received {} bytes of data", data.len());
                    // ACK will be sent in response
                    status_ack_until = Some(Instant::now() + indicator_display_duration);
                    if let Ok(mut state) = app_state.lock() {
                        state.rx_buffer_bytes += data.len();
                        // Queue data for sending to TCP client
                        state.rx_data_queue.extend_from_slice(&data);
                    }
                }
                SessionEvent::SpeedChanged { level } => {
                    // Clamp speed level for bandwidth (500Hz max is mode 13)
                    let clamped_level = clamp_mode_for_bandwidth(level, config.bandwidth);
                    info!("Speed changed to level {}", clamped_level);
                    if let Ok(mut state) = app_state.lock() {
                        state.current_speed_level = clamped_level;
                    }
                }
                SessionEvent::Error { message } => {
                    warn!("Session error: {}", message);
                    // NACK or error condition
                    status_nack_until = Some(Instant::now() + indicator_display_duration);
                }
                SessionEvent::ReadyToSend => {
                    // Session is ready to accept more data for transmission
                    // Check if there's queued TX data from TCP and send it
                    debug!("Session ready to send more data");
                    if let Ok(state) = app_state.lock() {
                        if !state.tx_data_queue.is_empty() {
                            // Data is available - it will be processed in the next loop iteration
                            debug!("TX data queue has {} bytes pending", state.tx_data_queue.len());
                        }
                    }
                }
            }
        }

        // Handle tune mode - generate continuous two-tone signal
        // Only generate if buffer is low to prevent memory exhaustion
        let tune_buffer_threshold = (sample_rate * 0.1) as usize; // 100ms of audio
        if tune_mode && tx_buffer.len() < tune_buffer_threshold {
            // Generate two-tone test signal with continuous phase
            let tune_duration_samples = (sample_rate * 0.05) as usize; // 50ms chunks
            let mut tune_samples = Vec::with_capacity(tune_duration_samples);

            for _ in 0..tune_duration_samples {
                // Two-tone signal at 1000 Hz and 2000 Hz with continuous phase
                let sample = 0.5 * tune_phase1.sin() + 0.5 * tune_phase2.sin();
                tune_samples.push(sample * drive_level);

                // Advance phases using proper modulo arithmetic
                tune_phase1 = wrap_phase_positive(tune_phase1 + tune_phase_inc1);
                tune_phase2 = wrap_phase_positive(tune_phase2 + tune_phase_inc2);
            }
            tx_buffer.extend_from_slice(&tune_samples);
        } else if !tune_mode {
            // Reset tune phases when tune mode is off
            tune_phase1 = 0.0;
            tune_phase2 = 0.0;
        }

        // Process TX: check if we need to transmit
        // Use session.state() for decisions - it's the authoritative state
        let session_state = session.state();

        // Debug: log when we have data
        if let Ok(state) = app_state.lock() {
            if !state.tx_data_queue.is_empty() {
                debug!("TX queue has {} bytes, session is {:?}", state.tx_data_queue.len(), session_state);
            }
        }

        if !tune_mode {
        // Handle connection initiation: app_state says Connecting, session is still Disconnected
        if current_session_state == SessionState::Connecting && session_state == SessionState::Disconnected {
            if tx_buffer.is_empty() {
                if let Some(ref remote_call) = connecting_call {
                    info!("Generating connection request to {}", remote_call);

                    // Generate connect frame FIRST (before preamble)
                    match session.connect(remote_call) {
                        Ok(frame) => {
                            // Add preamble first
                            let preamble_samples = preamble.samples();
                            let preamble_len = preamble_samples.len();
                            tx_buffer.extend_from_slice(preamble_samples);

                            let frame_bytes = frame.encode();
                            debug!("TX Frame: {} bytes", frame_bytes.len());
                            // Convert bytes to bits for turbo encoding
                            let bits: Vec<u8> = frame_bytes.iter()
                                .flat_map(|&b| (0..8).rev().map(move |i| (b >> i) & 1))
                                .collect();
                            // Pad to block size
                            let block_size = turbo_encoder.block_size();
                            let mut padded_bits = bits.clone();
                            padded_bits.resize(block_size, 0);
                            let encoded = turbo_encoder.encode(&padded_bits);
                            debug!("Turbo encoded: {} bits -> {} bits", block_size, encoded.len());
                            let modulated = if is_fsk_mode {
                                if let Some(ref mut fsk_mod) = fsk_modulator {
                                    fsk_mod.reset();
                                    fsk_mod.modulate_bits(&encoded)
                                } else {
                                    modulator.reset();
                                    modulator.modulate(&encoded)
                                }
                            } else {
                                modulator.reset();
                                modulator.modulate(&encoded)
                            };
                            debug!("TX: preamble={} samples, data={} samples, total={}",
                                   preamble_len, modulated.len(), preamble_len + modulated.len());
                            tx_buffer.extend_from_slice(&modulated);

                            if let Ok(mut state) = app_state.lock() {
                                state.frames_sent += 1;
                            }
                        }
                        Err(e) => {
                            warn!("Failed to generate connect frame: {}", e);
                            // Reset app state on error
                            if let Ok(mut state) = app_state.lock() {
                                state.session_state = SessionState::Disconnected;
                                state.busy = false;
                            }
                        }
                    }
                }
            }
        }

        // Handle session state (separate from connection initiation)
        if session_state == SessionState::Connecting {
            // Session is waiting for connection ACK - don't do anything, just wait
            // The receive logic will handle the ACK
            debug!("Waiting for connection ACK...");
        } else if session_state == SessionState::Connected {
                // Queue TX data from TCP client to session
                {
                    let mut data_to_send = Vec::new();
                    let mut queue_len = 0;
                    match app_state.lock() {
                        Ok(mut state) => {
                            queue_len = state.tx_data_queue.len();
                            if !state.tx_data_queue.is_empty() {
                                // Take up to max_payload_bytes for current mode
                                // Account for encryption overhead (28 bytes: 12 nonce + 16 GCM tag)
                                let encryption_overhead = if encryption_enabled { 28 } else { 0 };
                                let max_payload = current_frame_config.max_payload_bytes.saturating_sub(encryption_overhead);
                                let take_len = state.tx_data_queue.len().min(max_payload);
                                data_to_send = state.tx_data_queue.drain(..take_len).collect();
                                debug!("Took {} bytes from TX queue (max_payload={})", take_len, max_payload);
                            }
                        }
                        Err(e) => {
                            warn!("Failed to lock app_state for TX data: {}", e);
                        }
                    }
                    if !data_to_send.is_empty() {
                        let len = data_to_send.len();
                        debug!("Processing {} bytes from TX queue (was {} total)", len, queue_len);
                        if let Err(e) = session.send(data_to_send) {
                            warn!("Failed to queue data for transmission: {}", e);
                        } else {
                            debug!("Queued {} bytes for transmission", len);
                        }
                    }
                }

                // Get next frame from session
                // Wait for frame_gap after TX completion before sending next DATA frame
                // This allows time for the remote station to send ACK
                // Only send ONE data frame at a time, then wait for frame_gap before next
                let frame_gap = std::time::Duration::from_millis(800); // Gap for HF radio (ACK TX ~500ms + radio turnaround ~100ms + margin)
                let time_since_data_frame = last_data_frame_time.map(|t| t.elapsed());
                let can_send_data_frame = time_since_data_frame
                    .map(|elapsed| elapsed >= frame_gap)
                    .unwrap_or(true); // First transmission

                // Also wait for TX completion before sending next frame
                let time_since_tx = tx_complete_time.map(|t| t.elapsed());
                let tx_cooldown_over = time_since_tx
                    .map(|elapsed| elapsed >= tx_cooldown_duration)
                    .unwrap_or(true);

                let can_send_next = can_send_data_frame && tx_cooldown_over;
                // Don't transmit while receiving a frame (especially important for slow FSK modes)
                let receiving_frame = pending_preamble_offset.is_some();

                // Debug: log when we can't send
                if !tx_buffer.is_empty() || !can_send_next {
                    let tx_buf_len = tx_buffer.len();
                    let time_ms = time_since_data_frame.map(|d| d.as_millis()).unwrap_or(9999);
                    if tx_buf_len > 0 || time_ms < 1500 {
                        debug!("TX blocked: tx_buffer={}, time_since_data_frame={}ms (need {}ms)",
                               tx_buf_len, time_ms, frame_gap.as_millis());
                    }
                }

                if tx_buffer.is_empty() && can_send_next && !receiving_frame {
                    debug!("TX: can generate frames (tx_buffer empty, can_send_next=true, not receiving)");
                    if let Some(frame) = session.get_tx_frame() {
                        debug!("Got TX frame from session: type={:?}, seq={}", frame.frame_type(), frame.sequence());
                        last_data_frame_time = Some(Instant::now());
                        // Generate preamble for each data transmission
                        tx_buffer.extend_from_slice(preamble.samples());

                        // Encode and modulate frame
                        let frame_bytes = frame.encode();
                        debug!("TX Data Frame: {} bytes: {:02x?}", frame_bytes.len(), &frame_bytes);
                        let bits: Vec<u8> = frame_bytes.iter()
                            .flat_map(|&b| (0..8).rev().map(move |i| (b >> i) & 1))
                            .collect();
                        let block_size = turbo_encoder.block_size();
                        let mut padded_bits = bits.clone();
                        padded_bits.resize(block_size, 0);
                        let encoded = turbo_encoder.encode(&padded_bits);
                        let modulated = if is_fsk_mode {
                            if let Some(ref mut fsk_mod) = fsk_modulator {
                                fsk_mod.reset();
                                fsk_mod.modulate_bits(&encoded)
                            } else {
                                modulator.reset();
                                modulator.modulate(&encoded)
                            }
                        } else {
                            modulator.reset();
                            modulator.modulate(&encoded)
                        };
                        tx_buffer.extend_from_slice(&modulated);

                        if let Ok(mut state) = app_state.lock() {
                            state.frames_sent += 1;
                            state.bytes_sent = session.bytes_sent();
                        }
                    }
                }

                // Generate ACK/keepalive/pong - only need tx_cooldown, not full frame_gap
                // This allows prompt ACK responses without waiting 800ms
                // For FSK modes with long frames, don't transmit while receiving a frame
                if tx_buffer.is_empty() && tx_cooldown_over && !receiving_frame {
                    if let Some(ack_frame) = session.generate_ack() {
                        debug!("Generating data ACK for seq {}", ack_frame.sequence());
                        // Set ACK status indicator
                        status_ack_until = Some(Instant::now() + indicator_display_duration);

                        let preamble_samples = preamble.samples();
                        tx_buffer.extend_from_slice(preamble_samples);

                        let frame_bytes = ack_frame.encode();
                        let bits: Vec<u8> = frame_bytes.iter()
                            .flat_map(|&b| (0..8).rev().map(move |i| (b >> i) & 1))
                            .collect();
                        let block_size = turbo_encoder.block_size();
                        let mut padded_bits = bits.clone();
                        padded_bits.resize(block_size, 0);
                        let encoded = turbo_encoder.encode(&padded_bits);
                        let modulated = if is_fsk_mode {
                            if let Some(ref mut fsk_mod) = fsk_modulator {
                                fsk_mod.reset();
                                fsk_mod.modulate_bits(&encoded)
                            } else {
                                modulator.reset();
                                modulator.modulate(&encoded)
                            }
                        } else {
                            modulator.reset();
                            modulator.modulate(&encoded)
                        };
                        tx_buffer.extend_from_slice(&modulated);
                    }

                    // Generate keepalive if needed (idle frame to maintain connection)
                    if let Some(keepalive_frame) = session.generate_keepalive() {
                        debug!("Sending keepalive frame");
                        tx_buffer.extend_from_slice(preamble.samples());

                        let frame_bytes = keepalive_frame.encode();
                        let bits: Vec<u8> = frame_bytes.iter()
                            .flat_map(|&b| (0..8).rev().map(move |i| (b >> i) & 1))
                            .collect();
                        let block_size = turbo_encoder.block_size();
                        let mut padded_bits = bits.clone();
                        padded_bits.resize(block_size, 0);
                        let encoded = turbo_encoder.encode(&padded_bits);
                        let modulated = if is_fsk_mode {
                            if let Some(ref mut fsk_mod) = fsk_modulator {
                                fsk_mod.reset();
                                fsk_mod.modulate_bits(&encoded)
                            } else {
                                modulator.reset();
                                modulator.modulate(&encoded)
                            }
                        } else {
                            modulator.reset();
                            modulator.modulate(&encoded)
                        };
                        tx_buffer.extend_from_slice(&modulated);
                    }

                    // Generate pong response if ping was received
                    if let Some(pong_frame) = session.generate_pong() {
                        debug!("Sending pong response");
                        tx_buffer.extend_from_slice(preamble.samples());

                        let frame_bytes = pong_frame.encode();
                        let bits: Vec<u8> = frame_bytes.iter()
                            .flat_map(|&b| (0..8).rev().map(move |i| (b >> i) & 1))
                            .collect();
                        let block_size = turbo_encoder.block_size();
                        let mut padded_bits = bits.clone();
                        padded_bits.resize(block_size, 0);
                        let encoded = turbo_encoder.encode(&padded_bits);
                        let modulated = if is_fsk_mode {
                            if let Some(ref mut fsk_mod) = fsk_modulator {
                                fsk_mod.reset();
                                fsk_mod.modulate_bits(&encoded)
                            } else {
                                modulator.reset();
                                modulator.modulate(&encoded)
                            }
                        } else {
                            modulator.reset();
                            modulator.modulate(&encoded)
                        };
                        tx_buffer.extend_from_slice(&modulated);
                    }
                }
            }
        } else if session_state == SessionState::Disconnecting {
            // Generate disconnect frame
            if tx_buffer.is_empty() {
                if let Ok(frame) = session.disconnect() {
                    tx_buffer.extend_from_slice(preamble.samples());

                    let frame_bytes = frame.encode();
                    let bits: Vec<u8> = frame_bytes.iter()
                        .flat_map(|&b| (0..8).rev().map(move |i| (b >> i) & 1))
                        .collect();
                    let block_size = turbo_encoder.block_size();
                    let mut padded_bits = bits.clone();
                    padded_bits.resize(block_size, 0);
                    let encoded = turbo_encoder.encode(&padded_bits);
                    let modulated = if is_fsk_mode {
                        if let Some(ref mut fsk_mod) = fsk_modulator {
                            fsk_mod.reset();
                            fsk_mod.modulate_bits(&encoded)
                        } else {
                            modulator.reset();
                            modulator.modulate(&encoded)
                        }
                    } else {
                        modulator.reset();
                        modulator.modulate(&encoded)
                    };
                    tx_buffer.extend_from_slice(&modulated);
                }
            }
        } // end if !tune_mode

        // Transmit audio
        if !tx_buffer.is_empty() {
            let chunk_size = 1024;
            let chunk: Vec<f32> = tx_buffer.drain(0..chunk_size.min(tx_buffer.len())).collect();

            // Activate PTT
            let _ = ptt.key();

            // Send to audio - log if channel is full and samples are dropped!
            match tx_channel.try_send(chunk.clone()) {
                Ok(_) => {
                    tx_samples_sent += chunk.len();
                },
                Err(crossbeam_channel::TrySendError::Full(_)) => {
                    eprintln!("[TX-OVERFLOW] Channel full! {} samples dropped, tx_buffer remaining: {}",
                        chunk.len(), tx_buffer.len());
                },
                Err(crossbeam_channel::TrySendError::Disconnected(_)) => {
                    error!("TX channel disconnected!");
                }
            }

            if let Ok(mut state) = app_state.lock() {
                state.tx_active = true;
                state.tx_buffer_bytes = tx_buffer.len();
            }
            // Reset sample counter at start of new TX session
            if !was_transmitting {
                tx_samples_sent = chunk.len();
            }
            was_transmitting = true;
        } else {
            // Deactivate PTT when no data
            let _ = ptt.unkey();

            // When transitioning from TX to RX, clear RX buffer to remove own signal (echo)
            // Only clear the samples we transmitted - preserve any response that may have arrived
            if was_transmitting {
                // Add 10% margin for timing variations
                let samples_to_clear = (tx_samples_sent as f32 * 1.1) as usize;
                let actual_clear = samples_to_clear.min(rx_buffer.len());
                rx_buffer.drain(0..actual_clear);
                preamble_detector.reset();
                pending_preamble_offset = None;
                pending_preamble_time = None;
                tx_complete_time = Some(Instant::now()); // Track TX completion for frame gap
                tx_samples_sent = 0; // Reset for next TX session
                // Only drain channel chunks that correspond to our TX (approximate)
                let chunks_to_drain = (samples_to_clear / 1024).saturating_sub(1);
                for _ in 0..chunks_to_drain {
                    if rx_channel.try_recv().is_err() { break; }
                }
                // Start cooldown period to let any residual audio fade
                tx_cooldown_until = Some(Instant::now() + tx_cooldown_duration);
            }
            was_transmitting = false;

            if let Ok(mut state) = app_state.lock() {
                state.tx_active = false;
                state.tx_buffer_bytes = 0;
            }
        }

        // Check session timeout
        session.check_timeout();

        // Update local SNR for rate negotiation
        session.update_local_snr(metrics.snr_db);

        // Check rate negotiation timeout (fall back to mode 2 if no keepalives received)
        session.check_rate_timeout();

        // Update GUI state
        if let Ok(mut gui) = gui_state.lock() {
            gui.snr_db = metrics.snr_db;
            gui.signal_level = audio_stats.rx_level_db;
            gui.noise_level = noise_floor_db;
            gui.afc_offset = afc.offset();

            // Update constellation points for display
            if !latest_constellation_points.is_empty() {
                gui.constellation_points = latest_constellation_points.clone();
            }

            // Update spectrum for waterfall display
            if !spectrum_data.is_empty() {
                gui.spectrum_data = spectrum_data.clone();
            }

            // Update status indicators based on session state
            let session_state = session.state();
            let now = Instant::now();

            gui.status_idle = session_state == SessionState::Disconnected
                && status_start_until.map_or(true, |t| now > t)
                && status_break_until.map_or(true, |t| now > t);
            gui.status_req = session_state == SessionState::Connecting;
            gui.status_start = status_start_until.map_or(false, |t| now <= t);
            gui.status_data = session_state == SessionState::Connected && !tx_buffer.is_empty();
            gui.status_ack = status_ack_until.map_or(false, |t| now <= t);
            gui.status_nack = status_nack_until.map_or(false, |t| now <= t);
            gui.status_break = status_break_until.map_or(false, |t| now <= t);
            gui.status_qrt = session_state == SessionState::Disconnecting;
        }

        // Update app state with signal metrics
        if let Ok(mut state) = app_state.lock() {
            state.snr_db = metrics.snr_db;
            state.vu_level = audio_stats.rx_level_db;
            state.afc_offset_hz = afc.offset();
            let corr_peak = preamble_detector.correlation_peak();
            state.busy = corr_peak > 0.3;

            // Debug logging for correlation values
            if corr_peak > 0.1 {
                debug!("Preamble correlation: {:.3}", corr_peak);
            }

            // RX active if we detected signal
            state.rx_active = state.busy || corr_peak > 0.2;
        }

        // CPU usage and bitrate calculation (once per second)
        _cpu_samples += 1;
        cpu_time += loop_start.elapsed();

        if last_cpu_check.elapsed().as_secs() >= 1 {
            let cpu_percent = (cpu_time.as_secs_f32() / last_cpu_check.elapsed().as_secs_f32()) * 100.0;
            let elapsed_secs = last_bitrate_calc.elapsed().as_secs_f32();

            if let Ok(mut state) = app_state.lock() {
                state.cpu_usage = cpu_percent.min(100.0);

                // Calculate bitrate (bytes/sec * 8 = bits/sec)
                if elapsed_secs > 0.0 {
                    let bytes_delta = (state.bytes_sent - last_bytes_sent)
                        + (state.bytes_received - last_bytes_received);
                    state.bitrate_bps = (bytes_delta as f32 * 8.0) / elapsed_secs;
                    last_bytes_sent = state.bytes_sent;
                    last_bytes_received = state.bytes_received;
                }
            }

            last_cpu_check = Instant::now();
            last_bitrate_calc = Instant::now();
            _cpu_samples = 0;
            cpu_time = std::time::Duration::ZERO;
        }

        // Sleep to maintain ~100Hz loop rate
        let elapsed = loop_start.elapsed();
        let target = std::time::Duration::from_millis(10);
        if elapsed < target {
            std::thread::sleep(target - elapsed);
        }
    }
}
