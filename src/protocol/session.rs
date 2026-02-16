//! Session management
//!
//! Handles connection establishment, maintenance, and teardown

use std::time::{Duration, Instant};
use super::{Frame, FrameType, ArqController, ArqConfig, ProtocolError, TimerConfig, ConnectionMode};
use super::{Encryptor, EncryptionConfig};
use super::rate::{RateNegotiator, NegotiationState, max_mode_for_bandwidth, FrameSize};
use crate::fec::crc::{session_seed, generate_random_seed};
use crate::Bandwidth;

/// Session state
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SessionState {
    /// No active session
    Disconnected,
    /// Connection request sent, waiting for response
    Connecting,
    /// Connection established
    Connected,
    /// Disconnect request sent, waiting for response
    Disconnecting,
}

/// Session configuration
#[derive(Debug, Clone)]
pub struct SessionConfig {
    /// Local callsign
    pub local_call: String,
    /// Connection mode
    pub mode: ConnectionMode,
    /// Timer configuration
    pub timers: TimerConfig,
    /// ARQ configuration
    pub arq: ArqConfig,
    /// Initial speed level
    pub speed_level: u8,
    /// Encryption configuration
    pub encryption: EncryptionConfig,
    /// Bandwidth for rate negotiation
    pub bandwidth: Bandwidth,
}

impl Default for SessionConfig {
    fn default() -> Self {
        Self {
            local_call: String::new(),
            mode: ConnectionMode::P2P,
            timers: TimerConfig::default(),
            arq: ArqConfig::default(),
            speed_level: 9,
            encryption: EncryptionConfig::default(),
            bandwidth: Bandwidth::Hz2300,
        }
    }
}

/// Session events
#[derive(Debug, Clone)]
pub enum SessionEvent {
    /// Connection established
    Connected { remote_call: String },
    /// Connection request received
    ConnectRequest { remote_call: String },
    /// Disconnected
    Disconnected { reason: String },
    /// Data received
    DataReceived { data: Vec<u8> },
    /// Ready to send more data
    ReadyToSend,
    /// Speed level changed
    SpeedChanged { level: u8 },
    /// Error occurred
    Error { message: String },
}

/// Keepalive interval in seconds (15s for idle frame transmission)
const KEEPALIVE_INTERVAL_SECS: u64 = 15;

/// Number of consecutive successes required before frame size can grow
const FRAME_SIZE_GROW_THRESHOLD: usize = 5;

/// SNR thresholds for frame size selection (dB)
const SHORT_SNR_THRESHOLD: f32 = 6.0;
const MEDIUM_SNR_THRESHOLD: f32 = 12.0;
const STANDARD_SNR_THRESHOLD: f32 = 18.0;

/// Frame size negotiator for adaptive frame sizing
#[derive(Debug, Clone)]
pub struct FrameSizeNegotiator {
    current_size: FrameSize,
    success_count: usize,
    retransmit_count: usize,
    last_snr: f32,
    remote_size: FrameSize,
}

impl FrameSizeNegotiator {
    pub fn new() -> Self {
        Self {
            current_size: FrameSize::Standard,
            success_count: 0,
            retransmit_count: 0,
            last_snr: 0.0,
            remote_size: FrameSize::Standard,
        }
    }

    /// Called when a frame is successfully acknowledged
    pub fn on_success(&mut self, snr: f32) -> bool {
        self.success_count += 1;
        self.retransmit_count = 0;
        self.last_snr = snr;

        if self.success_count >= FRAME_SIZE_GROW_THRESHOLD {
            let snr_supported_size = self.snr_recommended_size(snr);
            let new_size = self.current_size.larger();

            if new_size != self.current_size && snr_supported_size >= new_size {
                self.current_size = new_size;
                self.success_count = 0;
                log::info!("Frame size increased to {:?} (SNR={:.1}dB)", self.current_size, snr);
                return true;
            }
        }
        false
    }

    /// Called when a retransmission is needed
    pub fn on_retransmit(&mut self) -> bool {
        self.retransmit_count += 1;
        self.success_count = 0;

        let new_size = self.current_size.smaller();
        if new_size != self.current_size {
            self.current_size = new_size;
            log::info!("Frame size decreased to {:?} due to retransmit", self.current_size);
            return true;
        }
        false
    }

    pub fn update_remote_size(&mut self, size: FrameSize) {
        self.remote_size = size;
    }

    fn snr_recommended_size(&self, snr: f32) -> FrameSize {
        if snr < SHORT_SNR_THRESHOLD {
            FrameSize::Short
        } else if snr < MEDIUM_SNR_THRESHOLD {
            FrameSize::Medium
        } else if snr < STANDARD_SNR_THRESHOLD {
            FrameSize::Standard
        } else {
            FrameSize::Long
        }
    }

    pub fn current_size(&self) -> FrameSize {
        self.current_size
    }

    pub fn set_size(&mut self, size: FrameSize) {
        if size != self.current_size {
            self.current_size = size;
            self.success_count = 0;
            self.retransmit_count = 0;
        }
    }

    pub fn reset(&mut self) {
        self.current_size = FrameSize::Standard;
        self.success_count = 0;
        self.retransmit_count = 0;
        self.last_snr = 0.0;
        self.remote_size = FrameSize::Standard;
    }
}

impl Default for FrameSizeNegotiator {
    fn default() -> Self {
        Self::new()
    }
}

/// Session manager
pub struct Session {
    config: SessionConfig,
    state: SessionState,
    session_id: u16,
    remote_call: String,
    arq: Option<ArqController>,
    current_speed: u8,

    // Encryption
    encryptor: Encryptor,

    // Timing
    last_activity: Instant,
    connect_time: Option<Instant>,
    last_keepalive: Instant,
    keepalive_pending: bool,

    // RTT measurement
    ping_sent_time: Option<Instant>,
    rtt_ms: f32,
    pending_pong: bool,

    // Pending incoming connection
    pending_connect: Option<Frame>,

    // Pending disconnect ACK to send (stores session_id)
    pending_disconnect_ack: Option<u16>,

    // SNR-based rate negotiation
    rate_negotiator: RateNegotiator,

    // Frame size adaptation
    frame_size_negotiator: FrameSizeNegotiator,

    // Event queue
    events: Vec<SessionEvent>,

    // Statistics
    bytes_sent: u64,
    bytes_received: u64,
}

impl Session {
    /// Create new session manager
    pub fn new(config: SessionConfig) -> Self {
        let encryptor = Encryptor::new(&config.encryption);
        let max_mode = max_mode_for_bandwidth(config.bandwidth);
        let mut rate_negotiator = RateNegotiator::new(max_mode);
        // Start at mode 2 for initial connection (most robust OFDM mode)
        rate_negotiator.agreed_mode = 2;
        Self {
            current_speed: 2, // Start at mode 2 for connection
            config,
            state: SessionState::Disconnected,
            session_id: 0,
            remote_call: String::new(),
            arq: None,
            encryptor,
            last_activity: Instant::now(),
            connect_time: None,
            last_keepalive: Instant::now(),
            keepalive_pending: false,
            ping_sent_time: None,
            rtt_ms: 0.0,
            pending_pong: false,
            pending_connect: None,
            pending_disconnect_ack: None,
            rate_negotiator,
            frame_size_negotiator: FrameSizeNegotiator::new(),
            events: Vec::new(),
            bytes_sent: 0,
            bytes_received: 0,
        }
    }

    /// Initiate connection to remote station
    pub fn connect(&mut self, remote_call: &str) -> Result<Frame, ProtocolError> {
        if self.state != SessionState::Disconnected {
            return Err(ProtocolError::NoSession);
        }

        self.remote_call = remote_call.to_uppercase();

        // Generate session ID:
        // session_seed = src_hash XOR dst_hash XOR random_seed
        let random_seed = generate_random_seed();
        self.session_id = session_seed(&self.config.local_call, &self.remote_call, random_seed);

        // Initialize encryption with session-specific salt
        let mut enc_config = self.config.encryption.clone();
        enc_config.set_session_salt(self.session_id, &self.config.local_call, &self.remote_call);
        self.encryptor = Encryptor::new(&enc_config);

        self.state = SessionState::Connecting;
        self.last_activity = Instant::now();

        // We are the connection initiator - we should reconnect after timeout
        self.rate_negotiator.set_initiator(true);

        Ok(Frame::connect(
            &self.config.local_call,
            &self.remote_call,
            self.session_id,
        ))
    }

    /// Accept incoming connection
    pub fn accept(&mut self, connect_frame: &Frame) -> Result<Frame, ProtocolError> {
        if self.state != SessionState::Disconnected {
            return Err(ProtocolError::NoSession);
        }

        // Extract callsigns from connect frame (8 bytes each for compact format)
        if connect_frame.payload.len() < 16 {
            return Err(ProtocolError::FrameTooShort);
        }

        let source_call = String::from_utf8_lossy(&connect_frame.payload[..8])
            .trim_end_matches('\0')
            .to_string();
        let _dest_call = String::from_utf8_lossy(&connect_frame.payload[8..16])
            .trim_end_matches('\0')
            .to_string();

        self.remote_call = source_call.clone();
        self.session_id = connect_frame.session_id();

        // Initialize encryption with session-specific salt
        let mut enc_config = self.config.encryption.clone();
        enc_config.set_session_salt(self.session_id, &self.config.local_call, &self.remote_call);
        self.encryptor = Encryptor::new(&enc_config);

        // Initialize ARQ controller
        self.arq = Some(ArqController::new(self.config.arq.clone(), self.session_id));

        self.state = SessionState::Connected;
        self.connect_time = Some(Instant::now());
        self.last_activity = Instant::now();

        // We accepted an incoming connection - we are NOT the initiator
        // The other station should reconnect after timeout
        self.rate_negotiator.set_initiator(false);

        self.events.push(SessionEvent::Connected {
            remote_call: source_call,
        });

        Ok(Frame::connect_ack(self.session_id, self.current_speed))
    }

    /// Handle received connect ACK
    pub fn handle_connect_ack(&mut self, frame: &Frame) -> Result<(), ProtocolError> {
        if self.state != SessionState::Connecting {
            return Ok(());
        }

        if frame.session_id() != self.session_id {
            return Ok(());
        }

        // Initialize ARQ controller
        self.arq = Some(ArqController::new(self.config.arq.clone(), self.session_id));

        self.current_speed = frame.header.speed_level;
        self.state = SessionState::Connected;
        self.connect_time = Some(Instant::now());
        self.last_activity = Instant::now();

        self.events.push(SessionEvent::Connected {
            remote_call: self.remote_call.clone(),
        });

        Ok(())
    }

    /// Initiate disconnection
    pub fn disconnect(&mut self) -> Result<Frame, ProtocolError> {
        if self.state != SessionState::Connected {
            return Err(ProtocolError::NoSession);
        }

        self.state = SessionState::Disconnecting;
        self.last_activity = Instant::now();

        Ok(Frame::disconnect(self.session_id))
    }

    /// Handle disconnect (received or confirmed)
    pub fn handle_disconnect(&mut self) {
        self.state = SessionState::Disconnected;
        self.arq = None;
        self.session_id = 0;

        self.events.push(SessionEvent::Disconnected {
            reason: "Normal disconnect".to_string(),
        });
    }

    /// Send data (queues for transmission, encryption applied per-frame)
    pub fn send(&mut self, data: Vec<u8>) -> Result<(), ProtocolError> {
        if self.state != SessionState::Connected {
            return Err(ProtocolError::NoSession);
        }

        if let Some(arq) = &mut self.arq {
            let len = data.len();
            if arq.send(data) {
                self.bytes_sent += len as u64;
            } else {
                return Err(ProtocolError::QueueFull);
            }
        }

        Ok(())
    }

    /// Get next frame to transmit (encrypts payload if encryption enabled)
    pub fn get_tx_frame(&mut self) -> Option<Frame> {
        match self.state {
            SessionState::Connected => {
                if let Some(arq) = &mut self.arq {
                    if let Some(mut frame) = arq.get_tx_frame() {
                        // Add SNR and proposed mode to data frames for rate negotiation
                        frame.header.set_measured_snr(self.rate_negotiator.get_measured_snr_i8());
                        frame.header.set_proposed_mode(self.rate_negotiator.my_proposed_mode);
                        frame.header.set_frame_size(self.frame_size_negotiator.current_size());

                        // Encrypt frame payload if encryption is enabled
                        if self.encryptor.is_enabled() && !frame.payload.is_empty() {
                            match self.encryptor.encrypt(&frame.payload) {
                                Ok(encrypted) => {
                                    frame.payload = encrypted;
                                }
                                Err(e) => {
                                    log::warn!("Frame encryption failed: {}", e);
                                    return None;
                                }
                            }
                        }
                        Some(frame)
                    } else {
                        None
                    }
                } else {
                    None
                }
            }
            _ => None,
        }
    }

    /// Process received frame
    pub fn receive(&mut self, frame: Frame) -> Result<(), ProtocolError> {
        self.last_activity = Instant::now();

        match frame.frame_type() {
            FrameType::Connect => {
                // Store for later accept() and notify
                self.pending_connect = Some(frame.clone());
                self.events.push(SessionEvent::ConnectRequest {
                    remote_call: String::from_utf8_lossy(&frame.payload[..8])
                        .trim_end_matches('\0')
                        .to_string(),
                });
            }
            FrameType::ConnectAck => {
                self.handle_connect_ack(&frame)?;
            }
            FrameType::Disconnect => {
                if frame.session_id() == self.session_id {
                    self.handle_disconnect();
                }
            }
            FrameType::DisconnectAck => {
                if self.state == SessionState::Disconnecting {
                    self.handle_disconnect();
                }
            }
            FrameType::Data => {
                // Validate session_id before processing
                if frame.session_id() != self.session_id {
                    return Ok(()); // Ignore frames from other sessions
                }

                // Update remote frame size from incoming frame
                self.frame_size_negotiator.update_remote_size(frame.header.frame_size());

                // Decrypt frame payload before passing to ARQ
                let mut decrypted_frame = frame;
                if self.encryptor.is_enabled() && !decrypted_frame.payload.is_empty() {
                    match self.encryptor.decrypt(&decrypted_frame.payload) {
                        Ok(decrypted) => {
                            decrypted_frame.payload = decrypted;
                        }
                        Err(e) => {
                            log::warn!("Frame decryption failed: {}", e);
                            self.events.push(SessionEvent::Error {
                                message: format!("Decryption failed: {}", e),
                            });
                            return Ok(()); // Skip this frame
                        }
                    }
                }

                if let Some(arq) = &mut self.arq {
                    arq.receive(decrypted_frame)?;

                    // Get received data (already decrypted)
                    for data in arq.get_received_data() {
                        self.bytes_received += data.len() as u64;
                        self.events.push(SessionEvent::DataReceived { data });
                    }
                }
            }
            FrameType::DataAck | FrameType::Nack => {
                if frame.session_id() == self.session_id {
                    // Update remote frame size from incoming frame
                    self.frame_size_negotiator.update_remote_size(frame.header.frame_size());

                    if let Some(arq) = &mut self.arq {
                        arq.receive(frame.clone())?;
                    }

                    // On successful ACK, notify frame size negotiator
                    if matches!(frame.frame_type(), FrameType::DataAck) {
                        let snr = frame.header.measured_snr() as f32;
                        self.frame_size_negotiator.on_success(snr);
                    }
                }
            }
            FrameType::SpeedChange => {
                // Manual speed change request from peer
                if frame.session_id() == self.session_id {
                    let requested_level = frame.header.speed_level;
                    log::info!("Received manual SpeedChange request for level {}", requested_level);
                    if self.rate_negotiator.handle_mode_request(requested_level) {
                        // Will send ACK
                    }
                }
            }
            FrameType::SpeedChangeAck => {
                // ACK for our speed change request - complete the switch
                if frame.session_id() == self.session_id {
                    let acked_level = frame.header.speed_level;
                    log::info!("SpeedChangeAck received for level {}", acked_level);
                    // Complete the switch on initiator side
                    if let Some(pending) = self.rate_negotiator.pending_mode {
                        if acked_level == pending {
                            self.current_speed = acked_level;
                            self.rate_negotiator.agreed_mode = acked_level;
                            self.rate_negotiator.pending_mode = None;
                            self.rate_negotiator.negotiation_state = NegotiationState::Idle;
                            self.events.push(SessionEvent::SpeedChanged {
                                level: self.current_speed,
                            });
                            log::info!("Mode switch complete: now at mode {}", acked_level);
                        }
                    }
                }
            }
            FrameType::Idle => {
                // Keepalive received - reset activity timer, process rate negotiation
                if frame.session_id() == self.session_id {
                    self.reset_keepalive();

                    // Process SNR info for rate negotiation
                    let measured_snr = frame.header.measured_snr();
                    let proposed_mode = frame.header.proposed_mode();
                    log::info!(
                        "Keepalive received: remote_snr={} dB, remote_proposed_mode={}, local_snr={:.1} dB, my_proposed_mode={}",
                        measured_snr, proposed_mode,
                        self.rate_negotiator.local_snr, self.rate_negotiator.my_proposed_mode
                    );
                    if self.rate_negotiator.process_received_snr(measured_snr, proposed_mode) {
                        // Mode change agreed - update current_speed
                        if let Some(new_mode) = self.rate_negotiator.pending_mode {
                            log::info!("Rate negotiation: mode change agreed to level {}", new_mode);
                            self.current_speed = new_mode;
                            self.events.push(SessionEvent::SpeedChanged {
                                level: self.current_speed,
                            });
                        }
                    }
                }
            }
            FrameType::Break => {
                // Break/abort - handle connection break
                if frame.session_id() == self.session_id && self.state == SessionState::Connected {
                    self.state = SessionState::Disconnected;
                    self.events.push(SessionEvent::Disconnected {
                        reason: "Remote station sent BREAK".to_string(),
                    });
                    self.cleanup_session();
                }
            }
            FrameType::Request => {
                // Retransmission request - trigger ARQ to resend
                if frame.session_id() == self.session_id {
                    if let Some(arq) = &mut self.arq {
                        let _ = arq.receive(frame);
                    }
                }
            }
            FrameType::Quit => {
                // Graceful termination request
                if self.state == SessionState::Connected || self.state == SessionState::Connecting {
                    self.state = SessionState::Disconnected;
                    self.events.push(SessionEvent::Disconnected {
                        reason: "Remote station sent QUIT".to_string(),
                    });
                    self.cleanup_session();
                }
            }
            FrameType::Ping => {
                // Ping received - queue a Pong response
                if self.state == SessionState::Connected {
                    self.pending_pong = true;
                }
            }
            FrameType::Pong => {
                // Pong received - calculate RTT
                if let Some(ping_time) = self.ping_sent_time.take() {
                    self.rtt_ms = ping_time.elapsed().as_secs_f32() * 1000.0;
                }
            }
        }

        Ok(())
    }

    /// Generate ACK if needed
    pub fn generate_ack(&mut self) -> Option<Frame> {
        if let Some(arq) = &mut self.arq {
            if let Some(mut frame) = arq.generate_ack() {
                // Add SNR and proposed mode to ACK frames for rate negotiation
                frame.header.set_measured_snr(self.rate_negotiator.get_measured_snr_i8());
                frame.header.set_proposed_mode(self.rate_negotiator.my_proposed_mode);
                frame.header.set_frame_size(self.frame_size_negotiator.current_size());
                Some(frame)
            } else {
                None
            }
        } else {
            None
        }
    }

    /// Check if there's a pending ACK to send
    pub fn has_pending_ack(&self) -> bool {
        if let Some(arq) = &self.arq {
            arq.has_pending_ack()
        } else {
            false
        }
    }

    /// Get pending events
    pub fn get_events(&mut self) -> Vec<SessionEvent> {
        std::mem::take(&mut self.events)
    }

    /// Check for timeout and trigger keepalive if needed
    pub fn check_timeout(&mut self) -> bool {
        let timeout = Duration::from_millis(
            self.config.mode.timeout(&self.config.timers) as u64,
        );

        if self.last_activity.elapsed() > timeout {
            match self.state {
                SessionState::Connecting | SessionState::Disconnecting => {
                    self.state = SessionState::Disconnected;
                    self.events.push(SessionEvent::Disconnected {
                        reason: "Timeout".to_string(),
                    });
                    return true;
                }
                SessionState::Connected => {
                    // Trigger keepalive if no activity for too long
                    let keepalive_timeout = Duration::from_secs(KEEPALIVE_INTERVAL_SECS);
                    if self.last_keepalive.elapsed() > keepalive_timeout {
                        self.keepalive_pending = true;
                    }

                    // If we haven't received anything for 3x keepalive interval, disconnect
                    if self.last_activity.elapsed() > keepalive_timeout * 3 {
                        self.state = SessionState::Disconnected;
                        self.events.push(SessionEvent::Disconnected {
                            reason: "Connection timeout - no keepalive response".to_string(),
                        });
                        return true;
                    }
                }
                SessionState::Disconnected => {}
            }
        }

        false
    }

    /// Check if keepalive frame should be sent
    pub fn needs_keepalive(&self) -> bool {
        self.keepalive_pending && self.state == SessionState::Connected
    }

    /// Generate keepalive frame
    pub fn generate_keepalive(&mut self) -> Option<Frame> {
        if !self.keepalive_pending || self.state != SessionState::Connected {
            return None;
        }

        self.keepalive_pending = false;
        self.last_keepalive = Instant::now();

        // Generate a keepalive frame (uses IDLE frame type)
        Some(Frame::idle(self.session_id))
    }

    /// Reset keepalive timer (call when activity occurs)
    pub fn reset_keepalive(&mut self) {
        self.last_keepalive = Instant::now();
        self.keepalive_pending = false;
    }

    /// Clean up session state after disconnect
    fn cleanup_session(&mut self) {
        self.arq = None;
        self.connect_time = None;
        self.ping_sent_time = None;
        self.pending_pong = false;
        self.keepalive_pending = false;
    }

    /// Check if a Pong response should be sent
    pub fn needs_pong(&self) -> bool {
        self.pending_pong && self.state == SessionState::Connected
    }

    /// Generate Pong response frame
    pub fn generate_pong(&mut self) -> Option<Frame> {
        if !self.pending_pong || self.state != SessionState::Connected {
            return None;
        }

        self.pending_pong = false;
        Some(Frame::pong(self.session_id))
    }

    /// Generate Ping frame for RTT measurement
    pub fn generate_ping(&mut self) -> Option<Frame> {
        if self.state != SessionState::Connected {
            return None;
        }

        self.ping_sent_time = Some(Instant::now());
        Some(Frame::ping(self.session_id))
    }

    /// Get current RTT estimate in milliseconds
    pub fn rtt_ms(&self) -> f32 {
        self.rtt_ms
    }

    /// Get current state
    pub fn state(&self) -> SessionState {
        self.state
    }

    /// Get session ID
    pub fn session_id(&self) -> u16 {
        self.session_id
    }

    /// Get remote callsign
    pub fn remote_call(&self) -> &str {
        &self.remote_call
    }

    /// Get current speed level
    pub fn speed_level(&self) -> u8 {
        self.current_speed
    }

    /// Set speed level
    /// Clamps to bandwidth-specific max mode to prevent desynchronization
    pub fn set_speed_level(&mut self, level: u8) {
        // Mode 1 disabled: needs soft-decision FSK decoding
        // Clamp to bandwidth-specific max_mode to prevent desync with rate negotiator
        let clamped = level.clamp(2, self.rate_negotiator.max_mode);
        self.current_speed = clamped;
        // Keep rate negotiator in sync
        self.rate_negotiator.agreed_mode = clamped;
        self.rate_negotiator.my_proposed_mode = clamped;
    }

    /// Get connection duration
    pub fn connection_duration(&self) -> Option<Duration> {
        self.connect_time.map(|t| t.elapsed())
    }

    /// Get bytes sent
    pub fn bytes_sent(&self) -> u64 {
        self.bytes_sent
    }

    /// Get bytes received
    pub fn bytes_received(&self) -> u64 {
        self.bytes_received
    }

    /// Check if session is connected
    pub fn is_connected(&self) -> bool {
        self.state == SessionState::Connected
    }

    /// Update ARQ configuration (e.g., when speed level changes)
    pub fn update_arq_config(&mut self, arq_config: ArqConfig) {
        self.config.arq = arq_config;
        // If ARQ controller exists, we'd need to update it too
        // For now, the new config will be used on next connection
    }

    /// Check if there's a pending incoming connection
    pub fn has_pending_connect(&self) -> bool {
        self.pending_connect.is_some() && self.state == SessionState::Disconnected
    }

    /// Accept the pending incoming connection
    pub fn accept_pending(&mut self) -> Option<Frame> {
        if self.state != SessionState::Disconnected {
            return None;
        }
        if let Some(connect_frame) = self.pending_connect.take() {
            match self.accept(&connect_frame) {
                Ok(ack_frame) => Some(ack_frame),
                Err(_) => None,
            }
        } else {
            None
        }
    }

    /// Request speed level change and notify remote station
    /// Queues a SpeedChange frame; actual switch happens when ACK is received
    pub fn request_speed_change(&mut self, level: u8) {
        let clamped = level.clamp(2, self.rate_negotiator.max_mode);
        if self.state == SessionState::Connected && clamped != self.current_speed {
            self.rate_negotiator.pending_mode = Some(clamped);
            self.rate_negotiator.my_proposed_mode = clamped;
        }
    }

    /// Check if we need to send a speed change frame
    pub fn needs_speed_change(&self) -> bool {
        self.state == SessionState::Connected && self.rate_negotiator.needs_mode_request()
    }

    /// Generate speed change frame to notify remote station
    /// Does NOT switch speed yet - waits for SpeedChangeAck
    pub fn generate_speed_change_frame(&mut self) -> Option<Frame> {
        if self.state != SessionState::Connected {
            return None;
        }

        if let Some(level) = self.rate_negotiator.pending_mode {
            if self.rate_negotiator.negotiation_state == NegotiationState::Idle
                && level != self.rate_negotiator.agreed_mode
            {
                // Set to awaiting ACK state with timeout tracking
                self.rate_negotiator.start_awaiting_ack();
                log::debug!("Generating SpeedChange frame for level {}, awaiting ACK", level);
                return Some(Frame::speed_change(self.session_id, level));
            }
        }

        None
    }

    /// Generate speed change ACK frame (responder side)
    /// Responder switches AFTER sending ACK
    pub fn generate_speed_change_ack(&mut self) -> Option<Frame> {
        if self.state != SessionState::Connected {
            return None;
        }

        if self.rate_negotiator.needs_ack() {
            if let Some(level) = self.rate_negotiator.pending_mode {
                log::info!("Generating SpeedChangeAck for level {}, switching now", level);
                // Responder switches after sending ACK
                self.current_speed = level;
                self.rate_negotiator.agreed_mode = level;
                self.rate_negotiator.pending_mode = None;
                self.rate_negotiator.negotiation_state = NegotiationState::Idle;
                self.events.push(SessionEvent::SpeedChanged {
                    level: self.current_speed,
                });
                return Some(Frame::speed_change_ack(self.session_id, level));
            }
        }

        None
    }

    /// Generate keepalive frame with rate negotiation info
    pub fn generate_keepalive_with_snr(&mut self) -> Option<Frame> {
        if !self.keepalive_pending || self.state != SessionState::Connected {
            return None;
        }

        self.keepalive_pending = false;
        self.last_keepalive = Instant::now();

        // Generate an idle frame with SNR and proposed mode info
        let mut frame = Frame::idle(self.session_id);
        frame.header.set_measured_snr(self.rate_negotiator.get_measured_snr_i8());
        frame.header.set_proposed_mode(self.rate_negotiator.my_proposed_mode);
        frame.header.set_frame_size(self.frame_size_negotiator.current_size());
        Some(frame)
    }

    /// Update local SNR measurement for rate negotiation
    pub fn update_local_snr(&mut self, snr: f32) {
        self.rate_negotiator.update_local_snr(snr);
    }

    /// Check rate negotiator timeout and fall back to mode 2 if needed
    /// Also checks for speed change ACK timeout and rolls back
    /// Returns true if any timeout/rollback occurred
    pub fn check_rate_timeout(&mut self) -> bool {
        if self.state != SessionState::Connected {
            return false;
        }

        // Check for keepalive timeout (fall back to mode 2)
        if self.rate_negotiator.check_timeout() {
            self.current_speed = 2;
            self.events.push(SessionEvent::SpeedChanged {
                level: self.current_speed,
            });
            return true;
        }

        // Check for speed change ACK timeout (rollback pending request)
        if self.rate_negotiator.check_awaiting_ack_timeout() {
            // Rollback occurred - my_proposed_mode reset to agreed_mode
            // No speed change event needed, we stayed at agreed_mode
            return true;
        }

        false
    }

    /// Get access to the rate negotiator (for reading state)
    pub fn rate_negotiator(&self) -> &RateNegotiator {
        &self.rate_negotiator
    }

    /// Get access to the frame size negotiator (for reading state)
    pub fn frame_size_negotiator(&self) -> &FrameSizeNegotiator {
        &self.frame_size_negotiator
    }

    /// Get mutable access to the frame size negotiator
    pub fn frame_size_negotiator_mut(&mut self) -> &mut FrameSizeNegotiator {
        &mut self.frame_size_negotiator
    }

    /// Check if this station should attempt reconnection after timeout
    /// Only the original connection initiator should reconnect to avoid collisions
    pub fn should_reconnect(&self) -> bool {
        self.rate_negotiator.should_reconnect()
    }

    /// Complete speed change after initiator receives ACK
    /// Call this after sending the confirm frame
    pub fn complete_speed_change(&mut self) {
        if let Some(level) = self.rate_negotiator.pending_mode {
            self.current_speed = level;
            self.rate_negotiator.complete_switch();
            self.events.push(SessionEvent::SpeedChanged {
                level: self.current_speed,
            });
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_session_connect() {
        let mut config = SessionConfig::default();
        config.local_call = "W1AW".to_string();

        let mut session = Session::new(config);
        let connect = session.connect("K1ABC").unwrap();

        assert_eq!(session.state(), SessionState::Connecting);
        assert_eq!(connect.frame_type(), FrameType::Connect);
    }

    #[test]
    fn test_session_accept() {
        let mut config = SessionConfig::default();
        config.local_call = "K1ABC".to_string();

        let mut session = Session::new(config);

        let connect = Frame::connect("W1AW", "K1ABC", 12345);
        let ack = session.accept(&connect).unwrap();

        assert_eq!(session.state(), SessionState::Connected);
        assert_eq!(ack.frame_type(), FrameType::ConnectAck);
    }

    #[test]
    fn test_session_disconnect() {
        let mut config = SessionConfig::default();
        config.local_call = "K1ABC".to_string();

        let mut session = Session::new(config);

        let connect = Frame::connect("W1AW", "K1ABC", 12345);
        session.accept(&connect).unwrap();

        let disconnect = session.disconnect().unwrap();
        assert_eq!(disconnect.frame_type(), FrameType::Disconnect);
    }

    #[test]
    fn test_set_speed_level_clamps_to_bandwidth_max() {
        use crate::Bandwidth;

        // Test 500Hz bandwidth - max mode is 13
        let mut config = SessionConfig::default();
        config.local_call = "W1AW".to_string();
        config.bandwidth = Bandwidth::Hz500;
        let mut session = Session::new(config);

        // Try to set mode 17 - should clamp to 13 (500Hz max)
        session.set_speed_level(17);
        assert_eq!(session.speed_level(), 13, "500Hz should clamp to mode 13");
        assert_eq!(session.rate_negotiator().agreed_mode, 13, "Rate negotiator should be in sync");
        assert_eq!(session.rate_negotiator().my_proposed_mode, 13, "Proposed mode should be in sync");

        // Test 2300Hz bandwidth - max mode is 16
        let mut config = SessionConfig::default();
        config.local_call = "W1AW".to_string();
        config.bandwidth = Bandwidth::Hz2300;
        let mut session = Session::new(config);

        // Try to set mode 17 - should clamp to 16 (2300Hz max)
        session.set_speed_level(17);
        assert_eq!(session.speed_level(), 16, "2300Hz should clamp to mode 16");

        // Test 2750Hz bandwidth - max mode is 17
        let mut config = SessionConfig::default();
        config.local_call = "W1AW".to_string();
        config.bandwidth = Bandwidth::Hz2750;
        let mut session = Session::new(config);

        // Set mode 17 - should work for 2750Hz
        session.set_speed_level(17);
        assert_eq!(session.speed_level(), 17, "2750Hz should allow mode 17");

        // Test lower bound - mode 1 should clamp to 2
        session.set_speed_level(1);
        assert_eq!(session.speed_level(), 2, "Mode 1 should clamp to mode 2");
    }
}
