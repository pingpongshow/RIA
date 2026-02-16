//! Rate adaptation algorithm
//!
//! Implements automatic speed level selection based on channel conditions.
//! Rate table with 17 speed levels.

use std::collections::VecDeque;
use std::time::{Duration, Instant};
use crate::fec::CodeRate;

/// Maximum valid speed level (array bounds limit)
pub const MAX_SPEED_LEVEL: u8 = 17;
/// Minimum valid speed level (mode 1 disabled, start at 2)
pub const MIN_SPEED_LEVEL: u8 = 2;

/// FFT size configuration
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FftSize {
    /// 2048-point FFT (robust, 23.44 Hz spacing)
    Fft2048,
    /// 1024-point FFT (standard, 46.88 Hz spacing)
    Fft1024,
    /// 512-point FFT (fast, 93.75 Hz spacing)
    Fft512,
}

impl FftSize {
    /// Get FFT size as usize
    pub fn size(&self) -> usize {
        match self {
            FftSize::Fft2048 => 2048,
            FftSize::Fft1024 => 1024,
            FftSize::Fft512 => 512,
        }
    }

    /// Get carrier spacing in Hz (at 48kHz sample rate)
    pub fn carrier_spacing(&self) -> f32 {
        48000.0 / self.size() as f32
    }

    /// Get symbol duration in seconds
    pub fn symbol_duration(&self) -> f32 {
        self.size() as f32 / 48000.0
    }
}

/// Modulation type
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ModulationType {
    /// Frequency Shift Keying (robust, 1 bit/symbol effective)
    Fsk,
    /// Binary PSK (1 bit/symbol)
    Bpsk,
    /// Quadrature PSK (2 bits/symbol)
    Qpsk,
    /// 8-PSK (3 bits/symbol)
    Psk8,
    /// 16-QAM (4 bits/symbol)
    Qam16,
    /// 32-QAM (5 bits/symbol)
    Qam32,
    /// 64-QAM (6 bits/symbol)
    Qam64,
}

impl ModulationType {
    /// Get bits per symbol for this modulation
    pub fn bits_per_symbol(&self) -> usize {
        match self {
            ModulationType::Fsk => 1,
            ModulationType::Bpsk => 1,
            ModulationType::Qpsk => 2,
            ModulationType::Psk8 => 3,
            ModulationType::Qam16 => 4,
            ModulationType::Qam32 => 5,
            ModulationType::Qam64 => 6,
        }
    }

    /// Get display name
    pub fn name(&self) -> &'static str {
        match self {
            ModulationType::Fsk => "FSK",
            ModulationType::Bpsk => "BPSK",
            ModulationType::Qpsk => "4PSK",
            ModulationType::Psk8 => "8PSK",
            ModulationType::Qam16 => "16QAM",
            ModulationType::Qam32 => "32QAM",
            ModulationType::Qam64 => "64QAM",
        }
    }
}

/// Frame overhead in bytes (header + CRC + length fields)
/// Used to calculate effective throughput from raw bitrate
pub const FRAME_OVERHEAD_BYTES: usize = 16;

/// Rate information for a speed level
///
/// NOTE: Carrier counts here are the canonical source for OFDM configuration.
/// The FRAME_CONFIG tables use these carrier counts to calculate FEC block sizes.
/// If modifying carriers, ensure FRAME_CONFIG fec_block_bits remains consistent.
#[derive(Debug, Clone)]
pub struct RateInfo {
    /// Speed level (1-17)
    pub level: u8,
    /// Raw data rate in bits per second (2300 Hz mode) - does not include frame overhead
    pub bps: f32,
    /// Raw data rate for 500 Hz narrow mode - does not include frame overhead
    pub bps_500: f32,
    /// Raw data rate for 2750 Hz tactical mode - does not include frame overhead
    pub bps_2750: f32,
    /// Minimum SNR required (dB)
    pub min_snr: f32,
    /// Modulation type
    pub modulation: ModulationType,
    /// Symbol rate (symbols per second)
    pub symbol_rate: f32,
    /// Number of carriers (2300 Hz mode) - canonical source for OFDM config
    pub carriers: usize,
    /// Number of carriers (500 Hz mode) - canonical source for OFDM config
    pub carriers_500: usize,
    /// Number of carriers (2750 Hz mode) - canonical source for OFDM config
    pub carriers_2750: usize,
    /// FFT size
    pub fft_size: FftSize,
    /// FEC code rate (approximate)
    pub code_rate: f32,
    /// Modulation type description (legacy)
    pub modulation_str: &'static str,
}

impl RateInfo {
    /// Get effective throughput for 2300 Hz mode accounting for frame overhead
    ///
    /// The raw bps doesn't account for protocol overhead (header, CRC, length fields).
    /// Effective throughput = raw_bps * (payload / (payload + overhead))
    pub fn effective_bps(&self, max_payload_bytes: usize) -> f32 {
        if max_payload_bytes == 0 {
            return 0.0;
        }
        let efficiency = max_payload_bytes as f32 / (max_payload_bytes + FRAME_OVERHEAD_BYTES) as f32;
        self.bps * efficiency
    }

    /// Get effective throughput for 500 Hz mode accounting for frame overhead
    pub fn effective_bps_500(&self, max_payload_bytes: usize) -> f32 {
        if max_payload_bytes == 0 {
            return 0.0;
        }
        let efficiency = max_payload_bytes as f32 / (max_payload_bytes + FRAME_OVERHEAD_BYTES) as f32;
        self.bps_500 * efficiency
    }

    /// Get effective throughput for 2750 Hz mode accounting for frame overhead
    pub fn effective_bps_2750(&self, max_payload_bytes: usize) -> f32 {
        if max_payload_bytes == 0 {
            return 0.0;
        }
        let efficiency = max_payload_bytes as f32 / (max_payload_bytes + FRAME_OVERHEAD_BYTES) as f32;
        self.bps_2750 * efficiency
    }

    /// Get carrier count for a specific bandwidth
    pub fn carriers_for_bandwidth(&self, bandwidth: Bandwidth) -> usize {
        match bandwidth {
            Bandwidth::Hz500 => self.carriers_500,
            Bandwidth::Hz2300 => self.carriers,
            Bandwidth::Hz2750 => self.carriers_2750,
        }
    }

    /// Get raw bps for a specific bandwidth
    pub fn bps_for_bandwidth(&self, bandwidth: Bandwidth) -> f32 {
        match bandwidth {
            Bandwidth::Hz500 => self.bps_500,
            Bandwidth::Hz2300 => self.bps,
            Bandwidth::Hz2750 => self.bps_2750,
        }
    }

    /// Get effective throughput for a specific bandwidth accounting for frame overhead
    pub fn effective_bps_for_bandwidth(&self, bandwidth: Bandwidth, max_payload_bytes: usize) -> f32 {
        match bandwidth {
            Bandwidth::Hz500 => self.effective_bps_500(max_payload_bytes),
            Bandwidth::Hz2300 => self.effective_bps(max_payload_bytes),
            Bandwidth::Hz2750 => self.effective_bps_2750(max_payload_bytes),
        }
    }
}

/// SNR thresholds for mode selection with hysteresis
/// enter_snr: SNR required to switch UP to this mode (higher threshold)
/// stay_snr: SNR required to REMAIN at this mode (lower threshold, 3dB hysteresis)
#[derive(Debug, Clone, Copy)]
pub struct ModeThreshold {
    pub level: u8,
    pub enter_snr: f32,  // SNR to enter this mode
    pub stay_snr: f32,   // SNR to stay at this mode (3dB lower)
}

/// Mode threshold table with 3dB hysteresis gap
/// Based on RATE_TABLE min_snr values with enter = min_snr + 3dB margin, stay = min_snr
pub const MODE_THRESHOLDS: [ModeThreshold; 17] = [
    ModeThreshold { level: 1,  enter_snr: -3.0,  stay_snr: -6.0 },   // FSK most robust
    ModeThreshold { level: 2,  enter_snr: 0.0,   stay_snr: -3.0 },   // FSK baseline
    ModeThreshold { level: 3,  enter_snr: 3.0,   stay_snr: 0.0 },    // FSK
    ModeThreshold { level: 4,  enter_snr: 6.0,   stay_snr: 3.0 },    // BPSK
    ModeThreshold { level: 5,  enter_snr: 9.0,   stay_snr: 6.0 },    // QPSK
    ModeThreshold { level: 6,  enter_snr: 12.0,  stay_snr: 9.0 },    // QPSK
    ModeThreshold { level: 7,  enter_snr: 15.0,  stay_snr: 12.0 },   // QPSK
    ModeThreshold { level: 8,  enter_snr: 18.0,  stay_snr: 15.0 },   // QPSK
    ModeThreshold { level: 9,  enter_snr: 20.0,  stay_snr: 17.0 },   // QPSK
    ModeThreshold { level: 10, enter_snr: 22.0,  stay_snr: 19.0 },   // QPSK
    ModeThreshold { level: 11, enter_snr: 24.0,  stay_snr: 21.0 },   // QPSK rate 2/3
    ModeThreshold { level: 12, enter_snr: 26.0,  stay_snr: 23.0 },   // QPSK rate 3/4
    ModeThreshold { level: 13, enter_snr: 28.0,  stay_snr: 25.0 },   // 8PSK
    ModeThreshold { level: 14, enter_snr: 30.0,  stay_snr: 27.0 },   // 8PSK rate 3/4
    ModeThreshold { level: 15, enter_snr: 32.0,  stay_snr: 29.0 },   // 16QAM
    ModeThreshold { level: 16, enter_snr: 34.0,  stay_snr: 31.0 },   // 32QAM
    ModeThreshold { level: 17, enter_snr: 36.0,  stay_snr: 33.0 },   // 32QAM max
];

/// Get mode threshold for a level
/// Returns None if level is outside valid range [1, MAX_SPEED_LEVEL]
pub fn get_mode_threshold(level: u8) -> Option<&'static ModeThreshold> {
    if level >= 1 && level <= MAX_SPEED_LEVEL {
        Some(&MODE_THRESHOLDS[(level - 1) as usize])
    } else {
        None
    }
}

/// Clamp a speed level to the valid range [MIN_SPEED_LEVEL, MAX_SPEED_LEVEL]
/// Also respects per-bandwidth max_mode limit
#[inline]
pub fn clamp_speed_level(level: u8, max_mode: u8) -> u8 {
    level.clamp(MIN_SPEED_LEVEL, max_mode.min(MAX_SPEED_LEVEL))
}

/// Calculate proposed mode based on SNR using threshold table
/// Uses enter_snr for upshift decisions, stay_snr for downshift
/// Returns the highest mode that the SNR supports
pub fn calculate_proposed_mode(snr: f32, current_mode: u8, max_mode: u8) -> u8 {
    let max_mode = max_mode.min(17);
    let current_mode = current_mode.clamp(2, max_mode);

    // Check if we need to downshift (SNR below stay threshold)
    if current_mode >= 2 {
        let stay_threshold = MODE_THRESHOLDS[(current_mode - 1) as usize].stay_snr;
        if snr < stay_threshold {
            // Find highest mode where SNR >= enter_threshold
            for level in (2..=max_mode).rev() {
                if snr >= MODE_THRESHOLDS[(level - 1) as usize].enter_snr {
                    return level;
                }
            }
            return 2; // Minimum mode
        }
    }

    // Check if we could upshift (SNR above next mode's enter threshold)
    if current_mode < max_mode {
        let next_enter = MODE_THRESHOLDS[current_mode as usize].enter_snr;
        if snr >= next_enter {
            // Find highest mode where SNR >= enter_threshold
            for level in (2..=max_mode).rev() {
                if snr >= MODE_THRESHOLDS[(level - 1) as usize].enter_snr {
                    return level;
                }
            }
        }
    }

    current_mode
}

/// State for mode change negotiation
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NegotiationState {
    /// No negotiation in progress
    Idle,
    /// We sent a mode change request, waiting for ACK
    AwaitingAck,
    /// We received a request and will send ACK
    SendingAck,
    /// We received ACK, will send confirm
    SendingConfirm,
}

/// Timeout for AwaitingAck state in seconds (rollback if no ACK received)
const SPEED_CHANGE_ACK_TIMEOUT_SECS: u64 = 10;

/// Minimum time between mode changes in milliseconds (debounce to prevent oscillation)
/// This prevents rapid upshift->downshift->upshift cycles when SNR fluctuates near threshold
const MODE_CHANGE_DEBOUNCE_MS: u64 = 500;

/// SNR-based rate negotiation state
#[derive(Debug, Clone)]
pub struct RateNegotiator {
    /// Current agreed mode (used for TX/RX)
    pub agreed_mode: u8,
    /// My proposed mode based on SNR
    pub my_proposed_mode: u8,
    /// Peer's proposed mode from last received frame
    pub peer_proposed_mode: u8,
    /// EWMA-smoothed effective SNR
    pub snr_ewma: f32,
    /// Last measured local SNR
    pub local_snr: f32,
    /// Last received remote SNR (what peer measured on our signal)
    pub remote_snr: f32,
    /// Consecutive good SNR readings for upshift
    pub upshift_counter: u8,
    /// Number of readings required for upshift (after initial rapid upshift)
    pub upshift_threshold: u8,
    /// EWMA alpha (0.3 = moderate smoothing)
    pub ewma_alpha: f32,
    /// Maximum mode for current bandwidth
    pub max_mode: u8,
    /// Last keepalive received time
    pub last_keepalive: Instant,
    /// Keepalive interval in seconds
    pub keepalive_interval_secs: u64,
    /// Negotiation state for 2-way handshake
    pub negotiation_state: NegotiationState,
    /// Pending mode change (mode we're trying to switch to)
    pub pending_mode: Option<u8>,
    /// Whether initial rapid upshift has been done
    pub initial_upshift_done: bool,
    /// Whether we are the connection initiator (for reconnection and collision priority)
    pub is_initiator: bool,
    /// Count of frames received since connection (for initial upshift timing)
    pub frames_since_connect: u8,
    /// Time when we entered AwaitingAck state (for timeout detection)
    awaiting_ack_since: Option<Instant>,
    /// Time of last mode change (for debounce)
    last_mode_change: Instant,
}

impl RateNegotiator {
    /// Create new rate negotiator starting at mode 2
    pub fn new(max_mode: u8) -> Self {
        Self {
            agreed_mode: 2,
            my_proposed_mode: 2,
            peer_proposed_mode: 2,
            snr_ewma: 0.0,
            local_snr: 0.0,
            remote_snr: 0.0,
            upshift_counter: 0,
            upshift_threshold: 3,
            ewma_alpha: 0.3,
            max_mode: max_mode.min(17),
            last_keepalive: Instant::now(),
            keepalive_interval_secs: 15,
            negotiation_state: NegotiationState::Idle,
            pending_mode: None,
            initial_upshift_done: false,
            is_initiator: false,
            frames_since_connect: 0,
            awaiting_ack_since: None,
            last_mode_change: Instant::now(),
        }
    }

    /// Set whether we are the connection initiator
    pub fn set_initiator(&mut self, is_initiator: bool) {
        self.is_initiator = is_initiator;
    }

    /// Check if we should be the one to reconnect after timeout
    /// Only the original initiator should attempt reconnection
    pub fn should_reconnect(&self) -> bool {
        self.is_initiator
    }

    /// Process received frame with SNR info
    /// Returns true if mode change should occur
    pub fn process_received_snr(&mut self, measured_snr: i8, peer_proposed: u8) -> bool {
        self.last_keepalive = Instant::now();
        self.remote_snr = measured_snr as f32;
        self.peer_proposed_mode = clamp_speed_level(peer_proposed, self.max_mode);
        self.frames_since_connect = self.frames_since_connect.saturating_add(1);

        // Calculate effective SNR (limited by worse direction)
        let effective_snr = self.local_snr.min(self.remote_snr);

        // Update EWMA
        if self.snr_ewma == 0.0 {
            self.snr_ewma = effective_snr;
        } else {
            self.snr_ewma = self.ewma_alpha * effective_snr + (1.0 - self.ewma_alpha) * self.snr_ewma;
        }

        // Calculate my proposed mode based on smoothed SNR
        let new_proposed = calculate_proposed_mode(self.snr_ewma, self.agreed_mode, self.max_mode);

        // Debounce: prevent mode changes too quickly to avoid oscillation
        let debounce_elapsed = self.last_mode_change.elapsed().as_millis() as u64;
        let debounce_ok = debounce_elapsed >= MODE_CHANGE_DEBOUNCE_MS;

        // Fast down: immediate if SNR below stay threshold (but respect debounce)
        if new_proposed < self.agreed_mode {
            self.my_proposed_mode = new_proposed;
            self.upshift_counter = 0;
            // NOTE: Do NOT reset initial_upshift_done on downshift
            // This prevents oscillation where we rapidly upshift after every downshift
            if debounce_ok {
                return self.check_mode_agreement();
            }
            return false;
        }

        // Rapid initial upshift: after first few frames at mode 2, jump directly to SNR-supported mode
        // This allows fast channel acquisition while still being conservative on subsequent changes
        if !self.initial_upshift_done && self.agreed_mode == 2 && self.frames_since_connect >= 2 {
            if new_proposed > self.agreed_mode && debounce_ok {
                log::info!("Rapid initial upshift from mode 2 to mode {}", new_proposed);
                self.my_proposed_mode = new_proposed;
                self.initial_upshift_done = true;
                self.upshift_counter = 0;
                return self.check_mode_agreement();
            }
        }

        // Slow up: require consecutive good readings (after initial upshift)
        if new_proposed > self.agreed_mode {
            self.upshift_counter += 1;
            if self.upshift_counter >= self.upshift_threshold && debounce_ok {
                self.my_proposed_mode = new_proposed;
                self.upshift_counter = 0;
                return self.check_mode_agreement();
            }
        } else {
            self.upshift_counter = 0;
        }

        self.my_proposed_mode = self.agreed_mode;
        self.check_mode_agreement()
    }

    /// Update local SNR measurement
    pub fn update_local_snr(&mut self, snr: f32) {
        self.local_snr = snr;
    }

    /// Check if we and peer agree on mode change
    /// Returns true if mode should change
    fn check_mode_agreement(&mut self) -> bool {
        // Conservative: use lower of the two proposals
        let agreed = self.my_proposed_mode.min(self.peer_proposed_mode);

        if agreed != self.agreed_mode {
            // Mode change agreed
            self.pending_mode = Some(agreed);
            return true;
        }
        false
    }

    /// Handle mode change request from peer
    /// Returns true if we should ACK
    ///
    /// Collision resolution: If both sides send SpeedChange simultaneously,
    /// the connection initiator wins (has priority). The non-initiator
    /// abandons its request and ACKs the initiator's request.
    pub fn handle_mode_request(&mut self, requested_mode: u8) -> bool {
        let requested = clamp_speed_level(requested_mode, self.max_mode);

        // Collision detection: If we're awaiting ACK for our own request
        // and peer also sent a SpeedChange, we have a collision
        if self.negotiation_state == NegotiationState::AwaitingAck {
            if self.is_initiator {
                // We have priority - ignore peer's request, keep waiting for ACK
                log::info!(
                    "SpeedChange collision: we are initiator, ignoring peer's request for mode {}",
                    requested
                );
                return false;
            } else {
                // Peer has priority - abandon our request and process theirs
                log::info!(
                    "SpeedChange collision: peer is initiator, abandoning our request, accepting mode {}",
                    requested
                );
                // Fall through to accept peer's request
            }
        }

        // Downshift requests always accepted
        // Upshift requests accepted if we also propose that high or higher
        if requested <= self.agreed_mode || requested <= self.my_proposed_mode {
            self.pending_mode = Some(requested.min(self.my_proposed_mode));
            self.negotiation_state = NegotiationState::SendingAck;
            return true;
        }
        false
    }

    /// Handle ACK received for our request
    pub fn handle_ack(&mut self, acked_mode: u8) {
        if let Some(pending) = self.pending_mode {
            if acked_mode == pending {
                self.negotiation_state = NegotiationState::SendingConfirm;
            }
        }
    }

    /// Handle confirm received, complete the switch
    pub fn handle_confirm(&mut self, confirmed_mode: u8) {
        if let Some(pending) = self.pending_mode {
            if confirmed_mode == pending {
                self.agreed_mode = confirmed_mode;
                self.pending_mode = None;
                self.negotiation_state = NegotiationState::Idle;
                self.last_mode_change = Instant::now();
                log::info!("Mode change confirmed: now at mode {}", self.agreed_mode);
            }
        }
    }

    /// Complete mode switch after sending confirm (initiator side)
    pub fn complete_switch(&mut self) {
        if let Some(pending) = self.pending_mode.take() {
            self.agreed_mode = pending;
            self.negotiation_state = NegotiationState::Idle;
            self.last_mode_change = Instant::now();
            log::info!("Mode switch complete: now at mode {}", self.agreed_mode);
        }
    }

    /// Check for keepalive timeout, fall back to mode 2
    /// Returns true if fallback occurred
    pub fn check_timeout(&mut self) -> bool {
        let timeout = Duration::from_secs(self.keepalive_interval_secs * 3);
        if self.last_keepalive.elapsed() > timeout {
            if self.agreed_mode != 2 {
                log::warn!("Keepalive timeout, falling back to mode 2");
                self.agreed_mode = 2;
                self.my_proposed_mode = 2;
                self.peer_proposed_mode = 2;
                self.snr_ewma = 0.0;
                self.upshift_counter = 0;
                self.negotiation_state = NegotiationState::Idle;
                self.pending_mode = None;
                self.last_mode_change = Instant::now();
                return true;
            }
        }
        false
    }

    /// Reset for new connection
    pub fn reset(&mut self) {
        self.agreed_mode = 2;
        self.my_proposed_mode = 2;
        self.peer_proposed_mode = 2;
        self.snr_ewma = 0.0;
        self.local_snr = 0.0;
        self.remote_snr = 0.0;
        self.upshift_counter = 0;
        self.last_keepalive = Instant::now();
        self.negotiation_state = NegotiationState::Idle;
        self.pending_mode = None;
        self.initial_upshift_done = false;
        self.frames_since_connect = 0;
        self.awaiting_ack_since = None;
        self.last_mode_change = Instant::now();
        // Note: is_initiator is NOT reset here - it persists for reconnection logic
    }

    /// Get measured SNR as i8 for frame transmission
    pub fn get_measured_snr_i8(&self) -> i8 {
        (self.local_snr.clamp(-128.0, 127.0)) as i8
    }

    /// Check if we need to initiate a mode change request
    pub fn needs_mode_request(&self) -> bool {
        self.negotiation_state == NegotiationState::Idle
            && self.my_proposed_mode != self.agreed_mode
    }

    /// Check if we need to send ACK
    pub fn needs_ack(&self) -> bool {
        self.negotiation_state == NegotiationState::SendingAck
    }

    /// Check if we need to send confirm
    pub fn needs_confirm(&self) -> bool {
        self.negotiation_state == NegotiationState::SendingConfirm
    }

    /// Start awaiting ACK for a mode change request
    /// Sets state to AwaitingAck and records timestamp for timeout detection
    pub fn start_awaiting_ack(&mut self) {
        self.negotiation_state = NegotiationState::AwaitingAck;
        self.awaiting_ack_since = Some(Instant::now());
    }

    /// Check for AwaitingAck timeout and rollback if needed
    /// Returns true if timeout occurred and state was rolled back
    pub fn check_awaiting_ack_timeout(&mut self) -> bool {
        if self.negotiation_state != NegotiationState::AwaitingAck {
            return false;
        }

        if let Some(since) = self.awaiting_ack_since {
            if since.elapsed() > Duration::from_secs(SPEED_CHANGE_ACK_TIMEOUT_SECS) {
                log::warn!(
                    "SpeedChange ACK timeout after {}s, rolling back to mode {}",
                    SPEED_CHANGE_ACK_TIMEOUT_SECS,
                    self.agreed_mode
                );
                // Rollback: cancel pending request, return to Idle
                self.pending_mode = None;
                self.my_proposed_mode = self.agreed_mode;
                self.negotiation_state = NegotiationState::Idle;
                self.awaiting_ack_since = None;
                return true;
            }
        }

        false
    }

    /// Cancel any pending speed change request (for collision resolution)
    pub fn cancel_pending_request(&mut self) {
        if self.negotiation_state == NegotiationState::AwaitingAck {
            log::info!("Cancelling pending speed change request");
            self.pending_mode = None;
            self.my_proposed_mode = self.agreed_mode;
            self.negotiation_state = NegotiationState::Idle;
            self.awaiting_ack_since = None;
        }
    }
}

/// Predefined rate table with 17 speed levels
const RATE_TABLE: [RateInfo; 17] = [
    // Level 1: Most robust, FSK, 23 sym/s, 32/11/40 carriers
    RateInfo {
        level: 1, bps: 18.0, bps_500: 18.0, bps_2750: 18.0,
        min_snr: -6.0, modulation: ModulationType::Fsk,
        symbol_rate: 23.0, carriers: 32, carriers_500: 11, carriers_2750: 40,
        fft_size: FftSize::Fft2048, code_rate: 0.125, modulation_str: "FSK 1/8"
    },
    // Level 2: FSK, 47 sym/s, 16/11/20 carriers
    RateInfo {
        level: 2, bps: 41.0, bps_500: 41.0, bps_2750: 41.0,
        min_snr: -3.0, modulation: ModulationType::Fsk,
        symbol_rate: 47.0, carriers: 16, carriers_500: 11, carriers_2750: 20,
        fft_size: FftSize::Fft1024, code_rate: 0.167, modulation_str: "FSK 1/6"
    },
    // Level 3: FSK, 47 sym/s, 16/11/20 carriers, higher code rate
    RateInfo {
        level: 3, bps: 82.0, bps_500: 61.0, bps_2750: 82.0,
        min_snr: 0.0, modulation: ModulationType::Fsk,
        symbol_rate: 47.0, carriers: 16, carriers_500: 11, carriers_2750: 20,
        fft_size: FftSize::Fft1024, code_rate: 0.25, modulation_str: "FSK 1/4"
    },
    // Level 4: FSK/BPSK, 94 sym/s, 16/11/20 carriers (free version limit)
    // 500Hz uses 11 carriers with FFT=1024 for adequate capacity
    RateInfo {
        level: 4, bps: 175.0, bps_500: 88.0, bps_2750: 175.0,
        min_snr: 3.0, modulation: ModulationType::Bpsk,
        symbol_rate: 94.0, carriers: 16, carriers_500: 11, carriers_2750: 20,
        fft_size: FftSize::Fft512, code_rate: 0.33, modulation_str: "BPSK 1/3"
    },
    // Level 5: 4PSK, 94 sym/s, 3/11/3 carriers
    // 500Hz uses 11 carriers with FFT=1024 for adequate capacity
    RateInfo {
        level: 5, bps: 270.0, bps_500: 177.0, bps_2750: 270.0,
        min_snr: 6.0, modulation: ModulationType::Qpsk,
        symbol_rate: 94.0, carriers: 3, carriers_500: 11, carriers_2750: 3,
        fft_size: FftSize::Fft512, code_rate: 0.5, modulation_str: "4PSK 1/2"
    },
    // Level 6: 4PSK, 94 sym/s, 4/11/4 carriers
    // 500Hz uses 11 carriers with FFT=1024 for adequate capacity
    RateInfo {
        level: 6, bps: 363.0, bps_500: 270.0, bps_2750: 363.0,
        min_snr: 9.0, modulation: ModulationType::Qpsk,
        symbol_rate: 94.0, carriers: 4, carriers_500: 11, carriers_2750: 4,
        fft_size: FftSize::Fft512, code_rate: 0.5, modulation_str: "4PSK 1/2"
    },
    // Level 7: 4PSK, 94 sym/s, 6/11/6 carriers
    RateInfo {
        level: 7, bps: 549.0, bps_500: 441.0, bps_2750: 549.0,
        min_snr: 12.0, modulation: ModulationType::Qpsk,
        symbol_rate: 94.0, carriers: 6, carriers_500: 11, carriers_2750: 6,
        fft_size: FftSize::Fft512, code_rate: 0.5, modulation_str: "4PSK 1/2"
    },
    // Level 8: 4PSK, 94 sym/s, 8/11/8 carriers
    RateInfo {
        level: 8, bps: 735.0, bps_500: 588.0, bps_2750: 735.0,
        min_snr: 15.0, modulation: ModulationType::Qpsk,
        symbol_rate: 94.0, carriers: 8, carriers_500: 11, carriers_2750: 8,
        fft_size: FftSize::Fft512, code_rate: 0.5, modulation_str: "4PSK 1/2"
    },
    // Level 9: 4PSK, 94 sym/s, 10/11/10 carriers
    RateInfo {
        level: 9, bps: 922.0, bps_500: 705.0, bps_2750: 922.0,
        min_snr: 17.0, modulation: ModulationType::Qpsk,
        symbol_rate: 94.0, carriers: 10, carriers_500: 11, carriers_2750: 10,
        fft_size: FftSize::Fft512, code_rate: 0.5, modulation_str: "4PSK 1/2"
    },
    // Level 10: 4PSK, 94/42 sym/s, 49/11/13 carriers
    // 2750Hz uses 13 carriers at 94 sym/s (FFT=512), others use 49/11 carriers at 42 sym/s
    RateInfo {
        level: 10, bps: 2011.0, bps_500: 884.0, bps_2750: 1203.0,
        min_snr: 19.0, modulation: ModulationType::Qpsk,
        symbol_rate: 42.0, carriers: 49, carriers_500: 11, carriers_2750: 13,
        fft_size: FftSize::Fft1024, code_rate: 0.5, modulation_str: "4PSK 1/2"
    },
    // Level 11: 4PSK, 42 sym/s, 49/11/59 carriers
    RateInfo {
        level: 11, bps: 2682.0, bps_500: 1060.0, bps_2750: 2423.0,
        min_snr: 21.0, modulation: ModulationType::Qpsk,
        symbol_rate: 42.0, carriers: 49, carriers_500: 11, carriers_2750: 59,
        fft_size: FftSize::Fft1024, code_rate: 0.67, modulation_str: "4PSK 2/3"
    },
    // Level 12: 4PSK, 42 sym/s, 49/11/59 carriers
    RateInfo {
        level: 12, bps: 3219.0, bps_500: 1286.0, bps_2750: 3230.0,
        min_snr: 23.0, modulation: ModulationType::Qpsk,
        symbol_rate: 42.0, carriers: 49, carriers_500: 11, carriers_2750: 59,
        fft_size: FftSize::Fft1024, code_rate: 0.75, modulation_str: "4PSK 3/4"
    },
    // Level 13: 8PSK, 42 sym/s, 49/11/59 carriers
    RateInfo {
        level: 13, bps: 4025.0, bps_500: 1543.0, bps_2750: 3877.0,
        min_snr: 25.0, modulation: ModulationType::Psk8,
        symbol_rate: 42.0, carriers: 49, carriers_500: 11, carriers_2750: 59,
        fft_size: FftSize::Fft1024, code_rate: 0.67, modulation_str: "8PSK 2/3"
    },
    // Level 14: 8PSK, 42 sym/s, 49/11/59 carriers
    RateInfo {
        level: 14, bps: 4830.0, bps_500: 1543.0, bps_2750: 4848.0,
        min_snr: 27.0, modulation: ModulationType::Psk8,
        symbol_rate: 42.0, carriers: 49, carriers_500: 11, carriers_2750: 59,
        fft_size: FftSize::Fft1024, code_rate: 0.75, modulation_str: "8PSK 3/4"
    },
    // Level 15: 16QAM, 42 sym/s, 49/11/59 carriers
    RateInfo {
        level: 15, bps: 5872.0, bps_500: 1543.0, bps_2750: 5817.0,
        min_snr: 29.0, modulation: ModulationType::Qam16,
        symbol_rate: 42.0, carriers: 49, carriers_500: 11, carriers_2750: 59,
        fft_size: FftSize::Fft1024, code_rate: 0.75, modulation_str: "16QAM 3/4"
    },
    // Level 16: 32QAM, 42 sym/s, 49/11/59 carriers
    RateInfo {
        level: 16, bps: 7050.0, bps_500: 1543.0, bps_2750: 7074.0,
        min_snr: 31.0, modulation: ModulationType::Qam32,
        symbol_rate: 42.0, carriers: 49, carriers_500: 11, carriers_2750: 59,
        fft_size: FftSize::Fft1024, code_rate: 0.83, modulation_str: "32QAM 5/6"
    },
    // Level 17: 32QAM, 42 sym/s, 59 carriers (2750 only, others limited)
    RateInfo {
        level: 17, bps: 7050.0, bps_500: 1543.0, bps_2750: 8489.0,
        min_snr: 33.0, modulation: ModulationType::Qam32,
        symbol_rate: 42.0, carriers: 49, carriers_500: 11, carriers_2750: 59,
        fft_size: FftSize::Fft1024, code_rate: 0.83, modulation_str: "32QAM 5/6"
    },
];

/// Channel quality sample
#[derive(Debug, Clone)]
struct QualitySample {
    timestamp: Instant,
    snr: f32,
    success: bool,
}

/// Rate adapter configuration
#[derive(Debug, Clone)]
pub struct RateAdapterConfig {
    /// SNR margin above minimum (dB)
    pub snr_margin: f32,
    /// Number of successful frames before rate increase
    pub success_threshold: usize,
    /// Number of failed frames before rate decrease
    pub failure_threshold: usize,
    /// Minimum time between rate changes (seconds)
    pub rate_change_interval: f32,
    /// History window size
    pub history_size: usize,
}

impl Default for RateAdapterConfig {
    fn default() -> Self {
        Self {
            snr_margin: 3.0,
            success_threshold: 10,
            failure_threshold: 2,
            rate_change_interval: 2.0,
            history_size: 50,
        }
    }
}

/// Rate adapter for automatic speed level selection
pub struct RateAdapter {
    config: RateAdapterConfig,
    current_level: u8,
    min_level: u8,
    max_level: u8,
    history: VecDeque<QualitySample>,
    last_rate_change: Instant,
    consecutive_success: usize,
    consecutive_failure: usize,
}

impl RateAdapter {
    /// Create new rate adapter
    pub fn new(config: RateAdapterConfig) -> Self {
        Self {
            config,
            current_level: 9, // Start at middle
            min_level: 2, // Mode 1 disabled: needs soft-decision FSK decoding
            max_level: 17,
            history: VecDeque::new(),
            last_rate_change: Instant::now(),
            consecutive_success: 0,
            consecutive_failure: 0,
        }
    }

    /// Create with default configuration
    pub fn default_adapter() -> Self {
        Self::new(RateAdapterConfig::default())
    }

    /// Set speed level range
    pub fn set_level_range(&mut self, min: u8, max: u8) {
        // Mode 1 disabled: needs soft-decision FSK decoding
        self.min_level = min.clamp(2, 17);
        self.max_level = max.clamp(self.min_level, 17);
        self.current_level = self.current_level.clamp(self.min_level, self.max_level);
    }

    /// Report frame transmission result
    pub fn report_frame(&mut self, snr: f32, success: bool) {
        let sample = QualitySample {
            timestamp: Instant::now(),
            snr,
            success,
        };

        self.history.push_back(sample);

        // Trim history
        while self.history.len() > self.config.history_size {
            self.history.pop_front();
        }

        // Update consecutive counters
        if success {
            self.consecutive_success += 1;
            self.consecutive_failure = 0;
        } else {
            self.consecutive_failure += 1;
            self.consecutive_success = 0;
        }

        // Check for rate adaptation
        self.adapt();
    }

    /// Get recommended speed level based on SNR
    pub fn recommend_level(&self, snr: f32) -> u8 {
        for info in RATE_TABLE.iter().rev() {
            if snr >= info.min_snr + self.config.snr_margin {
                return info.level.clamp(self.min_level, self.max_level);
            }
        }
        self.min_level
    }

    /// Perform rate adaptation
    fn adapt(&mut self) {
        // Check rate change interval
        if self.last_rate_change.elapsed()
            < Duration::from_secs_f32(self.config.rate_change_interval)
        {
            return;
        }

        // Fast down, slow up
        if self.consecutive_failure >= self.config.failure_threshold {
            // Decrease rate quickly
            self.decrease_level();
        } else if self.consecutive_success >= self.config.success_threshold {
            // Try increasing rate
            if self.should_increase() {
                self.increase_level();
            }
        }
    }

    /// Check if conditions support rate increase
    fn should_increase(&self) -> bool {
        if self.current_level >= self.max_level {
            return false;
        }

        // Calculate average SNR from recent samples
        let avg_snr = self.average_snr();

        // Check if average SNR supports next level
        let next_level = self.current_level + 1;
        if let Some(info) = self.get_rate_info(next_level) {
            avg_snr >= info.min_snr + self.config.snr_margin
        } else {
            false
        }
    }

    /// Increase speed level
    fn increase_level(&mut self) {
        if self.current_level < self.max_level {
            self.current_level += 1;
            self.last_rate_change = Instant::now();
            self.consecutive_success = 0;
        }
    }

    /// Decrease speed level
    fn decrease_level(&mut self) {
        if self.current_level > self.min_level {
            self.current_level -= 1;
            self.last_rate_change = Instant::now();
            self.consecutive_failure = 0;
        }
    }

    /// Calculate average SNR from history
    fn average_snr(&self) -> f32 {
        if self.history.is_empty() {
            return 0.0;
        }

        let sum: f32 = self.history.iter().map(|s| s.snr).sum();
        sum / self.history.len() as f32
    }

    /// Get current speed level
    pub fn current_level(&self) -> u8 {
        self.current_level
    }

    /// Set current speed level
    pub fn set_level(&mut self, level: u8) {
        self.current_level = level.clamp(self.min_level, self.max_level);
        self.last_rate_change = Instant::now();
    }

    /// Get rate info for level
    pub fn get_rate_info(&self, level: u8) -> Option<&'static RateInfo> {
        if level >= 1 && level <= 17 {
            Some(&RATE_TABLE[(level - 1) as usize])
        } else {
            None
        }
    }

    /// Get current rate info
    pub fn current_rate_info(&self) -> &'static RateInfo {
        &RATE_TABLE[(self.current_level - 1) as usize]
    }

    /// Get success rate from history
    pub fn success_rate(&self) -> f32 {
        if self.history.is_empty() {
            return 1.0;
        }

        let successes = self.history.iter().filter(|s| s.success).count();
        successes as f32 / self.history.len() as f32
    }

    /// Get effective throughput (bps * success rate)
    pub fn effective_throughput(&self) -> f32 {
        self.current_rate_info().bps * self.success_rate()
    }

    /// Reset adapter state
    pub fn reset(&mut self) {
        self.history.clear();
        self.consecutive_success = 0;
        self.consecutive_failure = 0;
        self.last_rate_change = Instant::now();
    }
}

/// Get rate info for all levels
pub fn all_rate_info() -> &'static [RateInfo; 17] {
    &RATE_TABLE
}

/// Get rate info for a specific level (1-17)
pub fn get_rate_info(level: u8) -> &'static RateInfo {
    let idx = (level.saturating_sub(1) as usize).min(16);
    &RATE_TABLE[idx]
}

use crate::Bandwidth;

/// Frame configuration for each speed level
/// These are hardcoded per-mode settings for frame structure
#[derive(Debug, Clone, Copy)]
pub struct FrameConfig {
    /// Speed level (1-17)
    pub level: u8,
    /// FFT size for this mode
    pub fft_size: usize,
    /// Number of OFDM symbols per frame
    pub symbols: usize,
    /// FEC block size in bits
    pub fec_block_bits: usize,
    /// Maximum payload bytes per frame
    pub max_payload_bytes: usize,
    /// Approximate frame duration in milliseconds
    pub frame_duration_ms: u32,
    /// Modulation type for this mode/bandwidth combination
    pub modulation: ModulationType,
    /// FEC code rate for this mode/bandwidth combination
    pub code_rate: CodeRate,
}

/// Get maximum speed level for a given bandwidth
/// 500Hz: modes 1-13 only
/// 2300Hz: modes 1-16 only (mode 17 is 2750Hz only)
/// 2750Hz: modes 1-17
pub fn max_mode_for_bandwidth(bandwidth: Bandwidth) -> u8 {
    match bandwidth {
        Bandwidth::Hz500 => 13,
        Bandwidth::Hz2300 => 16,
        Bandwidth::Hz2750 => 17,
    }
}

/// Clamp speed level to valid range for bandwidth
pub fn clamp_mode_for_bandwidth(level: u8, bandwidth: Bandwidth) -> u8 {
    level.clamp(1, max_mode_for_bandwidth(bandwidth))
}

/// Frame configurations for 2300 Hz bandwidth (49 carriers)
/// These are the proven working values - do not change without testing
/// Connection request frames are 36 bytes (288 bits), so FEC block must be >= 288 bits
/// For data frames: max_payload = FEC_bytes - 10 (header) - 2 (len) - 4 (CRC) = FEC_bytes - 16
pub const FRAME_CONFIG_2300: [FrameConfig; 17] = [
    // Mode 1-4: FSK, rate 1/2
    FrameConfig { level: 1, fft_size: 2048, symbols: 120, fec_block_bits: 288, max_payload_bytes: 20, frame_duration_ms: 5200, modulation: ModulationType::Fsk, code_rate: CodeRate::Rate1_2 },
    FrameConfig { level: 2, fft_size: 1024, symbols: 150, fec_block_bits: 288, max_payload_bytes: 20, frame_duration_ms: 3200, modulation: ModulationType::Fsk, code_rate: CodeRate::Rate1_2 },
    FrameConfig { level: 3, fft_size: 1024, symbols: 150, fec_block_bits: 288, max_payload_bytes: 20, frame_duration_ms: 3200, modulation: ModulationType::Fsk, code_rate: CodeRate::Rate1_2 },
    FrameConfig { level: 4, fft_size: 512, symbols: 150, fec_block_bits: 288, max_payload_bytes: 20, frame_duration_ms: 1600, modulation: ModulationType::Fsk, code_rate: CodeRate::Rate1_2 },
    // Mode 5-10: QPSK, rate 1/2
    FrameConfig { level: 5, fft_size: 512, symbols: 124, fec_block_bits: 240, max_payload_bytes: 14, frame_duration_ms: 1830, modulation: ModulationType::Qpsk, code_rate: CodeRate::Rate1_2 },
    FrameConfig { level: 6, fft_size: 512, symbols: 96, fec_block_bits: 280, max_payload_bytes: 19, frame_duration_ms: 1490, modulation: ModulationType::Qpsk, code_rate: CodeRate::Rate1_2 },
    FrameConfig { level: 7, fft_size: 512, symbols: 96, fec_block_bits: 472, max_payload_bytes: 43, frame_duration_ms: 1490, modulation: ModulationType::Qpsk, code_rate: CodeRate::Rate1_2 },
    FrameConfig { level: 8, fft_size: 512, symbols: 96, fec_block_bits: 664, max_payload_bytes: 67, frame_duration_ms: 1490, modulation: ModulationType::Qpsk, code_rate: CodeRate::Rate1_2 },
    FrameConfig { level: 9, fft_size: 512, symbols: 96, fec_block_bits: 760, max_payload_bytes: 79, frame_duration_ms: 1490, modulation: ModulationType::Qpsk, code_rate: CodeRate::Rate1_2 },
    // LDPC modes: fec_block_bits = num_codewords × k, where k = info_bits(1944, rate)
    // OFDM: 49 carriers, ~6 pilots = 43 data carriers
    // Rate 1/2: k=972, encoded=1944 per codeword
    // Rate 2/3: k=1296, encoded=1944 per codeword
    // Rate 3/4: k=1458, encoded=1944 per codeword
    // Rate 5/6: k=1620, encoded=1944 per codeword
    // Mode 10: QPSK (86 bits/sym), 49 sym → 4214 bits, 2 codewords (3888), k_total=1944
    FrameConfig { level: 10, fft_size: 1024, symbols: 49, fec_block_bits: 1944, max_payload_bytes: 227, frame_duration_ms: 1490, modulation: ModulationType::Qpsk, code_rate: CodeRate::Rate1_2 },
    // Mode 11: QPSK rate 2/3, 2 codewords (3888), k_total=2592
    FrameConfig { level: 11, fft_size: 1024, symbols: 49, fec_block_bits: 2592, max_payload_bytes: 308, frame_duration_ms: 1490, modulation: ModulationType::Qpsk, code_rate: CodeRate::Rate2_3 },
    // Mode 12: QPSK rate 3/4, 2 codewords (3888), k_total=2916
    FrameConfig { level: 12, fft_size: 1024, symbols: 49, fec_block_bits: 2916, max_payload_bytes: 348, frame_duration_ms: 1490, modulation: ModulationType::Qpsk, code_rate: CodeRate::Rate3_4 },
    // Mode 13: 8PSK (129 bits/sym), 49 sym → 6321 bits, 3 codewords (5832), k_total=3888
    FrameConfig { level: 13, fft_size: 1024, symbols: 49, fec_block_bits: 3888, max_payload_bytes: 470, frame_duration_ms: 1490, modulation: ModulationType::Psk8, code_rate: CodeRate::Rate2_3 },
    // Mode 14: 8PSK rate 3/4, 3 codewords (5832), k_total=4374
    FrameConfig { level: 14, fft_size: 1024, symbols: 49, fec_block_bits: 4374, max_payload_bytes: 530, frame_duration_ms: 1490, modulation: ModulationType::Psk8, code_rate: CodeRate::Rate3_4 },
    // Mode 15: 16QAM (172 bits/sym), 49 sym → 8428 bits, 4 codewords (7776), k_total=5832
    FrameConfig { level: 15, fft_size: 1024, symbols: 49, fec_block_bits: 5832, max_payload_bytes: 712, frame_duration_ms: 1490, modulation: ModulationType::Qam16, code_rate: CodeRate::Rate3_4 },
    // Mode 16: 32QAM (215 bits/sym), 49 sym → 10535 bits, 5 codewords (9720), k_total=8100
    FrameConfig { level: 16, fft_size: 1024, symbols: 49, fec_block_bits: 8100, max_payload_bytes: 996, frame_duration_ms: 1490, modulation: ModulationType::Qam32, code_rate: CodeRate::Rate5_6 },
    // Mode 17: NOT for 2300Hz - fallback entry only (uses same as mode 16)
    FrameConfig { level: 17, fft_size: 1024, symbols: 49, fec_block_bits: 8100, max_payload_bytes: 996, frame_duration_ms: 1490, modulation: ModulationType::Qam32, code_rate: CodeRate::Rate5_6 },
];

/// Frame configurations for 500 Hz bandwidth
/// FSK modes 1-3: 8 tones (3 bits/sym) - narrower than 2300Hz
/// OFDM modes 4-6: 2-3 carriers with BPSK/QPSK
/// OFDM modes 7-13: 11 carriers (9 data + 2 pilot)
/// 500Hz bandwidth supports modes 1-13 ONLY (no modes 14-17)
pub const FRAME_CONFIG_500: [FrameConfig; 17] = [
    // Mode 1-3: FSK, rate 1/2
    FrameConfig { level: 1, fft_size: 2048, symbols: 192, fec_block_bits: 288, max_payload_bytes: 20, frame_duration_ms: 8350, modulation: ModulationType::Fsk, code_rate: CodeRate::Rate1_2 },
    FrameConfig { level: 2, fft_size: 1024, symbols: 192, fec_block_bits: 288, max_payload_bytes: 20, frame_duration_ms: 4100, modulation: ModulationType::Fsk, code_rate: CodeRate::Rate1_2 },
    FrameConfig { level: 3, fft_size: 1024, symbols: 192, fec_block_bits: 288, max_payload_bytes: 20, frame_duration_ms: 4100, modulation: ModulationType::Fsk, code_rate: CodeRate::Rate1_2 },
    // Mode 4-6: BPSK/QPSK, rate 1/2, FFT=1024, 11 carriers (9 data + 2 pilot)
    // BPSK: 9 bits/symbol, QPSK: 18 bits/symbol
    // FEC adds 16 tail bits: encoded = fec_block_bits * 2 + 16
    // Mode 4 BPSK: 66 sym * 9 = 594 >= 288*2+16=592
    // Mode 5 QPSK: 33 sym * 18 = 594 >= 288*2+16=592
    // Mode 6 QPSK: 42 sym * 18 = 756 >= 360*2+16=736
    FrameConfig { level: 4, fft_size: 1024, symbols: 66, fec_block_bits: 288, max_payload_bytes: 20, frame_duration_ms: 1580, modulation: ModulationType::Bpsk, code_rate: CodeRate::Rate1_2 },
    FrameConfig { level: 5, fft_size: 1024, symbols: 33, fec_block_bits: 288, max_payload_bytes: 20, frame_duration_ms: 790, modulation: ModulationType::Qpsk, code_rate: CodeRate::Rate1_2 },
    FrameConfig { level: 6, fft_size: 1024, symbols: 42, fec_block_bits: 360, max_payload_bytes: 29, frame_duration_ms: 1010, modulation: ModulationType::Qpsk, code_rate: CodeRate::Rate1_2 },
    // Mode 7: QPSK, rate 1/2
    FrameConfig { level: 7, fft_size: 1024, symbols: 49, fec_block_bits: 432, max_payload_bytes: 38, frame_duration_ms: 1050, modulation: ModulationType::Qpsk, code_rate: CodeRate::Rate1_2 },
    // Mode 8: QPSK, rate 2/3
    FrameConfig { level: 8, fft_size: 1024, symbols: 49, fec_block_bits: 576, max_payload_bytes: 56, frame_duration_ms: 1050, modulation: ModulationType::Qpsk, code_rate: CodeRate::Rate2_3 },
    // Mode 9: QPSK, rate 3/4
    FrameConfig { level: 9, fft_size: 1024, symbols: 49, fec_block_bits: 648, max_payload_bytes: 65, frame_duration_ms: 1050, modulation: ModulationType::Qpsk, code_rate: CodeRate::Rate3_4 },
    // Mode 10: 8PSK, rate 2/3
    FrameConfig { level: 10, fft_size: 1024, symbols: 49, fec_block_bits: 864, max_payload_bytes: 92, frame_duration_ms: 1050, modulation: ModulationType::Psk8, code_rate: CodeRate::Rate2_3 },
    // Mode 11: 8PSK, rate 3/4
    FrameConfig { level: 11, fft_size: 1024, symbols: 49, fec_block_bits: 976, max_payload_bytes: 106, frame_duration_ms: 1050, modulation: ModulationType::Psk8, code_rate: CodeRate::Rate3_4 },
    // Mode 12: 16QAM, rate 3/4
    FrameConfig { level: 12, fft_size: 1024, symbols: 49, fec_block_bits: 1304, max_payload_bytes: 147, frame_duration_ms: 1050, modulation: ModulationType::Qam16, code_rate: CodeRate::Rate3_4 },
    // Mode 13: 32QAM, rate 3/4 - 500Hz MAX MODE
    FrameConfig { level: 13, fft_size: 1024, symbols: 49, fec_block_bits: 1640, max_payload_bytes: 189, frame_duration_ms: 1050, modulation: ModulationType::Qam32, code_rate: CodeRate::Rate3_4 },
    // Modes 14-17: DO NOT EXIST for 500Hz - fallback entries only
    FrameConfig { level: 14, fft_size: 1024, symbols: 49, fec_block_bits: 1640, max_payload_bytes: 189, frame_duration_ms: 1050, modulation: ModulationType::Qam32, code_rate: CodeRate::Rate3_4 },
    FrameConfig { level: 15, fft_size: 1024, symbols: 49, fec_block_bits: 1640, max_payload_bytes: 189, frame_duration_ms: 1050, modulation: ModulationType::Qam32, code_rate: CodeRate::Rate3_4 },
    FrameConfig { level: 16, fft_size: 1024, symbols: 49, fec_block_bits: 1640, max_payload_bytes: 189, frame_duration_ms: 1050, modulation: ModulationType::Qam32, code_rate: CodeRate::Rate3_4 },
    FrameConfig { level: 17, fft_size: 1024, symbols: 49, fec_block_bits: 1640, max_payload_bytes: 189, frame_duration_ms: 1050, modulation: ModulationType::Qam32, code_rate: CodeRate::Rate3_4 },
];

/// Frame configurations for 2750 Hz bandwidth
/// Modes 1-4: FSK (40/20/20/20 carriers)
/// Modes 5-10: QPSK with FFT=512 (3/4/6/8/10/13 carriers)
/// Modes 11-17: 59 carriers with FFT=1024
/// FEC adds 16 tail bits: encoded = fec_block_bits * rate_multiplier + 16
pub const FRAME_CONFIG_2750: [FrameConfig; 17] = [
    // Mode 1-4: FSK, rate 1/2
    FrameConfig { level: 1, fft_size: 2048, symbols: 120, fec_block_bits: 288, max_payload_bytes: 20, frame_duration_ms: 5200, modulation: ModulationType::Fsk, code_rate: CodeRate::Rate1_2 },
    FrameConfig { level: 2, fft_size: 1024, symbols: 150, fec_block_bits: 288, max_payload_bytes: 20, frame_duration_ms: 3200, modulation: ModulationType::Fsk, code_rate: CodeRate::Rate1_2 },
    FrameConfig { level: 3, fft_size: 1024, symbols: 150, fec_block_bits: 288, max_payload_bytes: 20, frame_duration_ms: 3200, modulation: ModulationType::Fsk, code_rate: CodeRate::Rate1_2 },
    FrameConfig { level: 4, fft_size: 512, symbols: 150, fec_block_bits: 288, max_payload_bytes: 20, frame_duration_ms: 1600, modulation: ModulationType::Fsk, code_rate: CodeRate::Rate1_2 },
    // Mode 5-10: QPSK, FFT=512, rate 1/2
    // Mode 5: 3 carriers (2 data + 1 pilot), 4 bits/sym, 148 sym → 592 raw >= 288*2+16=592
    FrameConfig { level: 5, fft_size: 512, symbols: 148, fec_block_bits: 288, max_payload_bytes: 20, frame_duration_ms: 1580, modulation: ModulationType::Qpsk, code_rate: CodeRate::Rate1_2 },
    // Mode 6: 4 carriers (3 data + 1 pilot), 6 bits/sym, 99 sym → 594 raw >= 592
    FrameConfig { level: 6, fft_size: 512, symbols: 99, fec_block_bits: 288, max_payload_bytes: 20, frame_duration_ms: 1060, modulation: ModulationType::Qpsk, code_rate: CodeRate::Rate1_2 },
    // Mode 7: 6 carriers (5 data + 1 pilot), 10 bits/sym, 100 sym → 1000 raw >= 480*2+16=976
    FrameConfig { level: 7, fft_size: 512, symbols: 100, fec_block_bits: 480, max_payload_bytes: 44, frame_duration_ms: 1070, modulation: ModulationType::Qpsk, code_rate: CodeRate::Rate1_2 },
    // Mode 8: 8 carriers (6 data + 2 pilots), 12 bits/sym, 100 sym → 1200 raw >= 584*2+16=1184
    FrameConfig { level: 8, fft_size: 512, symbols: 100, fec_block_bits: 584, max_payload_bytes: 57, frame_duration_ms: 1070, modulation: ModulationType::Qpsk, code_rate: CodeRate::Rate1_2 },
    // Mode 9: 10 carriers (8 data + 2 pilots), 16 bits/sym, 100 sym → 1600 raw >= 784*2+16=1584
    FrameConfig { level: 9, fft_size: 512, symbols: 100, fec_block_bits: 784, max_payload_bytes: 82, frame_duration_ms: 1070, modulation: ModulationType::Qpsk, code_rate: CodeRate::Rate1_2 },
    // Mode 10: 13 carriers (11 data + 2 pilots), 22 bits/sym, 100 sym → 2200 raw >= 1088*2+16=2192
    FrameConfig { level: 10, fft_size: 512, symbols: 100, fec_block_bits: 1088, max_payload_bytes: 120, frame_duration_ms: 1070, modulation: ModulationType::Qpsk, code_rate: CodeRate::Rate1_2 },
    // Mode 11-17: 59 carriers (51 data + 8 pilots), FFT=1024
    // QPSK: 51*2=102 bits/sym, 8PSK: 51*3=153 bits/sym, 16QAM: 51*4=204 bits/sym, 32QAM: 51*5=255 bits/sym
    // LDPC: Rate 1/2 k=972, Rate 2/3 k=1296, Rate 3/4 k=1458, Rate 5/6 k=1620
    // Mode 11: QPSK rate 1/2, 50 sym × 102 = 5100 bits, 2 codewords (3888), k_total=1944
    FrameConfig { level: 11, fft_size: 1024, symbols: 50, fec_block_bits: 1944, max_payload_bytes: 227, frame_duration_ms: 1200, modulation: ModulationType::Qpsk, code_rate: CodeRate::Rate1_2 },
    // Mode 12: QPSK rate 2/3, 2 codewords (3888), k_total=2592
    FrameConfig { level: 12, fft_size: 1024, symbols: 50, fec_block_bits: 2592, max_payload_bytes: 308, frame_duration_ms: 1200, modulation: ModulationType::Qpsk, code_rate: CodeRate::Rate2_3 },
    // Mode 13: QPSK rate 3/4, 2 codewords (3888), k_total=2916
    FrameConfig { level: 13, fft_size: 1024, symbols: 50, fec_block_bits: 2916, max_payload_bytes: 348, frame_duration_ms: 1200, modulation: ModulationType::Qpsk, code_rate: CodeRate::Rate3_4 },
    // Mode 14: 8PSK rate 2/3, 50 sym × 153 = 7650 bits, 3 codewords (5832), k_total=3888
    FrameConfig { level: 14, fft_size: 1024, symbols: 50, fec_block_bits: 3888, max_payload_bytes: 470, frame_duration_ms: 1200, modulation: ModulationType::Psk8, code_rate: CodeRate::Rate2_3 },
    // Mode 15: 8PSK rate 3/4, 3 codewords (5832), k_total=4374
    FrameConfig { level: 15, fft_size: 1024, symbols: 50, fec_block_bits: 4374, max_payload_bytes: 530, frame_duration_ms: 1200, modulation: ModulationType::Psk8, code_rate: CodeRate::Rate3_4 },
    // Mode 16: 16QAM rate 3/4, 50 sym × 204 = 10200 bits, 5 codewords (9720), k_total=7290
    FrameConfig { level: 16, fft_size: 1024, symbols: 50, fec_block_bits: 7290, max_payload_bytes: 895, frame_duration_ms: 1200, modulation: ModulationType::Qam16, code_rate: CodeRate::Rate3_4 },
    // Mode 17: 32QAM rate 5/6, 50 sym × 255 = 12750 bits, 6 codewords (11664), k_total=9720
    FrameConfig { level: 17, fft_size: 1024, symbols: 50, fec_block_bits: 9720, max_payload_bytes: 1199, frame_duration_ms: 1200, modulation: ModulationType::Qam32, code_rate: CodeRate::Rate5_6 },
];

/// Get frame configuration for a speed level and bandwidth
pub fn get_frame_config(level: u8, bandwidth: Bandwidth) -> &'static FrameConfig {
    let idx = (level.saturating_sub(1) as usize).min(16);
    match bandwidth {
        Bandwidth::Hz500 => &FRAME_CONFIG_500[idx],
        Bandwidth::Hz2300 => &FRAME_CONFIG_2300[idx],
        Bandwidth::Hz2750 => &FRAME_CONFIG_2750[idx],
    }
}

/// Get effective throughput (bps) for a speed level and bandwidth
///
/// This accounts for frame overhead (header, CRC, length fields) to give
/// the actual user data throughput rather than raw channel capacity.
pub fn get_effective_bps(level: u8, bandwidth: Bandwidth) -> f32 {
    let rate_info = get_rate_info(level);
    let frame_config = get_frame_config(level, bandwidth);
    rate_info.effective_bps_for_bandwidth(bandwidth, frame_config.max_payload_bytes)
}

/// Get throughput efficiency ratio for a speed level and bandwidth
///
/// Returns the ratio of effective throughput to raw throughput (0.0 to 1.0).
/// Higher payload modes have better efficiency due to lower overhead ratio.
pub fn get_throughput_efficiency(level: u8, bandwidth: Bandwidth) -> f32 {
    let frame_config = get_frame_config(level, bandwidth);
    let payload = frame_config.max_payload_bytes;
    if payload == 0 {
        return 0.0;
    }
    payload as f32 / (payload + FRAME_OVERHEAD_BYTES) as f32
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_rate_recommendation() {
        let adapter = RateAdapter::default_adapter();

        // With 3dB margin: level needs min_snr + 3dB
        // Mode 1 is disabled, so min_level is 2
        // Level 2: -3dB + 3 = 0dB, Level 3: 0dB + 3 = 3dB, Level 10: 19dB + 3 = 22dB
        assert_eq!(adapter.recommend_level(-10.0), 2);  // Below all thresholds, return min (2)
        assert_eq!(adapter.recommend_level(-2.0), 2);   // Below level 2 threshold, return min (2)
        assert_eq!(adapter.recommend_level(1.0), 2);    // 1dB >= 0dB for level 2
        assert_eq!(adapter.recommend_level(22.0), 10);  // 22dB >= 19+3=22dB for level 10
        assert_eq!(adapter.recommend_level(40.0), 17);  // High SNR gets max level
    }

    #[test]
    fn test_rate_table() {
        // Verify bitrates
        assert_eq!(RATE_TABLE[0].bps, 18.0);    // Level 1
        assert_eq!(RATE_TABLE[3].bps, 175.0);   // Level 4 (free version limit)
        assert_eq!(RATE_TABLE[9].bps, 2011.0);  // Level 10
        assert_eq!(RATE_TABLE[15].bps, 7050.0); // Level 16
        assert_eq!(RATE_TABLE[16].bps_2750, 8489.0); // Level 17 (2750 mode max)
    }

    #[test]
    fn test_modulation_progression() {
        // FSK for levels 1-4, 4PSK for 5-12, 8PSK for 13-14, etc.
        assert_eq!(RATE_TABLE[0].modulation, ModulationType::Fsk);  // Level 1
        assert_eq!(RATE_TABLE[3].modulation, ModulationType::Bpsk); // Level 4
        assert_eq!(RATE_TABLE[4].modulation, ModulationType::Qpsk); // Level 5
        assert_eq!(RATE_TABLE[11].modulation, ModulationType::Qpsk); // Level 12
        assert_eq!(RATE_TABLE[12].modulation, ModulationType::Psk8); // Level 13
        assert_eq!(RATE_TABLE[14].modulation, ModulationType::Qam16); // Level 15
        assert_eq!(RATE_TABLE[15].modulation, ModulationType::Qam32); // Level 16
    }

    #[test]
    fn test_carrier_counts() {
        // Verify carrier progression
        assert_eq!(RATE_TABLE[0].carriers, 32);  // Level 1: many carriers, low mod
        assert_eq!(RATE_TABLE[4].carriers, 3);   // Level 5: few carriers
        assert_eq!(RATE_TABLE[9].carriers, 49);  // Level 10: many carriers again
        assert_eq!(RATE_TABLE[16].carriers_2750, 59); // Level 17: max carriers in 2750 mode
    }

    #[test]
    fn test_rate_decrease_on_failure() {
        let mut config = RateAdapterConfig::default();
        config.rate_change_interval = 0.0; // Disable interval check for test
        config.failure_threshold = 2;

        let mut adapter = RateAdapter::new(config);
        adapter.set_level(10);

        // Simulate failures
        for _ in 0..5 {
            adapter.report_frame(10.0, false);
        }

        assert!(adapter.current_level() < 10);
    }

    #[test]
    fn test_rate_increase_on_success() {
        let mut config = RateAdapterConfig::default();
        config.rate_change_interval = 0.0; // Disable interval check
        config.success_threshold = 3;

        let mut adapter = RateAdapter::new(config);
        adapter.set_level(5);

        // Simulate successes with good SNR
        for _ in 0..10 {
            adapter.report_frame(20.0, true);
        }

        assert!(adapter.current_level() > 5);
    }

    #[test]
    fn test_level_range() {
        let mut adapter = RateAdapter::default_adapter();
        adapter.set_level_range(5, 12);

        adapter.set_level(1);
        assert_eq!(adapter.current_level(), 5);

        adapter.set_level(17);
        assert_eq!(adapter.current_level(), 12);
    }

    #[test]
    fn test_mode_threshold_bounds() {
        // Test get_mode_threshold bounds checking
        assert!(get_mode_threshold(0).is_none());
        assert!(get_mode_threshold(1).is_some());
        assert!(get_mode_threshold(17).is_some());
        assert!(get_mode_threshold(18).is_none());
        assert!(get_mode_threshold(255).is_none());

        // Verify threshold values are accessible
        let t1 = get_mode_threshold(1).unwrap();
        assert_eq!(t1.level, 1);
        let t17 = get_mode_threshold(17).unwrap();
        assert_eq!(t17.level, 17);
    }

    #[test]
    fn test_clamp_speed_level() {
        // Test clamp_speed_level function
        assert_eq!(clamp_speed_level(0, 17), MIN_SPEED_LEVEL);
        assert_eq!(clamp_speed_level(1, 17), MIN_SPEED_LEVEL);
        assert_eq!(clamp_speed_level(2, 17), 2);
        assert_eq!(clamp_speed_level(10, 17), 10);
        assert_eq!(clamp_speed_level(17, 17), 17);
        assert_eq!(clamp_speed_level(18, 17), 17);
        assert_eq!(clamp_speed_level(255, 17), 17);

        // Test with lower max_mode (500Hz bandwidth limits to mode 13)
        assert_eq!(clamp_speed_level(15, 13), 13);
        assert_eq!(clamp_speed_level(2, 13), 2);
    }

    #[test]
    fn test_calculate_proposed_mode_bounds() {
        // Test that calculate_proposed_mode doesn't panic with edge cases

        // Normal case
        let mode = calculate_proposed_mode(20.0, 10, 17);
        assert!(mode >= 2 && mode <= 17);

        // Edge case: current_mode at max
        let mode = calculate_proposed_mode(40.0, 17, 17);
        assert_eq!(mode, 17);

        // Edge case: current_mode at min
        let mode = calculate_proposed_mode(-10.0, 2, 17);
        assert_eq!(mode, 2);

        // Edge case: out of bounds inputs (should be clamped)
        let mode = calculate_proposed_mode(20.0, 0, 17);  // current_mode too low
        assert!(mode >= 2 && mode <= 17);

        let mode = calculate_proposed_mode(20.0, 20, 17);  // current_mode too high
        assert!(mode >= 2 && mode <= 17);

        // Edge case: max_mode limited (500Hz)
        let mode = calculate_proposed_mode(40.0, 12, 13);
        assert!(mode <= 13);
    }

    #[test]
    fn test_rate_negotiator_bounds() {
        let mut negotiator = RateNegotiator::new(17);

        // Test process_received_snr clamps peer_proposed
        negotiator.process_received_snr(20, 0);  // peer_proposed too low
        assert!(negotiator.peer_proposed_mode >= MIN_SPEED_LEVEL);

        negotiator.process_received_snr(20, 255);  // peer_proposed too high
        assert!(negotiator.peer_proposed_mode <= MAX_SPEED_LEVEL);

        // Test handle_mode_request clamps
        let _ = negotiator.handle_mode_request(0);  // too low
        let _ = negotiator.handle_mode_request(255);  // too high
        // Should not panic
    }

    #[test]
    fn test_speed_change_collision_initiator_wins() {
        // Test that connection initiator wins collision
        let mut initiator = RateNegotiator::new(17);
        initiator.set_initiator(true);
        initiator.agreed_mode = 10;
        initiator.my_proposed_mode = 12;
        initiator.pending_mode = Some(12);

        // Initiator is awaiting ACK for its mode 12 request
        initiator.start_awaiting_ack();
        assert_eq!(initiator.negotiation_state, NegotiationState::AwaitingAck);

        // Peer sends SpeedChange for mode 8 - collision!
        // Initiator should ignore peer's request (it has priority)
        let accepted = initiator.handle_mode_request(8);
        assert!(!accepted, "Initiator should reject peer's request during collision");
        assert_eq!(initiator.negotiation_state, NegotiationState::AwaitingAck);
        assert_eq!(initiator.pending_mode, Some(12));
    }

    #[test]
    fn test_speed_change_collision_non_initiator_yields() {
        // Test that non-initiator yields to peer in collision
        let mut responder = RateNegotiator::new(17);
        responder.set_initiator(false);
        responder.agreed_mode = 10;
        responder.my_proposed_mode = 12;
        responder.pending_mode = Some(12);

        // Responder is awaiting ACK for its mode 12 request
        responder.start_awaiting_ack();
        assert_eq!(responder.negotiation_state, NegotiationState::AwaitingAck);

        // Peer (initiator) sends SpeedChange for mode 8 - collision!
        // Responder should accept peer's request (peer has priority)
        let accepted = responder.handle_mode_request(8);
        assert!(accepted, "Non-initiator should accept peer's request during collision");
        assert_eq!(responder.negotiation_state, NegotiationState::SendingAck);
        assert_eq!(responder.pending_mode, Some(8));
    }

    #[test]
    fn test_speed_change_no_collision_when_idle() {
        // Normal case: no collision when state is Idle
        let mut negotiator = RateNegotiator::new(17);
        negotiator.agreed_mode = 10;
        negotiator.my_proposed_mode = 10;

        // Idle state - no collision
        assert_eq!(negotiator.negotiation_state, NegotiationState::Idle);

        // Peer sends SpeedChange for mode 8 (downshift)
        let accepted = negotiator.handle_mode_request(8);
        assert!(accepted, "Should accept downshift request when idle");
        assert_eq!(negotiator.negotiation_state, NegotiationState::SendingAck);
    }

    #[test]
    fn test_awaiting_ack_timeout_rollback() {
        let mut negotiator = RateNegotiator::new(17);
        negotiator.agreed_mode = 10;
        negotiator.pending_mode = Some(12);
        negotiator.my_proposed_mode = 12;

        // Start awaiting ACK
        negotiator.start_awaiting_ack();
        assert_eq!(negotiator.negotiation_state, NegotiationState::AwaitingAck);
        assert!(negotiator.awaiting_ack_since.is_some());

        // No timeout yet (just started)
        assert!(!negotiator.check_awaiting_ack_timeout());

        // Can't easily test actual timeout without waiting 10 seconds,
        // but we can verify the mechanism is in place
        assert_eq!(negotiator.negotiation_state, NegotiationState::AwaitingAck);
    }

    #[test]
    fn test_cancel_pending_request() {
        let mut negotiator = RateNegotiator::new(17);
        negotiator.agreed_mode = 10;
        negotiator.pending_mode = Some(12);
        negotiator.my_proposed_mode = 12;
        negotiator.start_awaiting_ack();

        // Cancel the request
        negotiator.cancel_pending_request();

        assert_eq!(negotiator.negotiation_state, NegotiationState::Idle);
        assert_eq!(negotiator.pending_mode, None);
        assert_eq!(negotiator.my_proposed_mode, 10);  // Reset to agreed_mode
        assert!(negotiator.awaiting_ack_since.is_none());
    }

    #[test]
    fn test_effective_bps() {
        // Test that effective_bps is always less than raw bps due to overhead
        for level in 1..=17 {
            let rate_info = get_rate_info(level);
            let frame_config_2300 = get_frame_config(level, Bandwidth::Hz2300);

            let raw_bps = rate_info.bps;
            let effective = rate_info.effective_bps(frame_config_2300.max_payload_bytes);

            // Effective should be less than raw due to overhead
            assert!(effective < raw_bps,
                "Level {}: effective {} should be < raw {}", level, effective, raw_bps);

            // Effective should be positive
            assert!(effective > 0.0,
                "Level {}: effective {} should be positive", level, effective);

            // Calculate expected efficiency
            let payload = frame_config_2300.max_payload_bytes;
            let expected_efficiency = payload as f32 / (payload + FRAME_OVERHEAD_BYTES) as f32;
            let actual_efficiency = effective / raw_bps;

            // Actual efficiency should match expected within tolerance
            assert!((actual_efficiency - expected_efficiency).abs() < 0.01,
                "Level {}: efficiency mismatch: actual {} vs expected {}",
                level, actual_efficiency, expected_efficiency);
        }
    }

    #[test]
    fn test_throughput_efficiency() {
        // Higher payload modes should have better efficiency
        let eff_mode_2 = get_throughput_efficiency(2, Bandwidth::Hz2300);
        let eff_mode_12 = get_throughput_efficiency(12, Bandwidth::Hz2300);
        let eff_mode_16 = get_throughput_efficiency(16, Bandwidth::Hz2300);

        // Mode 12 and 16 have larger payloads, so better efficiency
        assert!(eff_mode_12 > eff_mode_2,
            "Mode 12 efficiency {} should be > mode 2 efficiency {}", eff_mode_12, eff_mode_2);
        assert!(eff_mode_16 > eff_mode_12,
            "Mode 16 efficiency {} should be > mode 12 efficiency {}", eff_mode_16, eff_mode_12);

        // Efficiency should be between 0 and 1
        assert!(eff_mode_2 > 0.0 && eff_mode_2 < 1.0);
        assert!(eff_mode_16 > 0.0 && eff_mode_16 < 1.0);
    }

    #[test]
    fn test_get_effective_bps_convenience() {
        // Test the convenience function
        let eff_2300 = get_effective_bps(12, Bandwidth::Hz2300);
        let eff_500 = get_effective_bps(12, Bandwidth::Hz500);
        let eff_2750 = get_effective_bps(12, Bandwidth::Hz2750);

        // All should be positive
        assert!(eff_2300 > 0.0);
        assert!(eff_500 > 0.0);
        assert!(eff_2750 > 0.0);

        // 2300 and 2750 should have higher throughput than 500Hz
        assert!(eff_2300 > eff_500);
        assert!(eff_2750 > eff_500);
    }

    #[test]
    fn test_carriers_for_bandwidth() {
        let rate_info = get_rate_info(12);

        assert_eq!(rate_info.carriers_for_bandwidth(Bandwidth::Hz500), rate_info.carriers_500);
        assert_eq!(rate_info.carriers_for_bandwidth(Bandwidth::Hz2300), rate_info.carriers);
        assert_eq!(rate_info.carriers_for_bandwidth(Bandwidth::Hz2750), rate_info.carriers_2750);
    }
}
