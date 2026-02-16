//! Automatic Repeat Request (ARQ) protocol
//!
//! Implements selective repeat ARQ for reliable data transfer

use std::collections::{HashMap, VecDeque};
use std::time::{Duration, Instant};
use super::{Frame, FrameType, ProtocolError, MAX_RETRIES};

/// ARQ state
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ArqState {
    /// Idle, no active transmission
    Idle,
    /// Waiting for ACK
    WaitingAck,
    /// Transmitting data
    Transmitting,
    /// Receiving data
    Receiving,
}

/// ARQ configuration
#[derive(Debug, Clone)]
pub struct ArqConfig {
    /// Window size (number of unacknowledged frames allowed)
    pub window_size: usize,
    /// Timeout for ACK (milliseconds)
    pub ack_timeout_ms: u32,
    /// Maximum retransmission attempts per frame
    pub max_retries: usize,
    /// Enable selective repeat (vs go-back-N)
    pub selective_repeat: bool,
    /// Maximum TX queue size (bytes) to prevent memory exhaustion
    pub max_queue_bytes: usize,
}

impl Default for ArqConfig {
    fn default() -> Self {
        Self {
            window_size: 1,  // Stop-and-wait: send one frame, wait for ACK before next
            ack_timeout_ms: 15000, // Default 15s, should be overridden with for_frame_duration()
            max_retries: MAX_RETRIES,
            selective_repeat: true,
            max_queue_bytes: 1024 * 1024, // 1MB max queue
        }
    }
}

impl ArqConfig {
    /// Create ARQ config with timeout based on frame duration
    /// Timeout = 2 * frame_duration * 1.8 + 2000ms overhead
    /// This allows for round-trip (data + ACK) plus processing margin
    pub fn for_frame_duration(frame_duration_ms: u32) -> Self {
        // Round trip = 2 × frame_duration
        // With 80% margin for processing/propagation delays
        // Plus 2000ms fixed overhead
        let timeout_ms = (frame_duration_ms as u64 * 2 * 18 / 10 + 2000) as u32;

        Self {
            ack_timeout_ms: timeout_ms,
            ..Default::default()
        }
    }
}

/// Pending frame waiting for acknowledgment
#[derive(Debug, Clone)]
struct PendingFrame {
    frame: Frame,
    sent_time: Instant,
    retries: usize,
}

/// ARQ controller
pub struct ArqController {
    config: ArqConfig,
    state: ArqState,
    session_id: u16,

    // Transmit state
    tx_next_seq: u16,           // Next sequence number to assign
    tx_window_base: u16,        // Oldest unacknowledged sequence
    tx_pending: HashMap<u16, PendingFrame>,
    tx_queue: VecDeque<Vec<u8>>,

    // Receive state
    rx_expected_seq: u16,       // Next expected sequence number
    rx_buffer: HashMap<u16, Frame>,
    rx_delivered: Vec<Vec<u8>>, // Data ready for application
    rx_ack_pending: bool,       // True if we need to send an ACK for received data

    // Statistics
    stats: ArqStats,
}

/// ARQ statistics
#[derive(Debug, Clone, Default)]
pub struct ArqStats {
    pub frames_sent: u64,
    pub frames_received: u64,
    pub acks_sent: u64,
    pub acks_received: u64,
    pub nacks_sent: u64,
    pub nacks_received: u64,
    pub retransmissions: u64,
    pub timeouts: u64,
}

impl ArqController {
    /// Create new ARQ controller
    pub fn new(config: ArqConfig, session_id: u16) -> Self {
        Self {
            config,
            state: ArqState::Idle,
            session_id,
            tx_next_seq: 0,
            tx_window_base: 0,
            tx_pending: HashMap::new(),
            tx_queue: VecDeque::new(),
            rx_expected_seq: 0,
            rx_buffer: HashMap::new(),
            rx_delivered: Vec::new(),
            rx_ack_pending: false,
            stats: ArqStats::default(),
        }
    }

    /// Queue data for transmission
    ///
    /// Returns false if queue is full (exceeds max_queue_bytes).
    pub fn send(&mut self, data: Vec<u8>) -> bool {
        // Check queue size to prevent memory exhaustion
        let current_queue_bytes: usize = self.tx_queue.iter().map(|d| d.len()).sum();
        if current_queue_bytes + data.len() > self.config.max_queue_bytes {
            log::warn!("ARQ queue full ({} bytes), dropping {} bytes",
                       current_queue_bytes, data.len());
            return false;
        }
        self.tx_queue.push_back(data);
        true
    }

    /// Get next frame to transmit (if any)
    pub fn get_tx_frame(&mut self) -> Option<Frame> {
        // First, check for retransmissions needed
        let now = Instant::now();
        let timeout = Duration::from_millis(self.config.ack_timeout_ms as u64);

        for (&seq, pending) in &mut self.tx_pending {
            let elapsed = now.duration_since(pending.sent_time);
            if elapsed > timeout {
                if pending.retries >= self.config.max_retries {
                    // Too many retries - this will be handled separately
                    continue;
                }

                pending.retries += 1;
                pending.sent_time = now;
                self.stats.retransmissions += 1;
                self.stats.timeouts += 1;
                log::debug!("ARQ retransmit: seq={}, retry={}/{}", seq, pending.retries, self.config.max_retries);

                return Some(pending.frame.clone());
            }
        }

        // Check if window allows new transmission
        let window_used = self.tx_next_seq.wrapping_sub(self.tx_window_base) as usize;
        if window_used >= self.config.window_size {
            return None;
        }

        // Get new data from queue
        if let Some(data) = self.tx_queue.pop_front() {
            let seq = self.tx_next_seq;
            self.tx_next_seq = self.tx_next_seq.wrapping_add(1);

            let frame = Frame::data(seq, self.session_id, data);

            self.tx_pending.insert(seq, PendingFrame {
                frame: frame.clone(),
                sent_time: now,
                retries: 0,
            });

            self.state = ArqState::Transmitting;
            self.stats.frames_sent += 1;

            return Some(frame);
        }

        None
    }

    /// Process received frame
    pub fn receive(&mut self, frame: Frame) -> Result<(), ProtocolError> {
        // Validate session_id (defense-in-depth - caller should also validate)
        if frame.session_id() != self.session_id {
            log::debug!("ARQ ignoring frame with wrong session_id: {} != {}",
                       frame.session_id(), self.session_id);
            return Ok(());
        }

        match frame.frame_type() {
            FrameType::Data => self.handle_data(frame),
            FrameType::DataAck => self.handle_ack(frame),
            FrameType::Nack => self.handle_nack(frame),
            _ => Ok(()), // Ignore other frame types
        }
    }

    /// Handle received data frame
    fn handle_data(&mut self, frame: Frame) -> Result<(), ProtocolError> {
        let seq = frame.sequence();
        self.stats.frames_received += 1;
        log::debug!("ARQ handle_data: seq={}, expected={}, payload_len={}",
            seq, self.rx_expected_seq, frame.payload.len());

        if self.config.selective_repeat {
            // Selective repeat: buffer out-of-order frames
            if seq == self.rx_expected_seq {
                // In-order frame - deliver it
                self.rx_delivered.push(frame.payload.clone());
                self.rx_expected_seq = self.rx_expected_seq.wrapping_add(1);
                self.rx_ack_pending = true; // Need to ACK this data

                // Check for buffered frames that can now be delivered
                while let Some(buffered) = self.rx_buffer.remove(&self.rx_expected_seq) {
                    self.rx_delivered.push(buffered.payload);
                    self.rx_expected_seq = self.rx_expected_seq.wrapping_add(1);
                }
            } else if Self::seq_greater(seq, self.rx_expected_seq) {
                // Future frame - buffer it
                self.rx_buffer.insert(seq, frame);
                self.rx_ack_pending = true; // Need to ACK/NACK for out-of-order data
            }
            // else: old frame, ignore
        } else {
            // Go-back-N: only accept in-order frames
            if seq == self.rx_expected_seq {
                self.rx_delivered.push(frame.payload.clone());
                self.rx_expected_seq = self.rx_expected_seq.wrapping_add(1);
                self.rx_ack_pending = true; // Need to ACK this data
            }
        }

        self.state = ArqState::Receiving;
        Ok(())
    }

    /// Handle received ACK
    fn handle_ack(&mut self, frame: Frame) -> Result<(), ProtocolError> {
        let ack_seq = frame.sequence();
        self.stats.acks_received += 1;
        log::debug!("ARQ handle_ack: received ACK for seq={}", ack_seq);

        // Remove acknowledged frame
        self.tx_pending.remove(&ack_seq);

        // Advance window base
        while !self.tx_pending.contains_key(&self.tx_window_base)
            && self.tx_window_base != self.tx_next_seq
        {
            self.tx_window_base = self.tx_window_base.wrapping_add(1);
        }

        if self.tx_pending.is_empty() && self.tx_queue.is_empty() {
            self.state = ArqState::Idle;
        }

        Ok(())
    }

    /// Handle received NACK
    fn handle_nack(&mut self, frame: Frame) -> Result<(), ProtocolError> {
        self.stats.nacks_received += 1;

        // Parse missing sequence numbers from payload
        for chunk in frame.payload.chunks(2) {
            if chunk.len() == 2 {
                let missing_seq = ((chunk[0] as u16) << 8) | (chunk[1] as u16);

                // Mark for immediate retransmission
                if let Some(pending) = self.tx_pending.get_mut(&missing_seq) {
                    pending.sent_time = Instant::now() - Duration::from_secs(10);
                }
            }
        }

        Ok(())
    }

    /// Generate ACK frame for received data
    /// Only generates ACK if there's pending data to acknowledge (rx_ack_pending is true)
    pub fn generate_ack(&mut self) -> Option<Frame> {
        if !self.rx_ack_pending {
            return None;
        }

        // Clear the pending flag - we're generating the ACK now
        self.rx_ack_pending = false;
        self.stats.acks_sent += 1;
        let ack_seq = self.rx_expected_seq.wrapping_sub(1);
        log::debug!("ARQ generate_ack: creating ACK for seq={}", ack_seq);
        Some(Frame::ack(ack_seq, self.session_id))
    }

    /// Generate NACK frame for missing data
    pub fn generate_nack(&mut self) -> Option<Frame> {
        if self.rx_buffer.is_empty() {
            return None;
        }

        // Find gaps in received sequence numbers
        let mut missing = Vec::new();
        let mut seq = self.rx_expected_seq;

        while let Some(&max_seq) = self.rx_buffer.keys().max() {
            if !self.rx_buffer.contains_key(&seq) && Self::seq_less(seq, max_seq) {
                missing.push(seq);
            }
            seq = seq.wrapping_add(1);
            if seq == max_seq.wrapping_add(1) {
                break;
            }
        }

        if missing.is_empty() {
            return None;
        }

        self.stats.nacks_sent += 1;
        Some(Frame::nack(self.session_id, &missing))
    }

    /// Get data ready for application
    pub fn get_received_data(&mut self) -> Vec<Vec<u8>> {
        std::mem::take(&mut self.rx_delivered)
    }

    /// Check if any frame has exceeded max retries
    pub fn has_failed(&self) -> bool {
        self.tx_pending.values().any(|p| p.retries >= self.config.max_retries)
    }

    /// Get current state
    pub fn state(&self) -> ArqState {
        self.state
    }

    /// Get statistics
    pub fn stats(&self) -> &ArqStats {
        &self.stats
    }

    /// Check if transmit queue is empty
    pub fn tx_empty(&self) -> bool {
        self.tx_queue.is_empty() && self.tx_pending.is_empty()
    }

    /// Get number of pending frames
    pub fn pending_count(&self) -> usize {
        self.tx_pending.len()
    }

    /// Check if there's a pending ACK to send
    pub fn has_pending_ack(&self) -> bool {
        self.rx_ack_pending
    }

    /// Reset controller
    pub fn reset(&mut self) {
        self.state = ArqState::Idle;
        self.tx_next_seq = 0;
        self.tx_window_base = 0;
        self.tx_pending.clear();
        self.tx_queue.clear();
        self.rx_expected_seq = 0;
        self.rx_buffer.clear();
        self.rx_delivered.clear();
        self.rx_ack_pending = false;
    }

    /// Compare sequence numbers (handling wrap-around)
    fn seq_greater(a: u16, b: u16) -> bool {
        let diff = a.wrapping_sub(b);
        diff > 0 && diff < 32768
    }

    fn seq_less(a: u16, b: u16) -> bool {
        Self::seq_greater(b, a)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_arq_send_receive() {
        let config = ArqConfig::default();
        let mut tx = ArqController::new(config.clone(), 1);
        let mut rx = ArqController::new(config, 1);

        // Queue data
        tx.send(b"Hello".to_vec());
        tx.send(b"World".to_vec());

        // Get frames
        let frame1 = tx.get_tx_frame().unwrap();
        let frame2 = tx.get_tx_frame().unwrap();

        assert_eq!(frame1.sequence(), 0);
        assert_eq!(frame2.sequence(), 1);

        // Receive frames
        rx.receive(frame1).unwrap();
        rx.receive(frame2).unwrap();

        let data = rx.get_received_data();
        assert_eq!(data.len(), 2);
        assert_eq!(data[0], b"Hello");
        assert_eq!(data[1], b"World");
    }

    #[test]
    fn test_arq_out_of_order() {
        let config = ArqConfig::default();
        let mut rx = ArqController::new(config, 1);

        // Receive out of order
        let frame1 = Frame::data(1, 1, b"Second".to_vec());
        let frame0 = Frame::data(0, 1, b"First".to_vec());

        rx.receive(frame1).unwrap();
        assert!(rx.get_received_data().is_empty()); // Buffered

        rx.receive(frame0).unwrap();
        let data = rx.get_received_data();
        assert_eq!(data.len(), 2);
        assert_eq!(data[0], b"First");
        assert_eq!(data[1], b"Second");
    }

    #[test]
    fn test_arq_ack() {
        let config = ArqConfig::default();
        let mut tx = ArqController::new(config, 1);

        tx.send(b"Test".to_vec());
        let _ = tx.get_tx_frame().unwrap();

        assert_eq!(tx.pending_count(), 1);

        // Receive ACK
        let ack = Frame::ack(0, 1);
        tx.receive(ack).unwrap();

        assert_eq!(tx.pending_count(), 0);
    }
}
