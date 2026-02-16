//! Frame types and encoding
//!
//! Defines the RIA frame format for data transmission

use crate::fec::crc32;

/// Frame types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum FrameType {
    /// Connection request
    Connect = 0x01,
    /// Connection acknowledgment
    ConnectAck = 0x02,
    /// Disconnect request
    Disconnect = 0x03,
    /// Disconnect acknowledgment
    DisconnectAck = 0x04,
    /// Data frame
    Data = 0x10,
    /// Data acknowledgment
    DataAck = 0x11,
    /// Negative acknowledgment
    Nack = 0x12,
    /// Idle/keepalive
    Idle = 0x20,
    /// Break/abort
    Break = 0x21,
    /// Request retransmission
    Request = 0x22,
    /// Quit
    Quit = 0x23,
    /// Speed level change request
    SpeedChange = 0x30,
    /// Speed level change acknowledgment
    SpeedChangeAck = 0x31,
    /// Ping (for RTT measurement)
    Ping = 0x40,
    /// Pong (ping response)
    Pong = 0x41,
}

impl TryFrom<u8> for FrameType {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0x01 => Ok(FrameType::Connect),
            0x02 => Ok(FrameType::ConnectAck),
            0x03 => Ok(FrameType::Disconnect),
            0x04 => Ok(FrameType::DisconnectAck),
            0x10 => Ok(FrameType::Data),
            0x11 => Ok(FrameType::DataAck),
            0x12 => Ok(FrameType::Nack),
            0x20 => Ok(FrameType::Idle),
            0x21 => Ok(FrameType::Break),
            0x22 => Ok(FrameType::Request),
            0x23 => Ok(FrameType::Quit),
            0x30 => Ok(FrameType::SpeedChange),
            0x31 => Ok(FrameType::SpeedChangeAck),
            0x40 => Ok(FrameType::Ping),
            0x41 => Ok(FrameType::Pong),
            _ => Err(()),
        }
    }
}

/// Frame header
///
/// Rate negotiation uses:
/// - `speed_level`: Proposed mode based on SNR (what sender thinks mode should be)
/// - `flags`: Measured SNR as i8 (what sender measured on received frames from peer)
#[derive(Debug, Clone)]
pub struct FrameHeader {
    /// Protocol version
    pub version: u8,
    /// Frame type
    pub frame_type: FrameType,
    /// Sequence number (for data frames)
    pub sequence: u16,
    /// Session ID (reduced to 16-bit for compact frames)
    pub session_id: u16,
    /// Speed level / proposed mode (1-17) for rate negotiation
    pub speed_level: u8,
    /// Flags / measured SNR (as i8) for rate negotiation
    pub flags: u8,
}

impl FrameHeader {
    /// Header size in bytes (reduced from 10 to 8)
    pub const SIZE: usize = 8;

    /// Create new header
    pub fn new(frame_type: FrameType, sequence: u16, session_id: u16) -> Self {
        Self {
            version: super::PROTOCOL_VERSION,
            frame_type,
            sequence,
            session_id,
            speed_level: 9,
            flags: 0,
        }
    }

    /// Encode header to bytes
    pub fn encode(&self) -> [u8; Self::SIZE] {
        let mut bytes = [0u8; Self::SIZE];
        bytes[0] = self.version;
        bytes[1] = self.frame_type as u8;
        bytes[2] = (self.sequence >> 8) as u8;
        bytes[3] = (self.sequence & 0xFF) as u8;
        bytes[4] = (self.session_id >> 8) as u8;
        bytes[5] = (self.session_id & 0xFF) as u8;
        bytes[6] = self.speed_level;
        bytes[7] = self.flags;
        bytes
    }

    /// Decode header from bytes
    pub fn decode(bytes: &[u8]) -> Option<Self> {
        if bytes.len() < Self::SIZE {
            return None;
        }

        let frame_type = FrameType::try_from(bytes[1]).ok()?;
        let sequence = ((bytes[2] as u16) << 8) | (bytes[3] as u16);
        let session_id = ((bytes[4] as u16) << 8) | (bytes[5] as u16);

        Some(Self {
            version: bytes[0],
            frame_type,
            sequence,
            session_id,
            speed_level: bytes[6],
            flags: bytes[7],
        })
    }

    /// Get measured SNR from flags field (for rate negotiation)
    pub fn measured_snr(&self) -> i8 {
        self.flags as i8
    }

    /// Set measured SNR in flags field (for rate negotiation)
    pub fn set_measured_snr(&mut self, snr: i8) {
        self.flags = snr as u8;
    }

    /// Get proposed mode (alias for speed_level, for rate negotiation)
    pub fn proposed_mode(&self) -> u8 {
        self.speed_level
    }

    /// Set proposed mode (alias for speed_level, for rate negotiation)
    pub fn set_proposed_mode(&mut self, mode: u8) {
        self.speed_level = mode;
    }
}

/// Maximum payload length (limited by 2-byte length field)
pub const MAX_PAYLOAD_LEN: usize = 65535;

/// Complete frame with header, payload, and CRC
#[derive(Debug, Clone)]
pub struct Frame {
    /// Frame header
    pub header: FrameHeader,
    /// Payload data
    pub payload: Vec<u8>,
}

impl Frame {
    /// Create new frame
    pub fn new(header: FrameHeader, payload: Vec<u8>) -> Self {
        Self { header, payload }
    }

    /// Create data frame
    pub fn data(sequence: u16, session_id: u16, data: Vec<u8>) -> Self {
        let header = FrameHeader::new(FrameType::Data, sequence, session_id);
        Self::new(header, data)
    }

    /// Create ACK frame
    pub fn ack(sequence: u16, session_id: u16) -> Self {
        let header = FrameHeader::new(FrameType::DataAck, sequence, session_id);
        Self::new(header, Vec::new())
    }

    /// Create NACK frame with missing sequence numbers
    pub fn nack(session_id: u16, missing: &[u16]) -> Self {
        let header = FrameHeader::new(FrameType::Nack, 0, session_id);
        let mut payload = Vec::with_capacity(missing.len() * 2);
        for &seq in missing {
            payload.push((seq >> 8) as u8);
            payload.push((seq & 0xFF) as u8);
        }
        Self::new(header, payload)
    }

    /// Create connect frame
    pub fn connect(source_call: &str, dest_call: &str, session_id: u16) -> Self {
        let header = FrameHeader::new(FrameType::Connect, 0, session_id);

        let mut payload = Vec::new();
        // Source callsign (padded to 8 bytes - supports up to 8 character callsigns)
        let src = source_call.as_bytes();
        payload.extend_from_slice(&src[..src.len().min(8)]);
        payload.resize(8, 0);
        // Destination callsign (padded to 8 bytes)
        let dst = dest_call.as_bytes();
        payload.extend_from_slice(&dst[..dst.len().min(8)]);
        payload.resize(16, 0);

        Self::new(header, payload)
    }

    /// Create connect ACK frame
    pub fn connect_ack(session_id: u16, accepted_speed: u8) -> Self {
        let mut header = FrameHeader::new(FrameType::ConnectAck, 0, session_id);
        header.speed_level = accepted_speed;
        Self::new(header, Vec::new())
    }

    /// Create disconnect frame
    pub fn disconnect(session_id: u16) -> Self {
        let header = FrameHeader::new(FrameType::Disconnect, 0, session_id);
        Self::new(header, Vec::new())
    }

    /// Create idle frame
    pub fn idle(session_id: u16) -> Self {
        let header = FrameHeader::new(FrameType::Idle, 0, session_id);
        Self::new(header, Vec::new())
    }

    /// Create ping frame for RTT measurement
    pub fn ping(session_id: u16) -> Self {
        let header = FrameHeader::new(FrameType::Ping, 0, session_id);
        Self::new(header, Vec::new())
    }

    /// Create pong frame (ping response)
    pub fn pong(session_id: u16) -> Self {
        let header = FrameHeader::new(FrameType::Pong, 0, session_id);
        Self::new(header, Vec::new())
    }

    /// Create break frame (abort connection)
    pub fn break_frame(session_id: u16) -> Self {
        let header = FrameHeader::new(FrameType::Break, 0, session_id);
        Self::new(header, Vec::new())
    }

    /// Create quit frame (graceful termination)
    pub fn quit(session_id: u16) -> Self {
        let header = FrameHeader::new(FrameType::Quit, 0, session_id);
        Self::new(header, Vec::new())
    }

    /// Create speed change request frame
    pub fn speed_change(session_id: u16, speed_level: u8) -> Self {
        let mut header = FrameHeader::new(FrameType::SpeedChange, 0, session_id);
        header.speed_level = speed_level;
        Self::new(header, Vec::new())
    }

    /// Create speed change acknowledgment frame
    pub fn speed_change_ack(session_id: u16, speed_level: u8) -> Self {
        let mut header = FrameHeader::new(FrameType::SpeedChangeAck, 0, session_id);
        header.speed_level = speed_level;
        Self::new(header, Vec::new())
    }

    /// Encode frame to bytes (with CRC)
    pub fn encode(&self) -> Vec<u8> {
        let mut bytes = Vec::with_capacity(FrameHeader::SIZE + self.payload.len() + 4);

        // Header
        bytes.extend_from_slice(&self.header.encode());

        // Payload length
        bytes.push((self.payload.len() >> 8) as u8);
        bytes.push((self.payload.len() & 0xFF) as u8);

        // Payload
        bytes.extend_from_slice(&self.payload);

        // CRC-32
        let crc = crc32(&bytes);
        bytes.push((crc >> 24) as u8);
        bytes.push((crc >> 16) as u8);
        bytes.push((crc >> 8) as u8);
        bytes.push((crc & 0xFF) as u8);

        bytes
    }

    /// Decode frame from bytes (verify CRC)
    /// Handles padded buffers by reading payload length to determine actual frame size
    pub fn decode(bytes: &[u8]) -> Result<Self, super::ProtocolError> {
        // Minimum frame: header (10) + payload_len (2) + crc (4) = 16 bytes
        if bytes.len() < FrameHeader::SIZE + 6 {
            return Err(super::ProtocolError::FrameTooShort);
        }

        // Extract payload length first to determine actual frame size
        let payload_len = ((bytes[FrameHeader::SIZE] as usize) << 8)
            | (bytes[FrameHeader::SIZE + 1] as usize);

        // Calculate actual frame size (header + payload_len field + payload + CRC)
        let actual_frame_size = FrameHeader::SIZE + 2 + payload_len + 4;

        if bytes.len() < actual_frame_size {
            return Err(super::ProtocolError::FrameTooShort);
        }

        // CRC is at the end of the actual frame, not the end of the buffer
        let crc_idx = FrameHeader::SIZE + 2 + payload_len;
        let received_crc = ((bytes[crc_idx] as u32) << 24)
            | ((bytes[crc_idx + 1] as u32) << 16)
            | ((bytes[crc_idx + 2] as u32) << 8)
            | (bytes[crc_idx + 3] as u32);

        let calculated_crc = crc32(&bytes[..crc_idx]);

        if received_crc != calculated_crc {
            return Err(super::ProtocolError::CrcError);
        }

        // Decode header
        let header = FrameHeader::decode(bytes)
            .ok_or(super::ProtocolError::InvalidFrameType)?;

        // Check version
        if header.version != super::PROTOCOL_VERSION {
            return Err(super::ProtocolError::VersionMismatch);
        }

        // Extract payload
        let payload_start = FrameHeader::SIZE + 2;
        let payload_end = payload_start + payload_len;
        let payload = bytes[payload_start..payload_end].to_vec();

        Ok(Self { header, payload })
    }

    /// Get frame type
    pub fn frame_type(&self) -> FrameType {
        self.header.frame_type
    }

    /// Get sequence number
    pub fn sequence(&self) -> u16 {
        self.header.sequence
    }

    /// Get session ID
    pub fn session_id(&self) -> u16 {
        self.header.session_id
    }

    /// Check if this is a data frame
    pub fn is_data(&self) -> bool {
        self.header.frame_type == FrameType::Data
    }

    /// Check if this is an acknowledgment
    pub fn is_ack(&self) -> bool {
        matches!(self.header.frame_type, FrameType::DataAck | FrameType::ConnectAck)
    }
}

/// Frame builder for convenience
pub struct FrameBuilder {
    frame_type: FrameType,
    sequence: u16,
    session_id: u16,
    speed_level: u8,
    flags: u8,
    payload: Vec<u8>,
}

impl FrameBuilder {
    /// Create new builder
    pub fn new(frame_type: FrameType) -> Self {
        Self {
            frame_type,
            sequence: 0,
            session_id: 0,
            speed_level: 9,
            flags: 0,
            payload: Vec::new(),
        }
    }

    /// Set sequence number
    pub fn sequence(mut self, seq: u16) -> Self {
        self.sequence = seq;
        self
    }

    /// Set session ID
    pub fn session_id(mut self, id: u16) -> Self {
        self.session_id = id;
        self
    }

    /// Set speed level
    pub fn speed_level(mut self, level: u8) -> Self {
        self.speed_level = level;
        self
    }

    /// Set flags
    pub fn flags(mut self, flags: u8) -> Self {
        self.flags = flags;
        self
    }

    /// Set payload
    pub fn payload(mut self, data: Vec<u8>) -> Self {
        self.payload = data;
        self
    }

    /// Build the frame
    pub fn build(self) -> Frame {
        let mut header = FrameHeader::new(self.frame_type, self.sequence, self.session_id);
        header.speed_level = self.speed_level;
        header.flags = self.flags;
        Frame::new(header, self.payload)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_frame_encode_decode() {
        let frame = Frame::data(42, 0x1234, b"Hello, World!".to_vec());
        let encoded = frame.encode();
        let decoded = Frame::decode(&encoded).unwrap();

        assert_eq!(decoded.sequence(), 42);
        assert_eq!(decoded.session_id(), 0x1234);
        assert_eq!(decoded.payload, b"Hello, World!");
    }

    #[test]
    fn test_frame_crc_error() {
        let frame = Frame::data(1, 1, b"Test".to_vec());
        let mut encoded = frame.encode();

        // Corrupt data
        encoded[12] ^= 0xFF;

        let result = Frame::decode(&encoded);
        assert!(matches!(result, Err(super::super::ProtocolError::CrcError)));
    }

    #[test]
    fn test_connect_frame() {
        let frame = Frame::connect("W1AW", "K1ABC", 12345);
        let encoded = frame.encode();
        let decoded = Frame::decode(&encoded).unwrap();

        assert_eq!(decoded.frame_type(), FrameType::Connect);
        assert_eq!(decoded.payload.len(), 16); // 2 x 8-byte callsigns
    }

    #[test]
    fn test_frame_builder() {
        let frame = FrameBuilder::new(FrameType::Data)
            .sequence(100)
            .session_id(999)
            .speed_level(12)
            .payload(b"Test data".to_vec())
            .build();

        assert_eq!(frame.sequence(), 100);
        assert_eq!(frame.session_id(), 999);
        assert_eq!(frame.header.speed_level, 12);
    }
}
