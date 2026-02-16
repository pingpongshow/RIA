//! KISS TNC protocol support
//!
//! Implements KISS framing for TNC compatibility

/// KISS frame types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum KissCommand {
    /// Data frame
    DataFrame = 0x00,
    /// TX delay (in 10ms units)
    TxDelay = 0x01,
    /// Persistence parameter
    Persistence = 0x02,
    /// Slot time (in 10ms units)
    SlotTime = 0x03,
    /// TX tail (in 10ms units)
    TxTail = 0x04,
    /// Full duplex mode
    FullDuplex = 0x05,
    /// Set hardware
    SetHardware = 0x06,
    /// Exit KISS mode
    Return = 0xFF,
}

impl TryFrom<u8> for KissCommand {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value & 0x0F {
            0x00 => Ok(KissCommand::DataFrame),
            0x01 => Ok(KissCommand::TxDelay),
            0x02 => Ok(KissCommand::Persistence),
            0x03 => Ok(KissCommand::SlotTime),
            0x04 => Ok(KissCommand::TxTail),
            0x05 => Ok(KissCommand::FullDuplex),
            0x06 => Ok(KissCommand::SetHardware),
            _ if value == 0xFF => Ok(KissCommand::Return),
            _ => Err(()),
        }
    }
}

/// KISS special bytes
const FEND: u8 = 0xC0;  // Frame end
const FESC: u8 = 0xDB;  // Frame escape
const TFEND: u8 = 0xDC; // Transposed frame end
const TFESC: u8 = 0xDD; // Transposed frame escape

/// KISS frame
#[derive(Debug, Clone)]
pub struct KissFrame {
    /// Port number (0-15)
    pub port: u8,
    /// Command type
    pub command: KissCommand,
    /// Frame data
    pub data: Vec<u8>,
}

impl KissFrame {
    /// Create new data frame
    pub fn new_data(port: u8, data: Vec<u8>) -> Self {
        Self {
            port: port & 0x0F,
            command: KissCommand::DataFrame,
            data,
        }
    }

    /// Create command frame
    pub fn new_command(port: u8, command: KissCommand, value: u8) -> Self {
        Self {
            port: port & 0x0F,
            command,
            data: vec![value],
        }
    }

    /// Encode frame to bytes with KISS framing
    pub fn encode(&self) -> Vec<u8> {
        let mut output = Vec::new();

        // Start with FEND
        output.push(FEND);

        // Command byte (port << 4 | command)
        let cmd_byte = (self.port << 4) | (self.command as u8 & 0x0F);
        Self::encode_byte(&mut output, cmd_byte);

        // Data with escape sequences
        for &byte in &self.data {
            Self::encode_byte(&mut output, byte);
        }

        // End with FEND
        output.push(FEND);

        output
    }

    /// Encode a single byte with escaping
    fn encode_byte(output: &mut Vec<u8>, byte: u8) {
        match byte {
            FEND => {
                output.push(FESC);
                output.push(TFEND);
            }
            FESC => {
                output.push(FESC);
                output.push(TFESC);
            }
            _ => output.push(byte),
        }
    }

    /// Decode frame from bytes
    pub fn decode(data: &[u8]) -> Option<Self> {
        if data.len() < 2 {
            return None;
        }

        // First byte is command
        let cmd_byte = data[0];
        let port = (cmd_byte >> 4) & 0x0F;
        let command = KissCommand::try_from(cmd_byte).ok()?;

        // Unescape remaining data
        let mut frame_data = Vec::new();
        let mut escape = false;

        for &byte in &data[1..] {
            if escape {
                match byte {
                    TFEND => frame_data.push(FEND),
                    TFESC => frame_data.push(FESC),
                    _ => frame_data.push(byte), // Invalid escape, pass through
                }
                escape = false;
            } else if byte == FESC {
                escape = true;
            } else {
                frame_data.push(byte);
            }
        }

        Some(Self {
            port,
            command,
            data: frame_data,
        })
    }
}

/// KISS frame parser (handles stream of bytes)
pub struct KissFramer {
    buffer: Vec<u8>,
    in_frame: bool,
}

impl KissFramer {
    /// Create new framer
    pub fn new() -> Self {
        Self {
            buffer: Vec::new(),
            in_frame: false,
        }
    }

    /// Process incoming bytes and extract complete frames
    pub fn process(&mut self, data: &[u8]) -> Vec<KissFrame> {
        let mut frames = Vec::new();

        for &byte in data {
            match byte {
                FEND => {
                    if self.in_frame && !self.buffer.is_empty() {
                        // End of frame
                        if let Some(frame) = KissFrame::decode(&self.buffer) {
                            frames.push(frame);
                        }
                    }
                    // Start new frame
                    self.buffer.clear();
                    self.in_frame = true;
                }
                _ if self.in_frame => {
                    self.buffer.push(byte);
                }
                _ => {
                    // Byte outside frame, ignore
                }
            }
        }

        frames
    }

    /// Reset framer state
    pub fn reset(&mut self) {
        self.buffer.clear();
        self.in_frame = false;
    }
}

impl Default for KissFramer {
    fn default() -> Self {
        Self::new()
    }
}

/// KISS port configuration
#[derive(Debug, Clone)]
pub struct KissPort {
    /// Port number
    pub port: u8,
    /// TX delay (in 10ms units)
    pub tx_delay: u8,
    /// Persistence (0-255)
    pub persistence: u8,
    /// Slot time (in 10ms units)
    pub slot_time: u8,
    /// TX tail (in 10ms units)
    pub tx_tail: u8,
    /// Full duplex mode
    pub full_duplex: bool,
    /// Hardware-specific configuration data
    pub hardware_config: Vec<u8>,
    /// Flag indicating KISS mode should exit (Return command received)
    pub exit_kiss_mode: bool,
}

impl Default for KissPort {
    fn default() -> Self {
        Self {
            port: 0,
            tx_delay: 50,  // 500ms
            persistence: 63, // 25%
            slot_time: 10,  // 100ms
            tx_tail: 1,    // 10ms
            full_duplex: false,
            hardware_config: Vec::new(),
            exit_kiss_mode: false,
        }
    }
}

impl KissPort {
    /// Create configuration frames for this port
    pub fn config_frames(&self) -> Vec<KissFrame> {
        vec![
            KissFrame::new_command(self.port, KissCommand::TxDelay, self.tx_delay),
            KissFrame::new_command(self.port, KissCommand::Persistence, self.persistence),
            KissFrame::new_command(self.port, KissCommand::SlotTime, self.slot_time),
            KissFrame::new_command(self.port, KissCommand::TxTail, self.tx_tail),
            KissFrame::new_command(self.port, KissCommand::FullDuplex, self.full_duplex as u8),
        ]
    }

    /// Handle configuration command
    pub fn handle_command(&mut self, command: KissCommand, value: u8) {
        match command {
            KissCommand::TxDelay => self.tx_delay = value,
            KissCommand::Persistence => self.persistence = value,
            KissCommand::SlotTime => self.slot_time = value,
            KissCommand::TxTail => self.tx_tail = value,
            KissCommand::FullDuplex => self.full_duplex = value != 0,
            KissCommand::SetHardware => {
                // Hardware-specific configuration - store the value
                // In a real implementation, this would configure hardware-specific features
                self.hardware_config.push(value);
            }
            KissCommand::Return => {
                // Exit KISS mode - set flag to indicate the interface should
                // return to command mode (for hardware TNCs) or close
                self.exit_kiss_mode = true;
            }
            KissCommand::DataFrame => {
                // Data frames are handled separately, not through this method
            }
        }
    }

    /// Handle SetHardware command with data payload
    pub fn handle_set_hardware(&mut self, data: &[u8]) {
        // Store hardware configuration data
        // This is TNC-specific and interpretation depends on the hardware
        self.hardware_config.clear();
        self.hardware_config.extend_from_slice(data);
    }

    /// Check if KISS mode should exit
    pub fn should_exit(&self) -> bool {
        self.exit_kiss_mode
    }

    /// Reset exit flag
    pub fn reset_exit(&mut self) {
        self.exit_kiss_mode = false;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_kiss_encode_decode() {
        let frame = KissFrame::new_data(0, b"Hello, World!".to_vec());
        let encoded = frame.encode();

        // Should start and end with FEND
        assert_eq!(encoded[0], FEND);
        assert_eq!(encoded[encoded.len() - 1], FEND);

        // Decode via framer
        let mut framer = KissFramer::new();
        let frames = framer.process(&encoded);

        assert_eq!(frames.len(), 1);
        assert_eq!(frames[0].data, b"Hello, World!");
    }

    #[test]
    fn test_kiss_escape() {
        let data = vec![FEND, FESC, 0x00, 0xFF];
        let frame = KissFrame::new_data(0, data.clone());
        let encoded = frame.encode();

        let mut framer = KissFramer::new();
        let frames = framer.process(&encoded);

        assert_eq!(frames.len(), 1);
        assert_eq!(frames[0].data, data);
    }

    #[test]
    fn test_kiss_multiple_frames() {
        let mut framer = KissFramer::new();

        let frame1 = KissFrame::new_data(0, b"First".to_vec());
        let frame2 = KissFrame::new_data(0, b"Second".to_vec());

        let mut data = frame1.encode();
        data.extend(frame2.encode());

        let frames = framer.process(&data);

        assert_eq!(frames.len(), 2);
        assert_eq!(frames[0].data, b"First");
        assert_eq!(frames[1].data, b"Second");
    }

    #[test]
    fn test_kiss_port() {
        let mut port = KissPort::default();
        port.handle_command(KissCommand::TxDelay, 100);

        assert_eq!(port.tx_delay, 100);
    }
}
