//! Command parsing and response generation
//!
//! Implements command interface for TCP control

use super::InterfaceError;

/// Commands supported by the modem
#[derive(Debug, Clone, PartialEq)]
pub enum Command {
    /// Connect to remote station
    Connect { callsign: String },
    /// Disconnect current session
    Disconnect,
    /// Abort connection attempt
    Abort,
    /// Set bandwidth mode
    Bandwidth { mode: String },
    /// Set compression mode
    Compression { enabled: bool },
    /// Enable/disable listen mode
    Listen { enabled: bool },
    /// Set callsign
    MyCall { callsign: String },
    /// Set auxiliary callsigns
    MyAux { callsigns: Vec<String> },
    /// Enable/disable chat mode
    ChatMode { enabled: bool },
    /// Get version
    Version,
    /// Get codec info
    Codec,
    /// Get PTT state
    PttState,
    /// Set BW mode (500/2300/2750)
    BwMode { hz: u32 },
    /// Set Winlink mode
    WinlinkSession { enabled: bool },
    /// Tune output
    Tune { enabled: bool },
    /// Set auto tune
    AutoTune { enabled: bool },
    /// Get/set CW ID
    CwId { callsign: Option<String> },
    /// Get busy state
    Busy,
    /// Get buffer status
    Buffer,
    /// Close command connection
    Close,
    /// Unknown command
    Unknown(String),
}

impl Command {
    /// Parse command from string
    pub fn parse(input: &str) -> Result<Self, InterfaceError> {
        let input = input.trim();
        let parts: Vec<&str> = input.splitn(2, ' ').collect();
        let cmd = parts[0].to_uppercase();
        let args = parts.get(1).map(|s| s.trim()).unwrap_or("");

        match cmd.as_str() {
            "CONNECT" => {
                if args.is_empty() {
                    return Err(InterfaceError::InvalidCommand("CONNECT requires callsign".to_string()));
                }
                Ok(Command::Connect { callsign: args.to_string() })
            }
            "DISCONNECT" => Ok(Command::Disconnect),
            "ABORT" => Ok(Command::Abort),
            "BW" | "BANDWIDTH" => {
                Ok(Command::Bandwidth { mode: args.to_string() })
            }
            "COMPRESSION" => {
                let enabled = args.eq_ignore_ascii_case("ON") || args == "1";
                Ok(Command::Compression { enabled })
            }
            "LISTEN" => {
                let enabled = args.eq_ignore_ascii_case("ON") || args == "1";
                Ok(Command::Listen { enabled })
            }
            "MYCALL" => {
                if args.is_empty() {
                    return Err(InterfaceError::InvalidCommand("MYCALL requires callsign".to_string()));
                }
                Ok(Command::MyCall { callsign: args.to_string() })
            }
            "MYAUX" => {
                let callsigns: Vec<String> = args.split(',')
                    .map(|s| s.trim().to_string())
                    .filter(|s| !s.is_empty())
                    .collect();
                Ok(Command::MyAux { callsigns })
            }
            "CHATMODE" => {
                let enabled = args.eq_ignore_ascii_case("ON") || args == "1";
                Ok(Command::ChatMode { enabled })
            }
            "VERSION" => Ok(Command::Version),
            "CODEC" => Ok(Command::Codec),
            "PTTSTATE" | "PTT" => Ok(Command::PttState),
            "BWMODE" => {
                let hz = args.parse().unwrap_or(2300);
                Ok(Command::BwMode { hz })
            }
            "WINLINK" | "WINLINKSESSION" => {
                let enabled = args.eq_ignore_ascii_case("ON") || args == "1";
                Ok(Command::WinlinkSession { enabled })
            }
            "TUNE" => {
                let enabled = args.eq_ignore_ascii_case("ON") || args == "1";
                Ok(Command::Tune { enabled })
            }
            "AUTOTUNE" => {
                let enabled = args.eq_ignore_ascii_case("ON") || args == "1";
                Ok(Command::AutoTune { enabled })
            }
            "CWID" => {
                let callsign = if args.is_empty() { None } else { Some(args.to_string()) };
                Ok(Command::CwId { callsign })
            }
            "BUSY" | "BUSYSTATE" => Ok(Command::Busy),
            "BUFFER" => Ok(Command::Buffer),
            "CLOSE" => Ok(Command::Close),
            _ => Ok(Command::Unknown(input.to_string())),
        }
    }
}

/// Response messages from the modem
#[derive(Debug, Clone)]
pub enum Response {
    /// Command accepted
    Ok,
    /// Error response
    Error(String),
    /// Connected to station
    Connected { callsign: String },
    /// Disconnected
    Disconnected,
    /// Connection pending
    Pending,
    /// Link broken (timeout/failure)
    LinkBroken,
    /// Busy channel detected
    Busy { detected: bool },
    /// Version info
    Version { version: String },
    /// Buffer status
    Buffer { bytes: usize },
    /// PTT state
    Ptt { active: bool },
    /// SNR report
    Snr { db: f32 },
    /// State change
    State { state: String },
    /// Data received indicator
    DataReceived { bytes: usize },
    /// Custom response
    Custom(String),
    /// Close connection (sends OK then closes)
    Close,
}

impl Response {
    /// Format response as string
    pub fn to_string(&self) -> String {
        match self {
            Response::Ok => "OK\r".to_string(),
            Response::Error(msg) => format!("ERROR {}\r", msg),
            Response::Connected { callsign } => format!("CONNECTED {}\r", callsign),
            Response::Disconnected => "DISCONNECTED\r".to_string(),
            Response::Pending => "PENDING\r".to_string(),
            Response::LinkBroken => "LINK BROKEN\r".to_string(),
            Response::Busy { detected } => format!("BUSY {}\r", if *detected { "ON" } else { "OFF" }),
            Response::Version { version } => format!("VERSION {}\r", version),
            Response::Buffer { bytes } => format!("BUFFER {}\r", bytes),
            Response::Ptt { active } => format!("PTT {}\r", if *active { "ON" } else { "OFF" }),
            Response::Snr { db } => format!("SNR {:.1}\r", db),
            Response::State { state } => format!("{}\r", state),
            Response::DataReceived { bytes } => format!("DATA {}\r", bytes),
            Response::Custom(msg) => format!("{}\r", msg),
            Response::Close => "OK\r".to_string(),
        }
    }

    /// Format as bytes
    pub fn to_bytes(&self) -> Vec<u8> {
        self.to_string().into_bytes()
    }
}

/// Command parser with buffering
pub struct CommandParser {
    buffer: String,
}

impl CommandParser {
    /// Create new parser
    pub fn new() -> Self {
        Self {
            buffer: String::new(),
        }
    }

    /// Add data to buffer and extract complete commands
    pub fn parse(&mut self, data: &[u8]) -> Vec<Command> {
        // Add new data to buffer
        if let Ok(s) = std::str::from_utf8(data) {
            self.buffer.push_str(s);
        }

        let mut commands = Vec::new();

        // Extract complete commands (terminated by \r or \n)
        while let Some(pos) = self.buffer.find(|c| c == '\r' || c == '\n') {
            let line = self.buffer[..pos].to_string();
            self.buffer = self.buffer[pos + 1..].to_string();

            // Skip empty lines
            if line.trim().is_empty() {
                continue;
            }

            if let Ok(cmd) = Command::parse(&line) {
                commands.push(cmd);
            }
        }

        commands
    }

    /// Clear parser buffer
    pub fn clear(&mut self) {
        self.buffer.clear();
    }
}

impl Default for CommandParser {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_connect() {
        let cmd = Command::parse("CONNECT W1AW").unwrap();
        assert!(matches!(cmd, Command::Connect { callsign } if callsign == "W1AW"));
    }

    #[test]
    fn test_parse_mycall() {
        let cmd = Command::parse("MYCALL K1ABC").unwrap();
        assert!(matches!(cmd, Command::MyCall { callsign } if callsign == "K1ABC"));
    }

    #[test]
    fn test_parse_listen() {
        let cmd = Command::parse("LISTEN ON").unwrap();
        assert!(matches!(cmd, Command::Listen { enabled: true }));

        let cmd = Command::parse("LISTEN OFF").unwrap();
        assert!(matches!(cmd, Command::Listen { enabled: false }));
    }

    #[test]
    fn test_response_format() {
        let resp = Response::Connected { callsign: "W1AW".to_string() };
        assert_eq!(resp.to_string(), "CONNECTED W1AW\r");

        let resp = Response::Snr { db: 15.5 };
        assert_eq!(resp.to_string(), "SNR 15.5\r");
    }

    #[test]
    fn test_command_parser() {
        let mut parser = CommandParser::new();

        let cmds = parser.parse(b"MYCALL W1AW\rLISTEN ON\r");
        assert_eq!(cmds.len(), 2);

        // Partial command
        let cmds = parser.parse(b"CONN");
        assert!(cmds.is_empty());

        let cmds = parser.parse(b"ECT K1ABC\r");
        assert_eq!(cmds.len(), 1);
    }
}
