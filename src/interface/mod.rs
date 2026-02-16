//! Interface module - External connectivity
//!
//! Provides TCP server for host application communication
//! and KISS TNC protocol support

mod tcp;
mod kiss;
mod commands;

#[allow(unused_imports)]
pub use tcp::{TcpServer, TcpClient, TcpConfig};
#[allow(unused_imports)]
pub use kiss::{KissFrame, KissFramer, KissPort};
pub use commands::{Command, Response, CommandParser};

/// Default TCP port for control connection
pub const DEFAULT_CMD_PORT: u16 = 8300;

/// Default TCP port for data connection
pub const DEFAULT_DATA_PORT: u16 = 8301;

/// Interface error types
#[derive(Debug, Clone)]
pub enum InterfaceError {
    /// Connection error
    ConnectionError(String),
    /// Parse error
    ParseError(String),
    /// Invalid command
    InvalidCommand(String),
    /// IO error
    IoError(String),
}

impl std::fmt::Display for InterfaceError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            InterfaceError::ConnectionError(msg) => write!(f, "Connection error: {}", msg),
            InterfaceError::ParseError(msg) => write!(f, "Parse error: {}", msg),
            InterfaceError::InvalidCommand(msg) => write!(f, "Invalid command: {}", msg),
            InterfaceError::IoError(msg) => write!(f, "IO error: {}", msg),
        }
    }
}

impl std::error::Error for InterfaceError {}

impl From<std::io::Error> for InterfaceError {
    fn from(err: std::io::Error) -> Self {
        InterfaceError::IoError(err.to_string())
    }
}
