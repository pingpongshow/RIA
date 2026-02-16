//! TCP server for host application communication
//!
//! Implements TCP interface for command and data ports

use std::io::{Read, Write};
use std::net::{TcpListener, TcpStream, SocketAddr};
use std::time::Duration;

use super::{Command, Response, CommandParser, InterfaceError};
use super::{DEFAULT_CMD_PORT, DEFAULT_DATA_PORT};

/// TCP server configuration
#[derive(Debug, Clone)]
pub struct TcpConfig {
    /// Command port
    pub cmd_port: u16,
    /// Data port
    pub data_port: u16,
    /// Bind address
    pub bind_addr: String,
}

impl Default for TcpConfig {
    fn default() -> Self {
        Self {
            cmd_port: DEFAULT_CMD_PORT,
            data_port: DEFAULT_DATA_PORT,
            bind_addr: "127.0.0.1".to_string(),
        }
    }
}

/// TCP client connection state
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ClientState {
    Connected,
    Disconnected,
}

/// TCP client handler
pub struct TcpClient {
    stream: TcpStream,
    parser: CommandParser,
    state: ClientState,
}

impl TcpClient {
    /// Create from accepted connection
    pub fn new(stream: TcpStream) -> Result<Self, InterfaceError> {
        stream.set_nonblocking(true)?;
        stream.set_read_timeout(Some(Duration::from_millis(100)))?;
        stream.set_write_timeout(Some(Duration::from_millis(1000)))?; // 1s write timeout

        Ok(Self {
            stream,
            parser: CommandParser::new(),
            state: ClientState::Connected,
        })
    }

    /// Read and parse commands (non-blocking)
    pub fn read_commands(&mut self) -> Vec<Command> {
        let mut buf = [0u8; 4096];

        match self.stream.read(&mut buf) {
            Ok(0) => {
                self.state = ClientState::Disconnected;
                Vec::new()
            }
            Ok(n) => self.parser.parse(&buf[..n]),
            Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => Vec::new(),
            Err(_) => {
                self.state = ClientState::Disconnected;
                Vec::new()
            }
        }
    }

    /// Send response
    pub fn send(&mut self, response: &Response) -> Result<(), InterfaceError> {
        self.stream.write_all(&response.to_bytes())?;
        // Close connection after sending if Response::Close
        if matches!(response, Response::Close) {
            self.state = ClientState::Disconnected;
        }
        Ok(())
    }

    /// Send raw data
    pub fn send_raw(&mut self, data: &[u8]) -> Result<(), InterfaceError> {
        self.stream.write_all(data)?;
        Ok(())
    }

    /// Check if connected
    pub fn is_connected(&self) -> bool {
        self.state == ClientState::Connected
    }

    /// Get peer address
    pub fn peer_addr(&self) -> Option<SocketAddr> {
        self.stream.peer_addr().ok()
    }
}

/// TCP server for command and data connections
pub struct TcpServer {
    config: TcpConfig,
    cmd_listener: Option<TcpListener>,
    data_listener: Option<TcpListener>,
    cmd_clients: Vec<TcpClient>,
    data_client: Option<TcpClient>,
    running: bool,
}

impl TcpServer {
    /// Create new TCP server
    pub fn new(config: TcpConfig) -> Self {
        Self {
            config,
            cmd_listener: None,
            data_listener: None,
            cmd_clients: Vec::new(),
            data_client: None,
            running: false,
        }
    }

    /// Start listening
    pub fn start(&mut self) -> Result<(), InterfaceError> {
        let cmd_addr = format!("{}:{}", self.config.bind_addr, self.config.cmd_port);
        let data_addr = format!("{}:{}", self.config.bind_addr, self.config.data_port);

        let cmd_listener = TcpListener::bind(&cmd_addr)?;
        cmd_listener.set_nonblocking(true)?;

        let data_listener = TcpListener::bind(&data_addr)?;
        data_listener.set_nonblocking(true)?;

        self.cmd_listener = Some(cmd_listener);
        self.data_listener = Some(data_listener);
        self.running = true;

        Ok(())
    }

    /// Stop listening
    pub fn stop(&mut self) {
        self.running = false;
        self.cmd_listener = None;
        self.data_listener = None;
        self.cmd_clients.clear();
        self.data_client = None;
    }

    /// Maximum number of command clients allowed
    const MAX_CMD_CLIENTS: usize = 10;

    /// Accept new connections (non-blocking)
    pub fn accept(&mut self) -> Result<(), InterfaceError> {
        // Accept command connections (with limit to prevent resource exhaustion)
        if self.cmd_clients.len() < Self::MAX_CMD_CLIENTS {
            if let Some(listener) = &self.cmd_listener {
                if let Ok((stream, _addr)) = listener.accept() {
                    if let Ok(client) = TcpClient::new(stream) {
                        self.cmd_clients.push(client);
                    }
                }
            }
        }

        // Accept data connection (only one at a time)
        if self.data_client.is_none() {
            if let Some(listener) = &self.data_listener {
                if let Ok((stream, _addr)) = listener.accept() {
                    if let Ok(client) = TcpClient::new(stream) {
                        self.data_client = Some(client);
                    }
                }
            }
        }

        Ok(())
    }

    /// Read commands from all clients, returning (client_index, command) pairs
    pub fn read_commands(&mut self) -> Vec<(usize, Command)> {
        let mut commands = Vec::new();

        for (idx, client) in self.cmd_clients.iter_mut().enumerate() {
            for cmd in client.read_commands() {
                commands.push((idx, cmd));
            }
        }

        // Remove disconnected clients
        self.cmd_clients.retain(|c| c.is_connected());

        // Check data client
        if let Some(client) = &self.data_client {
            if !client.is_connected() {
                self.data_client = None;
            }
        }

        commands
    }

    /// Send response to all command clients
    pub fn send_response(&mut self, response: &Response) {
        let mut failed_indices = Vec::new();
        for (idx, client) in self.cmd_clients.iter_mut().enumerate() {
            if let Err(e) = client.send(response) {
                log::warn!("Failed to send response to command client: {}", e);
                failed_indices.push(idx);
            }
        }
        // Mark failed clients as disconnected so they get cleaned up
        for idx in failed_indices {
            if idx < self.cmd_clients.len() {
                self.cmd_clients[idx].state = ClientState::Disconnected;
            }
        }
    }

    /// Send response to a specific command client by index
    pub fn send_response_to(&mut self, client_idx: usize, response: &Response) {
        if client_idx < self.cmd_clients.len() {
            if let Err(e) = self.cmd_clients[client_idx].send(response) {
                log::warn!("Failed to send response to command client {}: {}", client_idx, e);
                self.cmd_clients[client_idx].state = ClientState::Disconnected;
            }
        }
    }

    /// Send data to data client
    pub fn send_data(&mut self, data: &[u8]) -> Result<(), InterfaceError> {
        if let Some(client) = &mut self.data_client {
            client.send_raw(data)?;
        }
        Ok(())
    }

    /// Read data from data client
    pub fn read_data(&mut self) -> Option<Vec<u8>> {
        if let Some(client) = &mut self.data_client {
            let mut buf = [0u8; 4096];
            match client.stream.read(&mut buf) {
                Ok(0) => {
                    client.state = ClientState::Disconnected;
                    None
                }
                Ok(n) => Some(buf[..n].to_vec()),
                Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => None,
                Err(_) => {
                    client.state = ClientState::Disconnected;
                    None
                }
            }
        } else {
            None
        }
    }

    /// Check if any command clients are connected
    pub fn has_cmd_clients(&self) -> bool {
        !self.cmd_clients.is_empty()
    }

    /// Check if data client is connected
    pub fn has_data_client(&self) -> bool {
        self.data_client.is_some()
    }

    /// Get number of command clients
    pub fn cmd_client_count(&self) -> usize {
        self.cmd_clients.len()
    }

    /// Check if server is running
    pub fn is_running(&self) -> bool {
        self.running
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_tcp_config_default() {
        let config = TcpConfig::default();
        assert_eq!(config.cmd_port, DEFAULT_CMD_PORT);
        assert_eq!(config.data_port, DEFAULT_DATA_PORT);
    }

    // Network tests would require actual port binding
    // which may not be available in all test environments
}
