// Interface module - TCP host interface for external program control
// Based on RIA modem TCP interface pattern

#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace ultra {
namespace interface {

// Default port numbers (matching RIA modem)
constexpr uint16_t DEFAULT_CMD_PORT = 8300;
constexpr uint16_t DEFAULT_DATA_PORT = 8301;
constexpr uint16_t DEFAULT_KISS_PORT = 8302;

// TCP server configuration
struct TcpConfig {
    uint16_t cmd_port = DEFAULT_CMD_PORT;
    uint16_t data_port = DEFAULT_DATA_PORT;
    uint16_t kiss_port = DEFAULT_KISS_PORT;
    std::string bind_addr = "0.0.0.0";
    bool kiss_enabled = false;
    bool enabled = false;
};

// Interface error types
enum class InterfaceError {
    None,
    ConnectionError,
    ParseError,
    InvalidCommand,
    IoError,
    SocketError,
    BindError
};

// Error to string conversion
inline const char* errorToString(InterfaceError err) {
    switch (err) {
        case InterfaceError::None: return "None";
        case InterfaceError::ConnectionError: return "Connection error";
        case InterfaceError::ParseError: return "Parse error";
        case InterfaceError::InvalidCommand: return "Invalid command";
        case InterfaceError::IoError: return "IO error";
        case InterfaceError::SocketError: return "Socket error";
        case InterfaceError::BindError: return "Bind error";
        default: return "Unknown error";
    }
}

} // namespace interface
} // namespace ultra
