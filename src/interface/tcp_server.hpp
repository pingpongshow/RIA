// TCP server for host application communication
// Ported from RIA modem tcp.rs

#pragma once

#include "interface.hpp"
#include "command_parser.hpp"
#include "response.hpp"

#include <string>
#include <vector>
#include <memory>
#include <optional>
#include <tuple>

// Cross-platform socket handling
#ifdef _WIN32
    #include <winsock2.h>
    #include <ws2tcpip.h>
    using socket_t = SOCKET;
    #define INVALID_SOCKET_VALUE INVALID_SOCKET
    #define CLOSE_SOCKET closesocket
#else
    using socket_t = int;
    #define INVALID_SOCKET_VALUE -1
    #define CLOSE_SOCKET close
#endif

namespace ultra {
namespace interface {

// Client connection state
enum class ClientState {
    Connected,
    Disconnected
};

// TCP client handler
class TcpClient {
public:
    explicit TcpClient(socket_t socket_fd);
    ~TcpClient();

    // Non-copyable
    TcpClient(const TcpClient&) = delete;
    TcpClient& operator=(const TcpClient&) = delete;

    // Move semantics
    TcpClient(TcpClient&& other) noexcept;
    TcpClient& operator=(TcpClient&& other) noexcept;

    // Read commands (non-blocking)
    std::vector<ParsedCommand> readCommands();

    // Send response
    bool send(const Response& response);

    // Send raw data
    bool sendRaw(const uint8_t* data, size_t len);

    // Read raw data (non-blocking)
    std::optional<std::vector<uint8_t>> readData();

    // Check if connected
    bool isConnected() const { return state_ == ClientState::Connected; }

    // Mark as disconnected
    void disconnect() { state_ = ClientState::Disconnected; }

    // Get peer address (for logging)
    std::string peerAddress() const;

private:
    socket_t socket_fd_ = INVALID_SOCKET_VALUE;
    CommandParser parser_;
    ClientState state_ = ClientState::Connected;
};

// TCP server for command and data connections
class TcpServer {
public:
    explicit TcpServer(const TcpConfig& config);
    ~TcpServer();

    // Non-copyable, non-movable
    TcpServer(const TcpServer&) = delete;
    TcpServer& operator=(const TcpServer&) = delete;

    // Start listening
    bool start();

    // Stop listening and close all connections
    void stop();

    // Accept new connections (non-blocking)
    void accept();

    // Read commands from all clients
    // Returns vector of (client_index, command)
    std::vector<std::pair<size_t, ParsedCommand>> readCommands();

    // Send response to all command clients
    void sendResponse(const Response& response);

    // Send response to specific client
    void sendResponseTo(size_t client_idx, const Response& response);

    // Data port operations
    bool sendData(const uint8_t* data, size_t len);
    std::optional<std::vector<uint8_t>> readData();

    // Status queries
    bool hasCmdClients() const { return !cmd_clients_.empty(); }
    bool hasDataClient() const { return !data_clients_.empty(); }
    size_t cmdClientCount() const { return cmd_clients_.size(); }
    bool isRunning() const { return running_; }

    // Get last error
    InterfaceError lastError() const { return last_error_; }

private:
    TcpConfig config_;
    socket_t cmd_listener_ = INVALID_SOCKET_VALUE;
    socket_t data_listener_ = INVALID_SOCKET_VALUE;
    std::vector<std::unique_ptr<TcpClient>> cmd_clients_;
    std::vector<std::unique_ptr<TcpClient>> data_clients_;
    bool running_ = false;
    InterfaceError last_error_ = InterfaceError::None;

    static constexpr size_t MAX_CMD_CLIENTS = 10;
    static constexpr size_t MAX_DATA_CLIENTS = 10;

    // Helper: create listening socket
    socket_t createListener(uint16_t port);

public:
    // Helper: set socket non-blocking (public for TcpClient)
    static bool setNonBlocking(socket_t sock);
};

// Platform-specific socket initialization
class SocketInit {
public:
    SocketInit();
    ~SocketInit();
    bool isInitialized() const { return initialized_; }
private:
    bool initialized_ = false;
};

// Global socket initialization (call before using sockets)
bool initializeSockets();

} // namespace interface
} // namespace ultra
