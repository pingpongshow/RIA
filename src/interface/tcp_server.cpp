// TCP server implementation
// Cross-platform socket handling

#include "tcp_server.hpp"
#include "ultra/logging.hpp"

#include <cstring>

#ifdef _WIN32
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #pragma comment(lib, "ws2_32.lib")
#else
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <netinet/tcp.h>
    #include <arpa/inet.h>
    #include <unistd.h>
    #include <fcntl.h>
    #include <errno.h>
#endif

namespace ultra {
namespace interface {

// Static initialization flag
static bool g_sockets_initialized = false;

SocketInit::SocketInit() {
#ifdef _WIN32
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) == 0) {
        initialized_ = true;
    }
#else
    initialized_ = true;
#endif
}

SocketInit::~SocketInit() {
#ifdef _WIN32
    if (initialized_) {
        WSACleanup();
    }
#endif
}

bool initializeSockets() {
    if (g_sockets_initialized) return true;

#ifdef _WIN32
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        return false;
    }
#endif

    g_sockets_initialized = true;
    return true;
}

// TcpClient implementation

TcpClient::TcpClient(socket_t socket_fd) : socket_fd_(socket_fd) {
    // Set non-blocking
    TcpServer::setNonBlocking(socket_fd_);

    // Set TCP_NODELAY for low latency
#ifdef _WIN32
    BOOL nodelay = TRUE;
    setsockopt(socket_fd_, IPPROTO_TCP, TCP_NODELAY,
               reinterpret_cast<const char*>(&nodelay), sizeof(nodelay));
#else
    int nodelay = 1;
    setsockopt(socket_fd_, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));
#endif
}

TcpClient::~TcpClient() {
    if (socket_fd_ != INVALID_SOCKET_VALUE) {
        CLOSE_SOCKET(socket_fd_);
    }
}

TcpClient::TcpClient(TcpClient&& other) noexcept
    : socket_fd_(other.socket_fd_),
      parser_(std::move(other.parser_)),
      state_(other.state_) {
    other.socket_fd_ = INVALID_SOCKET_VALUE;
    other.state_ = ClientState::Disconnected;
}

TcpClient& TcpClient::operator=(TcpClient&& other) noexcept {
    if (this != &other) {
        if (socket_fd_ != INVALID_SOCKET_VALUE) {
            CLOSE_SOCKET(socket_fd_);
        }
        socket_fd_ = other.socket_fd_;
        parser_ = std::move(other.parser_);
        state_ = other.state_;
        other.socket_fd_ = INVALID_SOCKET_VALUE;
        other.state_ = ClientState::Disconnected;
    }
    return *this;
}

std::vector<ParsedCommand> TcpClient::readCommands() {
    if (state_ != ClientState::Connected) {
        return {};
    }

    uint8_t buf[4096];

#ifdef _WIN32
    int n = recv(socket_fd_, reinterpret_cast<char*>(buf), sizeof(buf), 0);
    if (n == 0) {
        state_ = ClientState::Disconnected;
        return {};
    }
    if (n < 0) {
        int err = WSAGetLastError();
        if (err != WSAEWOULDBLOCK) {
            state_ = ClientState::Disconnected;
        }
        return {};
    }
#else
    ssize_t n = recv(socket_fd_, buf, sizeof(buf), 0);
    if (n == 0) {
        state_ = ClientState::Disconnected;
        return {};
    }
    if (n < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
            state_ = ClientState::Disconnected;
        }
        return {};
    }
#endif

    return parser_.parse(buf, static_cast<size_t>(n));
}

bool TcpClient::send(const Response& response) {
    if (state_ != ClientState::Connected) {
        return false;
    }

    auto bytes = response.toBytes();
    return sendRaw(bytes.data(), bytes.size());
}

bool TcpClient::sendRaw(const uint8_t* data, size_t len) {
    if (state_ != ClientState::Connected || len == 0) {
        return false;
    }

    size_t sent = 0;
    while (sent < len) {
#ifdef _WIN32
        int n = ::send(socket_fd_, reinterpret_cast<const char*>(data + sent),
                       static_cast<int>(len - sent), 0);
        if (n <= 0) {
            state_ = ClientState::Disconnected;
            return false;
        }
#else
        ssize_t n = ::send(socket_fd_, data + sent, len - sent, MSG_NOSIGNAL);
        if (n <= 0) {
            state_ = ClientState::Disconnected;
            return false;
        }
#endif
        sent += static_cast<size_t>(n);
    }

    return true;
}

std::optional<std::vector<uint8_t>> TcpClient::readData() {
    if (state_ != ClientState::Connected) {
        return std::nullopt;
    }

    uint8_t buf[4096];

#ifdef _WIN32
    int n = recv(socket_fd_, reinterpret_cast<char*>(buf), sizeof(buf), 0);
    if (n == 0) {
        state_ = ClientState::Disconnected;
        return std::nullopt;
    }
    if (n < 0) {
        int err = WSAGetLastError();
        if (err != WSAEWOULDBLOCK) {
            state_ = ClientState::Disconnected;
        }
        return std::nullopt;
    }
#else
    ssize_t n = recv(socket_fd_, buf, sizeof(buf), 0);
    if (n == 0) {
        state_ = ClientState::Disconnected;
        return std::nullopt;
    }
    if (n < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
            state_ = ClientState::Disconnected;
        }
        return std::nullopt;
    }
#endif

    return std::vector<uint8_t>(buf, buf + n);
}

std::string TcpClient::peerAddress() const {
    if (socket_fd_ == INVALID_SOCKET_VALUE) {
        return "unknown";
    }

    struct sockaddr_storage addr;
    socklen_t addrlen = sizeof(addr);

    if (getpeername(socket_fd_, reinterpret_cast<struct sockaddr*>(&addr), &addrlen) != 0) {
        return "unknown";
    }

    char ip[INET6_ADDRSTRLEN];
    uint16_t port = 0;

    if (addr.ss_family == AF_INET) {
        auto* s = reinterpret_cast<struct sockaddr_in*>(&addr);
        inet_ntop(AF_INET, &s->sin_addr, ip, sizeof(ip));
        port = ntohs(s->sin_port);
    } else if (addr.ss_family == AF_INET6) {
        auto* s = reinterpret_cast<struct sockaddr_in6*>(&addr);
        inet_ntop(AF_INET6, &s->sin6_addr, ip, sizeof(ip));
        port = ntohs(s->sin6_port);
    } else {
        return "unknown";
    }

    return std::string(ip) + ":" + std::to_string(port);
}

// TcpServer implementation

TcpServer::TcpServer(const TcpConfig& config) : config_(config) {}

TcpServer::~TcpServer() {
    stop();
}

bool TcpServer::start() {
    if (running_) {
        return true;
    }

    if (!initializeSockets()) {
        last_error_ = InterfaceError::SocketError;
        return false;
    }

    // Create command listener
    cmd_listener_ = createListener(config_.cmd_port);
    if (cmd_listener_ == INVALID_SOCKET_VALUE) {
        last_error_ = InterfaceError::BindError;
        LOG_MODEM(ERROR, "TcpServer: Failed to bind command port %d", config_.cmd_port);
        return false;
    }

    // Create data listener
    data_listener_ = createListener(config_.data_port);
    if (data_listener_ == INVALID_SOCKET_VALUE) {
        CLOSE_SOCKET(cmd_listener_);
        cmd_listener_ = INVALID_SOCKET_VALUE;
        last_error_ = InterfaceError::BindError;
        LOG_MODEM(ERROR, "TcpServer: Failed to bind data port %d", config_.data_port);
        return false;
    }

    running_ = true;
    LOG_MODEM(INFO, "TcpServer: Started (cmd=%d, data=%d, bind=%s)",
              config_.cmd_port, config_.data_port, config_.bind_addr.c_str());
    return true;
}

void TcpServer::stop() {
    running_ = false;

    // Close listeners
    if (cmd_listener_ != INVALID_SOCKET_VALUE) {
        CLOSE_SOCKET(cmd_listener_);
        cmd_listener_ = INVALID_SOCKET_VALUE;
    }
    if (data_listener_ != INVALID_SOCKET_VALUE) {
        CLOSE_SOCKET(data_listener_);
        data_listener_ = INVALID_SOCKET_VALUE;
    }

    // Close clients
    cmd_clients_.clear();
    data_clients_.clear();

    LOG_MODEM(INFO, "TcpServer: Stopped");
}

void TcpServer::accept() {
    if (!running_) return;

    // Accept command connections (up to MAX_CMD_CLIENTS)
    if (cmd_clients_.size() < MAX_CMD_CLIENTS) {
        struct sockaddr_storage addr;
        socklen_t addrlen = sizeof(addr);
        socket_t client_sock = ::accept(cmd_listener_,
                                         reinterpret_cast<struct sockaddr*>(&addr),
                                         &addrlen);
        if (client_sock != INVALID_SOCKET_VALUE) {
            auto client = std::make_unique<TcpClient>(client_sock);
            LOG_MODEM(INFO, "TcpServer: Command client connected from %s",
                      client->peerAddress().c_str());
            cmd_clients_.push_back(std::move(client));
        }
    }

    // Accept data connections
    if (data_clients_.size() < MAX_DATA_CLIENTS) {
        struct sockaddr_storage addr;
        socklen_t addrlen = sizeof(addr);
        socket_t client_sock = ::accept(data_listener_,
                                         reinterpret_cast<struct sockaddr*>(&addr),
                                         &addrlen);
        if (client_sock != INVALID_SOCKET_VALUE) {
            auto client = std::make_unique<TcpClient>(client_sock);
            LOG_MODEM(INFO, "TcpServer: Data client connected from %s",
                      client->peerAddress().c_str());
            data_clients_.push_back(std::move(client));
        }
    }
}

std::vector<std::pair<size_t, ParsedCommand>> TcpServer::readCommands() {
    std::vector<std::pair<size_t, ParsedCommand>> result;

    for (size_t i = 0; i < cmd_clients_.size(); ++i) {
        auto& client = cmd_clients_[i];
        auto cmds = client->readCommands();
        for (auto& cmd : cmds) {
            result.emplace_back(i, std::move(cmd));
        }
    }

    // Remove disconnected clients
    cmd_clients_.erase(
        std::remove_if(cmd_clients_.begin(), cmd_clients_.end(),
                       [](const std::unique_ptr<TcpClient>& c) {
                           return !c->isConnected();
                       }),
        cmd_clients_.end());

    // Remove disconnected data clients
    data_clients_.erase(
        std::remove_if(data_clients_.begin(), data_clients_.end(),
                       [](const std::unique_ptr<TcpClient>& c) {
                           return !c->isConnected();
                       }),
        data_clients_.end());

    return result;
}

void TcpServer::sendResponse(const Response& response) {
    for (auto& client : cmd_clients_) {
        if (!client->send(response)) {
            LOG_MODEM(WARN, "TcpServer: Failed to send response to command client");
        }
    }
}

void TcpServer::sendResponseTo(size_t client_idx, const Response& response) {
    if (client_idx < cmd_clients_.size()) {
        if (!cmd_clients_[client_idx]->send(response)) {
            LOG_MODEM(WARN, "TcpServer: Failed to send response to client %zu", client_idx);
        }
    }
}

bool TcpServer::sendData(const uint8_t* data, size_t len) {
    if (data_clients_.empty()) {
        return false;
    }

    bool any_sent = false;
    for (auto& client : data_clients_) {
        if (client->isConnected() && client->sendRaw(data, len)) {
            any_sent = true;
        }
    }

    // Prune disconnected clients after attempted broadcast.
    data_clients_.erase(
        std::remove_if(data_clients_.begin(), data_clients_.end(),
                       [](const std::unique_ptr<TcpClient>& c) {
                           return !c->isConnected();
                       }),
        data_clients_.end());

    return any_sent;
}

std::optional<std::vector<uint8_t>> TcpServer::readData() {
    if (data_clients_.empty()) {
        return std::nullopt;
    }

    std::optional<std::vector<uint8_t>> result = std::nullopt;
    for (auto& client : data_clients_) {
        auto data = client->readData();
        if (data && !data->empty()) {
            result = std::move(data);
            break;
        }
    }

    // Remove disconnected clients after poll.
    data_clients_.erase(
        std::remove_if(data_clients_.begin(), data_clients_.end(),
                       [](const std::unique_ptr<TcpClient>& c) {
                           return !c->isConnected();
                       }),
        data_clients_.end());

    return result;
}

socket_t TcpServer::createListener(uint16_t port) {
    socket_t sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock == INVALID_SOCKET_VALUE) {
        return INVALID_SOCKET_VALUE;
    }

    // Allow address reuse
    int optval = 1;
#ifdef _WIN32
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR,
               reinterpret_cast<const char*>(&optval), sizeof(optval));
#else
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));
#endif

    // Bind
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);

    if (config_.bind_addr == "0.0.0.0" || config_.bind_addr.empty()) {
        addr.sin_addr.s_addr = INADDR_ANY;
    } else {
        if (inet_pton(AF_INET, config_.bind_addr.c_str(), &addr.sin_addr) != 1) {
            CLOSE_SOCKET(sock);
            return INVALID_SOCKET_VALUE;
        }
    }

    if (bind(sock, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) != 0) {
        CLOSE_SOCKET(sock);
        return INVALID_SOCKET_VALUE;
    }

    // Listen
    if (listen(sock, 5) != 0) {
        CLOSE_SOCKET(sock);
        return INVALID_SOCKET_VALUE;
    }

    // Set non-blocking
    if (!setNonBlocking(sock)) {
        CLOSE_SOCKET(sock);
        return INVALID_SOCKET_VALUE;
    }

    return sock;
}

bool TcpServer::setNonBlocking(socket_t sock) {
#ifdef _WIN32
    u_long mode = 1;
    return ioctlsocket(sock, FIONBIO, &mode) == 0;
#else
    int flags = fcntl(sock, F_GETFL, 0);
    if (flags == -1) return false;
    return fcntl(sock, F_SETFL, flags | O_NONBLOCK) == 0;
#endif
}

} // namespace interface
} // namespace ultra
