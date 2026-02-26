#include "kenwood_tcp_backend.hpp"
#include "cat_controller.hpp"
#include <cstdio>
#include <cstring>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
#define SOCKET_ERROR_CODE WSAGetLastError()
#define CLOSE_SOCKET closesocket
#define INVALID_SOCK INVALID_SOCKET
typedef SOCKET socket_t;
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#define SOCKET_ERROR_CODE errno
#define CLOSE_SOCKET close
#define INVALID_SOCK -1
typedef int socket_t;
#endif

namespace ultra {
namespace cat {

KenwoodTcpBackend::KenwoodTcpBackend() = default;

KenwoodTcpBackend::KenwoodTcpBackend(const KenwoodTcpConfig& config)
    : config_(config) {}

KenwoodTcpBackend::~KenwoodTcpBackend() {
    disconnect();
}

bool KenwoodTcpBackend::connect() {
    std::lock_guard<std::mutex> lock(socket_mutex_);

    if (config_.host.empty()) {
        last_error_ = "No host configured";
        return false;
    }

#ifdef _WIN32
    if (socket_fd_ != INVALID_SOCK) {
#else
    if (socket_fd_ >= 0) {
#endif
        return true;  // Already connected
    }

    return socketConnect();
}

void KenwoodTcpBackend::disconnect() {
    std::lock_guard<std::mutex> lock(socket_mutex_);

    // Release PTT before disconnecting
    if (ptt_active_) {
#ifdef _WIN32
        if (socket_fd_ != INVALID_SOCK) {
#else
        if (socket_fd_ >= 0) {
#endif
            socketSend("RX;");  // PTT off
        }
        ptt_active_ = false;
    }

    socketClose();
}

bool KenwoodTcpBackend::isConnected() const {
    std::lock_guard<std::mutex> lock(socket_mutex_);
#ifdef _WIN32
    return socket_fd_ != INVALID_SOCK;
#else
    return socket_fd_ >= 0;
#endif
}

bool KenwoodTcpBackend::setPtt(bool active) {
    std::lock_guard<std::mutex> lock(socket_mutex_);

#ifdef _WIN32
    if (socket_fd_ == INVALID_SOCK) {
#else
    if (socket_fd_ < 0) {
#endif
        last_error_ = "Not connected";
        return false;
    }

    // Kenwood TX command: TX0 = PTT off, TX1 = PTT on
    // Also support RX for PTT off
    std::string cmd = active ? "TX1;" : "RX;";

    if (!socketSend(cmd)) {
        last_error_ = "Failed to send PTT command";
        return false;
    }

    ptt_active_ = active;
    fprintf(stderr, "[KenwoodTCP] PTT %s\n", active ? "ON" : "OFF");
    return true;
}

bool KenwoodTcpBackend::getPtt() const {
    return ptt_active_;
}

bool KenwoodTcpBackend::setFrequency(uint64_t hz) {
    std::lock_guard<std::mutex> lock(socket_mutex_);

#ifdef _WIN32
    if (socket_fd_ == INVALID_SOCK) {
#else
    if (socket_fd_ < 0) {
#endif
        last_error_ = "Not connected";
        return false;
    }

    // Kenwood FA command: FA<11-digit frequency>;
    // e.g., FA00014070000; for 14.070 MHz
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "FA%011llu;", (unsigned long long)hz);

    if (!socketSend(cmd)) {
        last_error_ = "Failed to send frequency command";
        return false;
    }

    fprintf(stderr, "[KenwoodTCP] Set frequency to %llu Hz\n", (unsigned long long)hz);
    return true;
}

uint64_t KenwoodTcpBackend::getFrequency() {
    std::lock_guard<std::mutex> lock(socket_mutex_);

#ifdef _WIN32
    if (socket_fd_ == INVALID_SOCK) {
#else
    if (socket_fd_ < 0) {
#endif
        return 0;
    }

    // Query frequency
    if (!socketSend("FA;")) {
        return 0;
    }

    std::string response = socketRecv(config_.timeout_ms);
    if (response.empty() || response.length() < 13) {
        return 0;
    }

    // Parse FA<11-digit>;
    if (response.substr(0, 2) == "FA") {
        try {
            return std::stoull(response.substr(2, 11));
        } catch (...) {
            return 0;
        }
    }

    return 0;
}

bool KenwoodTcpBackend::setMode(RadioMode mode) {
    std::lock_guard<std::mutex> lock(socket_mutex_);

#ifdef _WIN32
    if (socket_fd_ == INVALID_SOCK) {
#else
    if (socket_fd_ < 0) {
#endif
        last_error_ = "Not connected";
        return false;
    }

    std::string kenwood_mode = radioModeToKenwood(mode);
    if (kenwood_mode.empty()) {
        last_error_ = "Unsupported mode";
        return false;
    }

    // Kenwood MD command: MD<mode>;
    std::string cmd = "MD" + kenwood_mode + ";";

    if (!socketSend(cmd)) {
        last_error_ = "Failed to send mode command";
        return false;
    }

    fprintf(stderr, "[KenwoodTCP] Set mode to %s\n", radioModeToString(mode));
    return true;
}

RadioMode KenwoodTcpBackend::getMode() {
    std::lock_guard<std::mutex> lock(socket_mutex_);

#ifdef _WIN32
    if (socket_fd_ == INVALID_SOCK) {
#else
    if (socket_fd_ < 0) {
#endif
        return RadioMode::UNKNOWN;
    }

    // Query mode
    if (!socketSend("MD;")) {
        return RadioMode::UNKNOWN;
    }

    std::string response = socketRecv(config_.timeout_ms);
    if (response.empty() || response.length() < 3) {
        return RadioMode::UNKNOWN;
    }

    // Parse MD<mode>;
    if (response.substr(0, 2) == "MD") {
        return kenwoodToRadioMode(response.substr(2, 1));
    }

    return RadioMode::UNKNOWN;
}

CatStatus KenwoodTcpBackend::getStatus() const {
    CatStatus status;
    status.connected = isConnected();
    status.ptt_active = ptt_active_;
    status.error_message = last_error_;
    status.frequency_hz = 0;  // Would need to query
    status.mode = RadioMode::UNKNOWN;
    return status;
}

void KenwoodTcpBackend::setConfig(const KenwoodTcpConfig& config) {
    std::lock_guard<std::mutex> lock(socket_mutex_);

    bool need_reconnect = (config.host != config_.host) ||
                          (config.port != config_.port);

    config_ = config;

    if (need_reconnect) {
        socketClose();
    }
}

// ============================================================================
// Socket Operations
// ============================================================================

bool KenwoodTcpBackend::socketConnect() {
    // Initialize Winsock on Windows
#ifdef _WIN32
    static bool winsock_initialized = false;
    if (!winsock_initialized) {
        WSADATA wsaData;
        if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
            last_error_ = "Failed to initialize Winsock";
            return false;
        }
        winsock_initialized = true;
    }
#endif

    // Resolve hostname
    struct addrinfo hints, *result;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;

    char port_str[16];
    snprintf(port_str, sizeof(port_str), "%d", config_.port);

    int ret = getaddrinfo(config_.host.c_str(), port_str, &hints, &result);
    if (ret != 0) {
        last_error_ = "Failed to resolve host: " + config_.host;
        fprintf(stderr, "[KenwoodTCP] %s\n", last_error_.c_str());
        return false;
    }

    // Create socket
    socket_fd_ = socket(result->ai_family, result->ai_socktype, result->ai_protocol);
#ifdef _WIN32
    if (socket_fd_ == INVALID_SOCK) {
#else
    if (socket_fd_ < 0) {
#endif
        last_error_ = "Failed to create socket";
        freeaddrinfo(result);
        return false;
    }

    // Set socket timeout
#ifdef _WIN32
    DWORD timeout = config_.timeout_ms;
    setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout, sizeof(timeout));
    setsockopt(socket_fd_, SOL_SOCKET, SO_SNDTIMEO, (const char*)&timeout, sizeof(timeout));
#else
    struct timeval tv;
    tv.tv_sec = config_.timeout_ms / 1000;
    tv.tv_usec = (config_.timeout_ms % 1000) * 1000;
    setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    setsockopt(socket_fd_, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
#endif

    // Connect
    ret = ::connect(socket_fd_, result->ai_addr, (int)result->ai_addrlen);
    freeaddrinfo(result);

    if (ret != 0) {
        last_error_ = "Failed to connect to " + config_.host + ":" + std::to_string(config_.port);
        fprintf(stderr, "[KenwoodTCP] %s (error %d)\n", last_error_.c_str(), SOCKET_ERROR_CODE);
        socketClose();
        return false;
    }

    fprintf(stderr, "[KenwoodTCP] Connected to %s:%d\n",
            config_.host.c_str(), config_.port);
    last_error_.clear();
    return true;
}

void KenwoodTcpBackend::socketClose() {
#ifdef _WIN32
    if (socket_fd_ != INVALID_SOCK) {
        CLOSE_SOCKET(socket_fd_);
        socket_fd_ = INVALID_SOCK;
#else
    if (socket_fd_ >= 0) {
        CLOSE_SOCKET(socket_fd_);
        socket_fd_ = -1;
#endif
        fprintf(stderr, "[KenwoodTCP] Disconnected\n");
    }
}

bool KenwoodTcpBackend::socketSend(const std::string& data) {
#ifdef _WIN32
    int sent = send(socket_fd_, data.c_str(), (int)data.length(), 0);
#else
    ssize_t sent = send(socket_fd_, data.c_str(), data.length(), 0);
#endif
    return sent == (decltype(sent))data.length();
}

std::string KenwoodTcpBackend::socketRecv(int timeout_ms) {
    // Use poll/select for timeout
#ifdef _WIN32
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(socket_fd_, &readfds);
    struct timeval tv;
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    int ready = select((int)socket_fd_ + 1, &readfds, nullptr, nullptr, &tv);
#else
    struct pollfd pfd;
    pfd.fd = socket_fd_;
    pfd.events = POLLIN;
    int ready = poll(&pfd, 1, timeout_ms);
#endif

    if (ready <= 0) {
        return "";  // Timeout or error
    }

    char buffer[256];
#ifdef _WIN32
    int len = recv(socket_fd_, buffer, sizeof(buffer) - 1, 0);
#else
    ssize_t len = recv(socket_fd_, buffer, sizeof(buffer) - 1, 0);
#endif

    if (len <= 0) {
        return "";
    }

    buffer[len] = '\0';
    return std::string(buffer);
}

// ============================================================================
// Kenwood Mode Conversion
// ============================================================================

std::string KenwoodTcpBackend::radioModeToKenwood(RadioMode mode) {
    // Kenwood mode numbers:
    // 1=LSB, 2=USB, 3=CW, 4=FM, 5=AM, 6=RTTY, 7=CW-R, 9=RTTY-R
    switch (mode) {
        case RadioMode::LSB:    return "1";
        case RadioMode::USB:    return "2";
        case RadioMode::CW:     return "3";
        case RadioMode::FM:     return "4";
        case RadioMode::AM:     return "5";
        case RadioMode::RTTY:   return "6";
        case RadioMode::CW_R:   return "7";
        case RadioMode::RTTY_R: return "9";
        case RadioMode::DATA:   return "2";  // DATA usually uses USB
        case RadioMode::DATA_R: return "1";  // DATA-R uses LSB
        default:                return "";
    }
}

RadioMode KenwoodTcpBackend::kenwoodToRadioMode(const std::string& kenwood_mode) {
    if (kenwood_mode == "1") return RadioMode::LSB;
    if (kenwood_mode == "2") return RadioMode::USB;
    if (kenwood_mode == "3") return RadioMode::CW;
    if (kenwood_mode == "4") return RadioMode::FM;
    if (kenwood_mode == "5") return RadioMode::AM;
    if (kenwood_mode == "6") return RadioMode::RTTY;
    if (kenwood_mode == "7") return RadioMode::CW_R;
    if (kenwood_mode == "9") return RadioMode::RTTY_R;
    return RadioMode::UNKNOWN;
}

// Factory function
std::unique_ptr<CatBackend> createKenwoodTcpBackend(const CatConfig& config) {
    KenwoodTcpConfig tcp_config;
    tcp_config.host = config.tcp_host;
    tcp_config.port = config.tcp_port;
    tcp_config.slice = config.flex_slice;
    tcp_config.timeout_ms = 2000;

    return std::make_unique<KenwoodTcpBackend>(tcp_config);
}

} // namespace cat
} // namespace ultra
