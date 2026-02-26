#pragma once

#include "cat_backend.hpp"
#include <memory>
#include <mutex>
#include <string>

namespace ultra {
namespace cat {

// Forward declaration
struct CatConfig;

// Configuration for Kenwood TCP backend
struct KenwoodTcpConfig {
    std::string host = "localhost";
    uint16_t port = 4532;       // Default FlexRadio CAT port
    int slice = 0;              // Flex slice number (0-7)
    int timeout_ms = 2000;      // Read timeout
};

class KenwoodTcpBackend : public CatBackend {
public:
    KenwoodTcpBackend();
    explicit KenwoodTcpBackend(const KenwoodTcpConfig& config);
    ~KenwoodTcpBackend() override;

    // CatBackend interface
    bool connect() override;
    void disconnect() override;
    bool isConnected() const override;

    bool setPtt(bool active) override;
    bool getPtt() const override;

    bool setFrequency(uint64_t hz) override;
    uint64_t getFrequency() override;
    bool supportsFrequency() const override { return true; }

    bool setMode(RadioMode mode) override;
    RadioMode getMode() override;
    bool supportsMode() const override { return true; }

    CatStatus getStatus() const override;
    const char* backendName() const override { return "KenwoodTCP"; }

    // Configuration
    void setConfig(const KenwoodTcpConfig& config);
    const KenwoodTcpConfig& getConfig() const { return config_; }

private:
    KenwoodTcpConfig config_;
    bool ptt_active_ = false;
    std::string last_error_;
    mutable std::mutex socket_mutex_;

#ifdef _WIN32
    uintptr_t socket_fd_ = ~static_cast<uintptr_t>(0);  // INVALID_SOCKET
#else
    int socket_fd_ = -1;
#endif

    // Send Kenwood command and get response
    std::string sendCommand(const std::string& cmd);
    bool sendCommandNoResponse(const std::string& cmd);

    // Internal socket operations
    bool socketConnect();
    void socketClose();
    bool socketSend(const std::string& data);
    std::string socketRecv(int timeout_ms);

    // Kenwood command helpers
    static std::string radioModeToKenwood(RadioMode mode);
    static RadioMode kenwoodToRadioMode(const std::string& kenwood_mode);
};

// Factory function called by CatController
std::unique_ptr<CatBackend> createKenwoodTcpBackend(const CatConfig& config);

} // namespace cat
} // namespace ultra
