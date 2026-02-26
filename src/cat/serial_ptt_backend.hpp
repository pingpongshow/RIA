#pragma once

#include "cat_backend.hpp"
#include "gui/serial_ptt.hpp"
#include <memory>
#include <string>

namespace ultra {
namespace cat {

// Forward declaration
struct CatConfig;

// Configuration for serial PTT backend
struct SerialPttConfig {
    std::string port;
    int baud_rate = 9600;
    int ptt_line = 0;       // 0=DTR, 1=RTS (maps to gui::SerialPttLine)
    bool ptt_invert = false;
};

class SerialPttBackend : public CatBackend {
public:
    SerialPttBackend();
    explicit SerialPttBackend(const SerialPttConfig& config);
    ~SerialPttBackend() override;

    // CatBackend interface
    bool connect() override;
    void disconnect() override;
    bool isConnected() const override;

    bool setPtt(bool active) override;
    bool getPtt() const override;

    // Serial PTT does not support frequency/mode control
    bool setFrequency(uint64_t hz) override { (void)hz; return false; }
    uint64_t getFrequency() override { return 0; }
    bool supportsFrequency() const override { return false; }

    bool setMode(RadioMode mode) override { (void)mode; return false; }
    RadioMode getMode() override { return RadioMode::UNKNOWN; }
    bool supportsMode() const override { return false; }

    CatStatus getStatus() const override;
    const char* backendName() const override { return "SerialPTT"; }

    // Configuration
    void setConfig(const SerialPttConfig& config);
    const SerialPttConfig& getConfig() const { return config_; }

private:
    SerialPttConfig config_;
    gui::SerialPttController controller_;
    bool ptt_active_ = false;
    std::string last_error_;
};

// Factory function called by CatController
std::unique_ptr<CatBackend> createSerialPttBackend(const CatConfig& config);

} // namespace cat
} // namespace ultra
