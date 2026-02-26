#include "serial_ptt_backend.hpp"
#include "cat_controller.hpp"
#include <cstdio>

namespace ultra {
namespace cat {

SerialPttBackend::SerialPttBackend() = default;

SerialPttBackend::SerialPttBackend(const SerialPttConfig& config)
    : config_(config) {}

SerialPttBackend::~SerialPttBackend() {
    // Ensure PTT is released
    if (ptt_active_ && controller_.isOpen()) {
        auto line = static_cast<gui::SerialPttLine>(config_.ptt_line);
        controller_.setLine(line, config_.ptt_invert);  // Release = invert of active
    }
    disconnect();
}

bool SerialPttBackend::connect() {
    if (config_.port.empty()) {
        last_error_ = "No serial port configured";
        return false;
    }

    // Already connected to same port?
    if (controller_.isOpen() && controller_.matches(config_.port, config_.baud_rate)) {
        return true;
    }

    // Close if open to different port
    if (controller_.isOpen()) {
        controller_.close();
    }

    if (!controller_.open(config_.port, config_.baud_rate)) {
        last_error_ = "Failed to open serial port: " + config_.port;
        fprintf(stderr, "[SerialPTT] %s\n", last_error_.c_str());
        return false;
    }

    fprintf(stderr, "[SerialPTT] Connected to %s at %d baud\n",
            config_.port.c_str(), config_.baud_rate);
    last_error_.clear();
    return true;
}

void SerialPttBackend::disconnect() {
    if (controller_.isOpen()) {
        // Release PTT before closing
        if (ptt_active_) {
            auto line = static_cast<gui::SerialPttLine>(config_.ptt_line);
            controller_.setLine(line, config_.ptt_invert);  // Release
            ptt_active_ = false;
        }
        controller_.close();
        fprintf(stderr, "[SerialPTT] Disconnected\n");
    }
}

bool SerialPttBackend::isConnected() const {
    return controller_.isOpen();
}

bool SerialPttBackend::setPtt(bool active) {
    if (!controller_.isOpen()) {
        last_error_ = "Serial port not open";
        return false;
    }

    auto line = static_cast<gui::SerialPttLine>(config_.ptt_line);

    // Apply invert: if ptt_invert is true, we invert the logic
    // Note: CatController applies its own invert, so we receive the already-inverted value
    bool line_state = active;

    if (!controller_.setLine(line, line_state)) {
        last_error_ = "Failed to set PTT line";
        fprintf(stderr, "[SerialPTT] Failed to set %s to %s\n",
                config_.ptt_line == 0 ? "DTR" : "RTS",
                line_state ? "HIGH" : "LOW");
        return false;
    }

    ptt_active_ = active;
    fprintf(stderr, "[SerialPTT] PTT %s (%s=%s)\n",
            active ? "ON" : "OFF",
            config_.ptt_line == 0 ? "DTR" : "RTS",
            line_state ? "HIGH" : "LOW");
    return true;
}

bool SerialPttBackend::getPtt() const {
    return ptt_active_;
}

CatStatus SerialPttBackend::getStatus() const {
    CatStatus status;
    status.connected = controller_.isOpen();
    status.ptt_active = ptt_active_;
    status.frequency_hz = 0;  // Not supported
    status.mode = RadioMode::UNKNOWN;  // Not supported
    status.error_message = last_error_;
    return status;
}

void SerialPttBackend::setConfig(const SerialPttConfig& config) {
    // If port changed, need to reconnect
    bool need_reconnect = (config.port != config_.port) ||
                          (config.baud_rate != config_.baud_rate);

    config_ = config;

    if (need_reconnect && controller_.isOpen()) {
        disconnect();
    }
}

// Factory function
std::unique_ptr<CatBackend> createSerialPttBackend(const CatConfig& config) {
    SerialPttConfig serial_config;
    serial_config.port = config.serial_port;
    serial_config.baud_rate = config.serial_baud;
    serial_config.ptt_line = config.ptt_line;
    serial_config.ptt_invert = config.ptt_invert;

    return std::make_unique<SerialPttBackend>(serial_config);
}

} // namespace cat
} // namespace ultra
