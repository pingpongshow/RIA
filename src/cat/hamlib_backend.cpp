#include "hamlib_backend.hpp"
#include "cat_controller.hpp"
#include <cstdio>

#ifdef ULTRA_HAS_HAMLIB
#include <hamlib/rig.h>
#endif

namespace ultra {
namespace cat {

HamlibBackend::HamlibBackend() = default;

HamlibBackend::HamlibBackend(const HamlibConfig& config)
    : config_(config) {}

HamlibBackend::~HamlibBackend() {
    disconnect();
}

#ifdef ULTRA_HAS_HAMLIB

// ============================================================================
// Hamlib Available - Full Implementation
// ============================================================================

bool HamlibBackend::connect() {
    if (config_.model == 0) {
        last_error_ = "No rig model configured";
        return false;
    }

    if (config_.port.empty()) {
        last_error_ = "No port configured";
        return false;
    }

    // Already connected?
    if (rig_ != nullptr) {
        return true;
    }

    // Initialize rig
    rig_ = rig_init(config_.model);
    if (!rig_) {
        last_error_ = "Failed to initialize rig model " + std::to_string(config_.model);
        fprintf(stderr, "[Hamlib] %s\n", last_error_.c_str());
        return false;
    }

    // Set port
    strncpy(rig_->state.rigport.pathname, config_.port.c_str(),
            sizeof(rig_->state.rigport.pathname) - 1);

    // Set baud rate
    rig_->state.rigport.parm.serial.rate = config_.baud_rate;

    // Set PTT type
    switch (config_.ptt_type) {
        case 1:  // DTR
            rig_->state.pttport.type.ptt = RIG_PTT_SERIAL_DTR;
            break;
        case 2:  // RTS
            rig_->state.pttport.type.ptt = RIG_PTT_SERIAL_RTS;
            break;
        default:  // CAT
            rig_->state.pttport.type.ptt = RIG_PTT_RIG;
            break;
    }

    // Open connection
    int ret = rig_open(rig_);
    if (ret != RIG_OK) {
        last_error_ = "Failed to open rig: " + std::string(rigerror(ret));
        fprintf(stderr, "[Hamlib] %s\n", last_error_.c_str());
        rig_cleanup(rig_);
        rig_ = nullptr;
        return false;
    }

    fprintf(stderr, "[Hamlib] Connected to model %d on %s at %d baud\n",
            config_.model, config_.port.c_str(), config_.baud_rate);
    last_error_.clear();
    return true;
}

void HamlibBackend::disconnect() {
    if (rig_) {
        // Release PTT before disconnecting
        if (ptt_active_) {
            rig_set_ptt(rig_, RIG_VFO_CURR, RIG_PTT_OFF);
            ptt_active_ = false;
        }

        rig_close(rig_);
        rig_cleanup(rig_);
        rig_ = nullptr;
        fprintf(stderr, "[Hamlib] Disconnected\n");
    }
}

bool HamlibBackend::isConnected() const {
    return rig_ != nullptr;
}

bool HamlibBackend::setPtt(bool active) {
    if (!rig_) {
        last_error_ = "Not connected";
        return false;
    }

    ptt_t ptt = active ? RIG_PTT_ON : RIG_PTT_OFF;
    int ret = rig_set_ptt(rig_, RIG_VFO_CURR, ptt);

    if (ret != RIG_OK) {
        last_error_ = "Failed to set PTT: " + std::string(rigerror(ret));
        fprintf(stderr, "[Hamlib] %s\n", last_error_.c_str());
        return false;
    }

    ptt_active_ = active;
    fprintf(stderr, "[Hamlib] PTT %s\n", active ? "ON" : "OFF");
    return true;
}

bool HamlibBackend::getPtt() const {
    return ptt_active_;
}

bool HamlibBackend::setFrequency(uint64_t hz) {
    if (!rig_) {
        last_error_ = "Not connected";
        return false;
    }

    int ret = rig_set_freq(rig_, RIG_VFO_CURR, static_cast<freq_t>(hz));
    if (ret != RIG_OK) {
        last_error_ = "Failed to set frequency: " + std::string(rigerror(ret));
        return false;
    }

    fprintf(stderr, "[Hamlib] Set frequency to %llu Hz\n", (unsigned long long)hz);
    return true;
}

uint64_t HamlibBackend::getFrequency() {
    if (!rig_) {
        return 0;
    }

    freq_t freq = 0;
    int ret = rig_get_freq(rig_, RIG_VFO_CURR, &freq);
    if (ret != RIG_OK) {
        last_error_ = "Failed to get frequency: " + std::string(rigerror(ret));
        return 0;
    }

    return static_cast<uint64_t>(freq);
}

bool HamlibBackend::setMode(RadioMode mode) {
    if (!rig_) {
        last_error_ = "Not connected";
        return false;
    }

    rmode_t hamlib_mode = radioModeToHamlib(mode);
    if (hamlib_mode == RIG_MODE_NONE) {
        last_error_ = "Unsupported mode";
        return false;
    }

    // Use default passband
    pbwidth_t width = rig_passband_normal(rig_, hamlib_mode);

    int ret = rig_set_mode(rig_, RIG_VFO_CURR, hamlib_mode, width);
    if (ret != RIG_OK) {
        last_error_ = "Failed to set mode: " + std::string(rigerror(ret));
        return false;
    }

    fprintf(stderr, "[Hamlib] Set mode to %s\n", radioModeToString(mode));
    return true;
}

RadioMode HamlibBackend::getMode() {
    if (!rig_) {
        return RadioMode::UNKNOWN;
    }

    rmode_t mode = RIG_MODE_NONE;
    pbwidth_t width = 0;
    int ret = rig_get_mode(rig_, RIG_VFO_CURR, &mode, &width);
    if (ret != RIG_OK) {
        return RadioMode::UNKNOWN;
    }

    return hamlibToRadioMode(mode);
}

int HamlibBackend::radioModeToHamlib(RadioMode mode) {
    switch (mode) {
        case RadioMode::USB:    return RIG_MODE_USB;
        case RadioMode::LSB:    return RIG_MODE_LSB;
        case RadioMode::AM:     return RIG_MODE_AM;
        case RadioMode::FM:     return RIG_MODE_FM;
        case RadioMode::CW:     return RIG_MODE_CW;
        case RadioMode::CW_R:   return RIG_MODE_CWR;
        case RadioMode::RTTY:   return RIG_MODE_RTTY;
        case RadioMode::RTTY_R: return RIG_MODE_RTTYR;
        case RadioMode::DATA:   return RIG_MODE_PKTUSB;  // Most common for digital
        case RadioMode::DATA_R: return RIG_MODE_PKTLSB;
        default:                return RIG_MODE_NONE;
    }
}

RadioMode HamlibBackend::hamlibToRadioMode(int hamlib_mode) {
    switch (hamlib_mode) {
        case RIG_MODE_USB:    return RadioMode::USB;
        case RIG_MODE_LSB:    return RadioMode::LSB;
        case RIG_MODE_AM:     return RadioMode::AM;
        case RIG_MODE_FM:     return RadioMode::FM;
        case RIG_MODE_CW:     return RadioMode::CW;
        case RIG_MODE_CWR:    return RadioMode::CW_R;
        case RIG_MODE_RTTY:   return RadioMode::RTTY;
        case RIG_MODE_RTTYR:  return RadioMode::RTTY_R;
        case RIG_MODE_PKTUSB: return RadioMode::DATA;
        case RIG_MODE_PKTLSB: return RadioMode::DATA_R;
        default:              return RadioMode::UNKNOWN;
    }
}

bool HamlibBackend::isAvailable() {
    return true;
}

std::vector<std::pair<int, std::string>> HamlibBackend::getAvailableModels() {
    std::vector<std::pair<int, std::string>> models;

    // Return common models - a full implementation would enumerate from Hamlib
    models.emplace_back(HamlibModels::DUMMY, "Dummy (Test)");
    models.emplace_back(HamlibModels::KENWOOD_TS2000, "Kenwood TS-2000");
    models.emplace_back(HamlibModels::ICOM_IC7300, "Icom IC-7300");
    models.emplace_back(HamlibModels::ICOM_IC7100, "Icom IC-7100");
    models.emplace_back(HamlibModels::ICOM_IC7610, "Icom IC-7610");
    models.emplace_back(HamlibModels::YAESU_FT991, "Yaesu FT-991");
    models.emplace_back(HamlibModels::YAESU_FTDX10, "Yaesu FTDX-10");
    models.emplace_back(HamlibModels::ELECRAFT_K3, "Elecraft K3");
    models.emplace_back(HamlibModels::ELECRAFT_KX3, "Elecraft KX3");
    models.emplace_back(HamlibModels::FLEXRADIO_6000, "FlexRadio 6000");

    return models;
}

#else

// ============================================================================
// Hamlib NOT Available - Stub Implementation
// ============================================================================

bool HamlibBackend::connect() {
    last_error_ = "Hamlib support not compiled in";
    fprintf(stderr, "[Hamlib] %s\n", last_error_.c_str());
    return false;
}

void HamlibBackend::disconnect() {}

bool HamlibBackend::isConnected() const {
    return false;
}

bool HamlibBackend::setPtt(bool active) {
    (void)active;
    last_error_ = "Hamlib support not compiled in";
    return false;
}

bool HamlibBackend::getPtt() const {
    return false;
}

bool HamlibBackend::setFrequency(uint64_t hz) {
    (void)hz;
    last_error_ = "Hamlib support not compiled in";
    return false;
}

uint64_t HamlibBackend::getFrequency() {
    return 0;
}

bool HamlibBackend::setMode(RadioMode mode) {
    (void)mode;
    last_error_ = "Hamlib support not compiled in";
    return false;
}

RadioMode HamlibBackend::getMode() {
    return RadioMode::UNKNOWN;
}

bool HamlibBackend::isAvailable() {
    return false;
}

std::vector<std::pair<int, std::string>> HamlibBackend::getAvailableModels() {
    return {};
}

#endif  // ULTRA_HAS_HAMLIB

// ============================================================================
// Common Implementation (both with and without Hamlib)
// ============================================================================

CatStatus HamlibBackend::getStatus() const {
    CatStatus status;
    status.connected = isConnected();
    status.ptt_active = ptt_active_;
    status.error_message = last_error_;

    // Frequency and mode would be cached from last get operations
    // For now, return 0/UNKNOWN - a full implementation would poll or cache
    status.frequency_hz = 0;
    status.mode = RadioMode::UNKNOWN;

    return status;
}

void HamlibBackend::setConfig(const HamlibConfig& config) {
    bool need_reconnect = (config.model != config_.model) ||
                          (config.port != config_.port) ||
                          (config.baud_rate != config_.baud_rate);

    config_ = config;

    if (need_reconnect && isConnected()) {
        disconnect();
    }
}

// Factory function
std::unique_ptr<CatBackend> createHamlibBackend(const CatConfig& config) {
    HamlibConfig hamlib_config;
    hamlib_config.model = config.hamlib_model;
    hamlib_config.port = config.serial_port;
    hamlib_config.baud_rate = config.serial_baud;
    hamlib_config.ptt_type = 0;  // Default to CAT PTT

    return std::make_unique<HamlibBackend>(hamlib_config);
}

} // namespace cat
} // namespace ultra
