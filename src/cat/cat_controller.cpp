#include "cat_controller.hpp"
#include <cstdio>

// Forward declarations for backends (implemented in separate files)
// These are defined here to allow compilation before backends are implemented
namespace ultra {
namespace cat {

// Factory functions defined in backend implementation files
std::unique_ptr<CatBackend> createSerialPttBackend(const CatConfig& config);
std::unique_ptr<CatBackend> createHamlibBackend(const CatConfig& config);
std::unique_ptr<CatBackend> createKenwoodTcpBackend(const CatConfig& config);

} // namespace cat
} // namespace ultra

namespace ultra {
namespace cat {

CatController::CatController() = default;

CatController::~CatController() {
    // Ensure PTT is released on destruction
    if (ptt_active_ && backend_) {
        backend_->setPtt(false);
    }
    disconnect();
}

void CatController::configure(const CatConfig& config) {
    std::lock_guard<std::mutex> lock(mutex_);

    // If backend type changed or not connected, need to recreate
    bool need_reconnect = (config.backend_type != config_.backend_type) ||
                          (config.serial_port != config_.serial_port) ||
                          (config.tcp_host != config_.tcp_host) ||
                          (config.tcp_port != config_.tcp_port) ||
                          (config.hamlib_model != config_.hamlib_model);

    config_ = config;

    if (need_reconnect && backend_) {
        // Release PTT before disconnecting
        if (ptt_active_) {
            backend_->setPtt(false);
            ptt_active_ = false;
        }
        backend_->disconnect();
        backend_.reset();
    }

    // Clear state
    release_pending_ = false;
    watchdog_triggered_ = false;
}

bool CatController::connect() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!config_.enabled) {
        last_error_ = "CAT control is disabled";
        return false;
    }

    if (config_.backend_type == CatBackendType::None) {
        last_error_ = "No backend configured";
        return false;
    }

    // Create backend if needed
    if (!backend_) {
        backend_ = createBackend();
        if (!backend_) {
            last_error_ = "Failed to create backend";
            return false;
        }
    }

    // Connect
    if (!backend_->isConnected()) {
        if (!backend_->connect()) {
            last_error_ = "Backend failed to connect: " + backend_->getStatus().error_message;
            return false;
        }
    }

    last_error_.clear();
    return true;
}

void CatController::disconnect() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (backend_) {
        // Release PTT before disconnecting
        if (ptt_active_) {
            backend_->setPtt(false);
            ptt_active_ = false;
        }
        backend_->disconnect();
        backend_.reset();
    }

    release_pending_ = false;
    watchdog_triggered_ = false;
}

bool CatController::isConnected() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return backend_ && backend_->isConnected();
}

bool CatController::assertPtt(const char* reason) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!config_.enabled) {
        return true;  // Not enabled, assume success (no-op)
    }

    // Cancel any pending release
    release_pending_ = false;

    // Already asserted?
    if (ptt_active_) {
        return true;
    }

    return doSetPtt(true, reason);
}

void CatController::releasePtt(const char* reason) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!config_.enabled || !ptt_active_) {
        return;
    }

    // Schedule deferred release
    if (config_.ptt_tail_ms > 0) {
        release_pending_ = true;
        release_deadline_ = std::chrono::steady_clock::now() +
                           std::chrono::milliseconds(config_.ptt_tail_ms);
        logPtt(false, reason ? reason : "release_scheduled");
    } else {
        // Immediate release
        doSetPtt(false, reason);
    }
}

void CatController::releasePttImmediate(const char* reason) {
    std::lock_guard<std::mutex> lock(mutex_);

    release_pending_ = false;

    if (ptt_active_) {
        doSetPtt(false, reason ? reason : "immediate_release");
    }
}

bool CatController::isPttActive() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return ptt_active_;
}

bool CatController::poll() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!config_.enabled) {
        return false;
    }

    auto now = std::chrono::steady_clock::now();
    bool watchdog_fired = false;

    // Check deferred PTT release
    if (release_pending_ && ptt_active_) {
        if (now >= release_deadline_) {
            release_pending_ = false;
            doSetPtt(false, "tail_elapsed");
        }
    }

    // Check watchdog
    if (ptt_active_ && config_.watchdog_seconds > 0 && !watchdog_triggered_) {
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
            now - ptt_assert_time_).count();

        if (elapsed >= config_.watchdog_seconds) {
            // Watchdog triggered!
            watchdog_triggered_ = true;
            fprintf(stderr, "[CAT] WATCHDOG: TX exceeded %d seconds, forcing PTT release\n",
                    config_.watchdog_seconds);

            // Force PTT off
            doSetPtt(false, "watchdog_timeout");
            release_pending_ = false;

            // Notify callback
            if (watchdog_callback_) {
                // Release lock before callback to avoid deadlock
                mutex_.unlock();
                watchdog_callback_();
                mutex_.lock();
            }

            watchdog_fired = true;
        }
    }

    return watchdog_fired;
}

bool CatController::setFrequency(uint64_t hz) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!backend_ || !backend_->isConnected()) {
        last_error_ = "Not connected";
        return false;
    }

    if (!backend_->supportsFrequency()) {
        last_error_ = "Backend does not support frequency control";
        return false;
    }

    return backend_->setFrequency(hz);
}

uint64_t CatController::getFrequency() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!backend_ || !backend_->isConnected() || !backend_->supportsFrequency()) {
        return 0;
    }

    return backend_->getFrequency();
}

bool CatController::setMode(RadioMode mode) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!backend_ || !backend_->isConnected()) {
        last_error_ = "Not connected";
        return false;
    }

    if (!backend_->supportsMode()) {
        last_error_ = "Backend does not support mode control";
        return false;
    }

    return backend_->setMode(mode);
}

RadioMode CatController::getMode() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!backend_ || !backend_->isConnected() || !backend_->supportsMode()) {
        return RadioMode::UNKNOWN;
    }

    return backend_->getMode();
}

CatStatus CatController::getStatus() const {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!backend_) {
        CatStatus status;
        status.error_message = last_error_;
        return status;
    }

    CatStatus status = backend_->getStatus();
    status.ptt_active = ptt_active_;
    return status;
}

bool CatController::supportsFrequency() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return backend_ && backend_->supportsFrequency();
}

bool CatController::supportsMode() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return backend_ && backend_->supportsMode();
}

std::string CatController::getLastError() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return last_error_;
}

int CatController::getWatchdogRemaining() const {
    std::lock_guard<std::mutex> lock(mutex_);

    if (config_.watchdog_seconds <= 0) {
        return -1;  // Disabled
    }

    if (!ptt_active_) {
        return config_.watchdog_seconds;  // Full time remaining
    }

    if (watchdog_triggered_) {
        return 0;
    }

    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::steady_clock::now() - ptt_assert_time_).count();

    int remaining = config_.watchdog_seconds - static_cast<int>(elapsed);
    return remaining > 0 ? remaining : 0;
}

void CatController::resetWatchdog() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (ptt_active_) {
        ptt_assert_time_ = std::chrono::steady_clock::now();
        watchdog_triggered_ = false;
    }
}

std::unique_ptr<CatBackend> CatController::createBackend() {
    switch (config_.backend_type) {
        case CatBackendType::SerialPtt:
            return createSerialPttBackend(config_);

        case CatBackendType::Hamlib:
            return createHamlibBackend(config_);

        case CatBackendType::KenwoodTcp:
            return createKenwoodTcpBackend(config_);

        case CatBackendType::None:
        default:
            return nullptr;
    }
}

bool CatController::doSetPtt(bool active, const char* reason) {
    if (!backend_) {
        // Try to create and connect
        backend_ = createBackend();
        if (!backend_) {
            last_error_ = "No backend available";
            return false;
        }
        if (!backend_->connect()) {
            last_error_ = "Failed to connect backend";
            backend_.reset();
            return false;
        }
    }

    if (!backend_->isConnected()) {
        if (!backend_->connect()) {
            last_error_ = "Backend not connected";
            return false;
        }
    }

    // Apply PTT with invert if configured
    bool line_state = config_.ptt_invert ? !active : active;
    if (!backend_->setPtt(line_state)) {
        last_error_ = "Failed to set PTT";
        return false;
    }

    ptt_active_ = active;

    if (active) {
        ptt_assert_time_ = std::chrono::steady_clock::now();
        watchdog_triggered_ = false;
    }

    logPtt(active, reason);
    return true;
}

void CatController::logPtt(bool active, const char* reason) {
    const char* backend_name = backend_ ? backend_->backendName() : "none";
    fprintf(stderr, "[CAT] PTT %s via %s%s%s\n",
            active ? "ON" : "OFF",
            backend_name,
            reason ? " (" : "",
            reason ? reason : "");
    if (reason && !active) {
        fprintf(stderr, ")\n");
    }
}

} // namespace cat
} // namespace ultra
