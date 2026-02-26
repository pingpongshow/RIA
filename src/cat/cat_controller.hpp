#pragma once

#include "cat_backend.hpp"
#include <memory>
#include <mutex>
#include <chrono>
#include <functional>
#include <string>

namespace ultra {
namespace cat {

// Configuration for CAT controller
struct CatConfig {
    bool enabled = false;
    CatBackendType backend_type = CatBackendType::None;

    // SerialPTT settings
    std::string serial_port;
    int serial_baud = 9600;
    int ptt_line = 0;           // 0=DTR, 1=RTS
    bool ptt_invert = false;

    // Hamlib settings
    int hamlib_model = 0;       // Hamlib rig model ID

    // KenwoodTCP settings
    std::string tcp_host = "localhost";
    uint16_t tcp_port = 4532;
    int flex_slice = 0;

    // Timing
    int ptt_lead_ms = 50;       // Delay after PTT assert before TX audio
    int ptt_tail_ms = 50;       // Hold PTT after TX audio ends

    // Watchdog
    int watchdog_seconds = 120; // Max TX duration (0=disabled)
};

// Watchdog callback type
using WatchdogCallback = std::function<void()>;

class CatController {
public:
    CatController();
    ~CatController();

    // Configuration
    void configure(const CatConfig& config);
    const CatConfig& getConfig() const { return config_; }

    // Connection management
    bool connect();
    void disconnect();
    bool isConnected() const;
    bool isEnabled() const { return config_.enabled; }

    // PTT control with timing
    // Returns true if PTT was successfully asserted
    bool assertPtt(const char* reason = nullptr);

    // Schedule PTT release after tail delay
    void releasePtt(const char* reason = nullptr);

    // Immediate PTT release (bypasses tail delay)
    void releasePttImmediate(const char* reason = nullptr);

    // Check if PTT is currently asserted
    bool isPttActive() const;

    // Called from main loop to handle deferred PTT release and watchdog
    // Returns true if watchdog was triggered
    bool poll();

    // Frequency/mode pass-through (if backend supports)
    bool setFrequency(uint64_t hz);
    uint64_t getFrequency();
    bool setMode(RadioMode mode);
    RadioMode getMode();

    // Status
    CatStatus getStatus() const;
    bool supportsFrequency() const;
    bool supportsMode() const;
    std::string getLastError() const;

    // Watchdog
    void setWatchdogCallback(WatchdogCallback cb) { watchdog_callback_ = std::move(cb); }
    int getWatchdogRemaining() const;  // Seconds remaining, -1 if disabled, 0 if triggered
    void resetWatchdog();              // Reset watchdog timer (e.g., on new TX activity)

private:
    CatConfig config_;
    std::unique_ptr<CatBackend> backend_;
    mutable std::mutex mutex_;
    std::string last_error_;

    // PTT state
    bool ptt_active_ = false;
    std::chrono::steady_clock::time_point ptt_assert_time_;

    // Deferred release
    bool release_pending_ = false;
    std::chrono::steady_clock::time_point release_deadline_;

    // Watchdog
    WatchdogCallback watchdog_callback_;
    bool watchdog_triggered_ = false;

    // Backend factory
    std::unique_ptr<CatBackend> createBackend();

    // Internal helpers
    bool doSetPtt(bool active, const char* reason);
    void logPtt(bool active, const char* reason);
};

} // namespace cat
} // namespace ultra
