#pragma once

#include <string>
#include <cstdint>
#include <vector>

namespace ultra {
namespace cat {

// Radio operating modes
enum class RadioMode {
    USB,        // Upper sideband
    LSB,        // Lower sideband
    AM,         // Amplitude modulation
    FM,         // Frequency modulation
    CW,         // Continuous wave
    CW_R,       // CW reverse
    RTTY,       // RTTY
    RTTY_R,     // RTTY reverse
    DATA,       // Data mode (AFSK, digital)
    DATA_R,     // Data reverse
    UNKNOWN
};

// CAT backend types
enum class CatBackendType {
    None = 0,       // No CAT control
    SerialPtt = 1,  // Serial DTR/RTS PTT only (no freq/mode)
    Hamlib = 2,     // Hamlib library (200+ radios)
    KenwoodTcp = 3  // Kenwood protocol over TCP (FlexRadio SmartSDR)
};

// Backend status
struct CatStatus {
    bool connected = false;
    uint64_t frequency_hz = 0;
    RadioMode mode = RadioMode::UNKNOWN;
    bool ptt_active = false;
    std::string error_message;
};

// Abstract CAT backend interface
class CatBackend {
public:
    virtual ~CatBackend() = default;

    // Connection management
    virtual bool connect() = 0;
    virtual void disconnect() = 0;
    virtual bool isConnected() const = 0;

    // PTT control (required for all backends)
    virtual bool setPtt(bool active) = 0;
    virtual bool getPtt() const = 0;

    // Frequency control (optional - may return false if not supported)
    virtual bool setFrequency(uint64_t hz) = 0;
    virtual uint64_t getFrequency() = 0;
    virtual bool supportsFrequency() const = 0;

    // Mode control (optional - may return false if not supported)
    virtual bool setMode(RadioMode mode) = 0;
    virtual RadioMode getMode() = 0;
    virtual bool supportsMode() const = 0;

    // Status and diagnostics
    virtual CatStatus getStatus() const = 0;
    virtual const char* backendName() const = 0;
};

// String conversions
const char* radioModeToString(RadioMode mode);
RadioMode stringToRadioMode(const std::string& str);
const char* backendTypeToString(CatBackendType type);
CatBackendType stringToBackendType(const std::string& str);

// Get list of backend type names for UI
std::vector<const char*> getBackendTypeNames();

} // namespace cat
} // namespace ultra
