#pragma once

#include "cat_backend.hpp"
#include <memory>
#include <string>
#include <vector>

// Forward declare Hamlib types (only if Hamlib is available)
#ifdef ULTRA_HAS_HAMLIB
struct rig;
typedef struct rig RIG;
#endif

namespace ultra {
namespace cat {

// Forward declaration
struct CatConfig;

// Configuration for Hamlib backend
struct HamlibConfig {
    int model = 0;              // Hamlib rig model number (e.g., 2014=TS-2000, 3073=IC-7300)
    std::string port;           // Serial port or network address
    int baud_rate = 9600;
    int ptt_type = 0;           // 0=CAT, 1=DTR, 2=RTS
};

// Common Hamlib model IDs for reference
namespace HamlibModels {
    constexpr int DUMMY = 1;                // Hamlib dummy rig for testing
    constexpr int KENWOOD_TS2000 = 2014;
    constexpr int ICOM_IC7300 = 3073;
    constexpr int ICOM_IC7100 = 3070;
    constexpr int ICOM_IC7610 = 3078;
    constexpr int YAESU_FT991 = 1035;
    constexpr int YAESU_FTDX10 = 1042;
    constexpr int ELECRAFT_K3 = 2029;
    constexpr int ELECRAFT_KX3 = 2043;
    constexpr int FLEXRADIO_6000 = 2036;    // Note: TCP backend recommended for Flex
}

class HamlibBackend : public CatBackend {
public:
    HamlibBackend();
    explicit HamlibBackend(const HamlibConfig& config);
    ~HamlibBackend() override;

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
    const char* backendName() const override { return "Hamlib"; }

    // Configuration
    void setConfig(const HamlibConfig& config);
    const HamlibConfig& getConfig() const { return config_; }

    // Hamlib helpers
    static bool isAvailable();  // Check if Hamlib library is loaded
    static std::vector<std::pair<int, std::string>> getAvailableModels();

private:
    HamlibConfig config_;
    bool ptt_active_ = false;
    std::string last_error_;

#ifdef ULTRA_HAS_HAMLIB
    RIG* rig_ = nullptr;

    // Convert between RadioMode and Hamlib mode constants
    static int radioModeToHamlib(RadioMode mode);
    static RadioMode hamlibToRadioMode(int hamlib_mode);
#endif
};

// Factory function called by CatController
std::unique_ptr<CatBackend> createHamlibBackend(const CatConfig& config);

} // namespace cat
} // namespace ultra
