#pragma once

// MCDPSKWaveform - Multi-Carrier DPSK waveform implementation
//
// Wraps MultiCarrierDPSKModulator, MultiCarrierDPSKDemodulator, and ChirpSync
// to provide the IWaveform interface.
//
// Features:
// - Chirp preamble for robust sync at low SNR (-3 to +10 dB)
// - Multi-carrier DQPSK with frequency diversity
// - CFO-tolerant via complex correlation chirp detection
// - Used for connection establishment (CONNECT/DISCONNECT)

#include "waveform_interface.hpp"
#include "psk/multi_carrier_dpsk.hpp"
#include "sync/chirp_sync.hpp"
#include "sync/zc_sync.hpp"
#include <memory>

namespace ultra {

class MCDPSKWaveform : public IWaveform {
public:
    // Create with default configuration (8 carriers)
    MCDPSKWaveform();

    // Create with specific carrier count
    explicit MCDPSKWaveform(int num_carriers);

    // Create with full configuration
    explicit MCDPSKWaveform(const MultiCarrierDPSKConfig& config);

    ~MCDPSKWaveform() override = default;

    // ========================================================================
    // IWaveform - Identity
    // ========================================================================

    std::string getName() const override { return "MC-DPSK"; }
    protocol::WaveformMode getMode() const override { return protocol::WaveformMode::MC_DPSK; }
    WaveformCapabilities getCapabilities() const override;

    // ========================================================================
    // IWaveform - Configuration
    // ========================================================================

    void configure(Modulation mod, CodeRate rate) override;
    void setFrequencyOffset(float cfo_hz) override;
    void setTxFrequencyOffset(float cfo_hz) override;
    Modulation getModulation() const override { return modulation_; }
    CodeRate getCodeRate() const override { return code_rate_; }
    float getFrequencyOffset() const override { return cfo_hz_; }

    // ========================================================================
    // IWaveform - TX
    // ========================================================================

    // Full preamble: Chirp (for PING/PONG, cold-start)
    Samples generatePreamble() override;

    // Data preamble: ZC (for DATA/CONTROL when connected, 23x faster)
    // Uses ZC sequence which is 52ms vs 1200ms chirp
    Samples generateDataPreamble() override;

    // This waveform supports fast ZC preamble for DATA frames
    bool supportsDataPreamble() const override { return true; }

    Samples modulate(const Bytes& encoded_data) override;

    // ========================================================================
    // IWaveform - RX
    // ========================================================================

    // Detect sync - uses chirp for PING/PONG (cold-start)
    bool detectSync(SampleSpan samples, SyncResult& result, float threshold = 0.15f) override;

    // Detect ZC sync for DATA/CONTROL frames (when connected)
    // Uses fast ZC preamble (52ms vs 1200ms chirp)
    bool detectDataSync(SampleSpan samples, SyncResult& result,
                        float known_cfo_hz = 0.0f, float threshold = 0.2f) override;
    bool process(SampleSpan samples) override;
    std::vector<float> getSoftBits() override;
    void reset() override;

    // ========================================================================
    // IWaveform - Status
    // ========================================================================

    bool isSynced() const override;
    bool hasData() const override;
    float estimatedSNR() const override;
    float estimatedCFO() const override;
    std::vector<std::complex<float>> getConstellationSymbols() const override;

    // ========================================================================
    // IWaveform - GUI Display
    // ========================================================================

    std::string getStatusString() const override;
    int getCarrierCount() const override { return config_.num_carriers; }
    float getThroughput(CodeRate rate) const override;
    int getSamplesPerSymbol() const override { return config_.samples_per_symbol; }
    int getPreambleSamples() const override;
    int getDataPreambleSamples() const override;  // ZC + DPSK training (52ms)
    int getMinSamplesForFrame() const override;
    int getMinSamplesForCWCount(int num_cw) const override;

    // ========================================================================
    // MC-DPSK Specific
    // ========================================================================

    // Set number of carriers (3-20)
    void setCarrierCount(int carriers);

    // Set time-domain spreading mode (NONE, TIME_2X, TIME_4X)
    // Must be called before modulate()/process() to take effect
    void setSpreadingMode(SpreadingMode mode);

    // Get current spreading mode
    SpreadingMode getSpreadingMode() const { return config_.spreading_mode; }

    // Get configuration
    const MultiCarrierDPSKConfig& getConfig() const { return config_; }

    // ========================================================================
    // Channel Quality / Fading Detection (IWaveform override)
    // ========================================================================

    // Get fading index (0-1, higher = more fading)
    // Based on per-carrier signal magnitude variance
    float getFadingIndex() const override;

    // Check if channel appears to be fading (uses threshold 0.65)
    bool isFading() const override;

private:
    void initComponents();
    void initZCSync();

    MultiCarrierDPSKConfig config_;
    std::unique_ptr<MultiCarrierDPSKModulator> modulator_;
    std::unique_ptr<MultiCarrierDPSKDemodulator> demodulator_;
    std::unique_ptr<sync::ChirpSync> chirp_sync_;
    std::unique_ptr<sync::ZCSync> zc_sync_;  // For fast DATA/CONTROL preamble

    // State
    Modulation modulation_ = Modulation::DQPSK;
    CodeRate code_rate_ = CodeRate::R1_4;
    float cfo_hz_ = 0.0f;
    bool synced_ = false;
    bool connected_ = false;  // True after handshake, enables ZC preamble
    std::vector<float> soft_bits_;
    float last_snr_ = 0.0f;
    float last_cfo_ = 0.0f;
};

} // namespace ultra
