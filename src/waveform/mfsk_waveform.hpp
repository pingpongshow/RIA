#pragma once

// MFSKWaveform - Multi-Frequency Shift Keying waveform implementation
//
// Wraps MFSKModulator, MFSKDemodulator, and ChirpSync to provide the IWaveform interface.
//
// Features:
// - Chirp preamble for robust sync at very low SNR (-12 to +3 dB)
// - Adaptive tone count (2/4/8/16/32) for throughput vs robustness tradeoff
// - Non-coherent energy detection - inherently CFO-tolerant
// - Used for extreme low-SNR conditions where OFDM/MC-DPSK fail
//
// Performance (with R1/2 LDPC):
//   2-FSK:  ~30 bps  @ -12 dB reported SNR (-5 dB actual)
//   4-FSK:  ~45 bps  @ -8 dB reported
//   8-FSK:  ~62 bps  @ -4 dB reported
//   16-FSK: ~94 bps  @ 0 dB reported
//   32-FSK: ~156 bps @ +3 dB reported (then switch to MC-DPSK/OFDM)

#include "waveform_interface.hpp"
#include "fsk/mfsk.hpp"
#include "sync/chirp_sync.hpp"
#include <memory>

namespace ultra {

class MFSKWaveform : public IWaveform {
public:
    // Create with default configuration (8-FSK, balanced)
    MFSKWaveform();

    // Create with specific tone count (2, 4, 8, 16, or 32)
    explicit MFSKWaveform(int num_tones);

    // Create with full configuration
    explicit MFSKWaveform(const MFSKConfig& config);

    ~MFSKWaveform() override = default;

    // ========================================================================
    // IWaveform - Identity
    // ========================================================================

    std::string getName() const override { return "MFSK"; }
    protocol::WaveformMode getMode() const override { return protocol::WaveformMode::MFSK; }
    WaveformCapabilities getCapabilities() const override;

    // ========================================================================
    // IWaveform - Configuration
    // ========================================================================

    void configure(Modulation mod, CodeRate rate) override;
    void setFrequencyOffset(float cfo_hz) override;
    void setTxFrequencyOffset(float cfo_hz) override;
    Modulation getModulation() const override { return Modulation::DBPSK; }  // MFSK is orthogonal, closest is DBPSK
    CodeRate getCodeRate() const override { return code_rate_; }
    float getFrequencyOffset() const override { return cfo_hz_; }

    // ========================================================================
    // IWaveform - TX
    // ========================================================================

    Samples generatePreamble() override;
    Samples modulate(const Bytes& encoded_data) override;

    // ========================================================================
    // IWaveform - RX
    // ========================================================================

    bool detectSync(SampleSpan samples, SyncResult& result, float threshold = 0.15f) override;
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
    int getCarrierCount() const override { return config_.num_tones; }  // "carriers" = tones for MFSK
    float getThroughput(CodeRate rate) const override;
    int getSamplesPerSymbol() const override { return config_.samples_per_symbol; }
    int getPreambleSamples() const override;
    int getMinSamplesForFrame() const override;
    int getMinSamplesForCWCount(int num_cw) const override;

    // ========================================================================
    // MFSK Specific
    // ========================================================================

    // Set number of tones (2, 4, 8, 16, or 32)
    void setToneCount(int tones);

    // Get configuration
    const MFSKConfig& getConfig() const { return config_; }

    // ========================================================================
    // Channel Quality / Fading Detection (IWaveform override)
    // ========================================================================

    // MFSK is inherently robust to fading - returns low value
    float getFadingIndex() const override { return 0.1f; }

    // MFSK handles fading well via frequency diversity
    bool isFading() const override { return false; }

private:
    void initComponents();

    // Chirp sync configuration for MFSK
    sync::ChirpConfig getChirpConfig() const;

    MFSKConfig config_;
    std::unique_ptr<MFSKModulator> modulator_;
    std::unique_ptr<MFSKDemodulator> demodulator_;
    std::unique_ptr<sync::ChirpSync> chirp_sync_;

    // State
    CodeRate code_rate_ = CodeRate::R1_2;
    float cfo_hz_ = 0.0f;
    bool synced_ = false;
    std::vector<float> soft_bits_;
    float last_snr_ = 0.0f;
    float last_cfo_ = 0.0f;

    // Chirp parameters
    static constexpr float CHIRP_DURATION_MS = 500.0f;
    static constexpr float CHIRP_F_START = 300.0f;
    static constexpr float CHIRP_F_END = 2700.0f;
    static constexpr float CHIRP_GAP_MS = 50.0f;
};

} // namespace ultra
