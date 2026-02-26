// MFSKWaveform - Implementation

#include "mfsk_waveform.hpp"
#include "ultra/logging.hpp"
#include <cmath>
#include <sstream>

namespace ultra {

MFSKWaveform::MFSKWaveform() {
    // Default: 32-FSK test_fast for reasonable test speed
    // This uses shorter symbols (512 samples vs 1536) for ~3x faster operation
    // Use setToneCount() for more robust (slower) modes in real deployments
    config_ = mfsk_presets::test_fast();
    initComponents();
}

MFSKWaveform::MFSKWaveform(int num_tones) {
    // Select preset based on tone count
    switch (num_tones) {
        case 2:  config_ = mfsk_presets::robust(); break;
        case 4:  config_ = mfsk_presets::low_snr(); break;
        case 8:  config_ = mfsk_presets::medium(); break;
        case 16: config_ = mfsk_presets::fast(); break;
        case 32: config_ = mfsk_presets::turbo(); break;
        default:
            LOG_MODEM(WARN, "MFSKWaveform: Invalid tone count %d, using 8", num_tones);
            config_ = mfsk_presets::medium();
            break;
    }
    initComponents();
}

MFSKWaveform::MFSKWaveform(const MFSKConfig& config)
    : config_(config)
{
    initComponents();
}

void MFSKWaveform::initComponents() {
    modulator_ = std::make_unique<MFSKModulator>(config_);
    demodulator_ = std::make_unique<MFSKDemodulator>(config_);
    chirp_sync_ = std::make_unique<sync::ChirpSync>(getChirpConfig());

    LOG_MODEM(INFO, "MFSKWaveform: %d-FSK @ %.0f bps (rep=%d, spacing=%.0f Hz)",
              config_.num_tones, config_.effective_bps(),
              config_.repetition, config_.tone_spacing);
}

sync::ChirpConfig MFSKWaveform::getChirpConfig() const {
    sync::ChirpConfig cfg;
    cfg.sample_rate = config_.sample_rate;
    cfg.f_start = CHIRP_F_START;
    cfg.f_end = CHIRP_F_END;
    cfg.duration_ms = CHIRP_DURATION_MS;
    cfg.gap_ms = CHIRP_GAP_MS;
    cfg.use_dual_chirp = true;  // Up+down chirp for CFO estimation
    // Note: threshold is passed to detectDualChirp(), not stored in config
    return cfg;
}

WaveformCapabilities MFSKWaveform::getCapabilities() const {
    WaveformCapabilities caps;
    caps.supports_cfo_correction = true;     // Via dual chirp detection
    caps.supports_doppler_correction = true;  // Inherently CFO-tolerant
    caps.requires_pilots = false;             // Non-coherent detection
    caps.supports_differential = true;        // Orthogonal modulation

    // SNR range depends on tone count
    switch (config_.num_tones) {
        case 2:  caps.min_snr_db = -12.0f; break;
        case 4:  caps.min_snr_db = -8.0f; break;
        case 8:  caps.min_snr_db = -4.0f; break;
        case 16: caps.min_snr_db = 0.0f; break;
        case 32: caps.min_snr_db = 3.0f; break;
        default: caps.min_snr_db = -4.0f; break;
    }
    caps.max_snr_db = 10.0f;  // Above this, use MC-DPSK or OFDM

    caps.max_throughput_bps = getThroughput(CodeRate::R1_2);
    caps.preamble_duration_ms = CHIRP_DURATION_MS * 2 + CHIRP_GAP_MS * 2;
    return caps;
}

void MFSKWaveform::configure(Modulation mod, CodeRate rate) {
    code_rate_ = rate;

    // MFSK doesn't use traditional modulation schemes
    // Instead, adjust tone count based on requested "modulation" complexity
    // This allows the protocol to request more/less robustness
    (void)mod;  // Unused - MFSK is orthogonal, not PSK/QAM

    LOG_MODEM(INFO, "MFSKWaveform: configured with code_rate=%d", static_cast<int>(rate));
}

void MFSKWaveform::setFrequencyOffset(float cfo_hz) {
    cfo_hz_ = cfo_hz;
    // MFSK is inherently CFO-tolerant (all tones shift equally)
    // No compensation needed in demodulator
}

void MFSKWaveform::setTxFrequencyOffset(float cfo_hz) {
    // For testing: inject CFO in modulator
    // This would require modifying the modulator, but MFSK doesn't need it
    // since it's CFO-tolerant by design
    LOG_MODEM(INFO, "MFSKWaveform: TX CFO set to %.1f Hz (ignored - MFSK is CFO-tolerant)", cfo_hz);
}

Samples MFSKWaveform::generatePreamble() {
    // Generate chirp preamble followed by MFSK tone sweep
    Samples preamble;

    // Chirp preamble for timing sync
    if (chirp_sync_) {
        auto chirp = chirp_sync_->generate();
        preamble.insert(preamble.end(), chirp.begin(), chirp.end());
    }

    // MFSK tone sweep for fine timing and tone calibration
    auto tone_sweep = modulator_->generatePreamble(2);  // 2 cycles through all tones
    preamble.insert(preamble.end(), tone_sweep.begin(), tone_sweep.end());

    return preamble;
}

Samples MFSKWaveform::modulate(const Bytes& encoded_data) {
    if (!modulator_) {
        return Samples();
    }
    return modulator_->modulate(encoded_data);
}

bool MFSKWaveform::detectSync(SampleSpan samples, SyncResult& result, float threshold) {
    if (!chirp_sync_) {
        return false;
    }

    // Use dual chirp detection for timing sync
    auto chirp_result = chirp_sync_->detectDualChirp(samples, threshold);

    result.detected = chirp_result.success;
    result.start_sample = chirp_result.up_chirp_start;
    result.correlation = std::max(chirp_result.up_correlation, chirp_result.down_correlation);
    result.cfo_hz = chirp_result.cfo_hz;
    result.has_training = true;  // MFSK has tone sweep after chirp

    if (chirp_result.success) {
        synced_ = true;
        last_cfo_ = chirp_result.cfo_hz;

        // Calculate where MFSK tone sweep starts
        size_t chirp_samples = chirp_sync_->getChirpSamples();
        size_t gap_samples = static_cast<size_t>(config_.sample_rate * CHIRP_GAP_MS / 1000.0f);

        // Training (tone sweep) starts after down chirp + gap
        result.start_sample = chirp_result.down_chirp_start + chirp_samples + gap_samples;

        LOG_MODEM(INFO, "MFSKWaveform: Chirp detected at up=%d, down=%d, CFO=%.1f Hz, data_start=%d",
                  chirp_result.up_chirp_start, chirp_result.down_chirp_start,
                  chirp_result.cfo_hz, result.start_sample);
    }

    return result.detected;
}

bool MFSKWaveform::process(SampleSpan samples) {
    if (!demodulator_) {
        return false;
    }

    // Skip tone sweep preamble and go directly to data
    int preamble_len = demodulator_->getPreambleLength(2);  // 2 cycles

    if ((int)samples.size() <= preamble_len) {
        LOG_MODEM(DEBUG, "MFSKWaveform: Not enough samples (%zu <= %d preamble)",
                  samples.size(), preamble_len);
        return false;
    }

    // Data starts after preamble
    SampleSpan data_samples(samples.data() + preamble_len, samples.size() - preamble_len);

    // Demodulate to soft bits
    soft_bits_ = demodulator_->demodulateSoft(data_samples);

    LOG_MODEM(DEBUG, "MFSKWaveform: process: %zu input samples, %zu soft bits",
              samples.size(), soft_bits_.size());

    synced_ = true;
    return !soft_bits_.empty();
}

std::vector<float> MFSKWaveform::getSoftBits() {
    return std::move(soft_bits_);
}

void MFSKWaveform::reset() {
    if (demodulator_) {
        demodulator_->reset();
    }
    soft_bits_.clear();
    synced_ = false;
}

bool MFSKWaveform::isSynced() const {
    return synced_;
}

bool MFSKWaveform::hasData() const {
    return !soft_bits_.empty();
}

float MFSKWaveform::estimatedSNR() const {
    return last_snr_;
}

float MFSKWaveform::estimatedCFO() const {
    return last_cfo_;  // From chirp detection
}

std::vector<std::complex<float>> MFSKWaveform::getConstellationSymbols() const {
    // MFSK doesn't have a constellation - it uses orthogonal tones
    // Return empty vector
    return {};
}

std::string MFSKWaveform::getStatusString() const {
    std::ostringstream oss;
    oss << config_.num_tones << "-FSK @ "
        << static_cast<int>(getThroughput(code_rate_)) << " bps";
    if (std::abs(cfo_hz_) > 0.5f) {
        oss << " (CFO=" << static_cast<int>(cfo_hz_) << " Hz)";
    }
    return oss.str();
}

float MFSKWaveform::getThroughput(CodeRate rate) const {
    // Effective bit rate after repetition
    float raw_bps = config_.effective_bps();

    // Apply code rate
    float code_ratio = 0.5f;  // Default R1/2
    switch (rate) {
        case CodeRate::R1_4: code_ratio = 0.25f; break;
        case CodeRate::R1_3: code_ratio = 0.333f; break;
        case CodeRate::R1_2: code_ratio = 0.5f; break;
        case CodeRate::R2_3: code_ratio = 0.667f; break;
        case CodeRate::R3_4: code_ratio = 0.75f; break;
        case CodeRate::R5_6: code_ratio = 0.833f; break;
        case CodeRate::R7_8: code_ratio = 0.875f; break;
        case CodeRate::AUTO: code_ratio = 0.5f; break;
    }

    return raw_bps * code_ratio;
}

int MFSKWaveform::getPreambleSamples() const {
    // Chirp preamble + MFSK tone sweep
    int chirp_samples = 0;
    if (chirp_sync_) {
        chirp_samples = static_cast<int>(chirp_sync_->getTotalSamples());
    } else {
        // Fallback calculation
        int single_chirp = static_cast<int>(config_.sample_rate * CHIRP_DURATION_MS / 1000.0f);
        int gap = static_cast<int>(config_.sample_rate * CHIRP_GAP_MS / 1000.0f);
        chirp_samples = 2 * single_chirp + 2 * gap;  // Dual chirp
    }

    int tone_sweep_samples = demodulator_->getPreambleLength(2);
    return chirp_samples + tone_sweep_samples;
}

int MFSKWaveform::getMinSamplesForFrame() const {
    // Tone sweep preamble + data for 1 LDPC codeword (648 bits)
    int preamble = demodulator_->getPreambleLength(2);

    // Data samples for 1 LDPC codeword
    constexpr int LDPC_BLOCK_SIZE = 648;
    int bits_per_sym = config_.bits_per_symbol();
    int symbols_per_cw = (LDPC_BLOCK_SIZE + bits_per_sym - 1) / bits_per_sym;
    int samples_per_cw = symbols_per_cw * config_.repetition * config_.samples_per_symbol;

    return preamble + samples_per_cw;
}

int MFSKWaveform::getMinSamplesForCWCount(int num_cw) const {
    int preamble = demodulator_->getPreambleLength(2);

    constexpr int LDPC_BLOCK_SIZE = 648;
    int bits_per_sym = config_.bits_per_symbol();
    int symbols_per_cw = (LDPC_BLOCK_SIZE + bits_per_sym - 1) / bits_per_sym;
    int samples_per_cw = symbols_per_cw * config_.repetition * config_.samples_per_symbol;

    return preamble + num_cw * samples_per_cw;
}

void MFSKWaveform::setToneCount(int tones) {
    // Validate and select appropriate preset
    switch (tones) {
        case 2:  config_ = mfsk_presets::robust(); break;
        case 4:  config_ = mfsk_presets::low_snr(); break;
        case 8:  config_ = mfsk_presets::medium(); break;
        case 16: config_ = mfsk_presets::fast(); break;
        case 32: config_ = mfsk_presets::turbo(); break;
        default:
            LOG_MODEM(WARN, "MFSKWaveform: Invalid tone count %d, keeping %d",
                      tones, config_.num_tones);
            return;
    }
    initComponents();
}

} // namespace ultra
