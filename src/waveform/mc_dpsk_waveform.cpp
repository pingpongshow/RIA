// MCDPSKWaveform - Implementation

#include "mc_dpsk_waveform.hpp"
#include "gui/startup_trace.hpp"
#include "ultra/logging.hpp"
#include <cmath>

namespace ultra {

MCDPSKWaveform::MCDPSKWaveform() {
    // Default: 8 carriers for balanced performance
    config_.num_carriers = 8;
    initComponents();
}

MCDPSKWaveform::MCDPSKWaveform(int num_carriers) {
    config_.num_carriers = std::max(3, std::min(20, num_carriers));
    initComponents();
}

MCDPSKWaveform::MCDPSKWaveform(const MultiCarrierDPSKConfig& config)
    : config_(config)
{
    initComponents();
}

void MCDPSKWaveform::initComponents() {
    gui::startupTrace("MCDPSKWaveform", "init-components-enter");
    modulator_ = std::make_unique<MultiCarrierDPSKModulator>(config_);
    gui::startupTrace("MCDPSKWaveform", "modulator-created");
    demodulator_ = std::make_unique<MultiCarrierDPSKDemodulator>(config_);
    gui::startupTrace("MCDPSKWaveform", "demodulator-created");
    chirp_sync_ = std::make_unique<sync::ChirpSync>(config_.getChirpConfig());
    gui::startupTrace("MCDPSKWaveform", "chirp-sync-created");
    initZCSync();
    gui::startupTrace("MCDPSKWaveform", "zc-sync-created");

    // Keep constructor logging allocation-light for older Windows runtimes.
    gui::startupTrace("MCDPSKWaveform", "pre-created-log");
    // Avoid LOG_MODEM in this early constructor path on older Win10 runtimes.
    (void)config_.num_carriers;
    (void)config_.samples_per_symbol;
    gui::startupTrace("MCDPSKWaveform", "post-created-log");
    gui::startupTrace("MCDPSKWaveform", "init-components-exit");
}

void MCDPSKWaveform::initZCSync() {
    // Configure ZC sync for DATA/CONTROL preamble (52ms vs 1200ms chirp)
    // Uses DATA frame type by default - will be selected per-frame in detectDataSync
    sync::ZCConfig zc_cfg;
    zc_cfg.sample_rate = config_.sample_rate;
    zc_cfg.sequence_length = 127;    // Prime for good autocorrelation
    zc_cfg.upsample_factor = 8;      // 8 samples per chip
    zc_cfg.num_repetitions = 2;      // 2 reps for CFO estimation
    zc_cfg.carrier_freq = 1500.0f;   // Center frequency

    // Frame type encoding via ZC root index
    zc_cfg.root_ping = 1;     // Not used (PING uses chirp)
    zc_cfg.root_pong = 3;     // Not used (PONG uses chirp)
    zc_cfg.root_data = 5;     // DATA frames
    zc_cfg.root_control = 7;  // CONTROL frames (ACK, NACK, KEEPALIVE, etc.)

    zc_sync_ = std::make_unique<sync::ZCSync>(zc_cfg);
}

WaveformCapabilities MCDPSKWaveform::getCapabilities() const {
    WaveformCapabilities caps;
    caps.supports_cfo_correction = true;    // Via dual chirp detection
    caps.supports_doppler_correction = true; // Frequency diversity
    caps.requires_pilots = false;            // Differential modulation
    caps.supports_differential = true;
    caps.min_snr_db = -3.0f;                // Reliable threshold
    caps.max_snr_db = 15.0f;                // Above this, use OFDM
    caps.max_throughput_bps = getThroughput(CodeRate::R1_4);
    caps.preamble_duration_ms = config_.chirp_duration_ms * 2 + config_.getChirpConfig().gap_ms * 2;
    return caps;
}

void MCDPSKWaveform::configure(Modulation mod, CodeRate rate) {
    modulation_ = mod;
    code_rate_ = rate;

    // MC-DPSK always uses DQPSK internally
    // The modulation parameter affects how we interpret the throughput
    if (mod != Modulation::DQPSK && mod != Modulation::DBPSK && mod != Modulation::D8PSK) {
        LOG_MODEM(WARN, "MCDPSKWaveform: Unsupported modulation %d, using DQPSK",
                  static_cast<int>(mod));
        modulation_ = Modulation::DQPSK;
    }

    // Update demodulator if modulation changed
    int prev_bps = config_.bits_per_symbol;
    if (mod == Modulation::DBPSK) {
        config_.bits_per_symbol = 1;
    } else if (mod == Modulation::D8PSK) {
        config_.bits_per_symbol = 3;
    } else {
        config_.bits_per_symbol = 2;  // DQPSK default
    }

    LOG_MODEM(INFO, "MCDPSKWaveform::configure: mod=%s, rate=%s, bits_per_symbol=%d->%d, carriers=%d",
              mod == Modulation::DBPSK ? "DBPSK" :
              mod == Modulation::DQPSK ? "DQPSK" :
              mod == Modulation::D8PSK ? "D8PSK" : "other",
              rate == CodeRate::R1_4 ? "R1/4" :
              rate == CodeRate::R1_2 ? "R1/2" : "other",
              prev_bps, config_.bits_per_symbol, config_.num_carriers);

    // Reinitialize with new config
    initComponents();
}

void MCDPSKWaveform::setFrequencyOffset(float cfo_hz) {
    cfo_hz_ = cfo_hz;
    if (demodulator_) {
        demodulator_->setCFO(cfo_hz);
    }
}

void MCDPSKWaveform::setTxFrequencyOffset(float cfo_hz) {
    // Set TX CFO in config and reinitialize
    config_.tx_cfo_hz = cfo_hz;
    initComponents();

    LOG_MODEM(INFO, "MCDPSKWaveform: TX CFO set to %.1f Hz", cfo_hz);
}

Samples MCDPSKWaveform::generatePreamble() {
    // Full preamble: Chirp (for PING/PONG cold-start)
    if (!modulator_) {
        return Samples();
    }
    return modulator_->generatePreamble();
}

Samples MCDPSKWaveform::generateDataPreamble() {
    // Data preamble: ZC + DPSK training (for DATA/CONTROL when connected)
    // ZC is 52ms vs 1200ms chirp = 23x faster
    if (!zc_sync_ || !modulator_) {
        return generatePreamble();  // Fallback to chirp
    }

    // Generate ZC preamble (DATA frame type)
    Samples zc_preamble = zc_sync_->generatePreamble(sync::ZCFrameType::DATA);

    // Generate DPSK training + reference (needed for demodulation)
    Samples dpsk_training = modulator_->generateTrainingSequence();
    Samples dpsk_ref = modulator_->generateReferenceSymbol();

    // Combine: ZC + training + ref
    Samples result;
    result.reserve(zc_preamble.size() + dpsk_training.size() + dpsk_ref.size());
    result.insert(result.end(), zc_preamble.begin(), zc_preamble.end());
    result.insert(result.end(), dpsk_training.begin(), dpsk_training.end());
    result.insert(result.end(), dpsk_ref.begin(), dpsk_ref.end());

    // Debug: Log ZC preamble generation at INFO level
    float zc_rms = 0.0f;
    for (float s : zc_preamble) zc_rms += s * s;
    zc_rms = zc_preamble.empty() ? 0.0f : std::sqrt(zc_rms / zc_preamble.size());

    LOG_MODEM(INFO, "MCDPSKWaveform: generateDataPreamble: zc=%zu (rms=%.4f) + training=%zu + ref=%zu = %zu samples (%.1f ms)",
              zc_preamble.size(), zc_rms, dpsk_training.size(), dpsk_ref.size(), result.size(),
              result.size() / config_.sample_rate * 1000.0f);

    return result;
}

Samples MCDPSKWaveform::modulate(const Bytes& encoded_data) {
    if (!modulator_) {
        return Samples();
    }
    return modulator_->modulate(encoded_data);
}

bool MCDPSKWaveform::detectSync(SampleSpan samples, SyncResult& result, float threshold) {
    if (!chirp_sync_) {
        return false;
    }

    // Use dual chirp detection for CFO-tolerant sync
    auto chirp_result = chirp_sync_->detectDualChirp(samples, threshold);

    result.detected = chirp_result.success;
    result.start_sample = chirp_result.up_chirp_start;
    result.correlation = std::max(chirp_result.up_correlation, chirp_result.down_correlation);
    result.cfo_hz = chirp_result.cfo_hz;
    result.has_training = true;  // MC-DPSK has training sequence after chirp

    if (chirp_result.success) {
        synced_ = true;
        last_cfo_ = chirp_result.cfo_hz;

        // Calculate where TRAINING starts (process() needs training+ref+data)
        // Layout: [UP-CHIRP][GAP][DOWN-CHIRP][GAP][TRAINING][REF][DATA...]
        //                                         ^-- start_sample points here
        //
        // IMPORTANT: Use down_chirp position for training_start calculation!
        // With CFO, up_chirp and down_chirp positions shift in OPPOSITE directions:
        //   up_chirp: shifts by -CFO × cfo_to_samples
        //   down_chirp: shifts by +CFO × cfo_to_samples
        // Using up_chirp_start + fixed_offset gives growing error with CFO.
        // Using down_chirp_start gives more accurate training position.
        // (Same approach as OFDMChirpWaveform)
        size_t chirp_samples = chirp_sync_->getChirpSamples();
        size_t gap_samples = static_cast<size_t>(config_.sample_rate * config_.getChirpConfig().gap_ms / 1000.0f);

        if (config_.use_dual_chirp) {
            // Training starts after down chirp + gap
            result.start_sample = chirp_result.down_chirp_start +
                                  chirp_samples + gap_samples;
        } else {
            // Single chirp
            result.start_sample = chirp_result.up_chirp_start +
                                  chirp_samples + gap_samples;
        }
        // NOTE: Do NOT add training_samples + ref_samples - process() needs them

        LOG_MODEM(INFO, "MCDPSKWaveform: Chirp detected at up=%d, down=%d, CFO=%.1f Hz, training_start=%d",
                  chirp_result.up_chirp_start, chirp_result.down_chirp_start,
                  chirp_result.cfo_hz, result.start_sample);
    }

    return result.detected;
}

bool MCDPSKWaveform::detectDataSync(SampleSpan samples, SyncResult& result,
                                     float known_cfo_hz, float threshold) {
    // Detect ZC preamble for DATA/CONTROL frames (fast sync)
    // Uses 52ms ZC vs 1200ms chirp = 23x faster
    if (!zc_sync_) {
        // Fallback to chirp detection
        LOG_MODEM(WARN, "MCDPSKWaveform: detectDataSync: zc_sync_ is null, falling back to chirp!");
        return detectSync(samples, result, threshold);
    }

    // Debug: Check signal energy in search buffer
    float sig_energy = 0.0f;
    for (size_t i = 0; i < std::min(samples.size(), size_t(5000)); i++) {
        sig_energy += samples[i] * samples[i];
    }
    sig_energy = std::sqrt(sig_energy / std::min(samples.size(), size_t(5000)));

    LOG_MODEM(DEBUG, "MCDPSKWaveform: detectDataSync: using ZC, samples=%zu, threshold=%.2f, known_cfo=%.1f, sig_rms=%.4f",
              samples.size(), threshold, known_cfo_hz, sig_energy);

    // Keep demodulator state synchronized with known CFO for downstream processing.
    if (std::abs(known_cfo_hz) > 0.1f) {
        setFrequencyOffset(known_cfo_hz);
    }

    // In connected data mode, only DATA/CONTROL roots are valid.
    // Restricting roots reduces false PING/PONG acquisitions on payload/noise.
    constexpr uint8_t DATA_CONTROL_ROOTS = sync::ZC_ROOT_MASK_DATA | sync::ZC_ROOT_MASK_CONTROL;
    bool debug_zc = false;
    auto zc_result = zc_sync_->detect(samples, threshold, debug_zc, DATA_CONTROL_ROOTS, known_cfo_hz);

    LOG_MODEM(DEBUG, "MCDPSKWaveform: ZC result: detected=%d, corr=%.3f, cfo=%.1f Hz, start=%d",
              zc_result.detected ? 1 : 0, zc_result.correlation, zc_result.cfo_hz, zc_result.start_sample);

    result.detected = zc_result.detected;
    result.correlation = zc_result.correlation;
    result.cfo_hz = zc_result.cfo_hz;
    result.has_training = true;  // DPSK training follows ZC preamble

    if (zc_result.detected) {
        synced_ = true;
        connected_ = true;  // Mark as connected (ZC only used when connected)

        // ZC.start_sample points to where data starts (after ZC preamble)
        // For MC-DPSK, this is where training sequence starts
        result.start_sample = zc_result.start_sample;

        // Update CFO estimate (combine known CFO with ZC-detected residual).
        // ZCSync downconverts with (carrier + known_cfo_hz), so zc_result.cfo_hz
        // is residual CFO relative to known CFO.
        if (std::abs(known_cfo_hz) > 0.1f) {
            last_cfo_ = known_cfo_hz + zc_result.cfo_hz;
        } else {
            last_cfo_ = zc_result.cfo_hz;
        }
        cfo_hz_ = last_cfo_;

        LOG_MODEM(INFO, "MCDPSKWaveform: ZC detected at %d, corr=%.3f, frame_type=%s, "
                  "cfo=%.1f Hz (known=%.1f + residual=%.1f)",
                  result.start_sample, result.correlation,
                  sync::zcFrameTypeToString(zc_result.frame_type),
                  last_cfo_, known_cfo_hz, zc_result.cfo_hz);
    }

    return result.detected;
}

bool MCDPSKWaveform::process(SampleSpan samples) {
    if (!demodulator_) {
        return false;
    }

    // Debug: Check signal energy at different positions
    // Training should be at start (samples[0]), ref at training_end, data after
    size_t training_samples = config_.training_symbols * config_.samples_per_symbol;
    size_t ref_samples = config_.samples_per_symbol;

    auto calcRMS = [&](size_t start, size_t len) {
        float e = 0.0f;
        for (size_t i = start; i < start + len && i < samples.size(); i++) {
            e += samples[i] * samples[i];
        }
        return std::sqrt(e / std::min(len, samples.size() - start));
    };

    LOG_MODEM(DEBUG, "MCDPSKWaveform: process: samples=%zu, training=%zu, ref=%zu",
              samples.size(), training_samples, ref_samples);
    LOG_MODEM(DEBUG, "MCDPSKWaveform: RMS: training[0]=%f, ref[%zu]=%f, data[%zu]=%f",
              calcRMS(0, 512), training_samples, calcRMS(training_samples, 512),
              training_samples + ref_samples, calcRMS(training_samples + ref_samples, 512));

    // Tell demodulator that chirp was already detected externally via detectSync()
    // This puts it in GOT_CHIRP state so it processes data directly without
    // looking for chirp in the samples
    LOG_MODEM(DEBUG, "MCDPSKWaveform: process: setting CFO=%.1f Hz in demodulator", cfo_hz_);
    demodulator_->setChirpDetected(cfo_hz_);

    // Process samples through demodulator
    bool ready = demodulator_->process(samples);

    LOG_MODEM(DEBUG, "MCDPSKWaveform: process: input_samples=%zu, ready=%d, demod_cfo=%.1f",
              samples.size(), ready, demodulator_->getEstimatedCFO());

    if (ready) {
        // Get soft bits from demodulator's internal state (computed in processGotChirp)
        soft_bits_ = demodulator_->getSoftBits();
        LOG_MODEM(DEBUG, "MCDPSKWaveform: got %zu soft bits", soft_bits_.size());
        synced_ = true;
    }

    return ready;
}

std::vector<float> MCDPSKWaveform::getSoftBits() {
    return std::move(soft_bits_);
}

void MCDPSKWaveform::reset() {
    if (demodulator_) {
        demodulator_->reset();
    }
    soft_bits_.clear();
    synced_ = false;
    // NOTE: CFO is intentionally preserved across reset() for continuous tracking
    // Use setFrequencyOffset(0) to explicitly clear if needed
}

bool MCDPSKWaveform::isSynced() const {
    return synced_ || (demodulator_ && demodulator_->isSynced());
}

bool MCDPSKWaveform::hasData() const {
    return !soft_bits_.empty() || (demodulator_ && demodulator_->hasPendingData());
}

float MCDPSKWaveform::estimatedSNR() const {
    return last_snr_;
}

float MCDPSKWaveform::estimatedCFO() const {
    if (demodulator_) {
        return demodulator_->getEstimatedCFO();
    }
    return last_cfo_;
}

std::vector<std::complex<float>> MCDPSKWaveform::getConstellationSymbols() const {
    // MC-DPSK demodulator doesn't track constellation symbols
    // For now, return empty vector
    // TODO: Add constellation tracking to MultiCarrierDPSKDemodulator if needed for GUI
    return {};
}

std::string MCDPSKWaveform::getStatusString() const {
    std::ostringstream oss;
    oss << "MC-DPSK " << config_.num_carriers << " carriers @ "
        << static_cast<int>(getThroughput(code_rate_)) << " bps";
    if (std::abs(cfo_hz_) > 0.5f) {
        oss << " (CFO=" << static_cast<int>(cfo_hz_) << " Hz)";
    }
    return oss.str();
}

float MCDPSKWaveform::getThroughput(CodeRate rate) const {
    // Raw bit rate = symbol_rate * carriers * bits_per_symbol
    float raw_bps = config_.getRawBitRate();

    // Apply code rate
    float code_ratio = 0.25f;  // Default R1/4
    switch (rate) {
        case CodeRate::R1_4: code_ratio = 0.25f; break;
        case CodeRate::R1_3: code_ratio = 0.333f; break;
        case CodeRate::R1_2: code_ratio = 0.5f; break;
        case CodeRate::R2_3: code_ratio = 0.667f; break;
        case CodeRate::R3_4: code_ratio = 0.75f; break;
        case CodeRate::R5_6: code_ratio = 0.833f; break;
        case CodeRate::R7_8: code_ratio = 0.875f; break;
        case CodeRate::AUTO: code_ratio = 0.5f; break;  // Default to R1/2
    }

    return raw_bps * code_ratio;
}

int MCDPSKWaveform::getPreambleSamples() const {
    if (chirp_sync_) {
        return static_cast<int>(chirp_sync_->getTotalSamples());
    }
    // Fallback calculation
    size_t chirp_samples = static_cast<size_t>(config_.sample_rate * config_.chirp_duration_ms / 1000.0f);
    size_t gap_samples = static_cast<size_t>(config_.sample_rate * 100.0f / 1000.0f);  // 100ms gap
    return static_cast<int>(config_.use_dual_chirp ? 2 * chirp_samples + 2 * gap_samples
                                                   : chirp_samples + gap_samples);
}

int MCDPSKWaveform::getDataPreambleSamples() const {
    // ZC preamble + DPSK training + reference symbol
    // FIX (codex.md Issue #6): Use zc_sync_->getConfig().preambleSamples() instead of hardcoded math.
    // Previous hardcoded 127*8*2=2032 was missing the gap (480 samples). Actual ZC preamble is 2512.
    if (zc_sync_) {
        int zc_samples = zc_sync_->getConfig().preambleSamples();
        int training_samples = config_.training_symbols * config_.samples_per_symbol;
        int ref_samples = config_.samples_per_symbol;
        return zc_samples + training_samples + ref_samples;
    }
    // Fallback to full chirp preamble if ZC not initialized
    return getPreambleSamples();
}

void MCDPSKWaveform::setCarrierCount(int carriers) {
    config_.num_carriers = std::max(3, std::min(20, carriers));
    initComponents();
}

void MCDPSKWaveform::setSpreadingMode(SpreadingMode mode) {
    config_.spreading_mode = mode;
    LOG_MODEM(INFO, "MCDPSKWaveform::setSpreadingMode: mode=%s, carriers=%d",
              mode == SpreadingMode::TIME_4X ? "4×" :
              mode == SpreadingMode::TIME_2X ? "2×" : "NONE",
              config_.num_carriers);
    initComponents();  // Re-initialize with new spreading mode
}

int MCDPSKWaveform::getMinSamplesForFrame() const {
    // Training symbols + reference symbol + data for 1 LDPC codeword (648 bits)
    // MC-DPSK does NOT use frame interleaving - CW0 can be decoded independently
    // to parse the header and determine how many more CWs to request
    //
    // NOTE: Training and reference are NOT spread - only data symbols get spreading.
    // The modulator's generateTrainingSequence() and generateReferenceSymbol() produce
    // fixed-length output regardless of spreading mode.
    int spreading_factor = config_.getSpreadingFactor();
    int training_samples = config_.training_symbols * config_.samples_per_symbol;  // No spreading
    int ref_samples = config_.samples_per_symbol;  // No spreading

    // Data samples for 1 LDPC codeword (648 bits) - spreading applies here
    constexpr int LDPC_BLOCK_SIZE = 648;
    int bits_per_symbol = config_.num_carriers * config_.bits_per_symbol;
    int data_symbols = (LDPC_BLOCK_SIZE + bits_per_symbol - 1) / bits_per_symbol;
    int data_samples = data_symbols * config_.samples_per_symbol * spreading_factor;

    return training_samples + ref_samples + data_samples;
}

int MCDPSKWaveform::getMinSamplesForCWCount(int num_cw) const {
    // Training + reference + data for num_cw codewords
    //
    // NOTE: Training and reference are NOT spread - only data symbols get spreading.
    int spreading_factor = config_.getSpreadingFactor();
    int training_samples = config_.training_symbols * config_.samples_per_symbol;  // No spreading
    int ref_samples = config_.samples_per_symbol;  // No spreading

    constexpr int LDPC_BLOCK_SIZE = 648;
    int bits_per_symbol = config_.num_carriers * config_.bits_per_symbol;
    int data_symbols_per_cw = (LDPC_BLOCK_SIZE + bits_per_symbol - 1) / bits_per_symbol;
    int data_samples = num_cw * data_symbols_per_cw * config_.samples_per_symbol * spreading_factor;

    return training_samples + ref_samples + data_samples;
}

float MCDPSKWaveform::getFadingIndex() const {
    if (demodulator_) {
        return demodulator_->getFadingIndex();
    }
    return 0.0f;
}

bool MCDPSKWaveform::isFading() const {
    if (demodulator_) {
        return demodulator_->isFading();
    }
    return false;
}

} // namespace ultra
