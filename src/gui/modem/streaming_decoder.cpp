// StreamingDecoder - Continuous correlation receiver
//
// Architecture:
// - feedAudio(): Audio thread writes to ring buffer (fast, no processing)
// - processBuffer(): Decode thread runs state machine
//
// State machine:
//   SEARCHING → SYNC_FOUND → DECODING → SEARCHING
//
// Continuous correlation:
// - Search with small steps (100ms) to catch chirps quickly
// - Use RMS check to skip empty sections faster
// - FFT-based correlation for speed when signal present

#include "streaming_decoder.hpp"
#include "gui/startup_trace.hpp"
#include "waveform/ofdm_cox_waveform.hpp"
#include "waveform/ofdm_chirp_waveform.hpp"
#include "waveform/mfsk_waveform.hpp"
#include "waveform/mc_dpsk_waveform.hpp"
#include "fec/frame_interleaver.hpp"  // Frame-level interleaving for 4-CW frames
#include "fec/burst_interleaver.hpp"  // Burst-level long interleaver (4-frame groups)
#include "ultra/fec.hpp"              // LDPCDecoder for robust single-CW decode
#include "fec/ldpc_codec.hpp"         // getRecommendedIterations
#include "ultra/logging.hpp"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <fstream>

namespace ultra {
namespace gui {

namespace v2 = protocol::v2;

namespace {

// Return a conservative 1-CW control-frame sample requirement for connected OFDM.
// If data profile is high-order, the robust control profile (DQPSK R1/4) may need
// more symbols than the current data profile.
size_t getOFDMControlFrameSamples(IWaveform* waveform, Modulation data_mod, CodeRate data_rate) {
    if (!waveform) {
        return 0;
    }

    size_t default_samples = static_cast<size_t>(waveform->getMinSamplesForControlFrame());
    if (data_mod == Modulation::DQPSK && data_rate == CodeRate::R1_4) {
        return default_samples;
    }

    // Avoid waveform reconfigure here (it recreates internal DSP state and can
    // clear constellation history). Estimate robust control size analytically.
    const int carriers = waveform->getCarrierCount();
    const int samples_per_symbol = waveform->getSamplesPerSymbol();
    if (carriers <= 0 || samples_per_symbol <= 0) {
        return default_samples;
    }

    constexpr int pilot_spacing = 10;  // Differential DQPSK control profile
    const int pilot_count = (carriers + pilot_spacing - 1) / pilot_spacing;
    const int data_carriers = std::max(1, carriers - pilot_count);
    const int bits_per_symbol = data_carriers * 2;  // DQPSK
    const int data_symbols = (v2::LDPC_CODEWORD_BITS + bits_per_symbol - 1) / bits_per_symbol;
    const size_t robust_samples = static_cast<size_t>(2 + data_symbols) *
                                  static_cast<size_t>(samples_per_symbol);

    return std::max(default_samples, robust_samples);
}

}  // namespace

// ============================================================================
// DEBUG: Buffer snapshot for external analysis
// ============================================================================
// Dumps buffer contents to .f32 files at key sample counts
// Use: sox -t f32 -r 48000 -c 1 snapshot_*.f32 snapshot_*.wav
// Or: audacity can import raw 32-bit float

static bool g_debug_dumps_enabled = false;  // Set to true for debugging
static const char* g_dump_prefix = "/tmp/sd_debug";  // Dump file prefix

static void dumpBufferSnapshot(const std::vector<float>& buffer, size_t write_pos,
                                size_t total_fed, const std::string& label) {
    if (!g_debug_dumps_enabled) return;

    // Create filename with label and sample count
    char filename[256];
    snprintf(filename, sizeof(filename), "%s_%s_%zu.f32", g_dump_prefix, label.c_str(), total_fed);

    std::ofstream file(filename, std::ios::binary);
    if (!file) {
        LOG_MODEM(WARN, "StreamingDecoder: Failed to create dump file: %s", filename);
        return;
    }

    // Dump entire buffer in linear order (unwrap circular buffer)
    // Start from oldest data (write_pos) and wrap around
    size_t buf_size = buffer.size();
    size_t valid_samples = std::min(total_fed, buf_size);

    // For simplicity, dump from position 0 to write_pos (most recent data)
    // This is what the search sees
    if (total_fed < buf_size) {
        // Buffer hasn't wrapped yet - dump from 0 to write_pos
        file.write(reinterpret_cast<const char*>(buffer.data()), write_pos * sizeof(float));
    } else {
        // Buffer wrapped - dump from write_pos to end, then 0 to write_pos
        file.write(reinterpret_cast<const char*>(buffer.data() + write_pos),
                   (buf_size - write_pos) * sizeof(float));
        file.write(reinterpret_cast<const char*>(buffer.data()),
                   write_pos * sizeof(float));
    }

    file.close();

    // Also compute and log some stats
    float rms = 0, max_val = 0;
    for (size_t i = 0; i < valid_samples && i < 10000; i++) {
        size_t idx = (total_fed < buf_size) ? i : ((write_pos + i) % buf_size);
        float s = buffer[idx];
        rms += s * s;
        max_val = std::max(max_val, std::abs(s));
    }
    rms = std::sqrt(rms / std::min(valid_samples, size_t(10000)));

    LOG_MODEM(DEBUG, "StreamingDecoder: Dumped %s: %zu samples to %s (RMS=%.4f, peak=%.4f)",
              label.c_str(), valid_samples, filename, rms, max_val);
}

StreamingDecoder::StreamingDecoder() {
    startupTrace("StreamingDecoder", "ctor-enter");
    buffer_.resize(MAX_BUFFER_SAMPLES, 0.0f);
    startupTrace("StreamingDecoder", "buffer-resized");
    waveform_ = WaveformFactory::create(protocol::WaveformMode::MC_DPSK);
    startupTrace("StreamingDecoder", "waveform-created");
    if (waveform_) {
        // Disconnected handshake/control defaults: robust DBPSK R1/4.
        waveform_->configure(current_modulation_, code_rate_);
    }
    interleaver_ = std::make_unique<ChannelInterleaver>(16, v2::LDPC_CODEWORD_BITS);
    startupTrace("StreamingDecoder", "interleaver-created");
    codec_ = fec::CodecFactory::create(fec::CodecType::LDPC, CodeRate::R1_4);
    startupTrace("StreamingDecoder", "codec-created");

    // Initialize CSS sync for frame-type detection (optional, disabled by default)
    sync::CSSConfig css_config;
    css_config.sample_rate = 48000.0f;
    css_config.f_start = 300.0f;
    css_config.f_end = 2700.0f;
    css_config.duration_ms = 500.0f;
    css_config.gap_ms = 100.0f;
    css_config.num_shifts = 4;
    css_config.num_chirps = 1;  // Single chirp for frame type
    css_sync_ = std::make_unique<sync::CSSSync>(css_config);
    startupTrace("StreamingDecoder", "css-sync-created");

    // Initialize HARQ Chase Combining cache (enabled by default)
    fec::ChaseCache::Config chase_config;
    chase_config.enabled = true;
    chase_config.max_entries = 16;
    chase_config.entry_ttl = std::chrono::milliseconds(30000);
    chase_cache_ = std::make_unique<fec::ChaseCache>(chase_config);
    startupTrace("StreamingDecoder", "chase-cache-created");

    LOG_MODEM(INFO, "StreamingDecoder: Initialized (buffer=%zu samples, css=%s, chase=%s)",
              MAX_BUFFER_SAMPLES, css_enabled_ ? "enabled" : "disabled",
              chase_cache_->isEnabled() ? "enabled" : "disabled");
    startupTrace("StreamingDecoder", "ctor-exit");
}

StreamingDecoder::~StreamingDecoder() {
    stop();
}

void StreamingDecoder::setBurstInterleaveGroupSize(int size) {
    burst_group_size_ = std::clamp(size, 2, 8);
}

// ============================================================================
// AUDIO THREAD - Just buffer samples, nothing else
// ============================================================================

void StreamingDecoder::feedAudio(const float* samples, size_t count) {
    if (!samples || count == 0) return;

    std::lock_guard<std::mutex> lock(buffer_mutex_);

    size_t prev_total = total_fed_;

    // Check for buffer overflow - would we overwrite unsearched data?
    // Calculate how much unsearched data we have
    auto computeUnsearched = [this](size_t write_pos, size_t corr_pos) -> size_t {
        // Before first wrap, ring arithmetic is invalid if corr_pos exceeds write_pos.
        // Treat that as cursor drift and clamp instead of reporting near-full backlog.
        if (total_fed_ < MAX_BUFFER_SAMPLES) {
            return (write_pos >= corr_pos) ? (write_pos - corr_pos) : 0;
        }
        return (write_pos >= corr_pos)
            ? (write_pos - corr_pos)
            : (MAX_BUFFER_SAMPLES - corr_pos + write_pos);
    };

    if (total_fed_ < MAX_BUFFER_SAMPLES && correlation_pos_ > write_pos_) {
        const size_t old_corr = correlation_pos_;
        const size_t old_write = write_pos_;
        correlation_pos_ = write_pos_;

        // This invariant violation indicates stale decoder state (typically after
        // rapid RX reset/mode transitions while audio is still flowing). Drop any
        // in-flight frame context and force a clean resync from current audio.
        const bool had_inflight_state = (state_ != DecoderState::SEARCHING) ||
                                        (pending_total_cw_ != 0) ||
                                        !burst_soft_buffer_.empty();
        state_ = DecoderState::SEARCHING;
        pending_total_cw_ = 0;
        burst_blocks_decoded_ = 0;
        burst_soft_buffer_.clear();
        sync_reject_streak_ = 0;

        LOG_MODEM(WARN,
                  "[%s] Correlation cursor pre-wrap drift (corr=%zu > write=%zu), "
                  "clamped and reset state (inflight=%d)",
                  log_prefix_.c_str(), old_corr, old_write, had_inflight_state ? 1 : 0);
    }

    size_t unsearched = computeUnsearched(write_pos_, correlation_pos_);

    // Guard against pointer drift. If correlation_pos_ gets ahead of write_pos_
    // in wrapped space, unsearched can appear "almost full" and trigger false
    // overflow storms. Snap search pointer to newest audio in that case.
    if (total_fed_ >= MAX_BUFFER_SAMPLES &&
        unsearched > (MAX_BUFFER_SAMPLES - CORR_INVARIANT_GUARD)) {
        correlation_pos_ = write_pos_;
        unsearched = 0;
        LOG_MODEM(WARN, "[%s] Correlation pointer drift detected, resetting search cursor",
                  log_prefix_.c_str());
    }

    // If adding these samples would overflow, drop backlog aggressively enough
    // to recover in one step. Small drops (~1k) cause repeated overflow storms.
    if (unsearched + count >= MAX_BUFFER_SAMPLES) {
        const size_t incoming_total = unsearched + count;
        const size_t target_after_write = std::min(OVERFLOW_RECOVERY_KEEP, MAX_BUFFER_SAMPLES - 1);
        size_t need_to_drop = 0;
        if (incoming_total > target_after_write) {
            need_to_drop = std::min(unsearched, incoming_total - target_after_write);
        }

        correlation_pos_ = (correlation_pos_ + need_to_drop) % MAX_BUFFER_SAMPLES;

        // Once overloaded, any in-flight frame context is stale. Force a clean
        // resync from current audio instead of chasing old sync positions.
        bool reset_decode_state = false;
        if (state_ != DecoderState::SEARCHING) {
            state_ = DecoderState::SEARCHING;
            pending_total_cw_ = 0;
            burst_blocks_decoded_ = 0;
            burst_soft_buffer_.clear();
            reset_decode_state = true;
        }

        overflow_events_++;
        {
            std::lock_guard<std::mutex> slock(stats_mutex_);
            stats_.buffer_overflows = overflow_events_;
            stats_.overflow_samples_dropped += need_to_drop;
            if (reset_decode_state) {
                stats_.overflow_state_resets++;
            }
        }
        if (overflow_events_ <= 3 || (overflow_events_ % 25) == 0) {
            LOG_MODEM(WARN, "[%s] Buffer overflow, dropped %zu unsearched samples (corr_pos=%zu, keep=%zu, state_reset=%d, total=%llu)",
                      log_prefix_.c_str(), need_to_drop, correlation_pos_, target_after_write,
                      reset_decode_state ? 1 : 0,
                      static_cast<unsigned long long>(overflow_events_));
        }
    }

    // Write samples to circular buffer
    for (size_t i = 0; i < count; i++) {
        buffer_[write_pos_] = samples[i];
        write_pos_ = (write_pos_ + 1) % MAX_BUFFER_SAMPLES;
    }

    total_fed_ += count;

    // Update backlog telemetry for UI/CLI diagnostics.
    if (total_fed_ < MAX_BUFFER_SAMPLES && correlation_pos_ > write_pos_) {
        correlation_pos_ = write_pos_;
    }
    size_t unsearched_after = computeUnsearched(write_pos_, correlation_pos_);
    float backlog_ms = (static_cast<float>(unsearched_after) * 1000.0f) / 48000.0f;
    size_t used_samples = std::min(total_fed_, MAX_BUFFER_SAMPLES);
    float fill_pct = 100.0f * static_cast<float>(used_samples) /
                     static_cast<float>(MAX_BUFFER_SAMPLES);
    {
        std::lock_guard<std::mutex> slock(stats_mutex_);
        stats_.current_unsearched_samples = unsearched_after;
        stats_.peak_unsearched_samples = std::max<uint64_t>(stats_.peak_unsearched_samples,
                                                            static_cast<uint64_t>(unsearched_after));
        stats_.backlog_ms = backlog_ms;
        stats_.peak_backlog_ms = std::max(stats_.peak_backlog_ms, backlog_ms);
        stats_.buffer_fill_percent = fill_pct;
    }

    // DEBUG: Dump buffer snapshots at key sample counts
    // These help analyze what audio is actually in the buffer
    // Chirp structure: 7200 lead-in + 24000 up + 4800 gap + 24000 down + 4800 trail = 64800
    // So we should have full chirp around 72000 samples (64800 + margin)
    auto crossedThreshold = [prev_total, this](size_t threshold) {
        return prev_total < threshold && total_fed_ >= threshold;
    };

    if (crossedThreshold(24000)) {
        dumpBufferSnapshot(buffer_, write_pos_, total_fed_, "early_24k");
    }
    if (crossedThreshold(48000)) {
        dumpBufferSnapshot(buffer_, write_pos_, total_fed_, "mid_48k");
    }
    if (crossedThreshold(72000)) {
        dumpBufferSnapshot(buffer_, write_pos_, total_fed_, "full_chirp_72k");
    }
    if (crossedThreshold(96000)) {
        dumpBufferSnapshot(buffer_, write_pos_, total_fed_, "late_96k");
    }

    // Log at key thresholds to track audio arrival
    float audio_sec = total_fed_ / 48000.0f;
    auto crossedSecond = [prev_total, this](float sec) {
        size_t threshold = static_cast<size_t>(sec * 48000);
        return prev_total < threshold && total_fed_ >= threshold;
    };

    // Log every 0.5 seconds of audio
    if (crossedSecond(0.5f) || crossedSecond(1.0f) || crossedSecond(1.5f) ||
        crossedSecond(2.0f) || crossedSecond(2.5f) || crossedSecond(3.0f)) {
        float rms = 0.0f;
        for (size_t i = 0; i < count; i++) rms += samples[i] * samples[i];
        rms = std::sqrt(rms / count);
        LOG_MODEM(INFO, "[%s] feed: audio=%.2fs, RMS=%.4f",
                  log_prefix_.c_str(), audio_sec, rms);
    }

    // Wake up decode thread
    new_data_available_ = true;
    data_cv_.notify_one();
}

// ============================================================================
// DECODE THREAD - State machine for continuous correlation
// ============================================================================

void StreamingDecoder::processBuffer() {
    // Wait for new data or timeout
    bool woke_with_data = false;
    {
        std::unique_lock<std::mutex> lock(buffer_mutex_);
        woke_with_data = data_cv_.wait_for(lock, std::chrono::milliseconds(50), [this] {
            return shutdown_.load() || new_data_available_;
        });
        if (shutdown_.load()) return;
        if (!woke_with_data) return;  // Timeout with no new audio; avoid stale re-search churn
        new_data_available_ = false;  // Clear flag after waking
    }

    switch (state_) {
        case DecoderState::SEARCHING:
            searchForSync();
            break;

        case DecoderState::SYNC_FOUND:
            checkIfReadyToDecode();
            break;

        case DecoderState::DECODING:
            decodeCurrentFrame();
            break;

        case DecoderState::BURST_ACCUMULATING:
            accumulateBurstFrames();
            break;
    }
}

void StreamingDecoder::searchForSync() {
    size_t preamble = 0;
    bool waveform_supports_data_preamble = false;
    size_t waveform_data_preamble = 0;
    {
        std::lock_guard<std::mutex> wlock(waveform_mutex_);
        if (!waveform_) return;
        preamble = static_cast<size_t>(waveform_->getPreambleSamples());
        waveform_supports_data_preamble = waveform_->supportsDataPreamble();
        if (waveform_supports_data_preamble) {
            waveform_data_preamble = static_cast<size_t>(waveform_->getDataPreambleSamples());
        }
    }

    // Save generation counter - if reset() is called during our search,
    // we'll detect it and discard our results
    uint32_t gen_at_start = reset_generation_.load();

    // Search buffer sizing depends on sync mode:
    // - Chirp sync (disconnected): needs ~120k samples for dual chirp correlation
    // - Light sync (connected, LTS): needs only ~24k samples (LTS is ~1024 samples)
    // Using a smaller buffer when connected cuts per-hop latency from ~2.8s to <1s
    constexpr size_t CHIRP_MAX_SEARCH = 120000;   // ~2.5s for dual chirp detection
    constexpr size_t LIGHT_SEARCH_MIN = 9600;     // ~0.20s minimum for light sync

    size_t chirp_min_search = std::min(preamble + 65000, CHIRP_MAX_SEARCH);

    // Use waveform's data preamble size for connected mode search sizing.
    // For connected MC-DPSK (ZC), keep search windows tighter to reduce
    // per-search latency and prevent cursor lag.
    size_t min_search;
    bool weak_mc_dpsk_profile = connected_ &&
                                (mode_ == protocol::WaveformMode::MC_DPSK) &&
                                (current_modulation_ == Modulation::DBPSK) &&
                                (spreading_mode_ != SpreadingMode::NONE);
    bool connected_light_sync = connected_ && waveform_supports_data_preamble && !weak_mc_dpsk_profile;
    bool connected_zc_mode = connected_light_sync && (mode_ == protocol::WaveformMode::MC_DPSK);
    if (connected_light_sync) {
        size_t data_preamble = waveform_data_preamble;
        if (connected_zc_mode) {
            // ZC preamble is short (~52ms). Large windows only add compute delay and stale scans.
            constexpr size_t ZC_SEARCH_MARGIN = 24000;  // ~500ms margin
            constexpr size_t ZC_MAX_SEARCH = 48000;     // ~1.0s cap
            min_search = std::min(data_preamble + ZC_SEARCH_MARGIN, ZC_MAX_SEARCH);
        } else {
            // OFDM/LTS path retains larger correlation margin.
            min_search = std::min(data_preamble + 65000, CHIRP_MAX_SEARCH);
        }
        // But ensure minimum of LIGHT_SEARCH_MIN for true light sync modes
        min_search = std::max(min_search, LIGHT_SEARCH_MIN);
    } else {
        min_search = chirp_min_search;
    }

    std::vector<float> search_buffer;
    size_t search_start;

    {
        std::lock_guard<std::mutex> lock(buffer_mutex_);

        float audio_sec = total_fed_ / 48000.0f;

        // Need minimum samples before we can search
        if (total_fed_ < min_search) {
            static int skip_count = 0;
            if (++skip_count % 50 == 1)
                LOG_MODEM(INFO, "[%s] searchForSync: SKIP not enough samples, total=%.2fs, need=%.2fs",
                          log_prefix_.c_str(), audio_sec, min_search / 48000.0f);
            return;
        }

        // Initialize correlation_pos_ if needed
        if (correlation_pos_ == 0 && total_fed_ > 0) {
            if (total_fed_ < MAX_BUFFER_SAMPLES) {
                correlation_pos_ = 0;
            } else {
                correlation_pos_ = write_pos_;
            }
        }

        // Calculate unsearched data available
        size_t unsearched;
        if (write_pos_ >= correlation_pos_) {
            unsearched = write_pos_ - correlation_pos_;
        } else {
            unsearched = MAX_BUFFER_SAMPLES - correlation_pos_ + write_pos_;
        }

        // Need at least min_search unsearched samples
        if (unsearched < min_search) {
            static int skip_count2 = 0;
            if (++skip_count2 % 50 == 1)
                LOG_MODEM(INFO, "[%s] searchForSync: SKIP unsearched=%zu < min=%zu, total=%.2fs, corr_pos=%zu",
                          log_prefix_.c_str(), unsearched, min_search, audio_sec, correlation_pos_);
            return;
        }

        // Backlog-aware search-cursor catch-up for ZC mode:
        // If unsearched grows well beyond target window, the search cursor is lagging behind
        // incoming audio due to high detector cost (~0.9-1.0s per search).
        // This causes us to search stale audio that doesn't contain the current ZC preamble.
        // Fix: snap correlation_pos_ forward to near write_pos_ - min_search.
        if (connected_zc_mode) {
            // Target: keep unsearched close to min_search for responsiveness
            // ZC preamble is only 52ms - if we're searching stale audio, we miss it completely.
            // Be more aggressive than before: catch up whenever backlog exceeds 1.5x target.
            constexpr size_t BACKLOG_THRESHOLD_MODERATE = 15;  // 1.5x min_search (scaled by 10)
            constexpr size_t BACKLOG_THRESHOLD_SEVERE = 25;    // 2.5x min_search (scaled by 10)

            size_t scaled_unsearched = (unsearched * 10) / min_search;

            if (scaled_unsearched > BACKLOG_THRESHOLD_SEVERE) {
                // Severe lag: snap to near newest audio
                size_t new_pos = (write_pos_ + MAX_BUFFER_SAMPLES - min_search - 4800) % MAX_BUFFER_SAMPLES;
                size_t skipped = (unsearched > min_search) ? (unsearched - min_search) : 0;
                LOG_MODEM(WARN, "[%s] ZC backlog catch-up (severe): unsearched=%zu (%.1fx), "
                          "skipping %zu samples, corr_pos %zu -> %zu",
                          log_prefix_.c_str(), unsearched, static_cast<float>(unsearched) / min_search,
                          skipped, correlation_pos_, new_pos);
                correlation_pos_ = new_pos;
                unsearched = min_search + 4800;  // Update for downstream logic
            } else if (scaled_unsearched > BACKLOG_THRESHOLD_MODERATE) {
                // Moderate lag: gradual catch-up (always, not just on reject streak)
                // This is critical for ZC mode - we can't afford to search stale audio
                size_t excess = unsearched - min_search;
                size_t catch_up = excess / 2;  // Catch up half the excess each time
                catch_up = std::min(catch_up, size_t(24000));  // Max 500ms per step
                catch_up = std::max(catch_up, size_t(4800));   // Min 100ms per step
                size_t new_pos = (correlation_pos_ + catch_up) % MAX_BUFFER_SAMPLES;
                LOG_MODEM(INFO, "[%s] ZC backlog catch-up (moderate): unsearched=%zu (%.1fx), "
                          "advancing %zu samples, corr_pos %zu -> %zu",
                          log_prefix_.c_str(), unsearched, static_cast<float>(unsearched) / min_search,
                          catch_up, correlation_pos_, new_pos);
                correlation_pos_ = new_pos;
                unsearched -= catch_up;
            }
        }

        // Quick RMS check for signal presence
        float rms = 0.0f;
        for (size_t i = 0; i < 1000; i++) {
            float s = buffer_[(correlation_pos_ + i) % MAX_BUFFER_SAMPLES];
            rms += s * s;
        }
        rms = std::sqrt(rms / 1000.0f);

        // Adaptive RMS gate:
        // - Connected + data preamble: keep existing low-level OTA-friendly behavior.
        // - Disconnected chirp acquisition: avoid hard 0.05 gate starvation on
        //   valid low-amplitude links (observed ~0.03 RMS in fading sims).
        float noise_floor = std::max(0.001f, noise_floor_);
        if (rms < noise_floor * 3.0f) {
            noise_floor_ = 0.98f * noise_floor + 0.02f * rms;
        } else {
            noise_floor_ = 0.995f * noise_floor + 0.005f * rms;
        }

        float rms_gate = CORR_NOISE_THRESHOLD;
        if (connected_light_sync) {
            // Typical OTA values observed around 0.02-0.04 RMS; keep floor low enough
            // to avoid starving detectDataSync() while still skipping true silence.
            rms_gate = std::clamp(noise_floor_ * 2.2f, 0.015f, 0.040f);
            if (sync_reject_streak_ >= 8) {
                float relax = std::min(0.010f,
                                       0.001f * static_cast<float>(sync_reject_streak_ - 7));
                rms_gate = std::max(0.012f, rms_gate - relax);
            }
        } else {
            // Disconnected/legacy path: use adaptive gate instead of fixed 0.05.
            // Keep this permissive enough for low-RMS CONNECT/ACK chirps seen in
            // fading simulations (~0.03 RMS) while still filtering idle noise.
            rms_gate = std::clamp(noise_floor_ * 1.8f, 0.014f, 0.030f);
            if (sync_reject_streak_ >= 8) {
                float relax = std::min(0.008f,
                                       0.001f * static_cast<float>(sync_reject_streak_ - 7));
                rms_gate = std::max(0.010f, rms_gate - relax);
            }
        }

        if (rms < rms_gate) {
            // No signal - advance by small step (100ms = 4800 samples)
            static int rms_skip_count = 0;
            if (++rms_skip_count % 10 == 1)
                LOG_MODEM(INFO, "[%s] searchForSync: RMS skip, rms=%.4f < %.3f, corr_pos=%zu, total=%.2fs",
                          log_prefix_.c_str(), rms, rms_gate, correlation_pos_, audio_sec);
            correlation_pos_ = (correlation_pos_ + CORRELATION_STEP) % MAX_BUFFER_SAMPLES;
            return;
        }

        // Signal detected - log before running correlation (only occasionally to reduce spam)
        static int run_log_count = 0;
        if (++run_log_count % 10 == 1) {
            LOG_MODEM(INFO, "[%s] searchForSync: RUNNING correlation, rms=%.4f, corr_pos=%zu, total=%.2fs",
                      log_prefix_.c_str(), rms, correlation_pos_, audio_sec);
        }

        // Signal present - back up search start to catch preamble that might have started
        // in the lead-in silence. The TX lead-in is ~150ms (7200 samples), so we should
        // back up at least that much to ensure the preamble START is in our search window.
        //
        // ZC mode (MC-DPSK connected): ZC preamble is only 52ms (2512 samples).
        // With CORRELATION_STEP=4800, we can skip over the entire preamble!
        // Solution: Use larger backtrack and smaller step for ZC mode.
        constexpr size_t SEARCH_BACKTRACK_CHIRP = 9600;  // 200ms for chirp (1200ms preamble)
        constexpr size_t SEARCH_BACKTRACK_ZC = 19200;    // 400ms for ZC (52ms preamble, but need margin)
        constexpr size_t CORRELATION_STEP_ZC = 1200;     // 25ms step for ZC (don't skip over it)

        bool use_zc_params = connected_zc_mode;
        size_t backtrack = use_zc_params ? SEARCH_BACKTRACK_ZC : SEARCH_BACKTRACK_CHIRP;
        size_t step = use_zc_params ? CORRELATION_STEP_ZC : CORRELATION_STEP;

        if (correlation_pos_ >= backtrack) {
            search_start = correlation_pos_ - backtrack;
        } else if (total_fed_ < MAX_BUFFER_SAMPLES) {
            // Buffer hasn't wrapped yet, start from beginning
            search_start = 0;
        } else {
            // Buffer wrapped, handle underflow
            search_start = (MAX_BUFFER_SAMPLES + correlation_pos_ - backtrack) % MAX_BUFFER_SAMPLES;
        }
        search_buffer.resize(min_search);
        for (size_t i = 0; i < min_search; i++) {
            search_buffer[i] = buffer_[(search_start + i) % MAX_BUFFER_SAMPLES];
        }

        // Advance by appropriate step based on mode
        correlation_pos_ = (correlation_pos_ + step) % MAX_BUFFER_SAMPLES;
    }

    // DEBUG: Dump the search buffer on first few searches
    static int search_dump_count = 0;
    if (g_debug_dumps_enabled && search_dump_count < 5) {
        char label[64];
        snprintf(label, sizeof(label), "search_%d_pos%zu", search_dump_count, search_start);

        // Dump search buffer to file
        char filename[256];
        snprintf(filename, sizeof(filename), "%s_%s.f32", g_dump_prefix, label);
        std::ofstream file(filename, std::ios::binary);
        if (file) {
            file.write(reinterpret_cast<const char*>(search_buffer.data()),
                       search_buffer.size() * sizeof(float));
            file.close();

            // Compute stats
            float rms = 0, max_val = 0;
            for (size_t i = 0; i < std::min(search_buffer.size(), size_t(10000)); i++) {
                rms += search_buffer[i] * search_buffer[i];
                max_val = std::max(max_val, std::abs(search_buffer[i]));
            }
            rms = std::sqrt(rms / std::min(search_buffer.size(), size_t(10000)));

            LOG_MODEM(DEBUG, "StreamingDecoder: Search #%d: dumped %zu samples to %s (start=%zu, RMS=%.4f, peak=%.4f)",
                      search_dump_count, search_buffer.size(), filename, search_start, rms, max_val);
        }
        search_dump_count++;
    }

    // Search for sync (no lock held - this is the slow part)
    auto search_start_time = std::chrono::steady_clock::now();

    SyncResult sync_result;
    bool found = false;

    // When connected, use light sync only (LTS/ZC training symbols, no chirp).
    // In steady-state connected mode there is no chirp in payload frames.
    // A brief CHIRP fallback grace window is still allowed right after mode
    // transitions to handle handshake races.
    // Reject false positives where data autocorrelation produces spurious peaks
    // (observed up to 0.63). Real LTS correlation is always >0.81 even on
    // moderate fading.
    // Coherent modes need higher sync quality — badly-synced frames always fail
    // because stale LTS phases can't be recovered by DD tracking alone.
    // Coherent modes need higher sync quality than differential modes.
    // QAM is sensitive to phase errors but LTS correlation can be lower due to
    // amplitude variations. Use intermediate thresholds between PSK and differential.
    const bool is_coherent_psk = (current_modulation_ == Modulation::QPSK ||
                                  current_modulation_ == Modulation::BPSK);
    const bool is_coherent_qam = (current_modulation_ == Modulation::QAM16 ||
                                  current_modulation_ == Modulation::QAM32 ||
                                  current_modulation_ == Modulation::QAM64 ||
                                  current_modulation_ == Modulation::QAM256);
    const bool is_coherent = is_coherent_psk || is_coherent_qam;
    const float fading_hint = last_fading_index_.load();
    const float snr_hint = last_snr_.load();

    // ZC preamble (MC-DPSK): correlation ~0.25-0.30, use much lower threshold
    // LTS preamble (OFDM): correlation ~0.70-0.95, use higher threshold
    bool is_zc_mode = (mode_ == protocol::WaveformMode::MC_DPSK);

    // PSK: strict 0.90 threshold, QAM: relaxed 0.78 (LTS correlation varies more)
    // Differential: 0.72 base with adaptive relaxation
    // ZC: 0.40 base with normalized correlation (true peaks at ~0.55-0.60, noise at ~0.15-0.20)
    float light_sync_min_confidence = is_zc_mode ? 0.40f :
                                      (is_coherent_psk ? 0.90f : (is_coherent_qam ? 0.78f : 0.72f));
    float weak_sync_floor = is_zc_mode ? 0.30f :
                            (is_coherent_psk ? 0.85f : (is_coherent_qam ? 0.70f : 0.55f));
    if (!is_coherent && !is_zc_mode) {
        // OFDM LTS: OTA HF can produce valid LTS peaks in the 0.55-0.65 range during
        // fades. Start conservative, but avoid hard-locking to 0.70.
        if (fading_hint >= 1.00f || snr_hint < 10.0f) {
            light_sync_min_confidence = 0.62f;
        } else if (fading_hint >= 0.70f || snr_hint < 14.0f) {
            light_sync_min_confidence = 0.65f;
        } else if (fading_hint >= 0.50f || snr_hint < 18.0f) {
            light_sync_min_confidence = 0.68f;
        }

        if (connected_ && sync_reject_streak_ >= 8) {
            float extra_relax = std::min(0.12f,
                                         0.015f * static_cast<float>(sync_reject_streak_ - 7));
            light_sync_min_confidence = std::max(0.56f, light_sync_min_confidence - extra_relax);
        }
    }

    // ZC mode reject-loop breaker: after consecutive weak rejects, relax threshold
    // This prevents perpetual reject loops when valid ZC preambles produce borderline correlation
    // (e.g., due to residual CFO, amplitude variations, or window misalignment)
    if (is_zc_mode && connected_ && sync_reject_streak_ >= 4) {
        // Relax threshold gradually: -0.02 per reject after 4, floor at 0.25
        float extra_relax = std::min(0.15f,
                                     0.025f * static_cast<float>(sync_reject_streak_ - 3));
        float old_threshold = light_sync_min_confidence;
        light_sync_min_confidence = std::max(0.25f, light_sync_min_confidence - extra_relax);
        weak_sync_floor = std::max(0.20f, weak_sync_floor - extra_relax);
        if (sync_reject_streak_ <= 6 || (sync_reject_streak_ % 5) == 0) {
            LOG_MODEM(INFO, "[%s] ZC reject-loop breaker: streak=%llu, threshold %.2f -> %.2f",
                      log_prefix_.c_str(), static_cast<unsigned long long>(sync_reject_streak_),
                      old_threshold, light_sync_min_confidence);
        }
    }

    {
        std::lock_guard<std::mutex> wlock(waveform_mutex_);
        if (!waveform_) {
            return;
        }
        waveform_->reset();

    if (connected_light_sync) {
        float known_cfo = last_cfo_.load();
        // FIX: Pass mode-aware threshold to detectDataSync()
        // Previously used fixed CORR_DETECT_THRESHOLD (0.15), causing threshold contradiction:
        // - ZC detector found weak peaks at 0.17-0.21 (above 0.15)
        // - But acceptance required >= 0.40, causing perpetual reject loops
        // Now pass the actual acceptance threshold so detector and acceptance agree.
        float detect_threshold = is_zc_mode ? light_sync_min_confidence : CORR_DETECT_THRESHOLD;
        found = waveform_->detectDataSync(
            SampleSpan(search_buffer.data(), search_buffer.size()),
            sync_result, known_cfo, detect_threshold);

        // zc_sync::detect() now returns the best correlation even when below threshold.
        // Treat close misses as weak rejects so adaptive threshold relaxation can engage.
        if (is_zc_mode && !found) {
            if (sync_result.correlation >= weak_sync_floor) {
                sync_reject_streak_++;
                if (sync_reject_streak_ <= 6 || (sync_reject_streak_ % 5) == 0) {
                    LOG_MODEM(DEBUG, "[%s] ZC near-miss (corr=%.2f < %.2f, streak=%llu)",
                              log_prefix_.c_str(), sync_result.correlation, light_sync_min_confidence,
                              static_cast<unsigned long long>(sync_reject_streak_));
                }
            } else if (sync_reject_streak_ > 0) {
                sync_reject_streak_--;
            }
        }

        // Reject clear false positives (noise floor is ~0.2-0.4)
        bool weak_accept = false;
        if (found && sync_result.correlation < light_sync_min_confidence) {
            bool allow_weak_accept = !is_coherent &&
                                     connected_ &&
                                     sync_reject_streak_ >= 3 &&
                                     sync_result.correlation >= std::max(weak_sync_floor,
                                                                          light_sync_min_confidence - 0.08f);
            if (allow_weak_accept) {
                weak_accept = true;
                LOG_MODEM(INFO, "[%s] DATA sync weak-accepted (corr=%.2f < %.2f, streak=%llu)",
                          log_prefix_.c_str(), sync_result.correlation, light_sync_min_confidence,
                          static_cast<unsigned long long>(sync_reject_streak_));
            } else {
                sync_reject_streak_++;
                LOG_MODEM(INFO, "[%s] DATA sync rejected (corr=%.2f < %.2f, streak=%llu)",
                          log_prefix_.c_str(), sync_result.correlation, light_sync_min_confidence,
                          static_cast<unsigned long long>(sync_reject_streak_));
                found = false;
            }
        }

        if (found) {
            zc_chirp_fallback_attempts_ = 0;
            if (weak_accept) {
                sync_reject_streak_ = std::max<uint64_t>(1, sync_reject_streak_ / 2);
            } else {
                sync_reject_streak_ = 0;
            }
            LOG_MODEM(INFO, "[%s] DATA sync detected (training only, known CFO=%.1f Hz, corr=%.2f)",
                      log_prefix_.c_str(), known_cfo, sync_result.correlation);
        }

        // CHIRP fallback for MC-DPSK ZC mode:
        // Keep this only for a short grace period after entering connected mode.
        // Past that window, repeated chirp fallback is pure overhead/noise because
        // connected frames carry ZC preambles, not chirps.
        if (!found && is_zc_mode) {
            bool allow_chirp_fallback = false;
            int64_t connected_age_ms = -1;
            if (connected_since_.time_since_epoch().count() > 0) {
                connected_age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - connected_since_).count();
                allow_chirp_fallback = (connected_age_ms >= 0 &&
                                        connected_age_ms <= ZC_CHIRP_FALLBACK_GRACE_MS);
            }

            if (allow_chirp_fallback) {
                zc_chirp_fallback_attempts_++;
                bool log_fallback = (zc_chirp_fallback_attempts_ <= 3) ||
                                    ((zc_chirp_fallback_attempts_ % 25) == 0);
                if (log_fallback) {
                    LOG_MODEM(DEBUG, "[%s] ZC miss (corr=%.2f), trying CHIRP fallback (age=%lld ms, attempt=%llu)",
                              log_prefix_.c_str(), sync_result.correlation,
                              static_cast<long long>(connected_age_ms),
                              static_cast<unsigned long long>(zc_chirp_fallback_attempts_));
                }

                SyncResult chirp_result;
                bool chirp_found = waveform_->detectSync(
                    SampleSpan(search_buffer.data(), search_buffer.size()),
                    chirp_result, CORR_DETECT_THRESHOLD);

                if (chirp_found && chirp_result.correlation > 0.6f) {
                    LOG_MODEM(INFO, "[%s] Using CHIRP sync fallback (corr=%.2f, cfo=%.1f Hz, age=%lld ms)",
                              log_prefix_.c_str(), chirp_result.correlation, chirp_result.cfo_hz,
                              static_cast<long long>(connected_age_ms));
                    found = true;
                    sync_result = chirp_result;
                    sync_reject_streak_ = 0;
                    zc_chirp_fallback_attempts_ = 0;
                } else if (log_fallback) {
                    LOG_MODEM(DEBUG, "[%s] CHIRP fallback miss (corr=%.2f, cfo=%.1f Hz)",
                              log_prefix_.c_str(), chirp_result.correlation, chirp_result.cfo_hz);
                }
            }
        }
    } else {
        // Use full sync detection with chirp
        found = waveform_->detectSync(
            SampleSpan(search_buffer.data(), search_buffer.size()),
            sync_result, CORR_DETECT_THRESHOLD);
    }
    }

    auto search_end_time = std::chrono::steady_clock::now();
    float search_ms = std::chrono::duration<float, std::milli>(search_end_time - search_start_time).count();

    // Check if reset() was called during our search - if so, discard results
    if (reset_generation_.load() != gen_at_start) {
        LOG_MODEM(INFO, "[%s] searchForSync: ABORTED - reset() called during search", log_prefix_.c_str());
        return;
    }

    // Log timing: total_fed_ tells us how much audio has arrived
    float audio_sec = total_fed_ / 48000.0f;
    if (found || search_ms > 100) {  // Log if found or if search was slow
        // FIX (codex.md Issue #7): Removed fprintf - use LOG_MODEM only
        LOG_MODEM(INFO, "[%s] searchForSync: audio=%.2fs, search=%.1fms, found=%d, corr=%.3f",
                  log_prefix_.c_str(), audio_sec, search_ms, found ? 1 : 0, sync_result.correlation);
    }

    if (found) {
        LOG_MODEM(DEBUG, "[%s] Sync found! Setting SYNC_FOUND state", log_prefix_.c_str());
        std::lock_guard<std::mutex> lock(buffer_mutex_);

        sync_position_ = (search_start + sync_result.start_sample) % MAX_BUFFER_SAMPLES;

        // Convert ring-buffer index to absolute sample index.
        // Needed so waveform CFO phase starts from true stream time, not local window offset.
        auto ringPosToAbsolute = [this](size_t ring_pos) -> size_t {
            if (total_fed_ < MAX_BUFFER_SAMPLES) {
                // Buffer has not wrapped yet: ring index == absolute sample index.
                return ring_pos;
            }

            const size_t oldest_abs = total_fed_ - MAX_BUFFER_SAMPLES;
            const size_t oldest_pos = write_pos_;  // write_pos_ points to oldest sample after wrap
            const size_t offset = (ring_pos >= oldest_pos)
                ? (ring_pos - oldest_pos)
                : (MAX_BUFFER_SAMPLES - oldest_pos + ring_pos);
            return oldest_abs + offset;
        };

        // Anti-replay: reject sync at same position as last decoded frame (circular distance)
        if (last_decoded_sync_pos_ != SIZE_MAX) {
            size_t d1 = (sync_position_ >= last_decoded_sync_pos_)
                ? (sync_position_ - last_decoded_sync_pos_)
                : (MAX_BUFFER_SAMPLES - last_decoded_sync_pos_ + sync_position_);
            size_t dist = std::min(d1, MAX_BUFFER_SAMPLES - d1);
            if (dist < 200) {
                LOG_MODEM(INFO, "[%s] Anti-replay: duplicate sync at pos=%zu (prev=%zu), skipping",
                          log_prefix_.c_str(), sync_position_, last_decoded_sync_pos_);
                constexpr size_t SEARCH_BACKTRACK = 9600;
                correlation_pos_ = (sync_position_ + SEARCH_BACKTRACK + CORRELATION_STEP) % MAX_BUFFER_SAMPLES;
                return;
            }
        }

        // Provide absolute training position to waveform so initial CFO phase is aligned.
        {
            std::lock_guard<std::mutex> wlock(waveform_mutex_);
            if (waveform_) {
                const size_t abs_training_pos = ringPosToAbsolute(sync_position_);
                waveform_->setAbsoluteTrainingPosition(abs_training_pos);
            }
        }

        // CFO handling: On fading channels, chirp-based CFO measurement can be corrupted
        // by multipath (peaks shift differently for up vs down chirp).
        // When connected, trust the established CFO and limit drift.
        float new_cfo = sync_result.cfo_hz;
        float known_cfo = last_cfo_.load();

        if (connected_ && std::abs(known_cfo) > 0.01f) {
            // Limit CFO change to ±1 Hz per frame (oscillator drift is slow)
            constexpr float MAX_CFO_DRIFT_HZ = 1.0f;
            float cfo_diff = new_cfo - known_cfo;
            if (std::abs(cfo_diff) > MAX_CFO_DRIFT_HZ) {
                LOG_MODEM(INFO, "[%s] CFO sanity: measured=%.1f, known=%.1f, diff=%.1f > %.1f, using known",
                          log_prefix_.c_str(), new_cfo, known_cfo, cfo_diff, MAX_CFO_DRIFT_HZ);
                new_cfo = known_cfo;  // Trust established CFO over noisy measurement
            }
        }

        sync_cfo_ = new_cfo;
        sync_snr_ = estimateSNRFromChirp(sync_result.correlation, noise_floor_);
        sync_start_time_ = std::chrono::steady_clock::now();
        pending_total_cw_ = 0;

        state_ = DecoderState::SYNC_FOUND;

        last_snr_.store(sync_snr_);
        last_cfo_.store(sync_cfo_);

        LOG_MODEM(INFO, "[%s] SYNC at pos=%zu, CFO=%.1f Hz, SNR=%.1f dB",
                  log_prefix_.c_str(), sync_position_, sync_cfo_, sync_snr_);

        // NOTE: Do NOT advance correlation_pos_ past the frame here.
        // It was already advanced by CORRELATION_STEP at line 323 during search.
        // The post-decode skip at decodeCurrentFrame() line 718 handles advancing
        // past the decoded frame when we return to SEARCHING.
        //
        // Previously, this code jumped correlation_pos_ past the entire frame,
        // which could place it AHEAD of write_pos_ in circular buffer space
        // (especially after buffer wraps). This caused feedAudio()'s overflow
        // check to compute unsearched ≈ buffer_size, triggering spurious
        // buffer overflows and data loss during async decode.
    }
}

void StreamingDecoder::checkIfReadyToDecode() {
    // FIX (codex.md Issue #7): Removed fprintf from hot path
    {
        std::lock_guard<std::mutex> wlock(waveform_mutex_);
        if (!waveform_) {
            state_ = DecoderState::SEARCHING;
            return;
        }
    }

    // How many samples do we have from sync position?
    size_t available;
    {
        std::lock_guard<std::mutex> lock(buffer_mutex_);

        if (write_pos_ >= sync_position_) {
            available = write_pos_ - sync_position_;
        } else {
            available = MAX_BUFFER_SAMPLES - sync_position_ + write_pos_;
        }
    }

    // Check timeout - dynamic based on expected frame duration
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - sync_start_time_).count();

    int timeout_ms = getFrameTimeoutMs();
    if (elapsed > timeout_ms) {
        LOG_MODEM(WARN, "[%s] Frame timeout after %lld ms (limit=%d ms)", log_prefix_.c_str(),
                  (long long)elapsed, timeout_ms);
        state_ = DecoderState::SEARCHING;
        return;
    }

    // Calculate how much we need — must match decodeCurrentFrame() buffer sizing
    bool is_ofdm_here = (mode_ == protocol::WaveformMode::OFDM_CHIRP ||
                         mode_ == protocol::WaveformMode::OFDM_COX);
    size_t needed;
    {
        std::lock_guard<std::mutex> wlock(waveform_mutex_);
        if (!waveform_) {
            state_ = DecoderState::SEARCHING;
            return;
        }

        if (pending_total_cw_ > 0) {
            // We know exact CW count from CW0 peek — get exact sample count
            needed = static_cast<size_t>(waveform_->getMinSamplesForCWCount(pending_total_cw_));
        } else if (is_ofdm_here && connected_) {
            // Burst-interleaved frames MUST use full 4-CW buffer (marker is one-shot consumed by process())
            // Otherwise: 1-CW peek first — CW0 escalation handles data frames (including QAM)
            // ACK frames are always DQPSK R1/4 (1 CW), so control-first peek catches them even in QAM mode.
            // If it's a QAM data frame, CW0 peek will fail → escalate to 4 CWs.
            bool burst_latched = use_burst_interleave_ && waveform_->wasBurstInterleaved();
            if (!burst_latched) {
                // 1-CW peek first — escalate to 4-CW if CW0 indicates data frame
                needed = getOFDMControlFrameSamples(waveform_.get(), current_modulation_, code_rate_);
            } else {
                needed = static_cast<size_t>(waveform_->getMinSamplesForFrame());
            }
        } else {
            // MC-DPSK/disconnected: allow an early half-CW window so PING/PONG can
            // be classified quickly, then escalate to full-CW when needed.
            needed = static_cast<size_t>(waveform_->getMinSamplesForControlFrame());
            if (!connected_ && mode_ == protocol::WaveformMode::MC_DPSK &&
                current_modulation_ == Modulation::DBPSK) {
                needed = std::max<size_t>(needed / 2, 48000);  // ~1.0s minimum
            }
        }
    }

    // FIX (codex.md Issue #7): Removed fprintf from hot path
    if (available >= needed) {
        state_ = DecoderState::DECODING;
    }
}

// Observability counters for robust decode paths (check via debugger or periodic log)
static std::atomic<int> g_robust_retry_hits{0};   // CW0 peek: retry succeeded after initial fail
static std::atomic<int> g_salvage_hits{0};         // 1-CW control salvaged from 4-CW path

// Robust single-CW LDPC decode with Phase 0 decoder diversity (4 retry attempts)
// Uses standalone LDPCDecoder for setMinSumFactor (not available via ICodec interface)
// Pattern matches decodeFixedFrame() Phase 0 (frame_v2.cpp:1378-1395)
static std::pair<bool, Bytes> robustDecodeSingleCW(
    const float* cw_data, size_t cw_size, CodeRate rate, const char* log_prefix = nullptr)
{
    LDPCDecoder decoder(rate);
    decoder.setMaxIterations(fec::LDPCCodec::getRecommendedIterations(rate));
    decoder.setMinSumFactor(0.9375f);

    auto decoded = decoder.decodeSoft(std::span<const float>(cw_data, cw_size));
    bool ok = decoder.lastDecodeSuccess();

    if (!ok) {
        static constexpr float factors[] = {0.875f, 0.75f, 0.625f, 0.5f};
        for (int retry = 0; retry < 4 && !ok; retry++) {
            decoder.setMinSumFactor(factors[retry]);
            decoded = decoder.decodeSoft(std::span<const float>(cw_data, cw_size));
            ok = decoder.lastDecodeSuccess();
            if (ok) {
                g_robust_retry_hits.fetch_add(1, std::memory_order_relaxed);
                if (log_prefix) {
                    LOG_MODEM(INFO, "[%s] Robust CW0: RETRY OK (factor=%.3f, iters=%d, total_hits=%d)",
                              log_prefix, factors[retry], decoder.lastIterations(),
                              g_robust_retry_hits.load(std::memory_order_relaxed));
                }
            }
        }
    }

    Bytes data;
    if (ok) data.assign(decoded.begin(), decoded.end());
    return {ok, data};
}

void StreamingDecoder::decodeCurrentFrame() {
    // FIX (codex.md Issue #7): Removed fprintf from hot path
    if (!waveform_) {
        {
            std::lock_guard<std::mutex> lock(buffer_mutex_);
            correlation_pos_ = (sync_position_ + 4800) % MAX_BUFFER_SAMPLES;
        }
        state_ = DecoderState::SEARCHING;
        return;
    }

    bool is_ofdm = (mode_ == protocol::WaveformMode::OFDM_CHIRP ||
                    mode_ == protocol::WaveformMode::OFDM_COX);

    // Determine how many samples to copy from buffer
    // Strategy depends on mode and state:
    // - pending_total_cw_ > 0: exact size from prior CW0 peek
    // - Connected OFDM, first pass: full 4-CW buffer (most frames are data, frame-interleaved)
    // - MC-DPSK: 1-CW for peek (MC-DPSK getMinSamplesForFrame() == 1 CW by design)
    // - Disconnected: full frame (always MC-DPSK for handshake)
    size_t frame_len;
    if (pending_total_cw_ > 0) {
        frame_len = static_cast<size_t>(waveform_->getMinSamplesForCWCount(pending_total_cw_));
    } else if (is_ofdm && connected_) {
        // Burst-interleaved frames use full 4-CW buffer; otherwise 1-CW peek first
        // ACK frames are always DQPSK R1/4 (1 CW), so control-first peek catches them even in QAM mode.
        // If it's a QAM data frame, CW0 peek fails → escalate to 4 CWs via pending_total_cw_.
        bool burst_latched = use_burst_interleave_ && waveform_ && waveform_->wasBurstInterleaved();
        if (!burst_latched) {
            frame_len = getOFDMControlFrameSamples(waveform_.get(), current_modulation_, code_rate_);
        } else {
            frame_len = static_cast<size_t>(waveform_->getMinSamplesForFrame());
        }
    } else {
        // MC-DPSK or disconnected: 1-CW peek
        frame_len = static_cast<size_t>(waveform_->getMinSamplesForControlFrame());
    }

    // Copy frame samples from buffer
    std::vector<float> frame_buffer;
    {
        std::lock_guard<std::mutex> lock(buffer_mutex_);

        size_t available;
        if (write_pos_ >= sync_position_) {
            available = write_pos_ - sync_position_;
        } else {
            available = MAX_BUFFER_SAMPLES - sync_position_ + write_pos_;
        }

        frame_len = std::min(frame_len, available);

        frame_buffer.resize(frame_len);
        for (size_t i = 0; i < frame_len; i++) {
            frame_buffer[i] = buffer_[(sync_position_ + i) % MAX_BUFFER_SAMPLES];
        }
    }

    if (frame_buffer.empty()) {
        {
            std::lock_guard<std::mutex> lock(buffer_mutex_);
            correlation_pos_ = (sync_position_ + 4800) % MAX_BUFFER_SAMPLES;
        }
        state_ = DecoderState::SEARCHING;
        return;
    }

    // Check for PING (low energy after sync = chirp only, no data)
    // For MC-DPSK: training=4096, ref=512, so data starts at 4608
    // We need to check AFTER training region to detect PING correctly
    size_t training_skip = 0;
    if (mode_ == protocol::WaveformMode::MC_DPSK ||
        mode_ == protocol::WaveformMode::MFSK) {
        training_skip = 4608;  // training + ref samples (or tone sweep for MFSK)
    } else {
        // OFDM: check after light preamble (training symbols)
        // Must be waveform/config dependent (FFT/CP/carriers).
        int data_preamble = waveform_->getDataPreambleSamples();
        training_skip = (data_preamble > 0) ? static_cast<size_t>(data_preamble) : size_t(1024);
    }

    // Compute training region RMS (signal + noise)
    float training_rms = 0.0f;
    size_t train_len = std::min(training_skip, frame_buffer.size());
    if (train_len > 0) {
        for (size_t i = 0; i < train_len; i++) {
            training_rms += frame_buffer[i] * frame_buffer[i];
        }
        training_rms = std::sqrt(training_rms / train_len);
    }

    // Compute data region RMS
    float rms = 0.0f;
    size_t check_start = std::min(training_skip, frame_buffer.size());
    size_t check_len = std::min(frame_buffer.size() - check_start, size_t(5000));
    if (check_len > 0) {
        for (size_t i = 0; i < check_len; i++) {
            rms += frame_buffer[check_start + i] * frame_buffer[check_start + i];
        }
        rms = std::sqrt(rms / check_len);
    }

    // PING/PONG/DATA/CONTROL detection
    bool is_ping = false;
    bool is_pong = false;
    // Connected links carry framed ARQ traffic; probing chirps are disconnected-only.
    // Allowing PING classification while connected can misclassify short DATA frames
    // (for example tiny keep-alive payloads) and trigger retry storms.
    const bool allow_ping_detection = !connected_;

    if (allow_ping_detection) {
        if (css_enabled_ && css_sync_) {
            // CSS mode: detect frame type from CSS preamble
            // With hybrid approach: [legacy_preamble][CSS_chirp]
            // CSS chirp is appended after the full legacy preamble (including training+ref)
            // We need to read it directly from the ring buffer since frame_buffer may be too small

            // CSS chirp position relative to sync_position_ (training_start)
            // = legacy_preamble_samples - (dual_chirp_samples + 2*gap)
            // = training_samples + ref_samples (since training_start is after dual chirps)
            // For MC-DPSK: training = 4096, ref = 512, so CSS starts at offset ~4608
            const auto& css_cfg = css_sync_->getConfig();
            size_t css_chirp_samples = css_cfg.chirpSamples();
            size_t css_gap_samples = css_cfg.gapSamples();

            // Calculate CSS start offset from sync_position_
            // sync_position_ = training_start = after [up-chirp][gap][down-chirp][gap]
            // CSS is at end of legacy preamble = training_start + training + ref
            size_t css_offset_from_sync = training_skip;  // training + ref = ~4608 for MC-DPSK

            // Read CSS portion directly from ring buffer
            size_t css_needed = css_chirp_samples * css_cfg.num_chirps +
                               css_gap_samples * css_cfg.num_chirps;  // ~28800
            sync::CSSSyncResult css_result;

            {
                std::lock_guard<std::mutex> lock(buffer_mutex_);
                size_t css_start = (sync_position_ + css_offset_from_sync) % MAX_BUFFER_SAMPLES;
                size_t available = (write_pos_ >= css_start) ? (write_pos_ - css_start)
                                                              : (MAX_BUFFER_SAMPLES - css_start + write_pos_);
                if (available >= css_needed) {
                    Samples css_buffer(css_needed);
                    for (size_t i = 0; i < css_needed; i++) {
                        css_buffer[i] = buffer_[(css_start + i) % MAX_BUFFER_SAMPLES];
                    }
                    css_result = css_sync_->detect(css_buffer, 0.2f);
                }
            }
            sync::CSSFrameType css_frame_type = css_result.frame_type;

            is_ping = (css_frame_type == sync::CSSFrameType::PING);
            is_pong = (css_frame_type == sync::CSSFrameType::PONG);

            LOG_MODEM(INFO, "[%s] CSS frame type: %s (corr=%.3f, detected=%d, offset=%zu)",
                      log_prefix_.c_str(), sync::cssFrameTypeToString(css_frame_type),
                      css_result.correlation, css_result.detected ? 1 : 0, css_offset_from_sync);
            // FIX (codex.md Issue #7): Removed redundant fprintf
        } else {
            // Legacy mode: use ratio of data RMS to training RMS
            // PING (chirp only): data region is noise-only, ratio << 1 (typically 0.15-0.55)
            // DATA frame: data region has signal, ratio ~0.6-1.0
            // Threshold 0.6 handles low SNR (-3 dB) where noise pushes ratio toward 0.5-0.55
            float rms_ratio = (training_rms > 0.001f) ? rms / training_rms : 0.0f;
            is_ping = rms_ratio < 0.6f;

            // FIX (codex.md Issue #7): Removed redundant fprintf
            LOG_MODEM(INFO, "[%s] PING check: RMS=%.4f, train_RMS=%.4f, ratio=%.3f (threshold=0.6), sync_pos=%zu",
                      log_prefix_.c_str(), rms, training_rms, rms_ratio, sync_position_);
        }
    }

    if (is_ping || is_pong) {
        // PING or PONG detected
        const char* type_str = is_pong ? "PONG" : "PING";
        LOG_MODEM(INFO, "[%s] %s detected (RMS=%.4f), SNR=%.1f dB, CFO=%.1f Hz",
                  log_prefix_.c_str(), type_str, rms, sync_snr_, sync_cfo_);

        DecodeResult ping;
        ping.success = true;
        ping.is_ping = true;  // Both PING and PONG are "ping-like" (no data payload)
        ping.frame_type = is_pong ? v2::FrameType::PONG : v2::FrameType::PING;
        ping.snr_db = sync_snr_;
        ping.cfo_hz = sync_cfo_;

        {
            std::lock_guard<std::mutex> qlock(queue_mutex_);
            frame_queue_.push(ping);
        }

        if (ping_callback_) ping_callback_(sync_snr_, sync_cfo_);

        {
            std::lock_guard<std::mutex> slock(stats_mutex_);
            stats_.pings_received++;
        }

        // Skip past the PING
        {
            std::lock_guard<std::mutex> lock(buffer_mutex_);
            size_t min_frame = static_cast<size_t>(waveform_->getMinSamplesForFrame());
            correlation_pos_ = (sync_position_ + min_frame) % MAX_BUFFER_SAMPLES;
            last_decoded_sync_pos_ = sync_position_;
        }

        state_ = DecoderState::SEARCHING;
        return;
    }

    // Connected OFDM control-first hypothesis:
    // Try demodulating as DQPSK R1/4 control before using data profile.
    // This protects ACK/NACK decode when data modulation is higher order.
    bool first_pass_ofdm_peek = (pending_total_cw_ == 0 && is_ofdm && connected_
                                 && frame_len <= getOFDMControlFrameSamples(
                                     waveform_.get(), current_modulation_, code_rate_));
    if (first_pass_ofdm_peek) {
        constexpr size_t CONTROL_LDPC_BLOCK = v2::LDPC_CODEWORD_BITS;
        Modulation saved_mod = current_modulation_;
        CodeRate saved_rate = code_rate_;
        bool switched_profile = (saved_mod != Modulation::DQPSK || saved_rate != CodeRate::R1_4);

        if (switched_profile) {
            waveform_->configure(Modulation::DQPSK, CodeRate::R1_4);
        }

        waveform_->setFrequencyOffset(sync_cfo_);
        bool control_ok = waveform_->process(SampleSpan(frame_buffer.data(), frame_buffer.size()));
        if (control_ok) {
            captureConstellationSnapshot();
            auto control_soft_bits = waveform_->getSoftBits();
            if (control_soft_bits.size() >= CONTROL_LDPC_BLOCK) {
                auto [ok_r14, data_r14] = robustDecodeSingleCW(
                    control_soft_bits.data(), CONTROL_LDPC_BLOCK, CodeRate::R1_4, log_prefix_.c_str());
                size_t bpc_r14 = v2::getBytesPerCodeword(CodeRate::R1_4);

                if (ok_r14 && data_r14.size() >= 4
                    && data_r14[0] == 0x55 && data_r14[1] == 0x4C) {
                    if (data_r14.size() > bpc_r14) data_r14.resize(bpc_r14);
                    auto hdr = v2::parseHeader(data_r14);
                    if (hdr.valid && hdr.total_cw == 1 && v2::isControlFrame(hdr.type)) {
                        DecodeResult control_result;
                        control_result.success = true;
                        control_result.frame_data = data_r14;
                        control_result.frame_type = hdr.type;
                        control_result.snr_db = sync_snr_;
                        control_result.cfo_hz = sync_cfo_;
                        control_result.codewords_ok = 1;
                        control_result.codewords_failed = 0;

                        {
                            std::lock_guard<std::mutex> qlock(queue_mutex_);
                            frame_queue_.push(control_result);
                        }
                        if (frame_callback_) {
                            frame_callback_(control_result);
                        }
                        {
                            std::lock_guard<std::mutex> slock(stats_mutex_);
                            stats_.frames_decoded++;
                        }

                        last_fading_index_.store(waveform_->getFadingIndex());

                        if (switched_profile) {
                            waveform_->configure(saved_mod, saved_rate);
                        }

                        {
                            std::lock_guard<std::mutex> lock(buffer_mutex_);
                            correlation_pos_ = (sync_position_ + frame_len) % MAX_BUFFER_SAMPLES;
                            last_decoded_sync_pos_ = sync_position_;
                        }

                        LOG_MODEM(INFO, "[%s] OFDM control-profile decode SUCCESS (%s seq=%d)",
                                  log_prefix_.c_str(), v2::frameTypeToString(hdr.type), hdr.seq);
                        state_ = DecoderState::SEARCHING;
                        return;
                    }
                }
            }
        }

        if (switched_profile) {
            waveform_->configure(saved_mod, saved_rate);
        }
    }

    // Data frame - process audio to get soft bits
    waveform_->setFrequencyOffset(sync_cfo_);

    auto decode_start = std::chrono::steady_clock::now();
    bool ok = waveform_->process(SampleSpan(frame_buffer.data(), frame_buffer.size()));

    if (!ok) {
        LOG_MODEM(DEBUG, "[%s] process() failed", log_prefix_.c_str());
        {
            std::lock_guard<std::mutex> lock(buffer_mutex_);
            correlation_pos_ = (sync_position_ + frame_len) % MAX_BUFFER_SAMPLES;
        }
        state_ = DecoderState::SEARCHING;
        return;
    }
    captureConstellationSnapshot();

    auto soft_bits = waveform_->getSoftBits();
    if (soft_bits.empty()) {
        LOG_MODEM(DEBUG, "[%s] getSoftBits() returned empty", log_prefix_.c_str());
        {
            std::lock_guard<std::mutex> lock(buffer_mutex_);
            correlation_pos_ = (sync_position_ + frame_len) % MAX_BUFFER_SAMPLES;
        }
        state_ = DecoderState::SEARCHING;
        return;
    }
    LOG_MODEM(INFO, "[%s] Got %zu soft bits (%zu samples), proceeding to decode",
              log_prefix_.c_str(), soft_bits.size(), frame_buffer.size());

    last_fading_index_.store(waveform_->getFadingIndex());

    // Check for burst interleave marker (negated LTS detected by waveform)
    // Only for connected OFDM_CHIRP when burst interleaving is enabled
    bool burst_marker = use_burst_interleave_ && connected_ && is_ofdm
                        && mode_ == protocol::WaveformMode::OFDM_CHIRP
                        && waveform_->wasBurstInterleaved();

    if (burst_marker) {
        LOG_MODEM(INFO, "[%s] Burst interleave marker detected, entering accumulation",
                  log_prefix_.c_str());

        // Initialize accumulation state with first frame's soft bits
        burst_soft_buffer_.clear();
        burst_soft_buffer_.push_back(std::move(soft_bits));
        burst_min_block_ = static_cast<size_t>(waveform_->getMinSamplesForFrame());
        burst_next_pos_ = (sync_position_ + frame_len) % MAX_BUFFER_SAMPLES;
        burst_snr_ = sync_snr_;
        burst_cfo_ = sync_cfo_;
        burst_start_time_ = std::chrono::steady_clock::now();

        // Feed back CFO from first frame
        float corrected_cfo = waveform_->estimatedCFO();
        float current_cfo = last_cfo_.load();
        constexpr float MAX_PILOT_CFO_DRIFT_HZ_B = 2.0f;
        float drift = corrected_cfo - current_cfo;
        if (std::abs(drift) > MAX_PILOT_CFO_DRIFT_HZ_B) {
            corrected_cfo = current_cfo + std::copysign(MAX_PILOT_CFO_DRIFT_HZ_B, drift);
        }
        last_cfo_.store(corrected_cfo);
        burst_cfo_ = corrected_cfo;

        state_ = DecoderState::BURST_ACCUMULATING;
        return;  // processBuffer() will call accumulateBurstFrames() on next iteration
    }

    // Feed back pilot-corrected CFO to cached value (OFDM only).
    // MC-DPSK does not have pilot-based tracking; keep chirp-derived CFO.
    if (is_ofdm) {
        float corrected_cfo = waveform_->estimatedCFO();
        float current_cfo = last_cfo_.load();

        if (connected_) {
            constexpr float MAX_PILOT_CFO_DRIFT_HZ = 2.0f;
            float drift = corrected_cfo - current_cfo;
            if (std::abs(drift) > MAX_PILOT_CFO_DRIFT_HZ) {
                LOG_MODEM(WARN, "[%s] Pilot CFO drift clamped: %.2f → %.2f Hz (drift=%.2f, max=%.1f)",
                          log_prefix_.c_str(), current_cfo, corrected_cfo, drift, MAX_PILOT_CFO_DRIFT_HZ);
                corrected_cfo = current_cfo + std::copysign(MAX_PILOT_CFO_DRIFT_HZ, drift);
            }
        }

        if (std::abs(corrected_cfo - current_cfo) > 0.1f) {
            LOG_MODEM(INFO, "[%s] CFO updated: %.2f → %.2f Hz (pilot-corrected)",
                      log_prefix_.c_str(), current_cfo, corrected_cfo);
        }
        last_cfo_.store(corrected_cfo);
        sync_cfo_ = corrected_cfo;
    }

    constexpr size_t LDPC_BLOCK = v2::LDPC_CODEWORD_BITS;
    CodeRate rate = connected_ ? code_rate_ : CodeRate::R1_4;

    // ========================================================================
    // CW0 peek for MC-DPSK (non-OFDM, 1-CW buffer)
    // MC-DPSK starts with 1-CW buffer, needs to check total_cw before decode
    // ========================================================================

    if (pending_total_cw_ == 0 && !is_ofdm && soft_bits.size() >= LDPC_BLOCK) {
        int avail_cw = static_cast<int>(soft_bits.size() / LDPC_BLOCK);
        const bool disconnected_mc_dpsk = (!connected_ && mode_ == protocol::WaveformMode::MC_DPSK);
        const int min_handshake_cw = disconnected_mc_dpsk
            ? std::max<int>(2, v2::DataFrame::calculateCodewords(v2::ConnectFrame::PAYLOAD_SIZE, rate))
            : 0;

        // Use robust CW0 decode in disconnected MC-DPSK too.
        // Handshake CONNECT/CONNECT_ACK are multi-CW; if CW0 probe is brittle, we
        // fail to escalate and can lose the handshake frame intermittently.
        auto [peek_ok, peek_data] = robustDecodeSingleCW(
            soft_bits.data(), LDPC_BLOCK, rate, log_prefix_.c_str());
        if ((!peek_ok || peek_data.size() < 4 || peek_data[0] != 0x55 || peek_data[1] != 0x4C) &&
            use_mc_dpsk_channel_interleave_ && interleaver_) {
            std::vector<float> cw0(soft_bits.begin(), soft_bits.begin() + LDPC_BLOCK);
            auto cw0_deint = interleaver_->deinterleave(cw0);
            auto [peek_ok_i, peek_data_i] = robustDecodeSingleCW(
                cw0_deint.data(), LDPC_BLOCK, rate, log_prefix_.c_str());
            if (peek_ok_i && peek_data_i.size() >= 4 &&
                peek_data_i[0] == 0x55 && peek_data_i[1] == 0x4C) {
                peek_ok = true;
                peek_data = std::move(peek_data_i);
            }
        }

        if (peek_ok && peek_data.size() >= 4 && peek_data[0] == 0x55 && peek_data[1] == 0x4C) {
            auto hdr = v2::parseHeader(peek_data);
            if (hdr.valid) {
                int needed_cw = hdr.total_cw;
                const bool is_connect_handshake = v2::isConnectFrame(hdr.type) &&
                                                  hdr.type != v2::FrameType::DISCONNECT;
                if (is_connect_handshake && min_handshake_cw > 0 && needed_cw < min_handshake_cw) {
                    // Guard against CW0 false positives declaring an impossible 1-CW CONNECT.
                    LOG_MODEM(WARN, "[%s] CW0 peek: suspicious CONNECT total_cw=%d, forcing >=%d",
                              log_prefix_.c_str(), needed_cw, min_handshake_cw);
                    needed_cw = min_handshake_cw;
                }

                if (needed_cw > 1 && avail_cw < needed_cw) {
                    pending_total_cw_ = needed_cw;
                    state_ = DecoderState::SYNC_FOUND;
                    LOG_MODEM(INFO, "[%s] CW0 peek: need %d CWs (have %d) — waiting for %d samples",
                              log_prefix_.c_str(), needed_cw, avail_cw,
                              waveform_->getMinSamplesForCWCount(needed_cw));
                    return;
                }
            }
        }

        // Conservative disconnected-handshake fallback:
        // After non-PING detection, require enough samples for a full CONNECT/CACK
        // decode window even if CW0 probe is ambiguous at low SNR.
        if (min_handshake_cw > 1 && avail_cw < min_handshake_cw) {
            pending_total_cw_ = min_handshake_cw;
            state_ = DecoderState::SYNC_FOUND;
            LOG_MODEM(INFO, "[%s] MC-DPSK handshake fallback: waiting for %d CWs (have %d)",
                      log_prefix_.c_str(), min_handshake_cw, avail_cw);
            return;
        }
    }

    // ========================================================================
    // CW0 peek for connected OFDM (1-CW initial buffer)
    // Control frames (ACK/NACK) are 1-CW, non-interleaved → decode directly.
    // Data frames are 4-CW frame-interleaved → CW0 probe fails → escalate.
    // Skipped when burst interleave marker is latched (full buffer used instead).
    // ========================================================================
    if (pending_total_cw_ == 0 && is_ofdm && connected_
        && soft_bits.size() >= LDPC_BLOCK && soft_bits.size() < 2 * LDPC_BLOCK) {

        // Try R1/4 first — control frames are always encoded at R1/4 (hardened)
        bool peek_fell_through = false;
        {
            auto [ok_r14, data_r14] = robustDecodeSingleCW(
                soft_bits.data(), LDPC_BLOCK, CodeRate::R1_4, log_prefix_.c_str());
            size_t bpc_r14 = v2::getBytesPerCodeword(CodeRate::R1_4);
            if (ok_r14 && data_r14.size() >= 4
                && data_r14[0] == 0x55 && data_r14[1] == 0x4C) {
                if (data_r14.size() > bpc_r14) data_r14.resize(bpc_r14);
                auto hdr = v2::parseHeader(data_r14);
                if (hdr.valid && hdr.total_cw == 1) {
                    LOG_MODEM(DEBUG, "[%s] OFDM CW0 peek: R1/4 control fast-path OK",
                              log_prefix_.c_str());
                    peek_fell_through = true;
                } else if (hdr.valid && hdr.total_cw > 1) {
                    pending_total_cw_ = hdr.total_cw;
                    state_ = DecoderState::SYNC_FOUND;
                    LOG_MODEM(INFO, "[%s] OFDM CW0 peek: R1/4 shows %d CWs, escalating",
                              log_prefix_.c_str(), hdr.total_cw);
                    return;
                }
                // else: R1/4 decoded but invalid header — try code_rate_ fallback
            }
        }

        // Fallback: try code_rate_ if different from R1/4 and R1/4 didn't resolve
        if (!peek_fell_through && code_rate_ != CodeRate::R1_4) {
            size_t bpc = v2::getBytesPerCodeword(code_rate_);
            auto [ok_fb, data_fb] = robustDecodeSingleCW(
                soft_bits.data(), LDPC_BLOCK, code_rate_, log_prefix_.c_str());
            if (ok_fb && data_fb.size() >= 4
                && data_fb[0] == 0x55 && data_fb[1] == 0x4C) {
                if (data_fb.size() > bpc) data_fb.resize(bpc);
                auto hdr = v2::parseHeader(data_fb);
                if (hdr.valid && hdr.total_cw == 1) {
                    LOG_MODEM(DEBUG, "[%s] OFDM CW0 peek: 1-CW control (code_rate_ fallback)",
                              log_prefix_.c_str());
                    peek_fell_through = true;
                } else if (hdr.valid && hdr.total_cw > 1) {
                    pending_total_cw_ = hdr.total_cw;
                    state_ = DecoderState::SYNC_FOUND;
                    LOG_MODEM(INFO, "[%s] OFDM CW0 peek: need %d CWs, escalating",
                              log_prefix_.c_str(), hdr.total_cw);
                    return;
                } else {
                    pending_total_cw_ = v2::FIXED_FRAME_CODEWORDS;
                    state_ = DecoderState::SYNC_FOUND;
                    LOG_MODEM(INFO, "[%s] OFDM CW0 peek: invalid header, escalating to 4 CWs",
                              log_prefix_.c_str());
                    return;
                }
            }
        }

        if (!peek_fell_through) {
            pending_total_cw_ = v2::FIXED_FRAME_CODEWORDS;  // 4
            state_ = DecoderState::SYNC_FOUND;
            LOG_MODEM(INFO, "[%s] OFDM CW0 peek: decode failed, escalating to %d CWs",
                      log_prefix_.c_str(), pending_total_cw_);
            return;
        }
    }

    // ========================================================================
    // QAM partial-frame escalation:
    // In QAM mode, control-sized buffer gives ~3 CWs of soft bits (not enough
    // for 4-CW data frame). Escalate to full frame buffer if we have partial data.
    // ========================================================================
    if (pending_total_cw_ == 0 && is_ofdm && connected_
        && soft_bits.size() >= 2 * LDPC_BLOCK
        && soft_bits.size() < v2::FIXED_FRAME_CODEWORDS * LDPC_BLOCK) {
        bool is_qam_mod = (current_modulation_ == Modulation::QAM16 ||
                           current_modulation_ == Modulation::QAM32 ||
                           current_modulation_ == Modulation::QAM64 ||
                           current_modulation_ == Modulation::QAM256);
        if (is_qam_mod) {
            pending_total_cw_ = v2::FIXED_FRAME_CODEWORDS;  // 4
            state_ = DecoderState::SYNC_FOUND;
            LOG_MODEM(INFO, "[%s] QAM partial-frame: %zu soft bits (need %d), escalating to %d CWs",
                      log_prefix_.c_str(), soft_bits.size(),
                      v2::FIXED_FRAME_CODEWORDS * LDPC_BLOCK, pending_total_cw_);
            return;
        }
    }

    // Decode the frame using the soft bits
    if (!is_ofdm && !connected_ && pending_total_cw_ == 0 && soft_bits.size() < LDPC_BLOCK) {
        // Disconnected MC-DPSK with DBPSK can produce <1 CW soft bits in the
        // early window used for fast PING/PONG decision. Escalate to full 1-CW.
        pending_total_cw_ = 1;
        state_ = DecoderState::SYNC_FOUND;
        LOG_MODEM(INFO, "[%s] MC-DPSK early window: %zu soft bits (<%zu), waiting for full 1-CW (%d samples)",
                  log_prefix_.c_str(), soft_bits.size(), LDPC_BLOCK,
                  waveform_->getMinSamplesForCWCount(1));
        return;
    }

    if (!is_ofdm && !connected_ && mode_ == protocol::WaveformMode::MC_DPSK) {
        // After early-window expansion to 1-CW, require a full CONNECT/CACK
        // window before final decode to avoid latching onto 1-CW false positives.
        int min_handshake_cw = std::max<int>(
            2, v2::DataFrame::calculateCodewords(v2::ConnectFrame::PAYLOAD_SIZE, CodeRate::R1_4));
        int avail_cw = static_cast<int>(soft_bits.size() / LDPC_BLOCK);
        if (avail_cw < min_handshake_cw && pending_total_cw_ < min_handshake_cw) {
            pending_total_cw_ = min_handshake_cw;
            state_ = DecoderState::SYNC_FOUND;
            LOG_MODEM(INFO, "[%s] MC-DPSK handshake window: waiting for %d CWs (have %d)",
                      log_prefix_.c_str(), min_handshake_cw, avail_cw);
            return;
        }
    }

    DecodeResult result = decodeFrame(soft_bits, sync_snr_, sync_cfo_);

    // MC-DPSK handshake salvage:
    // If header CW decoded but full frame is incomplete, escalate to exact CW count
    // instead of treating this as a hard decode failure.
    if (!result.success && !is_ofdm && pending_total_cw_ == 0 && result.codewords_ok > 0) {
        auto hdr = v2::parseHeader(result.frame_data);
        if (hdr.valid && hdr.total_cw > 1) {
            int avail_cw = static_cast<int>(soft_bits.size() / LDPC_BLOCK);
            if (avail_cw < hdr.total_cw) {
                pending_total_cw_ = hdr.total_cw;
                state_ = DecoderState::SYNC_FOUND;
                LOG_MODEM(INFO, "[%s] MC-DPSK header salvage: need %d CWs (have %d) — waiting for %d samples",
                          log_prefix_.c_str(), hdr.total_cw, avail_cw,
                          waveform_->getMinSamplesForCWCount(hdr.total_cw));
                return;
            }
        }
    }

    // Disconnected-handshake modulation fallback:
    // If CONNECT/CONNECT_ACK CW0 still fails, retry once with alternate differential
    // modulation (DBPSK<->DQPSK) before giving up this sync candidate.
    if (!result.success && result.codewords_ok == 0 && !is_ofdm && !connected_) {
        Modulation primary_mod = current_modulation_;
        CodeRate primary_rate = connected_ ? code_rate_ : CodeRate::R1_4;
        Modulation retry_mod = Modulation::AUTO;

        if (primary_mod == Modulation::DQPSK) {
            retry_mod = Modulation::DBPSK;
        } else if (primary_mod == Modulation::DBPSK) {
            retry_mod = Modulation::DQPSK;
        }

        if (retry_mod != Modulation::AUTO) {
            LOG_MODEM(INFO, "[%s] MC-DPSK handshake decode retry with alternate modulation: %s -> %s",
                      log_prefix_.c_str(), modulationToString(primary_mod),
                      modulationToString(retry_mod));

            waveform_->configure(retry_mod, CodeRate::R1_4);
            waveform_->setFrequencyOffset(sync_cfo_);
            bool retry_ok = waveform_->process(SampleSpan(frame_buffer.data(), frame_buffer.size()));

            if (retry_ok) {
                captureConstellationSnapshot();
                auto retry_bits = waveform_->getSoftBits();
                if (retry_bits.size() >= LDPC_BLOCK) {
                    size_t retry_bytes_per_cw = v2::getBytesPerCodeword(CodeRate::R1_4);
                    auto retry_result = decodeMCDPSKFrame(retry_bits, CodeRate::R1_4,
                                                          retry_bytes_per_cw, sync_snr_, sync_cfo_);
                    // Handshake decode recovery must require a full frame.
                    // Accepting partial CWs here can consume buffer on false positives
                    // and cause intermittent CONNECT_ACK timeout flakes.
                    if (retry_result.success) {
                        LOG_MODEM(INFO, "[%s] MC-DPSK handshake decode recovered with %s",
                                  log_prefix_.c_str(), modulationToString(retry_mod));
                        result = std::move(retry_result);
                    }
                }
            }

            // Restore runtime waveform profile after one-shot handshake retry.
            waveform_->configure(primary_mod, primary_rate);
        }
    }

    // MC-DPSK disconnected handshake sync recovery:
    // If CONNECT/CONNECT_ACK CW0 decode fails, retry nearby sync offsets and
    // modulation hypotheses before timing out the whole handshake attempt.
    if (!result.success && result.codewords_ok == 0 && !is_ofdm && !connected_) {
        const int retry_deltas[] = {8, -8, 16, -16, 24, -24, 32, -32, 48, -48, 64, -64};
        Modulation primary_mod = current_modulation_;
        CodeRate primary_rate = CodeRate::R1_4;
        Modulation alt_mod = Modulation::AUTO;
        if (primary_mod == Modulation::DQPSK) {
            alt_mod = Modulation::DBPSK;
        } else if (primary_mod == Modulation::DBPSK) {
            alt_mod = Modulation::DQPSK;
        }

        auto ringPosToAbsolute = [this](size_t ring_pos) -> size_t {
            if (total_fed_ < MAX_BUFFER_SAMPLES) {
                return ring_pos;
            }
            const size_t oldest_abs = total_fed_ - MAX_BUFFER_SAMPLES;
            const size_t oldest_pos = write_pos_;
            const size_t offset = (ring_pos >= oldest_pos)
                ? (ring_pos - oldest_pos)
                : (MAX_BUFFER_SAMPLES - oldest_pos + ring_pos);
            return oldest_abs + offset;
        };

        bool recovered = false;
        int recovered_delta = 0;
        Modulation recovered_mod = primary_mod;

        for (int delta : retry_deltas) {
            size_t retry_sync = (sync_position_ + MAX_BUFFER_SAMPLES + delta) % MAX_BUFFER_SAMPLES;
            size_t retry_len = frame_len;
            std::vector<float> retry_buffer;

            {
                std::lock_guard<std::mutex> lock(buffer_mutex_);
                size_t available;
                if (write_pos_ >= retry_sync) {
                    available = write_pos_ - retry_sync;
                } else {
                    available = MAX_BUFFER_SAMPLES - retry_sync + write_pos_;
                }
                retry_len = std::min(retry_len, available);
                if (retry_len < static_cast<size_t>(waveform_->getMinSamplesForControlFrame())) {
                    continue;
                }
                retry_buffer.resize(retry_len);
                for (size_t i = 0; i < retry_len; ++i) {
                    retry_buffer[i] = buffer_[(retry_sync + i) % MAX_BUFFER_SAMPLES];
                }
            }

            for (int pass = 0; pass < 2; ++pass) {
                Modulation trial_mod = (pass == 0) ? primary_mod : alt_mod;
                if (trial_mod == Modulation::AUTO) {
                    continue;
                }

                waveform_->configure(trial_mod, primary_rate);
                waveform_->reset();
                waveform_->setAbsoluteTrainingPosition(ringPosToAbsolute(retry_sync));
                waveform_->setFrequencyOffset(sync_cfo_);

                if (!waveform_->process(SampleSpan(retry_buffer.data(), retry_buffer.size()))) {
                    continue;
                }
                captureConstellationSnapshot();

                auto retry_bits = waveform_->getSoftBits();
                if (retry_bits.empty()) {
                    continue;
                }

                auto retry_result = decodeFrame(retry_bits, sync_snr_, sync_cfo_);
                // Same rule as above: only accept full-frame recovery in disconnected
                // handshake path to avoid latching onto partial/false-positive CW0.
                if (retry_result.success) {
                    result = std::move(retry_result);
                    sync_position_ = retry_sync;
                    frame_len = retry_len;
                    recovered = true;
                    recovered_delta = delta;
                    recovered_mod = trial_mod;
                    last_fading_index_.store(waveform_->getFadingIndex());
                    break;
                }
            }

            if (recovered) {
                break;
            }
        }

        waveform_->configure(primary_mod, primary_rate);

        if (recovered) {
            LOG_MODEM(INFO,
                      "[%s] MC-DPSK handshake sync recovery: delta=%+d, mod=%s",
                      log_prefix_.c_str(), recovered_delta,
                      modulationToString(recovered_mod));
        } else {
            LOG_MODEM(DEBUG, "[%s] MC-DPSK handshake sync recovery: no nearby offset decoded",
                      log_prefix_.c_str());
        }
    }

    // ========================================================================
    // Small-frame recovery for OFDM connected mode:
    // If full 4-CW buffer decode failed, the frame might be a small non-data
    // frame (e.g. DISCONNECT = 2 CWs at R1/2) where trailing noise symbols
    // degraded LLR quality. Retry with 1-CW peek to determine actual size.
    // ========================================================================

    if (!result.success && is_ofdm && connected_) {
        size_t one_cw_s = static_cast<size_t>(waveform_->getMinSamplesForControlFrame());
        if (one_cw_s <= frame_buffer.size()) {
            waveform_->setFrequencyOffset(sync_cfo_);
            if (waveform_->process(SampleSpan(frame_buffer.data(), one_cw_s))) {
                captureConstellationSnapshot();
            }
            auto short_bits = waveform_->getSoftBits();

            if (short_bits.size() >= LDPC_BLOCK) {
                // Try R1/4 first (control frames hardened), then code_rate_ fallback
                auto trySmallFrame = [&](CodeRate sr) -> bool {
                    auto [ok2, data2] = robustDecodeSingleCW(
                        short_bits.data(), LDPC_BLOCK, sr, log_prefix_.c_str());
                    if (!ok2 || data2.size() < 4 || data2[0] != 0x55 || data2[1] != 0x4C)
                        return false;
                    auto hdr2 = v2::parseHeader(data2);
                    if (hdr2.valid && hdr2.total_cw == 1) {
                        // 1-CW control frame — decode via decodeFrame (has R1/4 fast-path)
                        result = decodeFrame(short_bits, sync_snr_, sync_cfo_);
                        frame_len = one_cw_s;
                        return true;
                    } else if (hdr2.valid && hdr2.total_cw > 1 && hdr2.total_cw < v2::FIXED_FRAME_CODEWORDS) {
                        // Variable-CW frame (2-3 CWs) — reprocess with exact size
                        size_t exact_size = static_cast<size_t>(
                            waveform_->getMinSamplesForCWCount(hdr2.total_cw));
                        exact_size = std::min(exact_size, frame_buffer.size());

                        LOG_MODEM(INFO, "[%s] Small-frame recovery: reprocessing %zu samples (%d CWs)",
                                  log_prefix_.c_str(), exact_size, hdr2.total_cw);
                        waveform_->setFrequencyOffset(sync_cfo_);
                        if (waveform_->process(SampleSpan(frame_buffer.data(), exact_size))) {
                            captureConstellationSnapshot();
                        }
                        auto recovered_bits = waveform_->getSoftBits();
                        result = decodeFrame(recovered_bits, sync_snr_, sync_cfo_);
                        frame_len = exact_size;
                        return true;
                    }
                    return false;
                };

                if (!trySmallFrame(CodeRate::R1_4) && rate != CodeRate::R1_4) {
                    trySmallFrame(rate);
                }
            }
        }
    }

    // Multi-candidate light-sync recovery (connected OFDM):
    // If decode fails at the detected sync point, retry nearby timing candidates.
    // detectDataSync() scans with coarse steps, and fading can shift the best
    // decode point by a few samples even when correlation looks valid.
    if (!result.success && result.codewords_ok == 0 && is_ofdm && connected_) {
        const int retry_deltas[] = {8, -8, 16, -16, 24, -24, 32, -32};
        bool recovered = false;
        int recovered_delta = 0;
        uint64_t recovery_attempts = 0;

        auto ringPosToAbsolute = [this](size_t ring_pos) -> size_t {
            if (total_fed_ < MAX_BUFFER_SAMPLES) {
                return ring_pos;
            }
            const size_t oldest_abs = total_fed_ - MAX_BUFFER_SAMPLES;
            const size_t oldest_pos = write_pos_;
            const size_t offset = (ring_pos >= oldest_pos)
                ? (ring_pos - oldest_pos)
                : (MAX_BUFFER_SAMPLES - oldest_pos + ring_pos);
            return oldest_abs + offset;
        };

        for (int delta : retry_deltas) {
            recovery_attempts++;
            size_t retry_sync = (sync_position_ + MAX_BUFFER_SAMPLES + delta) % MAX_BUFFER_SAMPLES;

            std::vector<float> retry_buffer;
            size_t retry_len = frame_len;
            {
                std::lock_guard<std::mutex> lock(buffer_mutex_);
                size_t available;
                if (write_pos_ >= retry_sync) {
                    available = write_pos_ - retry_sync;
                } else {
                    available = MAX_BUFFER_SAMPLES - retry_sync + write_pos_;
                }
                retry_len = std::min(retry_len, available);
                if (retry_len == 0) {
                    continue;
                }
                retry_buffer.resize(retry_len);
                for (size_t i = 0; i < retry_len; ++i) {
                    retry_buffer[i] = buffer_[(retry_sync + i) % MAX_BUFFER_SAMPLES];
                }
            }

            waveform_->reset();
            waveform_->setAbsoluteTrainingPosition(ringPosToAbsolute(retry_sync));
            waveform_->setFrequencyOffset(sync_cfo_);
            bool retry_ok = waveform_->process(SampleSpan(retry_buffer.data(), retry_buffer.size()));
            if (!retry_ok) {
                continue;
            }
            captureConstellationSnapshot();
            auto retry_bits = waveform_->getSoftBits();
            if (retry_bits.empty()) {
                continue;
            }

            auto retry_result = decodeFrame(retry_bits, sync_snr_, sync_cfo_);
            if (!(retry_result.success || retry_result.codewords_ok > 0)) {
                continue;
            }

            LOG_MODEM(INFO, "[%s] Multi-candidate sync recovery: delta=%+d samples succeeded",
                      log_prefix_.c_str(), delta);

            // Keep CFO tracking consistent with the accepted retry candidate.
            float corrected_cfo = waveform_->estimatedCFO();
            float current_cfo = last_cfo_.load();
            constexpr float MAX_PILOT_CFO_DRIFT_HZ = 2.0f;
            float drift = corrected_cfo - current_cfo;
            if (std::abs(drift) > MAX_PILOT_CFO_DRIFT_HZ) {
                corrected_cfo = current_cfo + std::copysign(MAX_PILOT_CFO_DRIFT_HZ, drift);
            }
            last_cfo_.store(corrected_cfo);
            sync_cfo_ = corrected_cfo;
            last_fading_index_.store(waveform_->getFadingIndex());

            sync_position_ = retry_sync;
            frame_len = retry_len;
            result = std::move(retry_result);
            recovered_delta = delta;
            recovered = true;
            break;
        }

        if (recovery_attempts > 0) {
            std::lock_guard<std::mutex> slock(stats_mutex_);
            stats_.sync_recovery_attempts += recovery_attempts;
            if (recovered) {
                stats_.sync_recovery_successes++;
                switch (recovered_delta) {
                    case 8: stats_.sync_recovery_delta_p8++; break;
                    case -8: stats_.sync_recovery_delta_m8++; break;
                    case 16: stats_.sync_recovery_delta_p16++; break;
                    case -16: stats_.sync_recovery_delta_m16++; break;
                    case 24: stats_.sync_recovery_delta_p24++; break;
                    case -24: stats_.sync_recovery_delta_m24++; break;
                    case 32: stats_.sync_recovery_delta_p32++; break;
                    case -32: stats_.sync_recovery_delta_m32++; break;
                    default: break;
                }
            }
        }

        if (!recovered) {
            LOG_MODEM(DEBUG, "[%s] Multi-candidate sync recovery: no nearby offset decoded",
                      log_prefix_.c_str());
        }
    }

    auto decode_end = std::chrono::steady_clock::now();
    float ms = std::chrono::duration<float, std::milli>(decode_end - decode_start).count();

    {
        std::lock_guard<std::mutex> slock(stats_mutex_);
        if (result.success) stats_.frames_decoded++;
        else stats_.frames_failed++;
        stats_.avg_decode_time_ms = 0.9f * stats_.avg_decode_time_ms + 0.1f * ms;
    }

    if (result.success || result.codewords_ok > 0) {
        {
            std::lock_guard<std::mutex> qlock(queue_mutex_);
            frame_queue_.push(result);
        }
        if (result.success && frame_callback_) frame_callback_(result);

        LOG_MODEM(INFO, "[%s] StreamingDecoder: Frame decoded, %d/%d CWs, SNR=%.1f dB, CFO=%.1f Hz",
                  log_prefix_.c_str(), result.codewords_ok, result.codewords_ok + result.codewords_failed,
                  sync_snr_, sync_cfo_);
    } else {
        LOG_MODEM(WARN, "[%s] StreamingDecoder: Decode failed (cw_ok=%d, cw_fail=%d, is_ping=%d)",
                  log_prefix_.c_str(), result.codewords_ok, result.codewords_failed, result.is_ping ? 1 : 0);
    }

    // Calculate consumed samples based on actual frame content
    // For non-data frames (control/connect), use exact sample count to avoid eating into next frame
    bool is_non_data_frame = false;
    if (result.success && result.frame_data.size() >= 3) {
        auto ft = static_cast<v2::FrameType>(result.frame_data[2]);
        is_non_data_frame = v2::isControlFrame(ft) || v2::isConnectFrame(ft);
    }

    size_t consumed = frame_len;
    if (result.success && is_non_data_frame && is_ofdm) {
        // Use exact sample count for the actual CW count
        int actual_cw = result.codewords_ok + result.codewords_failed;
        if (actual_cw > 0) {
            size_t exact_consumed = static_cast<size_t>(waveform_->getMinSamplesForCWCount(actual_cw));
            if (exact_consumed < consumed) {
                LOG_MODEM(INFO, "[%s] Non-data frame (%d CWs): advancing %zu samples (not %zu)",
                          log_prefix_.c_str(), actual_cw, exact_consumed, consumed);
                consumed = exact_consumed;
            }
        }
    }
    size_t next_block_pos = (sync_position_ + consumed) % MAX_BUFFER_SAMPLES;

    // After successful decode in connected OFDM mode, check for burst continuation
    // MC-DPSK never enters burst mode (uses window=1, full chirp preamble)
    // Skip burst continuation for non-data frames (control/connect are standalone)
    // Note: when burst interleaving is active, interleaved groups enter BURST_ACCUMULATING
    // above and return early. Non-interleaved bursts (< 4 frames) still need continuation.
    if (result.success && connected_ && is_ofdm && !is_non_data_frame) {
        size_t min_block = static_cast<size_t>(waveform_->getMinSamplesForFrame());

        // Loop to decode multiple burst continuation blocks
        while (burst_blocks_decoded_ < MAX_BURST_BLOCKS) {
            // Check if there are enough samples for another block
            size_t next_available;
            {
                std::lock_guard<std::mutex> lock(buffer_mutex_);
                if (write_pos_ >= next_block_pos) {
                    next_available = write_pos_ - next_block_pos;
                } else {
                    next_available = MAX_BUFFER_SAMPLES - next_block_pos + write_pos_;
                }
            }

            if (next_available < min_block) break;  // Not enough samples, burst over

            // Copy next block samples
            std::vector<float> next_block(min_block);
            {
                std::lock_guard<std::mutex> lock(buffer_mutex_);
                for (size_t i = 0; i < min_block; i++) {
                    next_block[i] = buffer_[(next_block_pos + i) % MAX_BUFFER_SAMPLES];
                }
            }

            // Check RMS energy to detect if there's actually a signal
            float next_rms = 0.0f;
            size_t burst_check_start = std::min(size_t(1024), min_block);  // Skip training area
            size_t burst_check_len = std::min(min_block - burst_check_start, size_t(5000));
            if (burst_check_len > 0) {
                for (size_t i = 0; i < burst_check_len; i++) {
                    next_rms += next_block[burst_check_start + i] * next_block[burst_check_start + i];
                }
                next_rms = std::sqrt(next_rms / burst_check_len);
            }

            constexpr float BURST_ENERGY_THRESHOLD = 0.04f;
            if (next_rms < BURST_ENERGY_THRESHOLD) break;  // No energy, burst over

            // Energy present - try to decode as continuation block
            burst_blocks_decoded_++;
            LOG_MODEM(INFO, "[%s] Burst continuation: block %d, RMS=%.4f, pos=%zu",
                      log_prefix_.c_str(), burst_blocks_decoded_, next_rms, next_block_pos);

            waveform_->setFrequencyOffset(sync_cfo_);
            bool next_ok = waveform_->process(SampleSpan(next_block.data(), next_block.size()));

            if (!next_ok) break;  // Process failed, burst over
            captureConstellationSnapshot();

            auto next_soft_bits = waveform_->getSoftBits();
            if (next_soft_bits.empty()) break;

            // Update CFO from pilot tracking
            float next_corrected_cfo = waveform_->estimatedCFO();
            float cfo_drift = next_corrected_cfo - sync_cfo_;
            constexpr float MAX_BURST_CFO_DRIFT_HZ = 2.0f;
            if (std::abs(cfo_drift) > MAX_BURST_CFO_DRIFT_HZ) {
                next_corrected_cfo = sync_cfo_ + std::copysign(MAX_BURST_CFO_DRIFT_HZ, cfo_drift);
            }
            sync_cfo_ = next_corrected_cfo;
            last_cfo_.store(next_corrected_cfo);
            last_fading_index_.store(waveform_->getFadingIndex());

            // Decode the continuation block
            DecodeResult next_result = decodeFrame(next_soft_bits, sync_snr_, sync_cfo_);

            {
                std::lock_guard<std::mutex> slock(stats_mutex_);
                if (next_result.success) stats_.frames_decoded++;
                else stats_.frames_failed++;
            }

            if (next_result.success || next_result.codewords_ok > 0) {
                {
                    std::lock_guard<std::mutex> qlock(queue_mutex_);
                    frame_queue_.push(next_result);
                }
                if (next_result.success && frame_callback_) frame_callback_(next_result);

                LOG_MODEM(INFO, "[%s] Burst block %d decoded: %d/%d CWs",
                          log_prefix_.c_str(), burst_blocks_decoded_,
                          next_result.codewords_ok, next_result.codewords_ok + next_result.codewords_failed);
            }

            // Advance position for next iteration (or final correlation_pos_)
            sync_position_ = next_block_pos;
            next_block_pos = (next_block_pos + min_block) % MAX_BUFFER_SAMPLES;

            // If decode failed completely, stop the burst
            if (!next_result.success && next_result.codewords_ok == 0) break;
        }
    }

    // Burst over (or non-burst) - skip past everything we decoded and return to SEARCHING
    burst_blocks_decoded_ = 0;
    {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        correlation_pos_ = (next_block_pos) % MAX_BUFFER_SAMPLES;
        last_decoded_sync_pos_ = sync_position_;
    }

    state_ = DecoderState::SEARCHING;
}

bool StreamingDecoder::hasFrame() const {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    return !frame_queue_.empty();
}

DecodeResult StreamingDecoder::getFrame() {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (frame_queue_.empty()) return DecodeResult{};
    DecodeResult r = std::move(frame_queue_.front());
    frame_queue_.pop();
    return r;
}

// ============================================================================
// MODE CONTROL
// ============================================================================

void StreamingDecoder::setMode(protocol::WaveformMode mode, bool connected) {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    std::lock_guard<std::mutex> waveform_lock(waveform_mutex_);

    if (mode_ == mode && connected_ == connected) {
        return;
    }

    LOG_MODEM(DEBUG, "StreamingDecoder::setMode old=(%s,%s) new=(%s,%s) state=%d corr_pos=%zu write_pos=%zu",
              protocol::waveformModeToString(mode_), connected_ ? "connected" : "disconnected",
              protocol::waveformModeToString(mode), connected ? "connected" : "disconnected",
              static_cast<int>(state_), correlation_pos_, write_pos_);

    auto old_mode = mode_;
    mode_ = mode;
    connected_ = connected;

    if (mode == protocol::WaveformMode::MC_DPSK) {
        waveform_ = WaveformFactory::createMCDPSK(mc_dpsk_carriers_);
        if (!connected_) {
            // Handshake path is transmitted as DBPSK R1/4; keep RX sizing/config
            // aligned so CW0 capture window is long enough under 2x/4x spreading.
            current_modulation_ = Modulation::DBPSK;
            code_rate_ = CodeRate::R1_4;
        }
        if (waveform_) {
            waveform_->configure(current_modulation_, code_rate_);
        }
        // Apply current spreading mode to the new waveform
        if (spreading_mode_ != SpreadingMode::NONE) {
            auto* mc_dpsk = dynamic_cast<MCDPSKWaveform*>(waveform_.get());
            if (mc_dpsk) {
                mc_dpsk->setSpreadingMode(spreading_mode_);
            }
        }
    } else {
        waveform_ = WaveformFactory::create(mode);
    }

    size_t bps = mc_dpsk_carriers_ * 2;
    if (mode == protocol::WaveformMode::OFDM_CHIRP || mode == protocol::WaveformMode::OFDM_COX) {
        bps = 60;
    }
    interleaver_ = std::make_unique<ChannelInterleaver>(bps, v2::LDPC_CODEWORD_BITS);

    auto old_state = state_;
    state_ = DecoderState::SEARCHING;
    pending_total_cw_ = 0;
    sync_reject_streak_ = 0;
    zc_chirp_fallback_attempts_ = 0;
    constellation_cache_.clear();
    constellation_cache_time_ = std::chrono::steady_clock::time_point{};

    // Clear burst interleave state on mode change
    burst_soft_buffer_.clear();
    use_burst_interleave_ = false;  // Re-enabled by caller if needed

    if (connected_) {
        connected_since_ = std::chrono::steady_clock::now();
    } else {
        connected_since_ = std::chrono::steady_clock::time_point{};
        // Clear chase cache on disconnect - cached soft bits from old connection are invalid
        if (chase_cache_) {
            chase_cache_->clear();
        }
    }

    // CRITICAL: Reset correlation_pos_ to current write position
    // Otherwise we'll search old data from previous mode
    correlation_pos_ = write_pos_;

    LOG_MODEM(INFO, "StreamingDecoder: Mode=%s (%s), reset corr_pos=%zu (old_mode=%s old_state=%d)",
              protocol::waveformModeToString(mode), connected ? "connected" : "disconnected",
              correlation_pos_, protocol::waveformModeToString(old_mode), static_cast<int>(old_state));
}

void StreamingDecoder::setMCDPSKCarriers(int n) {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    std::lock_guard<std::mutex> waveform_lock(waveform_mutex_);
    if (mc_dpsk_carriers_ == n) return;
    mc_dpsk_carriers_ = n;
    if (mode_ == protocol::WaveformMode::MC_DPSK) {
        waveform_ = WaveformFactory::createMCDPSK(n);
        // CRITICAL: Configure with current modulation (DBPSK vs DQPSK)
        // Without this, waveform defaults to DQPSK (2 bits/symbol)
        if (waveform_) {
            waveform_->configure(current_modulation_, code_rate_);
            // Apply current spreading mode to the new waveform
            if (spreading_mode_ != SpreadingMode::NONE) {
                auto* mc_dpsk = dynamic_cast<MCDPSKWaveform*>(waveform_.get());
                if (mc_dpsk) {
                    mc_dpsk->setSpreadingMode(spreading_mode_);
                }
            }
        }
        // Update interleaver with correct bits_per_symbol for current modulation
        int bits_per_symbol = getBitsPerSymbol(current_modulation_);
        interleaver_ = std::make_unique<ChannelInterleaver>(n * bits_per_symbol, v2::LDPC_CODEWORD_BITS);
    }
}

void StreamingDecoder::setSpreadingMode(SpreadingMode mode) {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    std::lock_guard<std::mutex> waveform_lock(waveform_mutex_);
    if (spreading_mode_ == mode) return;
    spreading_mode_ = mode;

    // Apply to MC-DPSK waveform if currently in that mode
    if (mode_ == protocol::WaveformMode::MC_DPSK && waveform_) {
        auto* mc_dpsk = dynamic_cast<MCDPSKWaveform*>(waveform_.get());
        if (mc_dpsk) {
            mc_dpsk->setSpreadingMode(mode);
        }
    }

    LOG_MODEM(INFO, "StreamingDecoder: Spreading mode set to %s",
              mode == SpreadingMode::TIME_4X ? "4×" :
              mode == SpreadingMode::TIME_2X ? "2×" : "NONE");
}

void StreamingDecoder::setChaseEnabled(bool enable) {
    if (chase_cache_) {
        chase_cache_->setEnabled(enable);
        LOG_MODEM(INFO, "StreamingDecoder: Chase combining %s",
                  enable ? "enabled" : "disabled");
    }
}

bool StreamingDecoder::getChaseEnabled() const {
    return chase_cache_ ? chase_cache_->isEnabled() : false;
}

fec::ChaseCache::Stats StreamingDecoder::getChaseStats() const {
    return chase_cache_ ? chase_cache_->getStats() : fec::ChaseCache::Stats{};
}

void StreamingDecoder::setOFDMConfig(const ModemConfig& config) {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    std::lock_guard<std::mutex> waveform_lock(waveform_mutex_);

    // Store carrier counts regardless of current mode (used when switching to OFDM later)
    ofdm_carriers_ = config.num_carriers;
    ofdm_data_carriers_ = config.getDataCarriers();

    // Only recreate waveform if currently in an OFDM mode.
    // When in MC-DPSK or MFSK mode (e.g., disconnected waiting for PINGs), do NOT replace
    // the waveform — that would break chirp-based sync detection.
    if (mode_ == protocol::WaveformMode::MC_DPSK ||
        mode_ == protocol::WaveformMode::MFSK) {
        LOG_MODEM(INFO, "StreamingDecoder: OFDM config stored (mode=%s, not replacing waveform)",
                  protocol::waveformModeToString(mode_));
        return;
    }

    if (mode_ == protocol::WaveformMode::OFDM_CHIRP) {
        waveform_ = std::make_unique<OFDMChirpWaveform>(config);
        LOG_MODEM(INFO, "StreamingDecoder: OFDM_CHIRP config set (FFT=%d, carriers=%d)",
                  config.fft_size, config.num_carriers);
    } else {
        waveform_ = std::make_unique<OFDMNvisWaveform>(config);
        LOG_MODEM(INFO, "StreamingDecoder: OFDM_COX config set (FFT=%d, carriers=%d)",
                  config.fft_size, config.num_carriers);
    }

    // Update interleaver for new carrier count (using current modulation)
    size_t bps = ofdm_data_carriers_ * getBitsPerSymbol(current_modulation_);
    interleaver_ = std::make_unique<ChannelInterleaver>(bps, v2::LDPC_CODEWORD_BITS);
}

void StreamingDecoder::setConnectedOFDMMode(protocol::WaveformMode mode,
                                            const ModemConfig& config,
                                            Modulation mod,
                                            CodeRate rate) {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    std::lock_guard<std::mutex> waveform_lock(waveform_mutex_);

    mode_ = mode;
    connected_ = true;
    code_rate_ = rate;
    current_modulation_ = mod;
    ofdm_carriers_ = config.num_carriers;
    ofdm_data_carriers_ = config.getDataCarriers();

    if (mode_ == protocol::WaveformMode::OFDM_CHIRP) {
        waveform_ = std::make_unique<OFDMChirpWaveform>(config);
    } else {
        waveform_ = std::make_unique<OFDMNvisWaveform>(config);
    }

    if (waveform_) {
        waveform_->configure(mod, rate);
    }

    // Query waveform for effective pilot layout after configure().
    int pilot_spacing = waveform_ ? waveform_->getPilotSpacing() : 0;
    if (pilot_spacing > 0) {
        int pilot_count = (ofdm_carriers_ + pilot_spacing - 1) / pilot_spacing;
        ofdm_data_carriers_ = ofdm_carriers_ - pilot_count;
    } else {
        ofdm_data_carriers_ = ofdm_carriers_;
    }

    size_t bps = ofdm_data_carriers_ * getBitsPerSymbol(mod);
    interleaver_ = std::make_unique<ChannelInterleaver>(bps, v2::LDPC_CODEWORD_BITS);

    state_ = DecoderState::SEARCHING;
    pending_total_cw_ = 0;
    sync_reject_streak_ = 0;
    zc_chirp_fallback_attempts_ = 0;
    connected_since_ = std::chrono::steady_clock::now();
    constellation_cache_.clear();
    constellation_cache_time_ = std::chrono::steady_clock::time_point{};
    burst_soft_buffer_.clear();
    correlation_pos_ = write_pos_;

    LOG_MODEM(INFO, "StreamingDecoder: connected OFDM mode=%s, mod=%s, rate=%s, carriers=%d data=%d bps=%zu",
              protocol::waveformModeToString(mode_),
              modulationToString(mod), codeRateToString(rate),
              ofdm_carriers_, ofdm_data_carriers_, bps);
}

void StreamingDecoder::setDataMode(Modulation mod, CodeRate rate) {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    std::lock_guard<std::mutex> waveform_lock(waveform_mutex_);
    code_rate_ = rate;
    current_modulation_ = mod;
    if (waveform_) waveform_->configure(mod, rate);

    // After configure(), the waveform has updated pilot config
    // Query waveform for actual pilot_spacing (coherent modes use denser pilots)
    if (mode_ != protocol::WaveformMode::MC_DPSK &&
        mode_ != protocol::WaveformMode::MFSK && waveform_) {
        int pilot_spacing = waveform_->getPilotSpacing();
        if (pilot_spacing > 0) {
            int pilot_count = (ofdm_carriers_ + pilot_spacing - 1) / pilot_spacing;
            ofdm_data_carriers_ = ofdm_carriers_ - pilot_count;
        } else {
            ofdm_data_carriers_ = ofdm_carriers_;
        }
    }

    // Update interleaver for new modulation
    // Use data carriers (not total) to account for pilot overhead
    // MFSK uses tones not carriers - use a reasonable default
    int carriers = mc_dpsk_carriers_;
    if (mode_ == protocol::WaveformMode::OFDM_CHIRP || mode_ == protocol::WaveformMode::OFDM_COX) {
        carriers = ofdm_data_carriers_;
    } else if (mode_ == protocol::WaveformMode::MFSK) {
        carriers = 8;  // Default MFSK tone count
    }
    size_t bps = carriers * getBitsPerSymbol(mod);
    interleaver_ = std::make_unique<ChannelInterleaver>(bps, v2::LDPC_CODEWORD_BITS);
    LOG_MODEM(INFO, "StreamingDecoder: interleaver updated for %s (%zu bits/symbol)",
              modulationToString(mod), bps);
}

void StreamingDecoder::setCodecType(fec::CodecType type) {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    if (codec_type_ == type) return;
    codec_type_ = type;
    codec_ = fec::CodecFactory::create(type, code_rate_);
}

// ============================================================================
// STATUS
// ============================================================================

float StreamingDecoder::getBufferFillPercent() const {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    size_t used = std::min(total_fed_, MAX_BUFFER_SAMPLES);
    return 100.0f * used / MAX_BUFFER_SAMPLES;
}

DecoderStats StreamingDecoder::getStats() const {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    return stats_;
}

StreamingDecoder::DecoderConfig StreamingDecoder::getConfig() const {
    std::lock_guard<std::mutex> lock(buffer_mutex_);

    DecoderConfig cfg;
    cfg.mode = mode_;
    cfg.modulation = current_modulation_;
    cfg.code_rate = code_rate_;
    cfg.num_carriers = ofdm_carriers_;
    cfg.data_carriers = ofdm_data_carriers_;
    cfg.bits_per_symbol = ofdm_data_carriers_ * getBitsPerSymbol(current_modulation_);

    // Get pilot config from waveform (coherent modes use denser pilots)
    cfg.use_pilots = true;
    cfg.pilot_spacing = waveform_ ? waveform_->getPilotSpacing() : 10;

    // Interleaving settings
    cfg.use_channel_interleave = use_channel_interleave_;
    cfg.use_frame_interleave = (mode_ == protocol::WaveformMode::OFDM_CHIRP ||
                                 mode_ == protocol::WaveformMode::OFDM_COX);

    return cfg;
}

size_t StreamingDecoder::samplesInBuffer() const {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    return std::min(total_fed_, MAX_BUFFER_SAMPLES);
}

bool StreamingDecoder::isSynced() const {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    return state_ == DecoderState::SYNC_FOUND || state_ == DecoderState::DECODING
        || state_ == DecoderState::BURST_ACCUMULATING;
}

void StreamingDecoder::captureConstellationSnapshot() {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    if (!waveform_) {
        return;
    }

    auto symbols = waveform_->getConstellationSymbols();
    if (!symbols.empty()) {
        constellation_cache_ = std::move(symbols);
        constellation_cache_time_ = std::chrono::steady_clock::now();
    }
}

std::vector<std::complex<float>> StreamingDecoder::getConstellationSymbols() const {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    auto now = std::chrono::steady_clock::now();

    if (waveform_) {
        auto symbols = waveform_->getConstellationSymbols();
        if (!symbols.empty()) {
            constellation_cache_ = symbols;
            constellation_cache_time_ = now;
        }
        static int log_count = 0;
        if (log_count < 10 && !symbols.empty()) {
            // Phase histogram to diagnose constellation display
            int bins[8] = {0}; // 45° bins: 0-45, 45-90, ..., 315-360
            float mag_sum = 0, mag_min = 1e9, mag_max = 0;
            for (const auto& s : symbols) {
                float phase = std::atan2(s.imag(), s.real()) * 180.0f / 3.14159265f;
                if (phase < 0) phase += 360.0f;
                int bin = static_cast<int>(phase / 45.0f) % 8;
                bins[bin]++;
                float mag = std::abs(s);
                mag_sum += mag;
                if (mag < mag_min) mag_min = mag;
                if (mag > mag_max) mag_max = mag;
            }
            float mag_avg = mag_sum / symbols.size();
            LOG_MODEM(INFO, "getConstellationSymbols: %zu symbols (mode=%d) mag=[%.3f,%.3f,%.3f] phase_hist: %d %d %d %d %d %d %d %d",
                      symbols.size(), static_cast<int>(mode_),
                      mag_min, mag_avg, mag_max,
                      bins[0], bins[1], bins[2], bins[3], bins[4], bins[5], bins[6], bins[7]);
            // Log first 10 symbols
            if (log_count < 2) {
                for (size_t i = 0; i < std::min(size_t(20), symbols.size()); i++) {
                    float ph = std::atan2(symbols[i].imag(), symbols[i].real()) * 180.0f / 3.14159265f;
                    LOG_MODEM(INFO, "  sym[%zu]: (%.4f, %.4f) mag=%.4f phase=%.1f°",
                              i, symbols[i].real(), symbols[i].imag(), std::abs(symbols[i]), ph);
                }
            }
            log_count++;
        }
        if (!symbols.empty()) {
            return symbols;
        }
    }

    // Hold last non-empty constellation briefly so GUI doesn't flicker to empty
    // during control-profile reconfiguration between frames.
    if (!constellation_cache_.empty()) {
        auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - constellation_cache_time_).count();
        if (age_ms <= CONSTELLATION_CACHE_HOLD_MS) {
            return constellation_cache_;
        }
    }
    return {};
}

// ============================================================================
// LIFECYCLE
// ============================================================================

void StreamingDecoder::reset() {
    // Increment generation BEFORE acquiring lock - any ongoing search will see new value
    reset_generation_.fetch_add(1);

    std::lock_guard<std::mutex> lock(buffer_mutex_);

    write_pos_ = 0;
    correlation_pos_ = 0;
    sync_position_ = 0;
    samples_since_sync_ = 0;
    total_fed_ = 0;
    feed_iter_ = 0;
    overflow_events_ = 0;
    sync_reject_streak_ = 0;
    zc_chirp_fallback_attempts_ = 0;
    connected_since_ = std::chrono::steady_clock::time_point{};
    state_ = DecoderState::SEARCHING;
    pending_total_cw_ = 0;
    burst_blocks_decoded_ = 0;
    burst_soft_buffer_.clear();
    constellation_cache_.clear();
    constellation_cache_time_ = std::chrono::steady_clock::time_point{};
    use_burst_interleave_ = false;
    new_data_available_ = false;
    last_decoded_sync_pos_ = SIZE_MAX;

    std::fill(buffer_.begin(), buffer_.end(), 0.0f);
    {
        std::lock_guard<std::mutex> wlock(waveform_mutex_);
        if (waveform_) waveform_->reset();
    }

    {
        std::lock_guard<std::mutex> qlock(queue_mutex_);
        while (!frame_queue_.empty()) frame_queue_.pop();
    }

    {
        std::lock_guard<std::mutex> slock(stats_mutex_);
        stats_ = DecoderStats{};
    }

    noise_floor_ = 0.001f;
    last_snr_.store(0.0f);
    last_cfo_.store(0.0f);
    last_fading_index_.store(0.0f);
}

void StreamingDecoder::stop() {
    shutdown_.store(true);
    data_cv_.notify_all();
}

// ============================================================================
// HELPERS
// ============================================================================

float StreamingDecoder::estimateSNRFromChirp(float corr, float /*noise*/) {
    float snr = (corr - 0.15f) / 0.03f;
    return std::max(-5.0f, std::min(30.0f, snr));
}

// ============================================================================
// MC-DPSK: Simple sequential codeword decode (no frame interleaving)
// ============================================================================
DecodeResult StreamingDecoder::decodeMCDPSKFrame(const std::vector<float>& soft_bits,
                                                   CodeRate rate, size_t bytes_per_cw,
                                                   float snr, float cfo) {
    // FIX (codex.md Issue #7): Removed unconditional fprintf - use LOG_MODEM instead
    LOG_MODEM(DEBUG, "[%s] decodeMCDPSKFrame: %zu soft_bits, rate=%d, bytes_per_cw=%zu",
              log_prefix_.c_str(), soft_bits.size(), static_cast<int>(rate), bytes_per_cw);

    DecodeResult result;
    result.snr_db = snr;
    result.cfo_hz = cfo;

    constexpr size_t LDPC_BLOCK = v2::LDPC_CODEWORD_BITS;

    if (soft_bits.size() < LDPC_BLOCK) {
        LOG_MODEM(DEBUG, "[%s] decodeMCDPSKFrame: not enough bits (%zu < %zu)",
                  log_prefix_.c_str(), soft_bits.size(), LDPC_BLOCK);
        return result;
    }

    const bool mc_dpsk_ci_enabled = use_mc_dpsk_channel_interleave_ && static_cast<bool>(interleaver_);
    auto decode_cw = [&](const float* cw_bits, CodeRate cw_rate, bool deinterleave)
        -> std::pair<bool, Bytes> {
        if (deinterleave && interleaver_) {
            std::vector<float> tmp(cw_bits, cw_bits + LDPC_BLOCK);
            auto deint = interleaver_->deinterleave(tmp);
            return robustDecodeSingleCW(deint.data(), LDPC_BLOCK, cw_rate, log_prefix_.c_str());
        }
        return robustDecodeSingleCW(cw_bits, LDPC_BLOCK, cw_rate, log_prefix_.c_str());
    };

    // FIX (codex.md Issue #5): Use robustDecodeSingleCW with 4-retry decoder diversity
    // instead of single-shot codec_->decode(). This matches OFDM control-first robust path.

    // Decode CW0 (header codeword) with robust retry.
    // For MC-DPSK DATA when negotiated, try raw first (control/handshake compatibility),
    // then deinterleaved fallback.
    auto [ok0, data0] = decode_cw(soft_bits.data(), rate, false);
    bool cw0_deinterleaved = false;

    LOG_MODEM(DEBUG, "[%s] CW0 decode: ok=%d, data_size=%zu, magic=0x%02X%02X",
              log_prefix_.c_str(), ok0 ? 1 : 0, data0.size(),
              data0.size() >= 1 ? data0[0] : 0, data0.size() >= 2 ? data0[1] : 0);

    // If primary rate failed, try alternate rate for control frames
    // Control frames are encoded at R1/4 but decoder might be configured for higher rate
    if ((!ok0 || data0.size() < 2 || data0[0] != 0x55 || data0[1] != 0x4C) && rate != CodeRate::R1_4) {
        LOG_MODEM(DEBUG, "[%s] MC-DPSK: CW0 primary rate=%d failed, trying R1/4 fallback",
                  log_prefix_.c_str(), static_cast<int>(rate));
        auto [ok_r14, data_r14] = decode_cw(soft_bits.data(), CodeRate::R1_4, false);
        if (ok_r14 && data_r14.size() >= 2 && data_r14[0] == 0x55 && data_r14[1] == 0x4C) {
            ok0 = ok_r14;
            data0 = data_r14;
            LOG_MODEM(INFO, "[%s] MC-DPSK: CW0 R1/4 fallback succeeded", log_prefix_.c_str());
        }
    }

    if ((!ok0 || data0.size() < 2 || data0[0] != 0x55 || data0[1] != 0x4C) && mc_dpsk_ci_enabled) {
        LOG_MODEM(DEBUG, "[%s] MC-DPSK: CW0 raw decode failed, trying deinterleave path",
                  log_prefix_.c_str());

        auto [ok0_i, data0_i] = decode_cw(soft_bits.data(), rate, true);
        if ((!ok0_i || data0_i.size() < 2 || data0_i[0] != 0x55 || data0_i[1] != 0x4C) &&
            rate != CodeRate::R1_4) {
            auto [ok_i_r14, data_i_r14] = decode_cw(soft_bits.data(), CodeRate::R1_4, true);
            if (ok_i_r14 && data_i_r14.size() >= 2 &&
                data_i_r14[0] == 0x55 && data_i_r14[1] == 0x4C) {
                ok0_i = true;
                data0_i = std::move(data_i_r14);
            }
        }

        if (ok0_i && data0_i.size() >= 2 && data0_i[0] == 0x55 && data0_i[1] == 0x4C) {
            ok0 = true;
            data0 = std::move(data0_i);
            cw0_deinterleaved = true;
            LOG_MODEM(INFO, "[%s] MC-DPSK: CW0 recovered via channel deinterleave", log_prefix_.c_str());
        }
    }

    if (!ok0 || data0.size() < 2 || data0[0] != 0x55 || data0[1] != 0x4C) {
        LOG_MODEM(INFO, "[%s] MC-DPSK: CW0 decode failed (ok=%d, size=%zu, magic=0x%02X%02X)",
                  log_prefix_.c_str(), ok0 ? 1 : 0, data0.size(),
                  data0.size() >= 1 ? data0[0] : 0, data0.size() >= 2 ? data0[1] : 0);
        return result;
    }

    // Truncate to bytes_per_cw if needed
    if (data0.size() > bytes_per_cw) data0.resize(bytes_per_cw);

    // Parse header
    auto hdr = v2::parseHeader(data0);
    if (!hdr.valid) {
        // Log CRC details for control frames (ACK, etc.)
        if (data0.size() >= 20) {
            uint16_t received_crc = (static_cast<uint16_t>(data0[18]) << 8) | data0[19];

            // Full 18 bytes
            uint8_t full18[18];
            for (int i = 0; i < 18; i++) full18[i] = data0[i];
            uint16_t calc18 = v2::ControlFrame::calculateCRC(full18, 18);

            // Print all 20 bytes for inspection
            LOG_MODEM(INFO, "[%s] MC-DPSK: CRC fail - rcv=0x%04X calc18=0x%04X",
                      log_prefix_.c_str(), received_crc, calc18);
            LOG_MODEM(INFO, "[%s] MC-DPSK: bytes[0-9]  = %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
                      log_prefix_.c_str(),
                      data0[0], data0[1], data0[2], data0[3], data0[4],
                      data0[5], data0[6], data0[7], data0[8], data0[9]);
            LOG_MODEM(INFO, "[%s] MC-DPSK: bytes[10-19] = %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
                      log_prefix_.c_str(),
                      data0[10], data0[11], data0[12], data0[13], data0[14],
                      data0[15], data0[16], data0[17], data0[18], data0[19]);
        } else {
            LOG_MODEM(INFO, "[%s] MC-DPSK: Header invalid - data too small (%zu bytes)",
                      log_prefix_.c_str(), data0.size());
        }
        return result;
    }

    // CONNECT/CONNECT_ACK/CONNECT_NAK are multi-CW frames by construction.
    // Treat impossible 1-CW CONNECT headers as decode false-positives.
    if (v2::isConnectFrame(hdr.type) && hdr.type != v2::FrameType::DISCONNECT) {
        int min_connect_cw = std::max<int>(
            2, v2::DataFrame::calculateCodewords(v2::ConnectFrame::PAYLOAD_SIZE, rate));
        if (hdr.total_cw < min_connect_cw) {
            LOG_MODEM(WARN, "[%s] MC-DPSK: invalid CONNECT header total_cw=%d (<%d), rejecting",
                      log_prefix_.c_str(), hdr.total_cw, min_connect_cw);
            return result;
        }
    }

    result.frame_type = hdr.type;
    result.codewords_ok = 1;

    // Create chase cache key from header (for HARQ combining)
    fec::ChaseCacheKey cache_key{hdr.seq, hdr.src_hash, hdr.dst_hash};
    bool chase_enabled = chase_cache_ && chase_cache_->isEnabled();

    // 1-CW frame (control frame like ACK, PROBE, etc.)
    if (hdr.total_cw == 1) {
        result.success = true;
        result.frame_data = data0;
        LOG_MODEM(DEBUG, "[%s] MC-DPSK: Control frame (1 CW) decoded", log_prefix_.c_str());
        return result;
    }

    const bool deinterleave_data_cw = cw0_deinterleaved;

    // Multi-CW frame - decode remaining codewords sequentially
    int total_cw = hdr.total_cw;
    int avail_cw = static_cast<int>(soft_bits.size() / LDPC_BLOCK);

    if (avail_cw < total_cw) {
        // Not enough codewords available
        result.frame_data = data0;
        LOG_MODEM(DEBUG, "[%s] MC-DPSK: Need %d CWs, have %d - partial", log_prefix_.c_str(), total_cw, avail_cw);
        return result;
    }

    // Set up codeword status for reassembly
    v2::CodewordStatus cw_status;
    cw_status.decoded.resize(total_cw, false);
    cw_status.data.resize(total_cw);
    cw_status.decoded[0] = true;
    cw_status.data[0] = data0;

    // Decode CW1+ with robust retry and HARQ chase combining
    for (int i = 1; i < total_cw; i++) {
        size_t off = i * LDPC_BLOCK;
        auto [ok, data] = decode_cw(soft_bits.data() + off, rate, deinterleave_data_cw);

        // If decode failed, try HARQ chase combining
        if (!ok && chase_enabled) {
            // Store current soft bits in cache for future combining
            std::vector<float> cw_soft(soft_bits.begin() + off, soft_bits.begin() + off + LDPC_BLOCK);
            chase_cache_->store(cache_key, i, cw_soft, total_cw, hdr.type);

            // Try decoding with combined soft bits (if we have previous receptions)
            auto combined = chase_cache_->getCombined(cache_key, i);
            if (combined && combined->size() == LDPC_BLOCK) {
                int combine_count = chase_cache_->getCombineCount(cache_key, i);
                if (combine_count > 1) {
                    // We have combined data from multiple receptions - try again
                    auto [ok_chase, data_chase] = decode_cw(combined->data(), rate, deinterleave_data_cw);
                    if (ok_chase && data_chase.size() >= bytes_per_cw) {
                        ok = true;
                        data = std::move(data_chase);
                        chase_cache_->markDecoded(cache_key, i);
                        chase_cache_->incrementRecoveries();
                        LOG_MODEM(INFO, "[%s] MC-DPSK: Chase combining recovered CW%d (combines=%d)",
                                  log_prefix_.c_str(), i, combine_count);
                    }
                }
            }
        }

        if (ok && data.size() >= bytes_per_cw) {
            data.resize(bytes_per_cw);
            cw_status.decoded[i] = true;
            cw_status.data[i] = data;
            result.codewords_ok++;
            // Mark as decoded in chase cache (clears stored soft bits)
            if (chase_enabled) {
                chase_cache_->markDecoded(cache_key, i);
            }
        } else {
            result.codewords_failed++;
        }
    }

    if (cw_status.allSuccess()) {
        result.success = true;
        result.frame_data = cw_status.reassemble();
        // Remove cache entry on full success
        if (chase_enabled) {
            chase_cache_->removeEntry(cache_key);
        }
        LOG_MODEM(DEBUG, "[%s] MC-DPSK: %d/%d CWs decoded OK", log_prefix_.c_str(), total_cw, total_cw);
    } else {
        LOG_MODEM(DEBUG, "[%s] MC-DPSK: %d/%d CWs failed", log_prefix_.c_str(),
                  result.codewords_failed, total_cw);
    }

    return result;
}

DecodeResult StreamingDecoder::decodeFrame(const std::vector<float>& soft_bits, float snr, float cfo) {
    DecodeResult result;
    result.snr_db = snr;
    result.cfo_hz = cfo;

    constexpr size_t LDPC_BLOCK = v2::LDPC_CODEWORD_BITS;
    constexpr size_t FRAME_INTERLEAVE_BITS = fec::FrameInterleaver::TOTAL_FRAME_BITS;  // 2592

    if (soft_bits.size() < LDPC_BLOCK) return result;

    CodeRate rate = connected_ ? code_rate_ : CodeRate::R1_4;
    size_t bytes_per_cw = v2::getBytesPerCodeword(rate);
    codec_->setRate(rate);

    // Channel interleaving only applies to OFDM modes, NOT MC-DPSK
    bool is_ofdm = (mode_ == protocol::WaveformMode::OFDM_CHIRP ||
                    mode_ == protocol::WaveformMode::OFDM_COX);
    bool apply_channel_deinterleave = use_channel_interleave_ && is_ofdm;

    // Helper to deinterleave a codeword if needed
    auto deinterleave_cw = [&](const std::vector<float>& cw) -> std::vector<float> {
        if (apply_channel_deinterleave) {
            return interleaver_->deinterleave(cw);
        }
        return cw;
    };

    // ========================================================================
    // MC-DPSK: Simple sequential decode (no frame interleaving ever)
    // OFDM: "Try Both" strategy with frame interleaving for 4-CW frames
    // ========================================================================

    // For MC-DPSK and MFSK, ALWAYS use simple sequential decode - skip all frame interleaving logic
    if (mode_ == protocol::WaveformMode::MC_DPSK ||
        mode_ == protocol::WaveformMode::MFSK) {
        return decodeMCDPSKFrame(soft_bits, rate, bytes_per_cw, snr, cfo);
    }

    // ========================================================================
    // OFDM: "Try Both" Strategy for Frame Interleaving
    // ========================================================================
    // 1. First, try to decode first 648 bits as non-interleaved CW0
    // 2. If it's a valid 1-CW control frame → done
    // 3. If decode fails OR it's a 4-CW frame → try frame-interleaved decode
    // ========================================================================

    // R1/4 fast-path: control frames are always encoded at R1/4 (hardened)
    // Try R1/4 first — if it's a valid 1-CW control frame, return immediately
    if (rate != CodeRate::R1_4 && soft_bits.size() >= LDPC_BLOCK) {
        codec_->setRate(CodeRate::R1_4);
        std::vector<float> cw0_r14(soft_bits.begin(), soft_bits.begin() + LDPC_BLOCK);
        auto [ok_r14, data_r14] = codec_->decode(cw0_r14);
        size_t bpc_r14 = v2::getBytesPerCodeword(CodeRate::R1_4);

        if (ok_r14 && data_r14.size() >= 2
            && data_r14[0] == 0x55 && data_r14[1] == 0x4C) {
            if (data_r14.size() > bpc_r14) data_r14.resize(bpc_r14);
            auto hdr_r14 = v2::parseHeader(data_r14);
            if (hdr_r14.valid && hdr_r14.total_cw == 1) {
                LOG_MODEM(INFO, "[%s] R1/4 control fast-path OK", log_prefix_.c_str());
                result.success = true;
                result.codewords_ok = 1;
                result.frame_data = data_r14;
                result.frame_type = hdr_r14.type;
                return result;
            }
        }
        // Restore rate for remaining decode paths
        codec_->setRate(rate);
    }

    // Step 1: Try to decode CW0 RAW (no channel deinterleave)
    // Control frames (ACK etc.) are never channel-interleaved, so probe without it.
    // If this is a 4-CW data frame (which IS interleaved), CW0 will likely fail here
    // and we'll fall through to decodeFixedFrame() which handles deinterleaving internally.
    std::vector<float> cw0_bits(soft_bits.begin(), soft_bits.begin() + LDPC_BLOCK);
    auto [ok0, data0] = codec_->decode(cw0_bits);

    bool try_frame_interleave = false;

    if (ok0 && data0.size() >= 2 && data0[0] == 0x55 && data0[1] == 0x4C) {
        // CW0 decoded and has valid magic - check if it's a control frame
        if (data0.size() > bytes_per_cw) data0.resize(bytes_per_cw);

        auto hdr = v2::parseHeader(data0);
        if (hdr.valid) {
            result.frame_type = hdr.type;

            if (hdr.total_cw == 1) {
                // === Control frame (1 CW) - no frame interleaving ===
                LOG_MODEM(DEBUG, "[%s] Control frame decoded (1 CW)", log_prefix_.c_str());
                result.success = true;
                result.codewords_ok = 1;
                result.frame_data = data0;
                return result;
            } else if (hdr.total_cw == v2::FIXED_FRAME_CODEWORDS) {
                // === Multi-CW frame with 4 CWs - try frame interleaving ===
                LOG_MODEM(DEBUG, "[%s] Header shows 4 CWs - trying frame deinterleave", log_prefix_.c_str());
                try_frame_interleave = true;
            } else {
                // === Multi-CW frame with != 4 CWs - old format, no frame interleaving ===
                // Fall through to legacy decode path
                LOG_MODEM(DEBUG, "[%s] Multi-CW frame (%d CWs) - legacy decode", log_prefix_.c_str(), hdr.total_cw);
            }
        }
    } else {
        // CW0 decode failed or invalid magic - might be interleaved
        LOG_MODEM(DEBUG, "[%s] CW0 decode failed/invalid - trying frame deinterleave", log_prefix_.c_str());
        try_frame_interleave = true;
    }

    // Step 2: Try frame-interleaved decode if needed
    if (try_frame_interleave && soft_bits.size() >= FRAME_INTERLEAVE_BITS) {
        LOG_MODEM(DEBUG, "[%s] Attempting 4-CW frame deinterleave decode", log_prefix_.c_str());

        // Use v2::decodeFixedFrame which handles frame + channel deinterleaving + LDPC decode
        // Channel deinterleaving restores the original bit order within each CW
        // Only enable for OFDM modes (MC-DPSK doesn't use channel interleaving)
        size_t bps = ofdm_data_carriers_ * getBitsPerSymbol(current_modulation_);
        auto cw_status = v2::decodeFixedFrame(soft_bits, rate, apply_channel_deinterleave, bps);

        result.codewords_ok = 0;
        result.codewords_failed = 0;
        for (int i = 0; i < v2::FIXED_FRAME_CODEWORDS; ++i) {
            if (cw_status.decoded[i]) {
                result.codewords_ok++;
            } else {
                result.codewords_failed++;
            }
        }

        if (cw_status.allSuccess()) {
            result.success = true;
            result.frame_data = cw_status.reassemble();

            if (result.frame_data.empty()) {
                // LDPC said 4/4 OK but reassemble failed — likely LDPC false positive
                result.success = false;
                LOG_MODEM(WARN, "[%s] Frame deinterleave: 4/4 CWs OK but reassemble FAILED (LDPC false positive?)",
                          log_prefix_.c_str());
            } else {
                // Parse header to get frame type
                if (result.frame_data.size() >= 3) {
                    result.frame_type = static_cast<v2::FrameType>(result.frame_data[2]);
                }
            }

            LOG_MODEM(INFO, "[%s] Frame deinterleave decode SUCCESS (%d/%d CWs, data=%zu bytes)",
                      log_prefix_.c_str(), result.codewords_ok, v2::FIXED_FRAME_CODEWORDS,
                      result.frame_data.size());
            return result;
        } else {
            LOG_MODEM(DEBUG, "[%s] Frame deinterleave decode FAILED (%d/%d CWs)",
                      log_prefix_.c_str(), result.codewords_ok, v2::FIXED_FRAME_CODEWORDS);

            // Step 2b: If frame deinterleave failed, check if it's a 1-CW control frame
            // Catches ACK frames that were escalated to 4-CW by a failed peek
            // Try R1/4 first (control frames hardened), then code_rate_ fallback
            {
                auto trySalvage = [&](CodeRate sr) -> bool {
                    size_t bpc = v2::getBytesPerCodeword(sr);
                    auto [rec_ok, rec_data] = robustDecodeSingleCW(
                        soft_bits.data(), LDPC_BLOCK, sr, log_prefix_.c_str());
                    if (rec_ok && rec_data.size() >= 2
                        && rec_data[0] == 0x55 && rec_data[1] == 0x4C) {
                        if (rec_data.size() > bpc) rec_data.resize(bpc);
                        auto hdr = v2::parseHeader(rec_data);
                        if (hdr.valid && hdr.total_cw == 1) {
                            g_salvage_hits.fetch_add(1, std::memory_order_relaxed);
                            LOG_MODEM(INFO, "[%s] Salvaged 1-CW control (rate=%d, total_hits=%d)",
                                      log_prefix_.c_str(), static_cast<int>(sr),
                                      g_salvage_hits.load(std::memory_order_relaxed));
                            result.success = true;
                            result.codewords_ok = 1;
                            result.codewords_failed = 0;
                            result.frame_data = rec_data;
                            result.frame_type = hdr.type;
                            return true;
                        }
                    }
                    return false;
                };

                if (trySalvage(CodeRate::R1_4))
                    return result;
                if (rate != CodeRate::R1_4 && trySalvage(rate))
                    return result;
            }
        }
    }

    // Step 3: Legacy decode path (non-interleaved multi-CW frames)
    // This handles old-format frames or when frame interleaving is disabled
    if (ok0 && data0.size() >= 2 && data0[0] == 0x55 && data0[1] == 0x4C) {
        if (data0.size() > bytes_per_cw) data0.resize(bytes_per_cw);
        result.codewords_ok = 1;

        auto hdr = v2::parseHeader(data0);
        if (!hdr.valid) return result;

        result.frame_type = hdr.type;
        int total_cw = hdr.total_cw;
        int avail_cw = static_cast<int>(soft_bits.size() / LDPC_BLOCK);

        if (avail_cw < total_cw) {
            result.frame_data = data0;
            return result;
        }

        v2::CodewordStatus cw_status;
        cw_status.decoded.resize(total_cw, false);
        cw_status.data.resize(total_cw);
        cw_status.decoded[0] = true;
        cw_status.data[0] = data0;

        for (int i = 1; i < total_cw; i++) {
            size_t off = i * LDPC_BLOCK;
            std::vector<float> bits(soft_bits.begin() + off, soft_bits.begin() + off + LDPC_BLOCK);
            bits = deinterleave_cw(bits);

            auto [ok, data] = codec_->decode(bits);
            if (ok && data.size() >= bytes_per_cw) {
                data.resize(bytes_per_cw);
                cw_status.decoded[i] = true;
                cw_status.data[i] = data;
                result.codewords_ok++;
            } else {
                result.codewords_failed++;
            }
        }

        if (cw_status.allSuccess()) {
            result.success = true;
            result.frame_data = cw_status.reassemble();
        }
    }

    return result;
}

// ============================================================================
// BURST INTERLEAVE ACCUMULATION
// ============================================================================

void StreamingDecoder::accumulateBurstFrames() {
    const int burst_group_size = std::max(2, burst_group_size_);
    const int burst_timeout_ms =
        static_cast<int>(BURST_TIMEOUT_MS_BASE * (burst_group_size / 4.0f));

    // Timeout check
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - burst_start_time_).count();
    if (elapsed > burst_timeout_ms) {
        LOG_MODEM(WARN, "[%s] Burst group timeout: got %zu/%d frames",
                  log_prefix_.c_str(), burst_soft_buffer_.size(), burst_group_size);
        // Discard — TX used 4-frame interleaving, partial is undecodable
        {
            std::lock_guard<std::mutex> slock(stats_mutex_);
            stats_.frames_failed += burst_group_size;
        }
        burst_soft_buffer_.clear();
        {
            std::lock_guard<std::mutex> lock(buffer_mutex_);
            correlation_pos_ = burst_next_pos_;
        }
        state_ = DecoderState::SEARCHING;
        return;
    }

    // Try to demodulate next frame
    auto result = tryDemodulateNextBurstFrame();

    if (result == BurstFrameResult::FAILED) {
        // Hard failure (energy lost or process error) — abort immediately
        LOG_MODEM(WARN, "[%s] Burst group aborted: hard failure at frame %zu/%d",
                  log_prefix_.c_str(), burst_soft_buffer_.size() + 1, burst_group_size);
        {
            std::lock_guard<std::mutex> slock(stats_mutex_);
            stats_.frames_failed += burst_group_size;
        }
        burst_soft_buffer_.clear();
        {
            std::lock_guard<std::mutex> lock(buffer_mutex_);
            correlation_pos_ = burst_next_pos_;
        }
        state_ = DecoderState::SEARCHING;
        return;
    }

    if (result == BurstFrameResult::WAITING) {
        return;  // Not enough samples yet — come back on next processBuffer() tick
    }

    // SUCCESS — check if group complete
    if (static_cast<int>(burst_soft_buffer_.size()) == burst_group_size) {
        finalizeBurstGroup();
        burst_soft_buffer_.clear();
        {
            std::lock_guard<std::mutex> lock(buffer_mutex_);
            correlation_pos_ = burst_next_pos_;
        }
        state_ = DecoderState::SEARCHING;
    }
    // else: still accumulating, return and wait for next processBuffer() call
}

StreamingDecoder::BurstFrameResult StreamingDecoder::tryDemodulateNextBurstFrame() {
    const int burst_group_size = std::max(2, burst_group_size_);

    // Check available samples at burst_next_pos_
    size_t next_available;
    {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        if (write_pos_ >= burst_next_pos_) {
            next_available = write_pos_ - burst_next_pos_;
        } else {
            next_available = MAX_BUFFER_SAMPLES - burst_next_pos_ + write_pos_;
        }
    }

    if (next_available < burst_min_block_) {
        return BurstFrameResult::WAITING;
    }

    // Copy samples from circular buffer
    std::vector<float> block(burst_min_block_);
    {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        for (size_t i = 0; i < burst_min_block_; i++) {
            block[i] = buffer_[(burst_next_pos_ + i) % MAX_BUFFER_SAMPLES];
        }
    }

    // Energy check (same as existing burst loop)
    float next_rms = 0.0f;
    size_t check_start = std::min(size_t(1024), burst_min_block_);
    size_t check_len = std::min(burst_min_block_ - check_start, size_t(5000));
    if (check_len > 0) {
        for (size_t i = 0; i < check_len; i++) {
            next_rms += block[check_start + i] * block[check_start + i];
        }
        next_rms = std::sqrt(next_rms / check_len);
    }
    constexpr float BURST_ENERGY_THRESHOLD = 0.04f;
    if (next_rms < BURST_ENERGY_THRESHOLD) {
        LOG_MODEM(WARN, "[%s] Burst frame %zu/%d: energy lost (RMS=%.4f)",
                  log_prefix_.c_str(), burst_soft_buffer_.size() + 1,
                  burst_group_size, next_rms);
        burst_next_pos_ = (burst_next_pos_ + burst_min_block_) % MAX_BUFFER_SAMPLES;
        return BurstFrameResult::FAILED;
    }

    // Demodulate
    waveform_->setFrequencyOffset(burst_cfo_);
    bool ok = waveform_->process(SampleSpan(block.data(), block.size()));
    if (!ok) {
        LOG_MODEM(WARN, "[%s] Burst frame %zu/%d: process() failed",
                  log_prefix_.c_str(), burst_soft_buffer_.size() + 1, burst_group_size);
        burst_next_pos_ = (burst_next_pos_ + burst_min_block_) % MAX_BUFFER_SAMPLES;
        return BurstFrameResult::FAILED;
    }
    captureConstellationSnapshot();

    auto soft = waveform_->getSoftBits();
    if (soft.empty()) {
        burst_next_pos_ = (burst_next_pos_ + burst_min_block_) % MAX_BUFFER_SAMPLES;
        return BurstFrameResult::FAILED;
    }

    // Update CFO from pilot tracking (same drift-limiting as existing burst loop)
    float corrected_cfo = waveform_->estimatedCFO();
    float drift = corrected_cfo - burst_cfo_;
    constexpr float MAX_BURST_CFO_DRIFT_HZ = 2.0f;
    if (std::abs(drift) > MAX_BURST_CFO_DRIFT_HZ) {
        corrected_cfo = burst_cfo_ + std::copysign(MAX_BURST_CFO_DRIFT_HZ, drift);
    }
    burst_cfo_ = corrected_cfo;
    last_cfo_.store(corrected_cfo);
    last_fading_index_.store(waveform_->getFadingIndex());

    burst_soft_buffer_.push_back(std::move(soft));
    burst_next_pos_ = (burst_next_pos_ + burst_min_block_) % MAX_BUFFER_SAMPLES;

    LOG_MODEM(INFO, "[%s] Burst frame %zu/%d demodulated, RMS=%.4f",
              log_prefix_.c_str(), burst_soft_buffer_.size(),
              burst_group_size, next_rms);
    return BurstFrameResult::SUCCESS;
}

void StreamingDecoder::finalizeBurstGroup() {
    const int burst_group_size = std::max(2, burst_group_size_);
    LOG_MODEM(INFO, "[%s] Burst group complete (%d frames), deinterleaving...",
              log_prefix_.c_str(), burst_group_size);

    auto logical_soft = fec::BurstInterleaver::deinterleave(burst_soft_buffer_);

    for (int i = 0; i < burst_group_size; i++) {
        DecodeResult result = decodeFrame(logical_soft[i], burst_snr_, burst_cfo_);

        {
            std::lock_guard<std::mutex> slock(stats_mutex_);
            if (result.success) stats_.frames_decoded++;
            else stats_.frames_failed++;
        }

        if (result.success || result.codewords_ok > 0) {
            {
                std::lock_guard<std::mutex> qlock(queue_mutex_);
                frame_queue_.push(result);
            }
            if (result.success && frame_callback_) frame_callback_(result);
        }

        LOG_MODEM(INFO, "[%s] Burst logical frame %d/%d: %s (%d/%d CWs)",
                  log_prefix_.c_str(), i + 1, burst_group_size,
                  result.success ? "OK" : "FAIL",
                  result.codewords_ok, result.codewords_ok + result.codewords_failed);
    }
}

// Legacy methods - do nothing but required by header
bool StreamingDecoder::runCorrelationSearch(size_t) { return false; }
bool StreamingDecoder::tryDecodeFrame() { return false; }
std::vector<float> StreamingDecoder::copySamplesFrom(size_t, size_t) { return {}; }
size_t StreamingDecoder::samplesAvailableFrom(size_t) const { return 0; }
bool StreamingDecoder::isPingOnly(const std::vector<float>&, size_t) { return false; }
void StreamingDecoder::updateNoiseFloor(const float*, size_t) {}

int StreamingDecoder::getFrameTimeoutMs() const {
    // Dynamic timeout based on expected frame duration
    // For slow modes (5-carrier DBPSK), frames can take 11+ seconds
    // For fast modes (10-carrier DQPSK), frames take ~2-3 seconds

    std::lock_guard<std::mutex> wlock(waveform_mutex_);
    if (!waveform_) {
        return MIN_FRAME_TIMEOUT_MS;
    }

    // Get expected frame samples from waveform
    int frame_samples;
    if (pending_total_cw_ > 0) {
        // We know exact CW count from CW0 peek
        frame_samples = waveform_->getMinSamplesForCWCount(pending_total_cw_);
    } else {
        // Use full frame estimate (4 CWs for data)
        frame_samples = waveform_->getMinSamplesForFrame();
    }

    // Add preamble samples.
    // Weak-signal MC-DPSK profile (DBPSK + 2x/4x) uses full CHIRP even when connected.
    bool weak_mc_dpsk_profile = connected_ &&
                                (mode_ == protocol::WaveformMode::MC_DPSK) &&
                                (current_modulation_ == Modulation::DBPSK) &&
                                (spreading_mode_ != SpreadingMode::NONE);
    int preamble_samples = (connected_ && !weak_mc_dpsk_profile && waveform_->supportsDataPreamble())
                               ? waveform_->getDataPreambleSamples()
                               : waveform_->getPreambleSamples();
    int total_samples = frame_samples + preamble_samples;

    // Convert to milliseconds at 48kHz
    constexpr int SAMPLE_RATE = 48000;
    int frame_duration_ms = static_cast<int>(std::ceil((total_samples * 1000.0) / SAMPLE_RATE));

    // Guard against decode-thread scheduling jitter and frame-duration estimation error.
    // MC-DPSK + spreading needs a wider margin than OFDM.
    bool is_mc_dpsk = (mode_ == protocol::WaveformMode::MC_DPSK);
    double timeout_scale = 1.75;
    int guard_ms = 750;
    if (is_mc_dpsk) {
        switch (spreading_mode_) {
            case SpreadingMode::TIME_4X:
                timeout_scale = 2.40;
                guard_ms = 1500;
                break;
            case SpreadingMode::TIME_2X:
                timeout_scale = 2.10;
                guard_ms = 1000;
                break;
            case SpreadingMode::NONE:
            default:
                timeout_scale = 1.90;
                guard_ms = 800;
                break;
        }
    }
    int timeout_ms = static_cast<int>(std::ceil(frame_duration_ms * timeout_scale)) + guard_ms;

    // Ensure minimum timeout
    return std::max(timeout_ms, MIN_FRAME_TIMEOUT_MS);
}

} // namespace gui
} // namespace ultra
