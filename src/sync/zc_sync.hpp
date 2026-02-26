// Zadoff-Chu (ZC) Sync - Compact preamble for sync, CFO, and frame-type encoding
//
// Based on working implementation from spire/zadoff_chu.cpp
// Uses upsampling with interpolation and proper I/Q modulation.

#pragma once

#include "ultra/types.hpp"
#include <vector>
#include <complex>
#include <cmath>
#include <algorithm>
#include <map>
#include <cstdio>

namespace ultra {
namespace sync {

enum class ZCFrameType : uint8_t {
    PING = 0,
    PONG = 1,
    DATA = 2,
    CONTROL = 3,
    UNKNOWN = 255
};

// Root-selection mask for detection. Connected MC-DPSK data paths can restrict
// detection to DATA/CONTROL roots to avoid false PING/PONG acquisitions.
constexpr uint8_t ZC_ROOT_MASK_PING = 1u << 0;
constexpr uint8_t ZC_ROOT_MASK_PONG = 1u << 1;
constexpr uint8_t ZC_ROOT_MASK_DATA = 1u << 2;
constexpr uint8_t ZC_ROOT_MASK_CONTROL = 1u << 3;
constexpr uint8_t ZC_ROOT_MASK_ALL = ZC_ROOT_MASK_PING |
                                     ZC_ROOT_MASK_PONG |
                                     ZC_ROOT_MASK_DATA |
                                     ZC_ROOT_MASK_CONTROL;

inline const char* zcFrameTypeToString(ZCFrameType type) {
    switch (type) {
        case ZCFrameType::PING: return "PING";
        case ZCFrameType::PONG: return "PONG";
        case ZCFrameType::DATA: return "DATA";
        case ZCFrameType::CONTROL: return "CONTROL";
        default: return "UNKNOWN";
    }
}

// Detection and estimation thresholds
constexpr float ZC_DEFAULT_DETECT_THRESHOLD = 0.3f;    // Minimum correlation for detection
constexpr float ZC_REP1_ADJUST_THRESHOLD = 0.4f;       // Min ratio to prefer earlier rep (40% of peak)
constexpr float ZC_AMPLITUDE_SCALE = 0.8f;             // Normalized amplitude target
constexpr float ZC_CFO_CONFIDENCE_THRESHOLD = 0.1f;    // Min correlation for CFO estimation
constexpr float ZC_LOW_SNR_COHERENT_THRESHOLD = 0.25f; // Enable coherent combining below this (codex.md Issue #8)

// CFO range limitation: With repetition spacing T_rep, unambiguous CFO range is ±1/(2*T_rep)
// For 127 chips × 8 upsample = 1016 samples at 48kHz: T_rep = 21.17ms, range = ±23.6 Hz
// CFO beyond this range will alias. See CFO_problems.md for details.
constexpr float ZC_MAX_UNAMBIGUOUS_CFO_HZ = 23.6f;

struct ZCConfig {
    float sample_rate = 48000.0f;
    int sequence_length = 127;       // N - ZC sequence length (prime)
    int upsample_factor = 8;         // Samples per ZC chip
    int num_repetitions = 2;         // Repeat for robustness
    float carrier_freq = 1500.0f;    // Center frequency
    float gap_ms = 10.0f;            // Gap after preamble

    // Root indices for each frame type (coprime with N)
    int root_ping = 1;
    int root_pong = 3;
    int root_data = 5;
    int root_control = 7;

    int gapSamples() const {
        return static_cast<int>(sample_rate * gap_ms / 1000.0f);
    }

    int singleRepSamples() const {
        return sequence_length * upsample_factor;
    }

    int preambleSamples() const {
        return singleRepSamples() * num_repetitions + gapSamples();
    }

    float preambleDurationMs() const {
        return preambleSamples() / sample_rate * 1000.0f;
    }

    int getRootForType(ZCFrameType type) const {
        switch (type) {
            case ZCFrameType::PING: return root_ping;
            case ZCFrameType::PONG: return root_pong;
            case ZCFrameType::DATA: return root_data;
            case ZCFrameType::CONTROL: return root_control;
            default: return root_data;
        }
    }

    ZCFrameType getTypeForRoot(int root) const {
        if (root == root_ping) return ZCFrameType::PING;
        if (root == root_pong) return ZCFrameType::PONG;
        if (root == root_data) return ZCFrameType::DATA;
        if (root == root_control) return ZCFrameType::CONTROL;
        return ZCFrameType::UNKNOWN;
    }
};

struct ZCSyncResult {
    bool detected = false;
    ZCFrameType frame_type = ZCFrameType::UNKNOWN;
    int start_sample = -1;
    float correlation = 0.0f;
    float cfo_hz = 0.0f;
    float snr_estimate = 0.0f;
    int root_detected = -1;
};

class ZCSync {
public:
    explicit ZCSync(const ZCConfig& config = ZCConfig())
        : config_(config)
    {
        generateTemplates();
    }

    Samples generatePreamble(ZCFrameType frame_type) const {
        int root = config_.getRootForType(frame_type);
        return generatePreambleForRoot(root);
    }

    // Generate preamble using upsampling with interpolation (from spire implementation)
    Samples generatePreambleForRoot(int root) const {
        const auto& zc = getZCTemplate(root);
        int length = config_.sequence_length;
        int upsample = config_.upsample_factor;
        int single_rep_len = length * upsample;

        Samples result;
        int total = config_.preambleSamples();
        result.reserve(total);

        for (int rep = 0; rep < config_.num_repetitions; rep++) {
            for (int i = 0; i < single_rep_len; ++i) {
                // Interpolate ZC sequence
                float chip_pos = static_cast<float>(i) / upsample;
                int chip_idx = static_cast<int>(chip_pos);
                float frac = chip_pos - chip_idx;

                // Linear interpolation between chips
                Complex interp;
                if (chip_idx < length - 1) {
                    interp = zc[chip_idx] * (1.0f - frac) + zc[chip_idx + 1] * frac;
                } else {
                    interp = zc[chip_idx];
                }

                // Modulate to center frequency
                // Use continuous time across repetitions for phase continuity
                int global_i = rep * single_rep_len + i;
                float t = static_cast<float>(global_i) / config_.sample_rate;
                float carrier_phase = 2.0f * M_PI * config_.carrier_freq * t;

                // I/Q modulation: s(t) = Re(zc)*cos(wt) - Im(zc)*sin(wt)
                float sample = interp.real() * std::cos(carrier_phase)
                             - interp.imag() * std::sin(carrier_phase);
                result.push_back(sample);
            }
        }

        // Normalize amplitude to target level
        float max_amp = 0.0f;
        for (float s : result) {
            max_amp = std::max(max_amp, std::abs(s));
        }
        if (max_amp > 0.0f) {
            float scale = ZC_AMPLITUDE_SCALE / max_amp;
            for (float& s : result) {
                s *= scale;
            }
        }

        // Add gap
        int gap = config_.gapSamples();
        for (int i = 0; i < gap; i++) {
            result.push_back(0.0f);
        }

        return result;
    }

    ZCSyncResult detect(SampleSpan samples,
                        float threshold = ZC_DEFAULT_DETECT_THRESHOLD,
                        bool debug = false,
                        uint8_t root_mask = ZC_ROOT_MASK_ALL,
                        float known_cfo_hz = 0.0f) const {
        ZCSyncResult result;

        int single_rep_len = config_.singleRepSamples();
        int preamble_len = config_.preambleSamples();

        if (static_cast<int>(samples.size()) < single_rep_len) {
            if (debug) fprintf(stderr, "[ZC] samples too small: %zu < %d\n", samples.size(), single_rep_len);
            return result;
        }

        if (debug) {
            fprintf(stderr, "[ZC] detecting in %zu samples (rep_len=%d, preamble=%d, known_cfo=%.1f)\n",
                    samples.size(), single_rep_len, preamble_len, known_cfo_hz);
        }

        // Try selected root templates
        float best_corr = 0.0f;
        int best_root = -1;
        int best_pos = -1;
        float best_cfo = 0.0f;
        struct RootCandidate {
            int root;
            uint8_t bit;
        };
        const RootCandidate candidates[] = {
            {config_.root_ping, ZC_ROOT_MASK_PING},
            {config_.root_pong, ZC_ROOT_MASK_PONG},
            {config_.root_data, ZC_ROOT_MASK_DATA},
            {config_.root_control, ZC_ROOT_MASK_CONTROL}
        };

        for (const auto& candidate : candidates) {
            if ((root_mask & candidate.bit) == 0) {
                continue;
            }
            int root = candidate.root;
            const auto& zc = getZCTemplate(root);
            auto correlation = correlate(samples, zc, debug, root, known_cfo_hz);

            // Find peak - look for EARLIEST strong peak (first ZC repetition)
            // With num_repetitions=2, both reps can produce peaks. We want the first one.
            float peak_mag = 0.0f;
            int peak_pos = 0;
            int single_rep = config_.singleRepSamples();

            for (size_t i = 0; i < correlation.size(); i++) {
                float mag = std::abs(correlation[i]);
                if (mag > peak_mag) {
                    peak_mag = mag;
                    peak_pos = static_cast<int>(i);
                }
            }

            // If peak is found, check if there's a comparable peak one rep earlier
            // (i.e., we might have found rep2 instead of rep1)
            // IMPORTANT: Keep original peak_mag for detection confidence, only adjust position for timing
            // See CFO_problems.md #3 for rationale
            int timing_pos = peak_pos;  // Position used for timing (may be adjusted to rep1)
            if (peak_mag > threshold && peak_pos >= single_rep) {
                int earlier_pos = peak_pos - single_rep;
                if (earlier_pos >= 0) {
                    // Explicitly compute correlation at earlier position
                    float earlier_mag = computeCorrelationMag(samples, zc, earlier_pos, known_cfo_hz);
                    // If earlier position has significant correlation, use it for timing
                    // but keep original peak_mag for detection confidence
                    if (earlier_mag > peak_mag * ZC_REP1_ADJUST_THRESHOLD) {
                        if (debug) {
                            fprintf(stderr, "  [DEBUG] Adjusting timing from rep2 (%d) to rep1 (%d), "
                                    "keeping detect mag=%.3f (earlier=%.3f)\n",
                                    peak_pos, earlier_pos, peak_mag, earlier_mag);
                        }
                        timing_pos = earlier_pos;
                        // Note: peak_mag is NOT updated - keeps detection confidence intact
                    } else if (debug) {
                        fprintf(stderr, "  [DEBUG] Earlier pos %d has mag=%.4f (%.1f%% of peak), keeping rep2 timing\n",
                                earlier_pos, earlier_mag, 100.0f * earlier_mag / peak_mag);
                    }
                }
            }

            // FIX (codex.md Issue #8): Coherent combining of repetitions for low-SNR improvement
            // When peak_mag is weak, try combining both repetitions for ~3 dB gain
            float combined_mag = peak_mag;
            if (peak_mag > 0.0f && peak_mag < ZC_LOW_SNR_COHERENT_THRESHOLD) {
                int rep2_pos = timing_pos + single_rep;
                if (rep2_pos + single_rep <= static_cast<int>(samples.size())) {
                    float rep1_mag = computeCorrelationMag(samples, zc, timing_pos, known_cfo_hz);
                    float rep2_mag = computeCorrelationMag(samples, zc, rep2_pos, known_cfo_hz);
                    // Non-coherent combining: sqrt(sum of powers) normalized
                    // This provides ~1.5 dB gain vs single peak
                    combined_mag = std::sqrt(rep1_mag * rep1_mag + rep2_mag * rep2_mag) / std::sqrt(2.0f);
                    // Use whichever is higher: single peak or combined
                    combined_mag = std::max(combined_mag, peak_mag);
                    if (debug && combined_mag > peak_mag) {
                        fprintf(stderr, "  [DEBUG] Low-SNR combining: peak=%.4f, rep1=%.4f, rep2=%.4f, combined=%.4f\n",
                                peak_mag, rep1_mag, rep2_mag, combined_mag);
                    }
                }
            }

            if (debug) {
                fprintf(stderr, "  [DEBUG] root=%d: peak_pos=%d, timing_pos=%d, peak_mag=%.4f, combined=%.4f\n",
                        root, peak_pos, timing_pos, peak_mag, combined_mag);
            }

            if (combined_mag > best_corr) {
                best_corr = combined_mag;  // FIX: Use combined magnitude for detection
                best_root = root;
                best_pos = timing_pos;  // Use timing-adjusted position for start_sample

                // CFO estimation using ZC-matched correlation phase difference
                // The phase difference between correlations at rep1 and rep2 gives CFO
                // Note: Unambiguous range is ±1/(2*T_rep) ≈ ±23.6 Hz (see ZC_MAX_UNAMBIGUOUS_CFO_HZ)
                int rx_len = static_cast<int>(samples.size());
                int rep2_pos = timing_pos + single_rep;

                if (rep2_pos + single_rep <= rx_len) {
                    int ref_chips = static_cast<int>(zc.size());
                    int upsample = config_.upsample_factor;
                    int ref_samples = ref_chips * upsample;

                    // Compute ZC-matched correlation at both repetition positions
                    // This reuses the correlation computation for efficiency
                    auto computeZCCorr = [&](int lag) -> Complex {
                        Complex sum(0.0f, 0.0f);
                        float downconvert_freq = config_.carrier_freq + known_cfo_hz;
                        for (int i = 0; i < ref_samples; ++i) {
                            float t = static_cast<float>(lag + i) / config_.sample_rate;
                            float phase = -2.0f * M_PI * downconvert_freq * t;
                            Complex bb = samples[lag + i] * Complex(std::cos(phase), std::sin(phase));

                            float chip_pos_f = static_cast<float>(i) / upsample;
                            int chip_idx = static_cast<int>(chip_pos_f);
                            float frac = chip_pos_f - chip_idx;
                            Complex zc_interp;
                            if (chip_idx < ref_chips - 1) {
                                zc_interp = zc[chip_idx] * (1.0f - frac) + zc[chip_idx + 1] * frac;
                            } else {
                                zc_interp = zc[chip_idx];
                            }
                            sum += bb * std::conj(zc_interp);
                        }
                        return sum;
                    };

                    Complex corr1 = computeZCCorr(timing_pos);
                    Complex corr2 = computeZCCorr(rep2_pos);

                    // Only estimate CFO if both correlations have sufficient confidence
                    float corr1_mag = std::abs(corr1) / ref_samples;
                    float corr2_mag = std::abs(corr2) / ref_samples;

                    if (corr1_mag > ZC_CFO_CONFIDENCE_THRESHOLD && corr2_mag > ZC_CFO_CONFIDENCE_THRESHOLD) {
                        // Phase difference between correlations gives CFO
                        float phase_diff = std::arg(corr2 * std::conj(corr1));
                        float rep_duration = static_cast<float>(single_rep) / config_.sample_rate;
                        best_cfo = phase_diff / (2.0f * M_PI * rep_duration);

                        if (debug) {
                            fprintf(stderr, "  [DEBUG] CFO: corr1=%.3f∠%.1f°, corr2=%.3f∠%.1f°, "
                                    "phase_diff=%.3f rad, cfo=%.1f Hz\n",
                                    corr1_mag, std::arg(corr1) * 180.0f / M_PI,
                                    corr2_mag, std::arg(corr2) * 180.0f / M_PI,
                                    phase_diff, best_cfo);
                        }
                    } else if (debug) {
                        fprintf(stderr, "  [DEBUG] CFO: low confidence (corr1=%.3f, corr2=%.3f), skipping\n",
                                corr1_mag, corr2_mag);
                    }
                }
            }
        }

        // Preserve best candidate diagnostics even when threshold is not met.
        result.correlation = best_corr;
        result.root_detected = best_root;
        if (best_root >= 0) {
            result.frame_type = config_.getTypeForRoot(best_root);
        }

        if (best_corr > threshold && best_root >= 0) {
            result.detected = true;
            result.cfo_hz = best_cfo;
            result.start_sample = best_pos + preamble_len;
            result.snr_estimate = correlationToSNR(best_corr);

            if (debug) {
                fprintf(stderr, "[ZC] DETECTED: root=%d (%s), corr=%.3f, pos=%d, cfo=%.1f Hz\n",
                        best_root, zcFrameTypeToString(result.frame_type),
                        best_corr, best_pos, result.cfo_hz);
            }
        }

        return result;
    }

    const ZCConfig& getConfig() const { return config_; }

    const std::vector<Complex>& getTemplate(int root) const {
        return getZCTemplate(root);
    }

private:
    ZCConfig config_;
    mutable std::map<int, std::vector<Complex>> zc_templates_;

    void generateTemplates() {
        for (int root : {config_.root_ping, config_.root_pong,
                         config_.root_data, config_.root_control}) {
            zc_templates_[root] = generateZC(root);
        }
    }

    const std::vector<Complex>& getZCTemplate(int root) const {
        auto it = zc_templates_.find(root);
        if (it != zc_templates_.end()) {
            return it->second;
        }
        zc_templates_[root] = generateZC(root);
        return zc_templates_[root];
    }

    // Generate ZC sequence (same formula as spire implementation)
    std::vector<Complex> generateZC(int root) const {
        int length = config_.sequence_length;
        std::vector<Complex> zc(length);
        bool even = (length % 2 == 0);

        for (int n = 0; n < length; ++n) {
            float phase;
            if (even) {
                phase = -M_PI * root * n * n / length;
            } else {
                phase = -M_PI * root * n * (n + 1) / length;
            }
            zc[n] = Complex(std::cos(phase), std::sin(phase));
        }

        return zc;
    }

    // Compute normalized correlation magnitude at a specific lag position
    // Used to explicitly check if earlier position has valid correlation
    // Returns scale-invariant value in [0, 1]
    float computeCorrelationMag(SampleSpan received,
                                const std::vector<Complex>& zc_ref,
                                int lag,
                                float known_cfo_hz = 0.0f) const {
        int ref_chips = static_cast<int>(zc_ref.size());
        int upsample = config_.upsample_factor;
        int ref_samples = ref_chips * upsample;
        int rx_len = static_cast<int>(received.size());

        if (lag < 0 || lag + ref_samples > rx_len) {
            return 0.0f;
        }

        Complex sum(0.0f, 0.0f);
        float rx_energy = 0.0f;
        float ref_energy = static_cast<float>(ref_samples);  // ZC has unit magnitude

        float downconvert_freq = config_.carrier_freq + known_cfo_hz;
        for (int i = 0; i < ref_samples; ++i) {
            // Downconvert to baseband
            float t = static_cast<float>(lag + i) / config_.sample_rate;
            float phase = -2.0f * M_PI * downconvert_freq * t;
            Complex bb = received[lag + i] * Complex(std::cos(phase), std::sin(phase));

            // Interpolate ZC reference
            float chip_pos = static_cast<float>(i) / upsample;
            int chip_idx = static_cast<int>(chip_pos);
            float frac = chip_pos - chip_idx;
            Complex zc_interp;
            if (chip_idx < ref_chips - 1) {
                zc_interp = zc_ref[chip_idx] * (1.0f - frac) + zc_ref[chip_idx + 1] * frac;
            } else {
                zc_interp = zc_ref[chip_idx];
            }
            sum += bb * std::conj(zc_interp);
            rx_energy += std::norm(bb);
        }

        // Normalized correlation
        float denom = std::sqrt(rx_energy * ref_energy);
        return (denom > 1e-10f) ? std::abs(sum) / denom : 0.0f;
    }

    // Correlate received signal with ZC reference (from spire implementation)
    std::vector<Complex> correlate(SampleSpan received,
                                   const std::vector<Complex>& zc_ref,
                                   bool debug,
                                   int root,
                                   float known_cfo_hz = 0.0f) const {
        int ref_chips = static_cast<int>(zc_ref.size());
        int upsample = config_.upsample_factor;
        int ref_samples = ref_chips * upsample;
        int rx_len = static_cast<int>(received.size());

        if (rx_len < ref_samples) {
            return {};
        }

        // Downconvert received signal to baseband
        std::vector<Complex> baseband(rx_len);
        float downconvert_freq = config_.carrier_freq + known_cfo_hz;
        for (int i = 0; i < rx_len; ++i) {
            float t = static_cast<float>(i) / config_.sample_rate;
            float phase = -2.0f * M_PI * downconvert_freq * t;
            baseband[i] = received[i] * Complex(std::cos(phase), std::sin(phase));
        }

        // Correlate with upsampled/interpolated ZC reference
        int corr_len = rx_len - ref_samples + 1;
        std::vector<Complex> correlation(corr_len);

        // Compute correlation in steps for efficiency
        int step = std::max(1, ref_samples / 32);

        // Debug: check baseband energy
        if (debug) {
            float bb_energy = 0.0f;
            for (int i = 0; i < std::min(rx_len, 2000); ++i) {
                bb_energy += std::abs(baseband[i]) * std::abs(baseband[i]);
            }
            bb_energy = std::sqrt(bb_energy / std::min(rx_len, 2000));
            fprintf(stderr, "[ZC] correlate root=%d: rx_len=%d, ref_samples=%d, corr_len=%d, step=%d, bb_rms=%.4f\n",
                    root, rx_len, ref_samples, corr_len, step, bb_energy);
        }

        // Coarse search
        int coarse_best_pos = 0;
        float coarse_best_mag = 0.0f;

        // Pre-compute reference energy (ZC has unit magnitude, so this is just ref_samples)
        float ref_energy = static_cast<float>(ref_samples);

        for (int lag = 0; lag < corr_len; lag += step) {
            Complex sum(0.0f, 0.0f);
            float rx_energy = 0.0f;
            for (int i = 0; i < ref_samples; ++i) {
                // Interpolate ZC reference for this sample
                float chip_pos = static_cast<float>(i) / upsample;
                int chip_idx = static_cast<int>(chip_pos);
                float frac = chip_pos - chip_idx;

                Complex zc_interp;
                if (chip_idx < ref_chips - 1) {
                    zc_interp = zc_ref[chip_idx] * (1.0f - frac) + zc_ref[chip_idx + 1] * frac;
                } else {
                    zc_interp = zc_ref[chip_idx];
                }

                Complex bb_sample = baseband[lag + i];
                sum += bb_sample * std::conj(zc_interp);
                rx_energy += std::norm(bb_sample);  // |bb|^2
            }
            // Normalized correlation: |corr| / sqrt(rx_energy * ref_energy)
            // This is scale-invariant and ranges [0, 1]
            float denom = std::sqrt(rx_energy * ref_energy);
            float mag = (denom > 1e-10f) ? std::abs(sum) / denom : 0.0f;
            if (mag > coarse_best_mag) {
                coarse_best_mag = mag;
                coarse_best_pos = lag;
            }
        }

        if (debug) {
            fprintf(stderr, "[ZC] correlate root=%d: coarse_best_pos=%d, coarse_best_mag=%.4f\n",
                    root, coarse_best_pos, coarse_best_mag);
        }

        // Fine search around coarse peak
        int fine_start = std::max(0, coarse_best_pos - step);
        int fine_end = std::min(corr_len, coarse_best_pos + step + 1);

        for (int lag = fine_start; lag < fine_end; ++lag) {
            Complex sum(0.0f, 0.0f);
            float rx_energy = 0.0f;
            for (int i = 0; i < ref_samples; ++i) {
                float chip_pos = static_cast<float>(i) / upsample;
                int chip_idx = static_cast<int>(chip_pos);
                float frac = chip_pos - chip_idx;

                Complex zc_interp;
                if (chip_idx < ref_chips - 1) {
                    zc_interp = zc_ref[chip_idx] * (1.0f - frac) + zc_ref[chip_idx + 1] * frac;
                } else {
                    zc_interp = zc_ref[chip_idx];
                }

                Complex bb_sample = baseband[lag + i];
                sum += bb_sample * std::conj(zc_interp);
                rx_energy += std::norm(bb_sample);
            }
            // Store normalized correlation (magnitude only for peak finding)
            float denom = std::sqrt(rx_energy * ref_energy);
            float norm_mag = (denom > 1e-10f) ? std::abs(sum) / denom : 0.0f;
            // Store as complex with normalized magnitude (phase preserved for CFO)
            correlation[lag] = (denom > 1e-10f) ? sum / denom : Complex(0.0f, 0.0f);
        }

        // Fill in coarse positions for peak finding (with normalized correlation)
        for (int lag = 0; lag < corr_len; lag += step) {
            if (lag < fine_start || lag >= fine_end) {
                // Recompute for this position
                Complex sum(0.0f, 0.0f);
                float rx_energy = 0.0f;
                for (int i = 0; i < ref_samples; ++i) {
                    float chip_pos = static_cast<float>(i) / upsample;
                    int chip_idx = static_cast<int>(chip_pos);
                    float frac = chip_pos - chip_idx;

                    Complex zc_interp;
                    if (chip_idx < ref_chips - 1) {
                        zc_interp = zc_ref[chip_idx] * (1.0f - frac) + zc_ref[chip_idx + 1] * frac;
                    } else {
                        zc_interp = zc_ref[chip_idx];
                    }

                    Complex bb_sample = baseband[lag + i];
                    sum += bb_sample * std::conj(zc_interp);
                    rx_energy += std::norm(bb_sample);
                }
                float denom = std::sqrt(rx_energy * ref_energy);
                correlation[lag] = (denom > 1e-10f) ? sum / denom : Complex(0.0f, 0.0f);
            }
        }

        return correlation;
    }

    float correlationToSNR(float corr) const {
        if (corr <= 0.01f) return -10.0f;
        if (corr >= 0.99f) return 30.0f;
        float snr = 20.0f * std::log10(corr / (1.0f - corr + 0.01f));
        return std::clamp(snr, -10.0f, 30.0f);
    }
};

} // namespace sync
} // namespace ultra
