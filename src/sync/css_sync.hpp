// CSS (Chirp Spread Spectrum) Sync with Cyclic Shift Encoding
//
// Replaces dual up/down chirp with single chirp + cyclic shift encoding.
// Frame type is encoded in the cyclic shift, detected via dechirp + FFT.
//
// Benefits:
// - Frame type known immediately from preamble (no energy ratio check)
// - Robust at low SNR - if chirp detected, frame type is known
// - Single architecture for all modes (MC-DPSK, OFDM, PING/PONG)
//
// Cyclic Shift Encoding:
// - 4 shifts = 2 bits, evenly spaced across chirp duration
// - Shift 0: PING
// - Shift 1: PONG
// - Shift 2: DATA (MC-DPSK or OFDM)
// - Shift 3: CONTROL (CONNECT, DISCONNECT, ACK, NACK)
//
// Detection:
// - Dechirp: multiply received signal by conjugate of base chirp
// - FFT: peak bin index indicates cyclic shift
// - CFO: estimated from FFT peak phase or bin offset

#pragma once

#include "ultra/types.hpp"
#include <vector>
#include <cmath>
#include <algorithm>

namespace ultra {
namespace sync {

// Frame types encoded in CSS cyclic shift
enum class CSSFrameType : uint8_t {
    PING = 0,      // Probe/presence check
    PONG = 1,      // Response to PING
    DATA = 2,      // Data frame (MC-DPSK or OFDM)
    CONTROL = 3,   // Control frame (CONNECT, DISCONNECT, ACK, NACK)
    UNKNOWN = 255  // Detection failed
};

inline const char* cssFrameTypeToString(CSSFrameType type) {
    switch (type) {
        case CSSFrameType::PING: return "PING";
        case CSSFrameType::PONG: return "PONG";
        case CSSFrameType::DATA: return "DATA";
        case CSSFrameType::CONTROL: return "CONTROL";
        default: return "UNKNOWN";
    }
}

// CSS configuration
struct CSSConfig {
    float sample_rate = 48000.0f;
    float f_start = 300.0f;       // Start frequency (Hz)
    float f_end = 2700.0f;        // End frequency (Hz)
    float duration_ms = 500.0f;   // Single chirp duration (ms)
    float gap_ms = 100.0f;        // Gap after chirp (for settling)
    int num_shifts = 4;           // Number of cyclic shifts (frame types)
    int num_chirps = 2;           // Number of chirps in preamble (for averaging)

    // Derived parameters
    int chirpSamples() const {
        return static_cast<int>(sample_rate * duration_ms / 1000.0f);
    }
    float bandwidth() const { return f_end - f_start; }
    int gapSamples() const {
        return static_cast<int>(sample_rate * gap_ms / 1000.0f);
    }
    int totalPreambleSamples() const {
        return num_chirps * chirpSamples() + (num_chirps - 1) * gapSamples() + gapSamples();
    }
};

// Result from CSS sync detection
struct CSSSyncResult {
    bool detected = false;
    CSSFrameType frame_type = CSSFrameType::UNKNOWN;
    int start_sample = -1;        // Sample position where data starts (after preamble)
    float correlation = 0.0f;     // Peak correlation strength (0-1)
    float cfo_hz = 0.0f;          // Estimated carrier frequency offset
    float snr_estimate = 0.0f;    // Estimated SNR from detection
};

class CSSSync {
public:
    explicit CSSSync(const CSSConfig& config = CSSConfig())
        : config_(config)
    {
        generateTemplates();
    }

    // Generate preamble for given frame type
    Samples generatePreamble(CSSFrameType frame_type) const {
        int shift = static_cast<int>(frame_type);
        if (shift < 0 || shift >= config_.num_shifts) {
            shift = static_cast<int>(CSSFrameType::DATA);
        }

        Samples result;
        int chirp_samples = config_.chirpSamples();
        int gap_samples = config_.gapSamples();

        // Generate num_chirps identical chirps with the cyclic shift
        for (int c = 0; c < config_.num_chirps; c++) {
            // Add chirp with cyclic shift
            auto chirp = generateChirpWithShift(shift);
            result.insert(result.end(), chirp.begin(), chirp.end());

            // Add gap (except after last chirp, we add it for settling)
            for (int i = 0; i < gap_samples; i++) {
                result.push_back(0.0f);
            }
        }

        return result;
    }

    // Detect CSS preamble in received samples
    // Returns detection result with frame type
    CSSSyncResult detect(SampleSpan samples, float threshold = 0.3f) const {
        CSSSyncResult result;

        int chirp_samples = config_.chirpSamples();
        int gap_samples = config_.gapSamples();
        int min_samples = chirp_samples + gap_samples;

        if (samples.size() < static_cast<size_t>(min_samples)) {
            return result;
        }

        // Slide through samples looking for chirp
        float best_corr = 0.0f;
        int best_pos = -1;
        int best_shift = -1;
        float best_cfo = 0.0f;

        // Search step - balance speed vs accuracy
        const int step = std::max(1, chirp_samples / 100);

        for (size_t pos = 0; pos + chirp_samples <= samples.size(); pos += step) {
            // Matched filter detection (works with real-valued audio)
            auto [shift, corr, cfo] = matchedFilterDetect(samples.data() + pos, chirp_samples);

            if (corr > best_corr && corr > threshold) {
                best_corr = corr;
                best_pos = static_cast<int>(pos);
                best_shift = shift;
                best_cfo = cfo;
            }
        }

        if (best_pos >= 0 && best_shift >= 0) {
            // Refine position with finer search
            int refine_start = std::max(0, best_pos - step);
            int refine_end = std::min(static_cast<int>(samples.size()) - chirp_samples, best_pos + step);

            for (int pos = refine_start; pos <= refine_end; pos++) {
                auto [shift, corr, cfo] = matchedFilterDetect(samples.data() + pos, chirp_samples);
                if (corr > best_corr) {
                    best_corr = corr;
                    best_pos = pos;
                    best_shift = shift;
                    best_cfo = cfo;
                }
            }

            result.detected = true;
            result.frame_type = static_cast<CSSFrameType>(best_shift);
            result.correlation = best_corr;
            result.cfo_hz = best_cfo;
            result.snr_estimate = correlationToSNR(best_corr);

            // Data starts after all chirps + gaps
            if (config_.num_chirps == 2) {
                int chirp_gap = chirp_samples + gap_samples;
                int search_range = chirp_samples / 50;  // ~1% of chirp duration
                bool found_pair = false;
                int first_chirp_pos = best_pos;

                // Helper to search for a chirp in a range
                auto searchChirp = [&](int center) -> std::tuple<int, int, float, float> {
                    int start = std::max(0, center - search_range);
                    int end = std::min(static_cast<int>(samples.size()) - chirp_samples, center + search_range);
                    float bc = 0.0f;
                    int bs = -1, bp = center;
                    float bcfo = 0.0f;
                    for (int p = start; p <= end; p += 10) {
                        auto [s, c, cf] = matchedFilterDetect(samples.data() + p, chirp_samples);
                        if (c > bc) { bc = c; bs = s; bp = p; bcfo = cf; }
                    }
                    return {bs, bp, bc, bcfo};
                };

                // Try 1: Look for second chirp AFTER best_pos
                auto [shift_after, pos_after, corr_after, cfo_after] = searchChirp(best_pos + chirp_gap);
                if (shift_after == best_shift && corr_after > threshold * 0.8f) {
                    found_pair = true;
                    result.correlation = (best_corr + corr_after) / 2.0f;
                    result.cfo_hz = (best_cfo + cfo_after) / 2.0f;
                    result.start_sample = pos_after + chirp_samples + gap_samples;
                }

                // Try 2: If after failed, look for first chirp BEFORE best_pos
                // (Maybe we found the second chirp first)
                if (!found_pair && best_pos >= chirp_gap) {
                    auto [shift_before, pos_before, corr_before, cfo_before] = searchChirp(best_pos - chirp_gap);
                    if (shift_before == best_shift && corr_before > threshold * 0.8f) {
                        found_pair = true;
                        first_chirp_pos = pos_before;
                        result.correlation = (corr_before + best_corr) / 2.0f;
                        result.cfo_hz = (cfo_before + best_cfo) / 2.0f;
                        result.start_sample = best_pos + chirp_samples + gap_samples;
                    }
                }

                if (!found_pair) {
                    // Pair validation failed - report with lower confidence
                    result.start_sample = first_chirp_pos + config_.totalPreambleSamples();
                    result.correlation *= 0.7f;
                }
            } else {
                result.start_sample = best_pos + chirp_samples + gap_samples;
            }
        }

        return result;
    }

    // Get current configuration
    const CSSConfig& getConfig() const { return config_; }

    // Get preamble duration in milliseconds
    float getPreambleDurationMs() const {
        return static_cast<float>(config_.totalPreambleSamples()) / config_.sample_rate * 1000.0f;
    }

private:
    CSSConfig config_;
    std::vector<Samples> chirp_templates_;  // One template per cyclic shift
    std::vector<float> template_energies_;   // Energy of each template

    void generateTemplates() {
        int N = config_.chirpSamples();
        float fs = config_.sample_rate;
        float T = config_.duration_ms / 1000.0f;

        // Total bandwidth divided into num_shifts bands
        float total_bw = config_.bandwidth();
        float band_bw = total_bw / config_.num_shifts;

        chirp_templates_.resize(config_.num_shifts);
        template_energies_.resize(config_.num_shifts);

        // Generate chirp for each frequency band (orthogonal approach)
        // Band 0: 300-900 Hz (PING)
        // Band 1: 900-1500 Hz (PONG)
        // Band 2: 1500-2100 Hz (DATA)
        // Band 3: 2100-2700 Hz (CONTROL)
        for (int band = 0; band < config_.num_shifts; band++) {
            float f0 = config_.f_start + band * band_bw;
            float f1 = f0 + band_bw;
            float k = (f1 - f0) / T;  // Chirp rate for this band

            chirp_templates_[band].resize(N);
            float energy = 0.0f;

            for (int i = 0; i < N; i++) {
                float t = static_cast<float>(i) / fs;
                float phase = 2.0f * M_PI * (f0 * t + 0.5f * k * t * t);
                chirp_templates_[band][i] = std::cos(phase);
                energy += chirp_templates_[band][i] * chirp_templates_[band][i];
            }
            template_energies_[band] = std::sqrt(energy);
        }
    }

    // Generate chirp with cyclic shift
    Samples generateChirpWithShift(int shift) const {
        if (shift >= 0 && shift < static_cast<int>(chirp_templates_.size())) {
            return chirp_templates_[shift];
        }
        return chirp_templates_[0];
    }

    // Detect cyclic shift using matched filtering with each template
    // Returns: (shift, correlation, cfo_hz)
    std::tuple<int, float, float> matchedFilterDetect(const float* samples, int count) const {
        int N = config_.chirpSamples();
        if (count < N) {
            return {-1, 0.0f, 0.0f};
        }

        // Calculate received signal energy
        float rx_energy = 0.0f;
        for (int i = 0; i < N; i++) {
            rx_energy += samples[i] * samples[i];
        }
        rx_energy = std::sqrt(rx_energy);

        if (rx_energy < 0.001f) {
            return {-1, 0.0f, 0.0f};
        }

        // Correlate with each shifted template
        float best_corr = 0.0f;
        int best_shift = 0;

        for (int shift = 0; shift < config_.num_shifts; shift++) {
            const Samples& templ = chirp_templates_[shift];
            float templ_energy = template_energies_[shift];

            // Cross-correlation at lag 0
            float corr = 0.0f;
            for (int i = 0; i < N; i++) {
                corr += samples[i] * templ[i];
            }

            // Normalize
            float norm_corr = corr / (rx_energy * templ_energy + 0.001f);
            norm_corr = std::abs(norm_corr);  // Handle phase inversion

            if (norm_corr > best_corr) {
                best_corr = norm_corr;
                best_shift = shift;
            }
        }

        // Simple CFO estimation using phase difference between signal halves
        // (This is a placeholder - more sophisticated methods exist)
        float cfo_hz = 0.0f;

        return {best_shift, best_corr, cfo_hz};
    }

    // Convert correlation strength to SNR estimate (dB)
    float correlationToSNR(float corr) const {
        // Empirical mapping: corr ~0.3 at 0dB, ~0.9 at 20dB
        if (corr <= 0.01f) return -10.0f;
        if (corr >= 0.99f) return 30.0f;

        // Approximate: SNR = 20 * log10(corr / (1 - corr))
        float snr = 20.0f * std::log10(corr / (1.0f - corr + 0.01f));
        return std::clamp(snr, -10.0f, 30.0f);
    }
};

} // namespace sync
} // namespace ultra
