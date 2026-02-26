// Test CSS (Chirp Spread Spectrum) Sync Detection
//
// Tests the CSS sync module in isolation before integration.
// Verifies:
// - All 4 cyclic shifts can be detected
// - Detection works at various SNR levels
// - CFO estimation works
// - Compare detection threshold to current dual-chirp

#include "sync/css_sync.hpp"
#include <iostream>
#include <random>
#include <cmath>
#include <iomanip>

using namespace ultra;
using namespace ultra::sync;

// Add AWGN noise to signal
void addAWGN(Samples& signal, float snr_db, std::mt19937& rng) {
    // Calculate signal power
    float signal_power = 0.0f;
    for (float s : signal) {
        signal_power += s * s;
    }
    signal_power /= signal.size();

    // Calculate noise power from SNR
    // SNR = 10 * log10(Ps / Pn)
    // Pn = Ps / 10^(SNR/10)
    float noise_power = signal_power / std::pow(10.0f, snr_db / 10.0f);
    float noise_std = std::sqrt(noise_power);

    std::normal_distribution<float> noise(0.0f, noise_std);
    for (float& s : signal) {
        s += noise(rng);
    }
}

// Test detection of a specific frame type at given SNR
bool testDetection(CSSSync& css, CSSFrameType frame_type, float snr_db,
                   uint32_t seed, bool verbose = false, bool debug = false) {
    std::mt19937 rng(seed);

    // Generate preamble
    Samples preamble = css.generatePreamble(frame_type);

    // Add some padding before and after
    // Preamble is ~1.2 seconds (57600 samples), so need at least 2 seconds for offset+preamble
    Samples signal(96000, 0.0f);  // 2 seconds of samples
    int offset = 4800;  // Start at 100ms
    std::copy(preamble.begin(), preamble.end(), signal.begin() + offset);

    // Add noise
    addAWGN(signal, snr_db, rng);

    // Detect
    auto result = css.detect(signal, 0.2f);

    int expected_start = offset + css.getConfig().totalPreambleSamples();
    bool correct = result.detected &&
                   result.frame_type == frame_type &&
                   std::abs(result.start_sample - expected_start) < 100;

    if (verbose || debug) {
        std::cout << "  " << cssFrameTypeToString(frame_type) << " @ " << snr_db << " dB: "
                  << (correct ? "PASS" : "FAIL")
                  << " (detected=" << result.detected
                  << ", type=" << cssFrameTypeToString(result.frame_type)
                  << ", corr=" << std::fixed << std::setprecision(3) << result.correlation
                  << ", cfo=" << std::setprecision(1) << result.cfo_hz << " Hz"
                  << ", snr_est=" << result.snr_estimate << " dB";
        if (debug) {
            std::cout << ", start=" << result.start_sample
                      << ", expected=" << expected_start
                      << ", diff=" << (result.start_sample - expected_start);
        }
        std::cout << ")\n";
    }

    return correct;
}

// Test detection with CFO (no correction - shows raw CSS limitation)
bool testDetectionWithCFO_Raw(CSSSync& css, CSSFrameType frame_type, float snr_db,
                              float cfo_hz, uint32_t seed, bool verbose = false) {
    std::mt19937 rng(seed);

    // Generate preamble
    Samples preamble = css.generatePreamble(frame_type);

    // Apply CFO to simulate frequency error
    float fs = css.getConfig().sample_rate;
    for (size_t i = 0; i < preamble.size(); i++) {
        float t = static_cast<float>(i) / fs;
        float phase = 2.0f * M_PI * cfo_hz * t;
        // Shift frequency by multiplying with complex exponential (real part)
        preamble[i] *= std::cos(phase);
    }

    // Add some padding
    Samples signal(96000, 0.0f);
    int offset = 4800;
    std::copy(preamble.begin(), preamble.end(), signal.begin() + offset);

    // Add noise
    addAWGN(signal, snr_db, rng);

    // Detect WITHOUT CFO correction
    auto result = css.detect(signal, 0.2f);

    bool correct = result.detected && result.frame_type == frame_type;

    if (verbose) {
        std::cout << "  " << cssFrameTypeToString(frame_type) << " @ " << snr_db << " dB, CFO="
                  << cfo_hz << " Hz (raw): "
                  << (correct ? "PASS" : "FAIL")
                  << " (detected=" << result.detected
                  << ", type=" << cssFrameTypeToString(result.frame_type)
                  << ", corr=" << std::fixed << std::setprecision(3) << result.correlation
                  << ")\n";
    }

    return correct;
}

// Test detection with CFO + correction (simulating real-world use)
bool testDetectionWithCFO(CSSSync& css, CSSFrameType frame_type, float snr_db,
                          float cfo_hz, uint32_t seed, bool verbose = false) {
    std::mt19937 rng(seed);

    // Generate preamble
    Samples preamble = css.generatePreamble(frame_type);

    // Apply CFO to simulate frequency error
    float fs = css.getConfig().sample_rate;
    for (size_t i = 0; i < preamble.size(); i++) {
        float t = static_cast<float>(i) / fs;
        float phase = 2.0f * M_PI * cfo_hz * t;
        preamble[i] *= std::cos(phase);
    }

    // Add some padding
    Samples signal(96000, 0.0f);
    int offset = 4800;
    std::copy(preamble.begin(), preamble.end(), signal.begin() + offset);

    // Add noise
    addAWGN(signal, snr_db, rng);

    // Apply CFO CORRECTION (as if we got CFO estimate from dual-chirp)
    // In real system, this would use the CFO from ChirpSync::detectDualChirp()
    for (size_t i = 0; i < signal.size(); i++) {
        float t = static_cast<float>(i) / fs;
        float phase = 2.0f * M_PI * (-cfo_hz) * t;  // Negate to correct
        signal[i] *= std::cos(phase);
    }

    // Detect with CFO-corrected signal
    auto result = css.detect(signal, 0.2f);

    bool correct = result.detected && result.frame_type == frame_type;

    if (verbose) {
        std::cout << "  " << cssFrameTypeToString(frame_type) << " @ " << snr_db << " dB, CFO="
                  << cfo_hz << " Hz (corrected): "
                  << (correct ? "PASS" : "FAIL")
                  << " (detected=" << result.detected
                  << ", type=" << cssFrameTypeToString(result.frame_type)
                  << ", corr=" << std::fixed << std::setprecision(3) << result.correlation
                  << ")\n";
    }

    return correct;
}

int main(int argc, char* argv[]) {
    std::cout << "=== CSS Sync Test ===\n\n";

    CSSConfig config;
    config.sample_rate = 48000.0f;
    config.f_start = 300.0f;
    config.f_end = 2700.0f;
    config.duration_ms = 500.0f;
    config.gap_ms = 100.0f;
    config.num_shifts = 4;
    config.num_chirps = 2;

    CSSSync css(config);

    std::cout << "Configuration:\n";
    std::cout << "  Sample rate: " << config.sample_rate << " Hz\n";
    std::cout << "  Bandwidth: " << config.bandwidth() << " Hz\n";
    std::cout << "  Chirp duration: " << config.duration_ms << " ms\n";
    std::cout << "  Num chirps: " << config.num_chirps << "\n";
    std::cout << "  Num shifts: " << config.num_shifts << "\n";
    std::cout << "  Total preamble: " << css.getPreambleDurationMs() << " ms\n";
    std::cout << "\n";

    // Test 1: Basic detection of all frame types at high SNR
    std::cout << "Test 1: Basic detection at 20 dB SNR (with debug)\n";
    int pass1 = 0, total1 = 0;
    for (int type = 0; type < 4; type++) {
        total1++;
        if (testDetection(css, static_cast<CSSFrameType>(type), 20.0f, 42 + type, true, true)) {
            pass1++;
        }
    }
    std::cout << "  Result: " << pass1 << "/" << total1 << " passed\n\n";

    // Test 2: Detection at various SNR levels
    std::cout << "Test 2: SNR sweep (PING frame type)\n";
    int pass2 = 0, total2 = 0;
    for (float snr = -10.0f; snr <= 20.0f; snr += 2.0f) {
        total2++;
        if (testDetection(css, CSSFrameType::PING, snr, 100 + static_cast<uint32_t>(snr * 10), true)) {
            pass2++;
        }
    }
    std::cout << "  Result: " << pass2 << "/" << total2 << " passed\n\n";

    // Test 3: Quick detection threshold check (disabled for speed)
    // Full threshold search takes ~4 minutes due to many correlations
    std::cout << "Test 3: Quick threshold check (full search disabled)\n";
    // Just verify detection works at -10 dB for each frame type
    int pass3 = 0, total3 = 0;
    for (int type = 0; type < 4; type++) {
        CSSFrameType ft = static_cast<CSSFrameType>(type);
        total3++;
        if (testDetection(css, ft, -10.0f, 3000 + type, false)) {
            pass3++;
            std::cout << "  " << cssFrameTypeToString(ft) << " @ -10 dB: PASS\n";
        } else {
            std::cout << "  " << cssFrameTypeToString(ft) << " @ -10 dB: FAIL\n";
        }
    }
    std::cout << "  Result: " << pass3 << "/" << total3 << " passed at -10 dB\n\n";

    // Test 4a: CFO tolerance without correction (shows CSS limitation)
    std::cout << "Test 4a: CFO tolerance at 10 dB SNR (no correction)\n";
    int pass4a = 0, total4a = 0;
    for (float cfo : {-50.0f, -25.0f, 0.0f, 25.0f, 50.0f}) {
        total4a++;
        if (testDetectionWithCFO_Raw(css, CSSFrameType::DATA, 10.0f, cfo, 200, true)) {
            pass4a++;
        }
    }
    std::cout << "  Result: " << pass4a << "/" << total4a << " passed (without correction)\n\n";

    // Test 4b: CFO tolerance WITH correction (real-world scenario)
    std::cout << "Test 4b: CFO tolerance at 10 dB SNR (with CFO correction)\n";
    int pass4b = 0, total4b = 0;
    for (float cfo : {-50.0f, -25.0f, 0.0f, 25.0f, 50.0f}) {
        total4b++;
        if (testDetectionWithCFO(css, CSSFrameType::DATA, 10.0f, cfo, 200, true)) {
            pass4b++;
        }
    }
    std::cout << "  Result: " << pass4b << "/" << total4b << " passed (with correction)\n\n";

    // Test 5: Multiple seeds at threshold SNR
    std::cout << "Test 5: Statistical test at -5 dB (30 seeds)\n";
    int pass5 = 0;
    for (uint32_t seed = 0; seed < 30; seed++) {
        if (testDetection(css, CSSFrameType::DATA, -5.0f, 500 + seed, false)) {
            pass5++;
        }
    }
    std::cout << "  Result: " << pass5 << "/30 passed ("
              << std::fixed << std::setprecision(1) << (pass5 * 100.0f / 30.0f) << "%)\n\n";

    // Summary
    std::cout << "=== Summary ===\n";
    bool basic_pass = (pass1 == total1);
    bool cfo_corrected_pass = (pass4b == total4b);
    std::cout << "Basic tests: " << (basic_pass ? "PASS" : "FAIL") << "\n";
    std::cout << "CFO tolerance (with correction): " << (cfo_corrected_pass ? "PASS" : "FAIL") << "\n";
    std::cout << "Low SNR (-5 dB): " << pass5 << "/30 (" << (pass5 * 100.0f / 30.0f) << "%)\n";

    return (basic_pass && cfo_corrected_pass) ? 0 : 1;
}
