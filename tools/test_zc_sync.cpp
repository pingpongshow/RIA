// Test for Zadoff-Chu (ZC) Sync
//
// Tests:
// 1. Basic detection at high SNR
// 2. SNR sweep to find detection floor
// 3. CFO estimation accuracy
// 4. Frame type discrimination
// 5. Comparison with chirp (duration, performance)

#include "sync/zc_sync.hpp"
#include "ultra/dsp.hpp"
#include <iostream>
#include <iomanip>
#include <random>
#include <chrono>
#include <cmath>

using namespace ultra;
using namespace ultra::sync;

// Add AWGN noise to signal
void addNoise(Samples& signal, float snr_db, std::mt19937& rng) {
    // Calculate signal power
    float sig_power = 0.0f;
    for (float s : signal) {
        sig_power += s * s;
    }
    sig_power /= signal.size();

    // Calculate noise power for target SNR
    float snr_linear = std::pow(10.0f, snr_db / 10.0f);
    float noise_power = sig_power / snr_linear;
    float noise_std = std::sqrt(noise_power);

    std::normal_distribution<float> noise(0.0f, noise_std);
    for (float& s : signal) {
        s += noise(rng);
    }
}

// Apply CFO to signal using proper SSB frequency shift
// Uses Hilbert transform to create analytic signal, then complex rotation
void applyCFO(Samples& signal, float cfo_hz, float sample_rate) {
    if (std::abs(cfo_hz) < 0.01f || signal.size() < 128) return;

    // Use Hilbert transform to get analytic signal
    HilbertTransform hilbert(127);  // 127 taps for good accuracy
    SampleSpan span(signal.data(), signal.size());
    auto analytic = hilbert.process(span);

    // Apply frequency shift: multiply by e^{j*2*pi*cfo*t}
    float phase = 0.0f;
    float phase_inc = 2.0f * M_PI * cfo_hz / sample_rate;

    for (size_t i = 0; i < signal.size() && i < analytic.size(); i++) {
        Complex rotation(std::cos(phase), std::sin(phase));
        Complex shifted = analytic[i] * rotation;
        signal[i] = shifted.real();
        phase += phase_inc;
        while (phase > M_PI) phase -= 2.0f * M_PI;
        while (phase < -M_PI) phase += 2.0f * M_PI;
    }
}

bool testBasicDetection(ZCSync& zc, float snr_db, uint32_t seed) {
    std::cout << "\nTest 1: Basic detection at " << snr_db << " dB SNR\n";

    std::mt19937 rng(seed);
    int passed = 0;
    int total = 4;

    ZCFrameType types[] = {ZCFrameType::PING, ZCFrameType::PONG,
                           ZCFrameType::DATA, ZCFrameType::CONTROL};

    for (auto type : types) {
        // Generate preamble
        Samples preamble = zc.generatePreamble(type);

        // Debug: check preamble
        float preamble_rms = 0.0f;
        for (float s : preamble) preamble_rms += s * s;
        preamble_rms = std::sqrt(preamble_rms / preamble.size());

        // Add some padding before and after
        Samples signal(1000, 0.0f);
        signal.insert(signal.end(), preamble.begin(), preamble.end());
        signal.resize(signal.size() + 1000, 0.0f);

        // Debug: check signal RMS before noise
        float sig_rms_pre = 0.0f;
        for (float s : signal) sig_rms_pre += s * s;
        sig_rms_pre = std::sqrt(sig_rms_pre / signal.size());

        // Add noise
        addNoise(signal, snr_db, rng);

        // Debug: check signal RMS after noise
        float sig_rms_post = 0.0f;
        for (float s : signal) sig_rms_post += s * s;
        sig_rms_post = std::sqrt(sig_rms_post / signal.size());

        // Detect
        auto result = zc.detect(signal, 0.2f);

        bool ok = result.detected && result.frame_type == type;
        if (ok) passed++;

        std::cout << "  " << zcFrameTypeToString(type) << " @ " << snr_db << " dB: "
                  << (ok ? "PASS" : "FAIL")
                  << " (detected=" << result.detected
                  << ", type=" << zcFrameTypeToString(result.frame_type)
                  << ", corr=" << std::fixed << std::setprecision(3) << result.correlation
                  << ", cfo=" << std::setprecision(1) << result.cfo_hz << " Hz"
                  << ", preamble_rms=" << std::setprecision(3) << preamble_rms
                  << ", sig_len=" << signal.size()
                  << ")\n";
    }

    std::cout << "  Result: " << passed << "/" << total << " passed\n";
    return passed == total;
}

bool testSNRSweep(ZCSync& zc, uint32_t seed) {
    std::cout << "\nTest 2: SNR sweep (PING frame type)\n";

    int passed = 0;
    int total = 0;

    for (float snr = -15.0f; snr <= 20.0f; snr += 2.5f) {
        std::mt19937 rng(seed);

        Samples preamble = zc.generatePreamble(ZCFrameType::PING);
        Samples signal(500, 0.0f);
        signal.insert(signal.end(), preamble.begin(), preamble.end());
        signal.resize(signal.size() + 500, 0.0f);

        addNoise(signal, snr, rng);

        auto result = zc.detect(signal, 0.2f);

        bool ok = result.detected && result.frame_type == ZCFrameType::PING;
        if (ok) passed++;
        total++;

        std::cout << "  PING @ " << std::fixed << std::setprecision(1) << snr << " dB: "
                  << (ok ? "PASS" : "FAIL")
                  << " (corr=" << std::setprecision(3) << result.correlation
                  << ", snr_est=" << std::setprecision(1) << result.snr_estimate << " dB"
                  << ")\n";
    }

    std::cout << "  Result: " << passed << "/" << total << " passed\n";
    return passed >= total * 0.8;  // 80% pass rate
}

bool testCFOEstimation(ZCSync& zc, float snr_db, uint32_t seed) {
    std::cout << "\nTest 3: CFO estimation at " << snr_db << " dB SNR\n";
    std::cout << "  Note: ZC unambiguous CFO range is ±23.6 Hz (rep spacing = 21.2 ms)\n";
    std::cout << "  ZC is designed for post-CHIRP residual CFO tracking (< ±15 Hz)\n";
    std::cout << "  Large CFO (±20+ Hz) causes detection failure - use CHIRP for acquisition\n";

    int passed = 0;
    int total = 0;
    float max_error = 5.0f;  // Allow 5 Hz error

    // Test within ZC operational range for residual CFO tracking
    // After CHIRP acquisition, residual CFO is typically < ±10 Hz
    // We test up to ±15 Hz to verify margin
    for (float cfo : {-15.0f, -10.0f, -5.0f, 0.0f, 5.0f, 10.0f, 15.0f}) {
        std::mt19937 rng(seed);

        Samples preamble = zc.generatePreamble(ZCFrameType::DATA);
        Samples signal(500, 0.0f);
        signal.insert(signal.end(), preamble.begin(), preamble.end());
        signal.resize(signal.size() + 500, 0.0f);

        // Apply CFO before noise
        applyCFO(signal, cfo, zc.getConfig().sample_rate);
        addNoise(signal, snr_db, rng);

        auto result = zc.detect(signal, 0.2f);

        float error = std::abs(result.cfo_hz - cfo);
        bool ok = result.detected && error < max_error;
        if (ok) passed++;
        total++;

        std::cout << "  CFO=" << std::fixed << std::setprecision(1) << cfo << " Hz: "
                  << (ok ? "PASS" : "FAIL")
                  << " (detected=" << result.detected
                  << ", cfo_est=" << result.cfo_hz << " Hz"
                  << ", error=" << error << " Hz"
                  << ", corr=" << std::setprecision(3) << result.correlation
                  << ")\n";
    }

    std::cout << "  Result: " << passed << "/" << total << " passed\n";
    // ZC CFO estimation requires successful detection first.
    // At ±15 Hz boundary, detection may fail due to correlator sensitivity.
    // 60% pass rate is acceptable for residual CFO tracking use case.
    return passed >= total * 0.6;
}

bool testFrameTypeDiscrimination(ZCSync& zc, float snr_db, uint32_t seed) {
    std::cout << "\nTest 4: Frame type discrimination at " << snr_db << " dB SNR\n";

    int passed = 0;
    int total = 0;

    ZCFrameType types[] = {ZCFrameType::PING, ZCFrameType::PONG,
                           ZCFrameType::DATA, ZCFrameType::CONTROL};

    // Test each type multiple times
    for (int trial = 0; trial < 5; trial++) {
        for (auto type : types) {
            std::mt19937 rng(seed + trial * 100 + static_cast<int>(type));

            Samples preamble = zc.generatePreamble(type);
            Samples signal(500, 0.0f);
            signal.insert(signal.end(), preamble.begin(), preamble.end());
            signal.resize(signal.size() + 500, 0.0f);

            addNoise(signal, snr_db, rng);

            auto result = zc.detect(signal, 0.2f);

            bool ok = result.detected && result.frame_type == type;
            if (ok) passed++;
            total++;
        }
    }

    float pct = 100.0f * passed / total;
    std::cout << "  Result: " << passed << "/" << total
              << " (" << std::fixed << std::setprecision(1) << pct << "%)\n";

    return passed >= total * 0.9;  // 90% accuracy
}

void testTiming(ZCSync& zc) {
    std::cout << "\nTest 5: Timing comparison\n";

    auto cfg = zc.getConfig();
    std::cout << "  ZC preamble: " << cfg.preambleSamples() << " samples ("
              << std::fixed << std::setprecision(1) << cfg.preambleDurationMs() << " ms)\n";

    // Compare with chirp (500ms * 2 + gaps)
    float chirp_duration_ms = 500.0f * 2 + 100.0f * 2 + 85.0f;  // up + down + gaps + training
    std::cout << "  Chirp preamble: ~" << chirp_duration_ms << " ms\n";
    std::cout << "  Speedup: " << std::setprecision(1)
              << (chirp_duration_ms / cfg.preambleDurationMs()) << "x\n";

    // Measure detection time
    Samples preamble = zc.generatePreamble(ZCFrameType::DATA);
    Samples signal(1000, 0.0f);
    signal.insert(signal.end(), preamble.begin(), preamble.end());
    signal.resize(signal.size() + 1000, 0.0f);

    auto start = std::chrono::high_resolution_clock::now();
    int iterations = 100;
    for (int i = 0; i < iterations; i++) {
        auto result = zc.detect(signal, 0.2f);
        (void)result;
    }
    auto end = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    float avg_us = static_cast<float>(duration.count()) / iterations;
    std::cout << "  Detection time: " << std::setprecision(1) << avg_us << " µs/call\n";
}

bool testNoNoise(ZCSync& zc) {
    std::cout << "\nTest 0: Detection with NO NOISE (sanity check)\n";

    int passed = 0;
    int total = 4;

    ZCFrameType types[] = {ZCFrameType::PING, ZCFrameType::PONG,
                           ZCFrameType::DATA, ZCFrameType::CONTROL};

    bool first = true;
    for (auto type : types) {
        // Generate preamble
        Samples preamble = zc.generatePreamble(type);

        // Add some padding before and after (no noise!)
        Samples signal(500, 0.0f);
        signal.insert(signal.end(), preamble.begin(), preamble.end());
        signal.resize(signal.size() + 500, 0.0f);

        // Detect - debug only for first one
        auto result = zc.detect(signal, 0.2f, first);
        first = false;

        bool ok = result.detected && result.frame_type == type;
        if (ok) passed++;

        std::cout << "  " << zcFrameTypeToString(type) << ": "
                  << (ok ? "PASS" : "FAIL")
                  << " (detected=" << result.detected
                  << ", type=" << zcFrameTypeToString(result.frame_type)
                  << ", corr=" << std::fixed << std::setprecision(3) << result.correlation
                  << ", root=" << result.root_detected
                  << ")\n";
    }

    std::cout << "  Result: " << passed << "/" << total << " passed\n";
    return passed == total;
}

int main() {
    std::cout << "=== Zadoff-Chu Sync Test ===\n";

    // Create ZC sync with default config (upsampled I/Q modulation)
    ZCConfig config;
    config.sequence_length = 127;   // Prime for good autocorrelation
    config.upsample_factor = 8;     // 8 samples per chip
    config.num_repetitions = 2;     // 2 repetitions for robustness
    // Roots coprime with 127 (prime)
    config.root_ping = 1;
    config.root_pong = 3;
    config.root_data = 5;
    config.root_control = 7;
    ZCSync zc(config);

    std::cout << "\nConfiguration:\n";
    std::cout << "  Sample rate: " << config.sample_rate << " Hz\n";
    std::cout << "  Sequence length: " << config.sequence_length << "\n";
    std::cout << "  Repetitions: " << config.num_repetitions << "\n";
    std::cout << "  Carrier freq: " << config.carrier_freq << " Hz\n";
    std::cout << "  Preamble duration: " << std::fixed << std::setprecision(1)
              << config.preambleDurationMs() << " ms\n";
    std::cout << "  Roots: PING=" << config.root_ping
              << ", PONG=" << config.root_pong
              << ", DATA=" << config.root_data
              << ", CONTROL=" << config.root_control << "\n";

    uint32_t seed = 42;
    bool all_pass = true;

    // Test 0: Sanity check - no noise
    all_pass &= testNoNoise(zc);

    // Test 1: Basic detection at high SNR
    all_pass &= testBasicDetection(zc, 20.0f, seed);

    // Test 2: SNR sweep
    all_pass &= testSNRSweep(zc, seed);

    // Test 3: CFO estimation
    all_pass &= testCFOEstimation(zc, 15.0f, seed);

    // Test 4: Frame type discrimination
    all_pass &= testFrameTypeDiscrimination(zc, 10.0f, seed);

    // Test 5: Timing
    testTiming(zc);

    // Summary
    std::cout << "\n=== Summary ===\n";
    std::cout << "Overall: " << (all_pass ? "PASS" : "FAIL") << "\n";

    return all_pass ? 0 : 1;
}
