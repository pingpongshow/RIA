// Test time-domain spreading for MC-DPSK
//
// Tests 2× and 4× time repetition spreading modes for improved SNR performance
// Expected gains: +3 dB for 2×, +6 dB for 4×

#include "psk/multi_carrier_dpsk.hpp"
#include "fec/ldpc_codec.hpp"
#include <iostream>
#include <iomanip>
#include <random>
#include <cmath>

using namespace ultra;

// Add AWGN noise to signal
void addNoise(Samples& signal, float snr_db, std::mt19937& rng) {
    float sig_power = 0.0f;
    for (float s : signal) {
        sig_power += s * s;
    }
    sig_power /= signal.size();

    float snr_linear = std::pow(10.0f, snr_db / 10.0f);
    float noise_power = sig_power / snr_linear;
    float noise_std = std::sqrt(noise_power);

    std::normal_distribution<float> noise(0.0f, noise_std);
    for (float& s : signal) {
        s += noise(rng);
    }
}

// Test spreading mode at a given SNR
struct TestResult {
    bool decoded;
    int bit_errors;
    int total_bits;
    float ber;
};

TestResult testAtSNR(float snr_db, SpreadingMode spreading, uint32_t seed, bool verbose = false) {
    TestResult result = {false, 0, 0, 1.0f};
    std::mt19937 rng(seed);

    // === Setup DBPSK with spreading ===
    MultiCarrierDPSKConfig dpsk_cfg = mc_dpsk_presets::level4_dbpsk();
    dpsk_cfg.spreading_mode = spreading;
    dpsk_cfg.use_dual_chirp = false;

    if (verbose) {
        std::cout << "  Config: " << dpsk_cfg.num_carriers << " carriers, "
                  << dpsk_cfg.bits_per_symbol << " bits/sym, "
                  << "spreading=" << dpsk_cfg.getSpreadingFactor() << "×\n";
        std::cout << "  Raw bit rate: " << dpsk_cfg.getRawBitRate() << " bps\n";
    }

    MultiCarrierDPSKModulator dpsk_mod(dpsk_cfg);
    MultiCarrierDPSKDemodulator dpsk_demod(dpsk_cfg);

    // === Setup LDPC ===
    fec::LDPCCodec ldpc(CodeRate::R1_2);

    // === Generate test data ===
    const int data_bytes = 40;  // 320 bits = 40 bytes
    Bytes tx_data(data_bytes);
    for (int i = 0; i < data_bytes; i++) {
        tx_data[i] = rng() & 0xFF;
    }

    // === TX: Encode and modulate ===
    Bytes encoded = ldpc.encode(tx_data);

    // Generate training + ref + data
    Samples training = dpsk_mod.generateTrainingSequence();
    Samples ref = dpsk_mod.generateReferenceSymbol();
    Samples data = dpsk_mod.modulate(encoded);

    // Build frame
    Samples frame;
    frame.reserve(training.size() + ref.size() + data.size());
    frame.insert(frame.end(), training.begin(), training.end());
    frame.insert(frame.end(), ref.begin(), ref.end());
    frame.insert(frame.end(), data.begin(), data.end());

    if (verbose) {
        std::cout << "  Frame: " << frame.size() << " samples ("
                  << frame.size() / 48.0f << " ms)\n";
        std::cout << "  Training: " << training.size() << ", Ref: " << ref.size()
                  << ", Data: " << data.size() << " samples\n";
    }

    // === Add noise ===
    addNoise(frame, snr_db, rng);

    // === RX: Demodulate ===
    // Process training
    SampleSpan train_span(frame.data(), training.size());
    dpsk_demod.processTraining(train_span);

    // Process reference
    SampleSpan ref_span(frame.data() + training.size(), ref.size());
    dpsk_demod.setReference(ref_span);

    // Demodulate data
    SampleSpan data_span(frame.data() + training.size() + ref.size(), data.size());
    std::vector<float> soft_bits = dpsk_demod.demodulateSoft(data_span);

    if (verbose) {
        std::cout << "  Soft bits: " << soft_bits.size() << " (expected ~648 for LDPC R1/2)\n";
        std::cout << "  First 10 soft bits: ";
        for (int i = 0; i < std::min(10, (int)soft_bits.size()); i++) {
            std::cout << soft_bits[i] << " ";
        }
        std::cout << "\n";
    }

    // === Decode ===
    auto [success, decoded] = ldpc.decode(soft_bits);

    // === Verify ===
    result.total_bits = data_bytes * 8;
    result.bit_errors = 0;

    if (success && decoded.size() >= tx_data.size()) {
        result.decoded = true;
        for (size_t i = 0; i < tx_data.size(); i++) {
            uint8_t xored = tx_data[i] ^ decoded[i];
            while (xored) {
                result.bit_errors += (xored & 1);
                xored >>= 1;
            }
        }
        result.ber = (float)result.bit_errors / result.total_bits;
    }

    return result;
}

int main(int argc, char* argv[]) {
    std::cout << "=== MC-DPSK Time Spreading Test ===\n\n";

    // Quick verification
    std::cout << "Verifying basic operation...\n";
    auto test1 = testAtSNR(0.0f, SpreadingMode::NONE, 1000, false);
    auto test2 = testAtSNR(0.0f, SpreadingMode::TIME_2X, 1000, false);
    auto test4 = testAtSNR(0.0f, SpreadingMode::TIME_4X, 1000, false);
    std::cout << "  NONE at 0dB: " << (test1.decoded && test1.bit_errors == 0 ? "PASS" : "FAIL") << "\n";
    std::cout << "  2× at 0dB: " << (test2.decoded && test2.bit_errors == 0 ? "PASS" : "FAIL") << "\n";
    std::cout << "  4× at 0dB: " << (test4.decoded && test4.bit_errors == 0 ? "PASS" : "FAIL") << "\n\n";

    // Test parameters - extend to find actual floors
    std::vector<float> snr_levels = {-16, -14, -12, -10, -8, -6, -4, -2, 0};
    const int num_trials = 20;

    std::cout << "Testing 10-carrier DBPSK with different spreading modes:\n";
    std::cout << "  - NONE: ~469 bps, floor ~-4 dB\n";
    std::cout << "  - TIME_2X: ~235 bps, floor ~-7 dB (+3 dB gain)\n";
    std::cout << "  - TIME_4X: ~117 bps, floor ~-10 dB (+6 dB gain)\n\n";

    // Test each spreading mode
    struct ModeConfig {
        SpreadingMode mode;
        const char* name;
    };
    std::vector<ModeConfig> modes = {
        {SpreadingMode::NONE, "NONE (1×)"},
        {SpreadingMode::TIME_2X, "TIME_2X"},
        {SpreadingMode::TIME_4X, "TIME_4X"}
    };

    for (const auto& mode_cfg : modes) {
        std::cout << "=== Spreading: " << mode_cfg.name << " ===\n";
        std::cout << std::setw(8) << "SNR(dB)" << " | "
                  << std::setw(8) << "Success" << " | "
                  << std::setw(10) << "Avg BER" << "\n";
        std::cout << std::string(32, '-') << "\n";

        for (float snr : snr_levels) {
            int successes = 0;
            float total_ber = 0.0f;

            for (int trial = 0; trial < num_trials; trial++) {
                auto result = testAtSNR(snr, mode_cfg.mode, 1000 + trial, false);
                if (result.decoded && result.bit_errors == 0) {
                    successes++;
                }
                total_ber += result.ber;
            }

            float avg_ber = total_ber / num_trials;
            std::cout << std::setw(8) << std::fixed << std::setprecision(1) << snr << " | "
                      << std::setw(6) << successes << "/" << num_trials << " | "
                      << std::scientific << std::setprecision(2) << avg_ber << "\n";
        }
        std::cout << "\n";
    }

    std::cout << "Test complete.\n";
    return 0;
}
