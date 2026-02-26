// Test ZC Sync with 10-carrier DBPSK
//
// Tests the combination of:
// - ZC preamble for sync (instead of chirp)
// - 10-carrier DBPSK data modulation
// - LDPC FEC (rate 1/2)
//
// Goal: Find the SNR floor when using ZC+DBPSK

#include "sync/zc_sync.hpp"
#include "psk/multi_carrier_dpsk.hpp"
#include "fec/ldpc_codec.hpp"
#include <iostream>
#include <iomanip>
#include <random>
#include <cmath>

using namespace ultra;
using namespace ultra::sync;

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

// Test ZC sync + DBPSK at a given SNR
struct TestResult {
    bool sync_detected;
    bool data_decoded;
    float sync_corr;
    float cfo_est;
    int bit_errors;
    int total_bits;
    float ber;
};

TestResult testAtSNR(float snr_db, uint32_t seed, bool verbose = false) {
    TestResult result = {false, false, 0.0f, 0.0f, 0, 0, 1.0f};
    std::mt19937 rng(seed);

    // === Setup ZC Sync ===
    ZCConfig zc_cfg;
    zc_cfg.sequence_length = 127;
    zc_cfg.upsample_factor = 8;
    zc_cfg.num_repetitions = 2;
    zc_cfg.root_ping = 1;
    zc_cfg.root_pong = 3;
    zc_cfg.root_data = 5;
    zc_cfg.root_control = 7;
    ZCSync zc_sync(zc_cfg);

    // === Setup DBPSK Modulator ===
    // Two modes available:
    //   level3_dbpsk_narrow(): 5 carriers, ~234 bps, floor -11 dB
    //   level4_dbpsk():        10 carriers, ~469 bps, floor -7.5 dB
    // Default to 10 carriers; set use_narrow_mode=true for ultra-low SNR
    bool use_narrow_mode = false;
    MultiCarrierDPSKConfig dpsk_cfg = use_narrow_mode
        ? mc_dpsk_presets::level3_dbpsk_narrow()
        : mc_dpsk_presets::level4_dbpsk();
    dpsk_cfg.use_dual_chirp = false;  // We're using ZC instead
    MultiCarrierDPSKModulator dpsk_mod(dpsk_cfg);
    MultiCarrierDPSKDemodulator dpsk_demod(dpsk_cfg);

    // === Setup LDPC ===
    fec::LDPCCodec ldpc(CodeRate::R1_2);  // 324 info bits -> 648 codeword bits

    // === Generate test data ===
    // LDPC R1/2 has 324 info bits = 40.5 bytes, use 40 bytes
    const int data_bytes = 40;
    Bytes tx_data(data_bytes);
    for (int i = 0; i < data_bytes; i++) {
        tx_data[i] = rng() & 0xFF;
    }

    // === TX: Build frame ===
    // 1. ZC preamble (for sync)
    Samples zc_preamble = zc_sync.generatePreamble(ZCFrameType::DATA);

    // 2. DPSK training + reference (for phase alignment)
    Samples dpsk_training = dpsk_mod.generateTrainingSequence();
    Samples dpsk_ref = dpsk_mod.generateReferenceSymbol();

    // 3. LDPC encode data (encode() takes Bytes and pads internally)
    Bytes encoded = ldpc.encode(tx_data);

    // 4. DPSK modulate encoded data
    dpsk_mod.reset();
    Samples dpsk_data = dpsk_mod.modulate(encoded);

    // 5. Combine: ZC preamble + DPSK (training + ref + data)
    Samples tx_signal;
    tx_signal.reserve(zc_preamble.size() + dpsk_training.size() + dpsk_ref.size() + dpsk_data.size() + 1000);

    // Add some silence before
    for (int i = 0; i < 500; i++) tx_signal.push_back(0.0f);
    tx_signal.insert(tx_signal.end(), zc_preamble.begin(), zc_preamble.end());
    tx_signal.insert(tx_signal.end(), dpsk_training.begin(), dpsk_training.end());
    tx_signal.insert(tx_signal.end(), dpsk_ref.begin(), dpsk_ref.end());
    tx_signal.insert(tx_signal.end(), dpsk_data.begin(), dpsk_data.end());
    // Add some silence after
    for (int i = 0; i < 500; i++) tx_signal.push_back(0.0f);

    // === Add noise ===
    addNoise(tx_signal, snr_db, rng);

    // === RX: Detect and decode ===

    // 1. ZC sync detection
    SampleSpan rx_span(tx_signal.data(), tx_signal.size());
    auto zc_result = zc_sync.detect(rx_span, 0.2f, false);

    result.sync_detected = zc_result.detected;
    result.sync_corr = zc_result.correlation;
    result.cfo_est = zc_result.cfo_hz;

    if (!zc_result.detected) {
        if (verbose) {
            std::cerr << "  ZC sync failed (corr=" << zc_result.correlation << ")\n";
        }
        return result;
    }

    // 2. Extract DPSK portion using ZC-detected position
    // Use zc_result.start_sample (where data starts after preamble)
    int dpsk_start = zc_result.start_sample;

    // Validate position
    if (dpsk_start < 0 || dpsk_start >= static_cast<int>(tx_signal.size()) - 1000) {
        if (verbose) {
            std::cerr << "  Invalid ZC start_sample: " << dpsk_start << "\n";
        }
        return result;
    }

    // 3. Extract DPSK samples (training + ref + data)
    Samples dpsk_samples(tx_signal.begin() + dpsk_start, tx_signal.end() - 500);

    // 4. DPSK demodulation using proper interface
    dpsk_demod.reset();

    // Tell demodulator the expected encoded data size (LDPC output bytes)
    dpsk_demod.setExpectedDataBytes(encoded.size());

    // Use ZC-detected CFO (now properly computed from rep-to-rep phase)
    dpsk_demod.setChirpDetected(zc_result.cfo_hz);

    // Feed all samples (training + ref + data) to the demodulator
    SampleSpan all_span(dpsk_samples.data(), dpsk_samples.size());

    // Process through demodulator state machine
    bool frame_ready = dpsk_demod.process(all_span);

    if (!frame_ready) {
        if (verbose) {
            std::cerr << "  DPSK frame not ready after process()\n";
        }
        return result;
    }

    // Get soft bits
    auto soft_bits = dpsk_demod.getSoftBits();

    // 5. LDPC decode
    if (verbose) {
        std::cerr << "  Got " << soft_bits.size() << " soft bits\n";
        // Check soft bit statistics
        float pos_count = 0, neg_count = 0, sum_mag = 0;
        for (float sb : soft_bits) {
            if (sb > 0) pos_count++;
            else neg_count++;
            sum_mag += std::abs(sb);
        }
        std::cerr << "  Soft bits: " << pos_count << " positive, " << neg_count << " negative\n";
        std::cerr << "  Average magnitude: " << (sum_mag / soft_bits.size()) << "\n";
    }

    if (soft_bits.size() < 648) {
        if (verbose) {
            std::cerr << "  Not enough soft bits for LDPC (" << soft_bits.size() << ")\n";
        }
        return result;
    }

    // Truncate to one codeword
    soft_bits.resize(648);
    auto [success, decoded] = ldpc.decode(soft_bits);

    if (verbose) {
        std::cerr << "  LDPC decode: " << (success ? "SUCCESS" : "FAILED") << "\n";
    }

    if (!success || decoded.empty()) {
        if (verbose) {
            std::cerr << "  LDPC decode failed\n";
        }
        return result;
    }

    // 6. Compare decoded data with original
    result.data_decoded = true;
    result.total_bits = data_bytes * 8;
    result.bit_errors = 0;

    for (int i = 0; i < data_bytes && i < static_cast<int>(decoded.size()); i++) {
        for (int b = 0; b < 8; b++) {
            int tx_bit = (tx_data[i] >> (7 - b)) & 1;
            int rx_bit = (decoded[i] >> (7 - b)) & 1;
            if (tx_bit != rx_bit) {
                result.bit_errors++;
            }
        }
    }

    result.ber = static_cast<float>(result.bit_errors) / result.total_bits;

    return result;
}

// Simple loopback test for DBPSK modulator/demodulator
void testDBPSKLoopback() {
    std::cout << "=== DBPSK Loopback Test (no ZC, no noise) ===\n";

    MultiCarrierDPSKConfig cfg = mc_dpsk_presets::level4_dbpsk();
    cfg.use_dual_chirp = false;
    MultiCarrierDPSKModulator mod(cfg);
    MultiCarrierDPSKDemodulator demod(cfg);

    // Test data: 10 bytes = 80 bits
    Bytes test_data = {0xAA, 0x55, 0xFF, 0x00, 0xA5, 0x5A, 0x12, 0x34, 0x56, 0x78};

    // Generate reference and data
    mod.reset();
    Samples ref = mod.generateReferenceSymbol();
    Samples data = mod.modulate(test_data);

    // Concatenate ref + data for demodulation
    Samples rx_signal;
    rx_signal.insert(rx_signal.end(), ref.begin(), ref.end());
    rx_signal.insert(rx_signal.end(), data.begin(), data.end());

    // Demodulate directly
    demod.reset();
    SampleSpan ref_span(rx_signal.data(), cfg.samples_per_symbol);
    demod.setReference(ref_span);

    SampleSpan data_span(rx_signal.data() + cfg.samples_per_symbol, data.size());
    auto soft_bits = demod.demodulateSoft(data_span);

    std::cout << "  TX bytes: " << test_data.size() << " (" << test_data.size() * 8 << " bits)\n";
    std::cout << "  RX soft bits: " << soft_bits.size() << "\n";

    // Count positive/negative
    int pos = 0, neg = 0;
    for (float sb : soft_bits) {
        if (sb > 0) pos++;
        else neg++;
    }
    std::cout << "  Positive: " << pos << ", Negative: " << neg << "\n";

    // Convert soft bits to hard bits and compare
    int bit_errors = 0;
    int total_bits = std::min((int)soft_bits.size(), (int)test_data.size() * 8);

    for (int i = 0; i < total_bits; i++) {
        int rx_bit = (soft_bits[i] < 0) ? 1 : 0;
        int tx_bit = (test_data[i / 8] >> (7 - (i % 8))) & 1;
        if (rx_bit != tx_bit) bit_errors++;
    }

    std::cout << "  Bit errors: " << bit_errors << "/" << total_bits << "\n";
    std::cout << "  BER: " << (100.0f * bit_errors / total_bits) << "%\n";

    if (bit_errors == 0) {
        std::cout << "  LOOPBACK TEST: PASS\n\n";
    } else {
        std::cout << "  LOOPBACK TEST: FAIL\n\n";
    }
}

// Test with ZC preamble but NO noise
void testWithZCNoNoise() {
    std::cout << "=== ZC + DBPSK + LDPC Test (NO NOISE) ===\n";

    // Setup ZC
    ZCConfig zc_cfg;
    zc_cfg.sequence_length = 127;
    zc_cfg.upsample_factor = 8;
    zc_cfg.num_repetitions = 2;
    zc_cfg.root_data = 5;
    ZCSync zc_sync(zc_cfg);

    // Setup DPSK
    MultiCarrierDPSKConfig dpsk_cfg = mc_dpsk_presets::level4_dbpsk();
    dpsk_cfg.use_dual_chirp = false;
    MultiCarrierDPSKModulator dpsk_mod(dpsk_cfg);
    MultiCarrierDPSKDemodulator dpsk_demod(dpsk_cfg);

    // Setup LDPC
    fec::LDPCCodec ldpc(CodeRate::R1_2);

    // Test data
    Bytes test_data(40);
    for (int i = 0; i < 40; i++) test_data[i] = i * 7;

    // === TX ===
    Samples zc_preamble = zc_sync.generatePreamble(ZCFrameType::DATA);
    Samples dpsk_training = dpsk_mod.generateTrainingSequence();
    Samples dpsk_ref = dpsk_mod.generateReferenceSymbol();
    Bytes encoded = ldpc.encode(test_data);
    dpsk_mod.reset();
    Samples dpsk_data = dpsk_mod.modulate(encoded);

    // Combine: silence + ZC + DPSK(training + ref + data) + silence
    Samples tx_signal;
    for (int i = 0; i < 500; i++) tx_signal.push_back(0.0f);
    tx_signal.insert(tx_signal.end(), zc_preamble.begin(), zc_preamble.end());
    tx_signal.insert(tx_signal.end(), dpsk_training.begin(), dpsk_training.end());
    tx_signal.insert(tx_signal.end(), dpsk_ref.begin(), dpsk_ref.end());
    tx_signal.insert(tx_signal.end(), dpsk_data.begin(), dpsk_data.end());
    for (int i = 0; i < 500; i++) tx_signal.push_back(0.0f);

    std::cout << "  ZC preamble: " << zc_preamble.size() << " samples\n";
    std::cout << "  DPSK training: " << dpsk_training.size() << " samples\n";
    std::cout << "  DPSK ref: " << dpsk_ref.size() << " samples\n";
    std::cout << "  DPSK data: " << dpsk_data.size() << " samples\n";
    std::cout << "  Total TX: " << tx_signal.size() << " samples\n";

    // === RX: ZC detection ===
    SampleSpan rx_span(tx_signal.data(), tx_signal.size());
    auto zc_result = zc_sync.detect(rx_span, 0.2f, false);
    std::cout << "  ZC detected: " << zc_result.detected << " (corr=" << zc_result.correlation
              << ", cfo=" << zc_result.cfo_hz << " Hz)\n";

    if (!zc_result.detected) {
        std::cout << "  ZC TEST: FAIL (no sync)\n\n";
        return;
    }

    // === Extract DPSK portion using ZC-detected position ===
    int calculated_start = 500 + zc_cfg.preambleSamples();  // For comparison only
    int dpsk_start = zc_result.start_sample;  // Use ZC-detected position

    std::cout << "  ZC detected start_sample: " << zc_result.start_sample << "\n";
    std::cout << "  Calculated DPSK start: " << calculated_start << "\n";
    std::cout << "  Difference: " << (zc_result.start_sample - calculated_start)
              << " samples (1 ZC rep = " << zc_cfg.singleRepSamples() << ")\n";

    // Validate ZC-detected position
    if (dpsk_start < 0 || dpsk_start >= static_cast<int>(tx_signal.size()) - 1000) {
        std::cout << "  ZC TEST: FAIL (invalid start_sample)\n\n";
        return;
    }

    Samples dpsk_samples(tx_signal.begin() + dpsk_start, tx_signal.end() - 500);
    std::cout << "  DPSK samples (ZC-detected position): " << dpsk_samples.size() << "\n";

    // DEBUG: Compare extracted samples with expected
    MultiCarrierDPSKModulator mod_standalone(dpsk_cfg);
    Samples training_standalone = mod_standalone.generateTrainingSequence();
    bool samples_match = true;
    for (size_t i = 0; i < 100 && i < dpsk_samples.size() && i < training_standalone.size(); i++) {
        float diff = std::abs(dpsk_samples[i] - training_standalone[i]);
        if (diff > 0.001f) {
            samples_match = false;
            if (i < 5) {  // Show first few mismatches
                std::cout << "  Sample mismatch at " << i << ": got=" << dpsk_samples[i]
                          << " expected=" << training_standalone[i] << " diff=" << diff << "\n";
            }
        }
    }
    std::cout << "  Samples match: " << (samples_match ? "YES" : "NO") << "\n";

    // === DPSK demodulation using ZC-detected CFO ===
    dpsk_demod.reset();
    dpsk_demod.setExpectedDataBytes(encoded.size());

    // Use ZC-detected CFO estimate (should be ~0 for no-noise test)
    std::cout << "  Using ZC CFO estimate: " << zc_result.cfo_hz << " Hz\n";
    dpsk_demod.setChirpDetected(zc_result.cfo_hz);

    SampleSpan all_span(dpsk_samples.data(), dpsk_samples.size());
    bool ready = dpsk_demod.process(all_span);

    std::cout << "  Frame ready: " << (ready ? "YES" : "NO") << "\n";

    if (!ready) {
        std::cout << "  ZC TEST: FAIL (frame not ready)\n\n";
        return;
    }

    auto soft_bits = dpsk_demod.getSoftBits();
    std::cout << "  RX soft bits: " << soft_bits.size() << "\n";

    int pos = 0, neg = 0;
    for (float sb : soft_bits) {
        if (sb > 0) pos++;
        else neg++;
    }
    std::cout << "  Positive: " << pos << ", Negative: " << neg << "\n";

    // LDPC decode
    if (soft_bits.size() >= 648) {
        soft_bits.resize(648);
        auto [success, decoded] = ldpc.decode(soft_bits);
        std::cout << "  LDPC decode: " << (success ? "SUCCESS" : "FAILED") << "\n";

        if (success && decoded.size() >= test_data.size()) {
            int byte_errors = 0;
            for (size_t i = 0; i < test_data.size(); i++) {
                if (decoded[i] != test_data[i]) byte_errors++;
            }
            std::cout << "  Byte errors: " << byte_errors << "/" << test_data.size() << "\n";

            if (byte_errors == 0) {
                std::cout << "  ZC TEST: PASS\n\n";
                return;
            }
        }
    }
    std::cout << "  ZC TEST: FAIL\n\n";
}

// Test with LDPC encoding (like ZC test)
void testDBPSKWithLDPC() {
    std::cout << "=== DBPSK Test with LDPC encoding ===\n";

    MultiCarrierDPSKConfig cfg = mc_dpsk_presets::level4_dbpsk();
    cfg.use_dual_chirp = false;
    MultiCarrierDPSKModulator mod(cfg);
    MultiCarrierDPSKDemodulator demod(cfg);

    // Setup LDPC
    fec::LDPCCodec ldpc(CodeRate::R1_2);

    // Test data: 40 bytes (same as ZC test)
    Bytes test_data(40);
    for (int i = 0; i < 40; i++) test_data[i] = i * 7;

    // LDPC encode
    Bytes encoded = ldpc.encode(test_data);
    std::cout << "  Raw data: " << test_data.size() << " bytes\n";
    std::cout << "  Encoded: " << encoded.size() << " bytes (" << encoded.size() * 8 << " bits)\n";

    // Generate training, reference and data
    Samples training = mod.generateTrainingSequence();
    Samples ref = mod.generateReferenceSymbol();
    mod.reset();
    Samples data = mod.modulate(encoded);

    std::cout << "  Data samples: " << data.size() << "\n";

    // Concatenate training + ref + data
    Samples rx_signal;
    rx_signal.insert(rx_signal.end(), training.begin(), training.end());
    rx_signal.insert(rx_signal.end(), ref.begin(), ref.end());
    rx_signal.insert(rx_signal.end(), data.begin(), data.end());

    // Use process() state machine
    demod.reset();
    demod.setExpectedDataBytes(encoded.size());
    demod.setChirpDetected(0.0f);

    SampleSpan all_span(rx_signal.data(), rx_signal.size());
    bool ready = demod.process(all_span);

    std::cout << "  Frame ready: " << (ready ? "YES" : "NO") << "\n";

    if (!ready) {
        std::cout << "  LDPC TEST: FAIL (frame not ready)\n\n";
        return;
    }

    auto soft_bits = demod.getSoftBits();
    std::cout << "  RX soft bits: " << soft_bits.size() << "\n";

    // Count positive/negative
    int pos = 0, neg = 0;
    for (float sb : soft_bits) {
        if (sb > 0) pos++;
        else neg++;
    }
    std::cout << "  Positive: " << pos << ", Negative: " << neg << "\n";

    // Truncate to LDPC codeword size and decode
    if (soft_bits.size() >= 648) {
        soft_bits.resize(648);
        auto [success, decoded] = ldpc.decode(soft_bits);
        std::cout << "  LDPC decode: " << (success ? "SUCCESS" : "FAILED") << "\n";

        if (success && decoded.size() >= test_data.size()) {
            int byte_errors = 0;
            for (size_t i = 0; i < test_data.size(); i++) {
                if (decoded[i] != test_data[i]) byte_errors++;
            }
            std::cout << "  Byte errors: " << byte_errors << "/" << test_data.size() << "\n";

            if (byte_errors == 0) {
                std::cout << "  LDPC TEST: PASS\n\n";
                return;
            }
        }
    }
    std::cout << "  LDPC TEST: FAIL\n\n";
}

// Test using process() state machine (like ZC test)
void testDBPSKWithProcess() {
    std::cout << "=== DBPSK Test with process() state machine ===\n";

    MultiCarrierDPSKConfig cfg = mc_dpsk_presets::level4_dbpsk();
    cfg.use_dual_chirp = false;
    MultiCarrierDPSKModulator mod(cfg);
    MultiCarrierDPSKDemodulator demod(cfg);

    // Test data: 10 bytes = 80 bits
    Bytes test_data = {0xAA, 0x55, 0xFF, 0x00, 0xA5, 0x5A, 0x12, 0x34, 0x56, 0x78};

    // Generate training, reference and data
    Samples training = mod.generateTrainingSequence();
    Samples ref = mod.generateReferenceSymbol();
    mod.reset();  // Reset before modulating data (as in main test)
    Samples data = mod.modulate(test_data);

    // Concatenate training + ref + data
    Samples rx_signal;
    rx_signal.insert(rx_signal.end(), training.begin(), training.end());
    rx_signal.insert(rx_signal.end(), ref.begin(), ref.end());
    rx_signal.insert(rx_signal.end(), data.begin(), data.end());

    // Use process() state machine (like in ZC test)
    demod.reset();
    demod.setExpectedDataBytes(test_data.size());
    demod.setChirpDetected(0.0f);  // No CFO

    SampleSpan all_span(rx_signal.data(), rx_signal.size());
    bool ready = demod.process(all_span);

    std::cout << "  Frame ready: " << (ready ? "YES" : "NO") << "\n";

    if (!ready) {
        std::cout << "  PROCESS TEST: FAIL (frame not ready)\n\n";
        return;
    }

    auto soft_bits = demod.getSoftBits();
    std::cout << "  RX soft bits: " << soft_bits.size() << "\n";

    // Count positive/negative
    int pos = 0, neg = 0;
    for (float sb : soft_bits) {
        if (sb > 0) pos++;
        else neg++;
    }
    std::cout << "  Positive: " << pos << ", Negative: " << neg << "\n";

    // Convert soft bits to hard bits and compare
    int bit_errors = 0;
    int total_bits = std::min((int)soft_bits.size(), (int)test_data.size() * 8);

    for (int i = 0; i < total_bits; i++) {
        int rx_bit = (soft_bits[i] < 0) ? 1 : 0;
        int tx_bit = (test_data[i / 8] >> (7 - (i % 8))) & 1;
        if (rx_bit != tx_bit) bit_errors++;
    }

    std::cout << "  Bit errors: " << bit_errors << "/" << total_bits << "\n";
    std::cout << "  BER: " << (100.0f * bit_errors / total_bits) << "%\n";

    if (bit_errors == 0) {
        std::cout << "  PROCESS TEST: PASS\n\n";
    } else {
        std::cout << "  PROCESS TEST: FAIL\n\n";
    }
}

// Test with training sequence included (like real frame)
void testDBPSKWithTraining() {
    std::cout << "=== DBPSK Test with Training (no ZC, no noise) ===\n";

    MultiCarrierDPSKConfig cfg = mc_dpsk_presets::level4_dbpsk();
    cfg.use_dual_chirp = false;
    MultiCarrierDPSKModulator mod(cfg);
    MultiCarrierDPSKDemodulator demod(cfg);

    // Test data: 10 bytes = 80 bits
    Bytes test_data = {0xAA, 0x55, 0xFF, 0x00, 0xA5, 0x5A, 0x12, 0x34, 0x56, 0x78};

    // Generate training, reference and data
    Samples training = mod.generateTrainingSequence();
    Samples ref = mod.generateReferenceSymbol();
    mod.reset();  // Reset before modulating data (as in main test)
    Samples data = mod.modulate(test_data);

    std::cout << "  Training: " << training.size() << " samples (" << cfg.training_symbols << " symbols)\n";
    std::cout << "  Reference: " << ref.size() << " samples\n";
    std::cout << "  Data: " << data.size() << " samples\n";

    // Concatenate training + ref + data
    Samples rx_signal;
    rx_signal.insert(rx_signal.end(), training.begin(), training.end());
    rx_signal.insert(rx_signal.end(), ref.begin(), ref.end());
    rx_signal.insert(rx_signal.end(), data.begin(), data.end());

    // Demodulate using direct path (bypassing state machine)
    demod.reset();

    // Calculate offsets
    size_t training_len = cfg.training_symbols * cfg.samples_per_symbol;
    size_t ref_len = cfg.samples_per_symbol;

    // Set reference from the reference symbol
    SampleSpan ref_span(rx_signal.data() + training_len, ref_len);
    demod.setReference(ref_span);

    // Demodulate data
    SampleSpan data_span(rx_signal.data() + training_len + ref_len, data.size());
    auto soft_bits = demod.demodulateSoft(data_span);

    std::cout << "  RX soft bits: " << soft_bits.size() << "\n";

    // Count positive/negative
    int pos = 0, neg = 0;
    for (float sb : soft_bits) {
        if (sb > 0) pos++;
        else neg++;
    }
    std::cout << "  Positive: " << pos << ", Negative: " << neg << "\n";

    // Convert soft bits to hard bits and compare
    int bit_errors = 0;
    int total_bits = std::min((int)soft_bits.size(), (int)test_data.size() * 8);

    for (int i = 0; i < total_bits; i++) {
        int rx_bit = (soft_bits[i] < 0) ? 1 : 0;
        int tx_bit = (test_data[i / 8] >> (7 - (i % 8))) & 1;
        if (rx_bit != tx_bit) bit_errors++;
    }

    std::cout << "  Bit errors: " << bit_errors << "/" << total_bits << "\n";
    std::cout << "  BER: " << (100.0f * bit_errors / total_bits) << "%\n";

    if (bit_errors == 0) {
        std::cout << "  TRAINING TEST: PASS\n\n";
    } else {
        std::cout << "  TRAINING TEST: FAIL\n\n";
    }
}

int main() {
    // First run loopback test to verify DBPSK works in isolation
    testDBPSKLoopback();

    // Test with training sequence included
    testDBPSKWithTraining();

    // Test using process() state machine
    testDBPSKWithProcess();

    // Test with LDPC encoding
    testDBPSKWithLDPC();

    // Test with ZC preamble but no noise
    testWithZCNoNoise();

    std::cout << "=== ZC Sync + 10-Carrier DBPSK Test ===\n\n";

    // DEBUG: First test at very high SNR to isolate issues
    std::cout << "DEBUG: Testing at 30 dB SNR (should definitely work):\n";
    auto debug_result = testAtSNR(30.0f, 42, true);
    std::cout << "  Result: sync=" << debug_result.sync_detected
              << ", decode=" << debug_result.data_decoded
              << ", bit_errors=" << debug_result.bit_errors << "/" << debug_result.total_bits
              << "\n\n";

    // Configuration summary
    ZCConfig zc_cfg;
    zc_cfg.sequence_length = 127;
    zc_cfg.upsample_factor = 8;
    zc_cfg.num_repetitions = 2;

    MultiCarrierDPSKConfig dpsk_cfg = mc_dpsk_presets::level4_dbpsk();

    std::cout << "ZC Preamble:\n";
    std::cout << "  Sequence length: " << zc_cfg.sequence_length << "\n";
    std::cout << "  Upsample factor: " << zc_cfg.upsample_factor << "\n";
    std::cout << "  Repetitions: " << zc_cfg.num_repetitions << "\n";
    std::cout << "  Duration: " << std::fixed << std::setprecision(1)
              << zc_cfg.preambleSamples() / 48.0f << " ms\n";

    std::cout << "\nDBPSK Data:\n";
    std::cout << "  Carriers: " << dpsk_cfg.num_carriers << "\n";
    std::cout << "  Bits/symbol: " << dpsk_cfg.bits_per_symbol << " (DBPSK)\n";
    std::cout << "  Symbol rate: " << std::setprecision(1)
              << dpsk_cfg.getSymbolRate() << " baud\n";
    std::cout << "  Raw bit rate: " << dpsk_cfg.getRawBitRate() << " bps\n";

    std::cout << "\nFEC: LDPC Rate 1/2\n";
    std::cout << "  Effective bit rate: ~" << dpsk_cfg.getRawBitRate() / 2.0f << " bps\n";

    // Compare with chirp preamble duration
    float chirp_duration_ms = 500.0f * 2 + 100.0f * 2;  // up + down + gaps
    float zc_duration_ms = zc_cfg.preambleSamples() / 48.0f;
    std::cout << "\nPreamble comparison:\n";
    std::cout << "  ZC: " << zc_duration_ms << " ms\n";
    std::cout << "  Chirp: ~" << chirp_duration_ms << " ms\n";
    std::cout << "  Speedup: " << std::setprecision(1)
              << chirp_duration_ms / zc_duration_ms << "x\n";

    std::cout << "\n" << std::string(60, '=') << "\n";
    std::cout << "SNR Sweep Test\n";
    std::cout << std::string(60, '=') << "\n\n";

    // Test across SNR range
    const int num_trials = 10;

    std::cout << std::setw(8) << "SNR(dB)"
              << std::setw(10) << "Sync%"
              << std::setw(10) << "Decode%"
              << std::setw(10) << "BER"
              << std::setw(12) << "Avg Corr"
              << "\n";
    std::cout << std::string(50, '-') << "\n";

    for (float snr = -15.0f; snr <= 10.0f; snr += 2.5f) {
        int sync_success = 0;
        int decode_success = 0;
        float total_ber = 0.0f;
        float total_corr = 0.0f;

        for (int trial = 0; trial < num_trials; trial++) {
            uint32_t seed = static_cast<uint32_t>(snr * 1000 + trial);
            auto result = testAtSNR(snr, seed);

            if (result.sync_detected) sync_success++;
            if (result.data_decoded && result.bit_errors == 0) decode_success++;
            total_ber += result.ber;
            total_corr += result.sync_corr;
        }

        float sync_pct = 100.0f * sync_success / num_trials;
        float decode_pct = 100.0f * decode_success / num_trials;
        float avg_ber = total_ber / num_trials;
        float avg_corr = total_corr / num_trials;

        std::cout << std::fixed << std::setprecision(1)
                  << std::setw(8) << snr
                  << std::setw(10) << sync_pct
                  << std::setw(10) << decode_pct
                  << std::setprecision(4) << std::setw(10) << avg_ber
                  << std::setprecision(3) << std::setw(12) << avg_corr
                  << "\n";
    }

    std::cout << "\n" << std::string(60, '=') << "\n";
    std::cout << "Finding SNR Floor (100% decode success)\n";
    std::cout << std::string(60, '=') << "\n\n";

    // Fine-grained search to find the floor
    float floor_snr = 10.0f;
    for (float snr = -5.0f; snr >= -15.0f; snr -= 1.0f) {
        int decode_success = 0;
        for (int trial = 0; trial < 20; trial++) {
            uint32_t seed = static_cast<uint32_t>(snr * 1000 + trial + 1000);
            auto result = testAtSNR(snr, seed);
            if (result.data_decoded && result.bit_errors == 0) decode_success++;
        }

        float decode_pct = 100.0f * decode_success / 20;
        std::cout << "SNR=" << std::fixed << std::setprecision(1) << snr
                  << " dB: " << decode_pct << "% success\n";

        if (decode_pct < 100.0f) {
            floor_snr = snr + 1.0f;
            break;
        }
        floor_snr = snr;
    }

    std::cout << "\n" << std::string(60, '=') << "\n";
    std::cout << "RESULT: SNR Floor = " << floor_snr << " dB\n";
    std::cout << "(100% decode success at " << floor_snr << " dB and above)\n";
    std::cout << std::string(60, '=') << "\n";

    return 0;
}
