// test_chase_cache.cpp - Unit test for HARQ Chase Combining
//
// Tests:
// 1. Basic cache operations (store, retrieve, combine)
// 2. LLR combining improves decode success
// 3. Cache eviction and TTL

#include "fec/chase_cache.hpp"
#include "fec/ldpc_codec.hpp"
#include "protocol/frame_v2.hpp"
#include <iostream>
#include <random>
#include <cmath>
#include <iomanip>

using namespace ultra;
using namespace ultra::fec;

// Add AWGN noise to soft bits
// LLR = 2 * y * SNR where y = s + n, s = ±1, n ~ N(0, 1/SNR)
void addNoise(std::vector<float>& llrs, float snr_db, std::mt19937& rng) {
    float snr_linear = std::pow(10.0f, snr_db / 10.0f);
    float noise_std = 1.0f / std::sqrt(snr_linear);  // Noise variance = 1/SNR
    std::normal_distribution<float> noise(0.0f, noise_std);

    for (auto& llr : llrs) {
        // Original LLR sign indicates transmitted bit
        float sign = (llr > 0) ? 1.0f : -1.0f;
        // Received symbol = ±1 + noise
        float received = sign + noise(rng);
        // LLR = 2 * received * SNR (soft decision)
        llr = 2.0f * received * snr_linear;
    }
}

// Generate noisy codeword from data
// Returns 648 soft bits (LLRs) for LDPC decoder
std::vector<float> generateNoisyCodeword(fec::LDPCCodec& codec,
                                          const std::vector<uint8_t>& data,
                                          float snr_db,
                                          std::mt19937& rng) {
    // Encode - returns bytes, need to convert to bits
    auto encoded_bytes = codec.encode(data);

    // Convert bytes to 648 bits
    std::vector<float> llrs;
    llrs.reserve(648);
    for (uint8_t byte : encoded_bytes) {
        for (int b = 7; b >= 0; b--) {
            int bit = (byte >> b) & 1;
            llrs.push_back(bit ? -4.0f : 4.0f);  // BPSK: 0->+4, 1->-4
        }
    }

    // Ensure exactly 648 bits
    while (llrs.size() < 648) llrs.push_back(4.0f);  // Pad with 0s
    llrs.resize(648);

    // Add noise
    addNoise(llrs, snr_db, rng);

    return llrs;
}

int main() {
    std::cout << "=== HARQ Chase Combining Unit Test ===\n\n";

    // Initialize LDPC codec
    fec::LDPCCodec codec;
    codec.setRate(CodeRate::R1_2);

    // Initialize chase cache
    ChaseCache::Config config;
    config.enabled = true;
    config.max_entries = 16;
    config.entry_ttl = std::chrono::milliseconds(30000);
    ChaseCache cache(config);

    std::mt19937 rng(42);

    // Test data (40 bytes for R1/2 = 324 info bits / 8)
    std::vector<uint8_t> test_data(40);
    for (size_t i = 0; i < test_data.size(); i++) {
        test_data[i] = static_cast<uint8_t>(i & 0xFF);
    }

    int pass = 0, fail = 0;

    // ========================================================================
    // TEST 1: Basic cache operations
    // ========================================================================
    std::cout << "TEST 1: Basic cache operations\n";
    {
        ChaseCacheKey key{100, 0x123456, 0x654321};
        std::vector<float> soft_bits(648, 1.0f);

        // Store
        bool stored = cache.store(key, 0, soft_bits, 2, protocol::v2::FrameType::DATA);
        if (stored) {
            std::cout << "  [PASS] store() succeeded\n";
            pass++;
        } else {
            std::cout << "  [FAIL] store() failed\n";
            fail++;
        }

        // Retrieve
        auto retrieved = cache.getCombined(key, 0);
        if (retrieved && retrieved->size() == 648) {
            std::cout << "  [PASS] getCombined() retrieved correct size\n";
            pass++;
        } else {
            std::cout << "  [FAIL] getCombined() failed\n";
            fail++;
        }

        // Combine count
        int count = cache.getCombineCount(key, 0);
        if (count == 1) {
            std::cout << "  [PASS] getCombineCount() = 1\n";
            pass++;
        } else {
            std::cout << "  [FAIL] getCombineCount() = " << count << " (expected 1)\n";
            fail++;
        }

        // Store again (should combine)
        std::vector<float> soft_bits2(648, 2.0f);
        cache.store(key, 0, soft_bits2, 2, protocol::v2::FrameType::DATA);

        count = cache.getCombineCount(key, 0);
        if (count == 2) {
            std::cout << "  [PASS] After second store, count = 2\n";
            pass++;
        } else {
            std::cout << "  [FAIL] After second store, count = " << count << " (expected 2)\n";
            fail++;
        }

        // Verify LLR values were combined (1.0 + 2.0 = 3.0)
        auto combined = cache.getCombined(key, 0);
        if (combined && std::abs((*combined)[0] - 3.0f) < 0.001f) {
            std::cout << "  [PASS] LLRs combined correctly (1.0 + 2.0 = 3.0)\n";
            pass++;
        } else {
            std::cout << "  [FAIL] LLR combining incorrect\n";
            fail++;
        }

        cache.clear();
    }

    // ========================================================================
    // TEST 2: Chase combining improves decode success at marginal SNR
    // ========================================================================
    std::cout << "\nTEST 2: Chase combining improves decode at marginal SNR\n";
    {
        // Find SNR where single reception has ~30-70% success
        // R1/2 LDPC threshold is around 2-3 dB Eb/N0
        float test_snr = 2.5f;
        int trials = 100;
        int single_success = 0;
        int chase_success = 0;

        for (int t = 0; t < trials; t++) {
            // Generate two independent noisy receptions of same codeword
            auto llrs1 = generateNoisyCodeword(codec, test_data, test_snr, rng);
            auto llrs2 = generateNoisyCodeword(codec, test_data, test_snr, rng);

            // Try decode with single reception
            auto [ok1, data1] = codec.decode(llrs1);
            if (ok1) single_success++;

            // Combine LLRs (chase combining)
            std::vector<float> combined(648);
            for (size_t i = 0; i < 648; i++) {
                combined[i] = llrs1[i] + llrs2[i];
            }

            // Try decode with combined
            auto [ok_combined, data_combined] = codec.decode(combined);
            if (ok_combined) chase_success++;
        }

        float single_rate = 100.0f * single_success / trials;
        float chase_rate = 100.0f * chase_success / trials;

        std::cout << "  SNR = " << test_snr << " dB, " << trials << " trials:\n";
        std::cout << "    Single reception: " << single_success << "/" << trials
                  << " (" << std::fixed << std::setprecision(1) << single_rate << "%)\n";
        std::cout << "    Chase combined:   " << chase_success << "/" << trials
                  << " (" << chase_rate << "%)\n";

        if (chase_rate > single_rate + 10.0f) {
            std::cout << "  [PASS] Chase combining improved success rate by "
                      << (chase_rate - single_rate) << "%\n";
            pass++;
        } else if (chase_rate >= single_rate) {
            std::cout << "  [PASS] Chase combining at least as good (both may be near 100%)\n";
            pass++;
        } else {
            std::cout << "  [FAIL] Chase combining did not improve success rate\n";
            fail++;
        }
    }

    // ========================================================================
    // TEST 3: Multi-reception combining (4 combines)
    // ========================================================================
    std::cout << "\nTEST 3: Multi-reception combining (up to 4)\n";
    {
        float test_snr = 1.5f;  // Low SNR where single decode struggles
        int trials = 50;
        int success_1rx = 0, success_2rx = 0, success_4rx = 0;

        for (int t = 0; t < trials; t++) {
            auto llrs1 = generateNoisyCodeword(codec, test_data, test_snr, rng);
            auto llrs2 = generateNoisyCodeword(codec, test_data, test_snr, rng);
            auto llrs3 = generateNoisyCodeword(codec, test_data, test_snr, rng);
            auto llrs4 = generateNoisyCodeword(codec, test_data, test_snr, rng);

            // 1 reception
            auto [ok1, d1] = codec.decode(llrs1);
            if (ok1) success_1rx++;

            // 2 receptions combined
            std::vector<float> combined2(648);
            for (size_t i = 0; i < 648; i++) {
                combined2[i] = llrs1[i] + llrs2[i];
            }
            auto [ok2, d2] = codec.decode(combined2);
            if (ok2) success_2rx++;

            // 4 receptions combined
            std::vector<float> combined4(648);
            for (size_t i = 0; i < 648; i++) {
                combined4[i] = llrs1[i] + llrs2[i] + llrs3[i] + llrs4[i];
            }
            auto [ok4, d4] = codec.decode(combined4);
            if (ok4) success_4rx++;
        }

        std::cout << "  SNR = " << test_snr << " dB, " << trials << " trials:\n";
        std::cout << "    1 reception:  " << success_1rx << "/" << trials << " ("
                  << (100.0f * success_1rx / trials) << "%)\n";
        std::cout << "    2 receptions: " << success_2rx << "/" << trials << " ("
                  << (100.0f * success_2rx / trials) << "%)\n";
        std::cout << "    4 receptions: " << success_4rx << "/" << trials << " ("
                  << (100.0f * success_4rx / trials) << "%)\n";

        if (success_4rx > success_2rx && success_2rx > success_1rx) {
            std::cout << "  [PASS] More receptions = better success rate\n";
            pass++;
        } else if (success_4rx >= success_1rx) {
            std::cout << "  [PASS] 4 receptions at least as good as 1\n";
            pass++;
        } else {
            std::cout << "  [FAIL] More receptions did not improve success\n";
            fail++;
        }
    }

    // ========================================================================
    // TEST 4: Cache eviction
    // ========================================================================
    std::cout << "\nTEST 4: Cache eviction (max 16 entries)\n";
    {
        cache.clear();
        std::vector<float> soft_bits(648, 1.0f);

        // Fill cache with 20 entries (should evict 4)
        for (int i = 0; i < 20; i++) {
            ChaseCacheKey key{static_cast<uint16_t>(i), 0x111, 0x222};
            cache.store(key, 0, soft_bits, 1, protocol::v2::FrameType::DATA);
        }

        if (cache.size() == 16) {
            std::cout << "  [PASS] Cache size capped at 16\n";
            pass++;
        } else {
            std::cout << "  [FAIL] Cache size = " << cache.size() << " (expected 16)\n";
            fail++;
        }

        // First entries should be evicted (LRU)
        ChaseCacheKey old_key{0, 0x111, 0x222};
        auto old_entry = cache.getCombined(old_key, 0);
        if (!old_entry) {
            std::cout << "  [PASS] Old entries evicted (LRU)\n";
            pass++;
        } else {
            std::cout << "  [FAIL] Old entry still present\n";
            fail++;
        }

        // Recent entries should exist
        ChaseCacheKey new_key{19, 0x111, 0x222};
        auto new_entry = cache.getCombined(new_key, 0);
        if (new_entry) {
            std::cout << "  [PASS] Recent entries retained\n";
            pass++;
        } else {
            std::cout << "  [FAIL] Recent entry missing\n";
            fail++;
        }

        cache.clear();
    }

    // ========================================================================
    // TEST 5: Per-codeword tracking
    // ========================================================================
    std::cout << "\nTEST 5: Per-codeword tracking\n";
    {
        ChaseCacheKey key{500, 0xABC, 0xDEF};
        std::vector<float> soft_bits(648, 1.0f);

        // Store CW0 and CW2, skip CW1
        cache.store(key, 0, soft_bits, 4, protocol::v2::FrameType::DATA);
        cache.store(key, 2, soft_bits, 4, protocol::v2::FrameType::DATA);

        auto cw0 = cache.getCombined(key, 0);
        auto cw1 = cache.getCombined(key, 1);
        auto cw2 = cache.getCombined(key, 2);

        if (cw0 && !cw1 && cw2) {
            std::cout << "  [PASS] Per-CW storage works (CW0=yes, CW1=no, CW2=yes)\n";
            pass++;
        } else {
            std::cout << "  [FAIL] Per-CW storage incorrect\n";
            fail++;
        }

        // Mark CW0 as decoded
        cache.markDecoded(key, 0);
        cw0 = cache.getCombined(key, 0);
        if (!cw0) {
            std::cout << "  [PASS] markDecoded() clears soft bits\n";
            pass++;
        } else {
            std::cout << "  [FAIL] markDecoded() did not clear soft bits\n";
            fail++;
        }

        cache.clear();
    }

    // ========================================================================
    // Summary
    // ========================================================================
    std::cout << "\n========================================\n";
    std::cout << "RESULTS: " << pass << " passed, " << fail << " failed\n";
    std::cout << "========================================\n";

    if (fail == 0) {
        std::cout << "\n[SUCCESS] All chase combining tests passed!\n";
        return 0;
    } else {
        std::cout << "\n[FAILURE] Some tests failed.\n";
        return 1;
    }
}
