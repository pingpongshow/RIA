// chase_cache.hpp - HARQ Chase Combining soft bit cache
//
// Stores soft bits (LLRs) from failed decode attempts and combines them
// with retransmission attempts. LLR addition provides ~3 dB SNR gain per
// doubling of combines (optimal for AWGN channels).
//
// Usage:
//   1. On decode failure: store(key, cw_idx, soft_bits, total_cw, frame_type)
//   2. On retransmission: getCombined(key, cw_idx) returns combined LLRs
//   3. On decode success: markDecoded(key, cw_idx) or removeEntry(key)

#pragma once

#include "protocol/frame_v2.hpp"
#include <vector>
#include <chrono>
#include <unordered_map>
#include <mutex>
#include <optional>
#include <cstdint>

namespace ultra {
namespace fec {

// Cache key for identifying retransmissions
// Combines sequence number with source/destination hashes for uniqueness
struct ChaseCacheKey {
    uint16_t seq_number;   // Frame sequence number from header
    uint32_t src_hash;     // Source callsign hash (24-bit, stored in 32)
    uint32_t dst_hash;     // Destination callsign hash (24-bit, stored in 32)

    bool operator==(const ChaseCacheKey& other) const {
        return seq_number == other.seq_number &&
               src_hash == other.src_hash &&
               dst_hash == other.dst_hash;
    }
};

// Hash function for ChaseCacheKey
struct ChaseCacheKeyHash {
    size_t operator()(const ChaseCacheKey& k) const {
        // Combine all fields into a single hash
        return std::hash<uint64_t>()(
            (static_cast<uint64_t>(k.seq_number) << 48) |
            (static_cast<uint64_t>(k.src_hash & 0xFFFFFF) << 24) |
            (k.dst_hash & 0xFFFFFF)
        );
    }
};

// Per-frame soft bit storage with per-codeword tracking
struct ChaseCacheEntry {
    ChaseCacheKey key;
    std::vector<std::vector<float>> cw_soft_bits;  // [cw_index][648 LLRs]
    std::vector<int> cw_combine_count;              // How many receptions combined per CW
    std::vector<bool> cw_decoded;                   // Which CWs already decoded OK
    std::chrono::steady_clock::time_point created;
    std::chrono::steady_clock::time_point last_access;
    int total_cw;                                   // Expected codeword count
    protocol::v2::FrameType frame_type;             // Frame type from header

    // Maximum combines before giving up on this entry
    static constexpr int MAX_COMBINES = 4;

    // LDPC codeword size
    static constexpr size_t LDPC_BLOCK_SIZE = 648;

    bool isExpired(std::chrono::milliseconds max_age) const {
        auto now = std::chrono::steady_clock::now();
        return (now - last_access) > max_age;
    }

    bool hasRoom(int cw_idx) const {
        return cw_idx >= 0 &&
               cw_idx < static_cast<int>(cw_combine_count.size()) &&
               cw_combine_count[cw_idx] < MAX_COMBINES;
    }

    bool isDecoded(int cw_idx) const {
        return cw_idx >= 0 &&
               cw_idx < static_cast<int>(cw_decoded.size()) &&
               cw_decoded[cw_idx];
    }
};

// Chase combining cache manager
class ChaseCache {
public:
    struct Config {
        bool enabled;
        size_t max_entries;
        std::chrono::milliseconds entry_ttl;
        bool log_combines;

        Config()
            : enabled(true)
            , max_entries(16)
            , entry_ttl(30000)
            , log_combines(true)
        {}
    };

    explicit ChaseCache(const Config& config = Config());
    ~ChaseCache() = default;

    // Non-copyable
    ChaseCache(const ChaseCache&) = delete;
    ChaseCache& operator=(const ChaseCache&) = delete;

    // Enable/disable chase combining
    void setEnabled(bool enable);
    bool isEnabled() const;

    // Store soft bits from failed decode
    // Returns true if stored (new entry or combined with existing)
    bool store(const ChaseCacheKey& key,
               int cw_index,
               const std::vector<float>& soft_bits,
               int total_cw,
               protocol::v2::FrameType frame_type);

    // Retrieve combined soft bits for a codeword
    // Returns nullopt if no cached data for this key/cw
    std::optional<std::vector<float>> getCombined(const ChaseCacheKey& key,
                                                   int cw_index) const;

    // Get combine count for a codeword (0 if not in cache)
    int getCombineCount(const ChaseCacheKey& key, int cw_index) const;

    // Mark a codeword as successfully decoded (no more combining needed)
    void markDecoded(const ChaseCacheKey& key, int cw_index);

    // Remove entry after successful full frame decode
    void removeEntry(const ChaseCacheKey& key);

    // Clear all entries (e.g., on disconnect)
    void clear();

    // Get current cache size
    size_t size() const;

    // Statistics
    struct Stats {
        uint64_t cache_hits = 0;        // getCombined() found entry
        uint64_t cache_misses = 0;      // getCombined() found nothing
        uint64_t stores = 0;            // store() calls
        uint64_t combines = 0;          // LLR combines performed
        uint64_t entries_evicted = 0;   // Evicted due to max_entries
        uint64_t entries_expired = 0;   // Evicted due to TTL
        uint64_t recoveries = 0;        // Successful decodes after combining (set externally)
    };

    Stats getStats() const;
    void resetStats();
    void incrementRecoveries();  // Called externally when chase combining succeeds

private:
    Config config_;
    mutable std::mutex mutex_;
    std::unordered_map<ChaseCacheKey, ChaseCacheEntry, ChaseCacheKeyHash> cache_;
    mutable Stats stats_;

    // Evict oldest entries when cache is full (caller holds mutex)
    void evictIfNeeded();

    // Remove expired entries (caller holds mutex)
    void pruneExpired();
};

} // namespace fec
} // namespace ultra
