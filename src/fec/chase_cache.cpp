// chase_cache.cpp - HARQ Chase Combining soft bit cache implementation

#include "chase_cache.hpp"
#include <algorithm>

namespace ultra {
namespace fec {

ChaseCache::ChaseCache(const Config& config)
    : config_(config)
{
}

void ChaseCache::setEnabled(bool enable) {
    std::lock_guard<std::mutex> lock(mutex_);
    config_.enabled = enable;
    if (!enable) {
        cache_.clear();
    }
}

bool ChaseCache::isEnabled() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return config_.enabled;
}

bool ChaseCache::store(const ChaseCacheKey& key,
                       int cw_index,
                       const std::vector<float>& soft_bits,
                       int total_cw,
                       protocol::v2::FrameType frame_type) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!config_.enabled) return false;
    if (soft_bits.size() != ChaseCacheEntry::LDPC_BLOCK_SIZE) return false;
    if (cw_index < 0 || cw_index >= total_cw) return false;
    if (total_cw <= 0 || total_cw > 16) return false;  // Sanity limit

    stats_.stores++;
    pruneExpired();

    auto it = cache_.find(key);
    if (it == cache_.end()) {
        // New entry
        evictIfNeeded();

        ChaseCacheEntry entry;
        entry.key = key;
        entry.cw_soft_bits.resize(total_cw);
        entry.cw_combine_count.resize(total_cw, 0);
        entry.cw_decoded.resize(total_cw, false);
        entry.created = std::chrono::steady_clock::now();
        entry.last_access = entry.created;
        entry.total_cw = total_cw;
        entry.frame_type = frame_type;

        // Store first reception
        entry.cw_soft_bits[cw_index] = soft_bits;
        entry.cw_combine_count[cw_index] = 1;

        cache_[key] = std::move(entry);
        return true;
    }

    // Existing entry - combine LLRs
    auto& entry = it->second;
    entry.last_access = std::chrono::steady_clock::now();

    if (cw_index >= entry.total_cw) return false;
    if (entry.isDecoded(cw_index)) return false;  // Already decoded
    if (!entry.hasRoom(cw_index)) return false;   // Max combines reached

    if (entry.cw_soft_bits[cw_index].empty()) {
        // First reception for this CW
        entry.cw_soft_bits[cw_index] = soft_bits;
        entry.cw_combine_count[cw_index] = 1;
    } else {
        // Combine with existing LLRs (simple addition - optimal for AWGN)
        auto& existing = entry.cw_soft_bits[cw_index];
        for (size_t i = 0; i < ChaseCacheEntry::LDPC_BLOCK_SIZE; ++i) {
            existing[i] += soft_bits[i];
        }
        entry.cw_combine_count[cw_index]++;
        stats_.combines++;
    }

    return true;
}

std::optional<std::vector<float>> ChaseCache::getCombined(
    const ChaseCacheKey& key, int cw_index) const {

    std::lock_guard<std::mutex> lock(mutex_);

    if (!config_.enabled) {
        stats_.cache_misses++;
        return std::nullopt;
    }

    auto it = cache_.find(key);
    if (it == cache_.end()) {
        stats_.cache_misses++;
        return std::nullopt;
    }

    const auto& entry = it->second;
    if (cw_index < 0 ||
        cw_index >= static_cast<int>(entry.cw_soft_bits.size()) ||
        entry.cw_soft_bits[cw_index].empty()) {
        stats_.cache_misses++;
        return std::nullopt;
    }

    // Don't return if already decoded
    if (entry.isDecoded(cw_index)) {
        stats_.cache_misses++;
        return std::nullopt;
    }

    stats_.cache_hits++;
    return entry.cw_soft_bits[cw_index];
}

int ChaseCache::getCombineCount(const ChaseCacheKey& key, int cw_index) const {
    std::lock_guard<std::mutex> lock(mutex_);

    auto it = cache_.find(key);
    if (it == cache_.end()) return 0;

    const auto& entry = it->second;
    if (cw_index < 0 || cw_index >= static_cast<int>(entry.cw_combine_count.size())) {
        return 0;
    }

    return entry.cw_combine_count[cw_index];
}

void ChaseCache::markDecoded(const ChaseCacheKey& key, int cw_index) {
    std::lock_guard<std::mutex> lock(mutex_);

    auto it = cache_.find(key);
    if (it == cache_.end()) return;

    auto& entry = it->second;
    if (cw_index >= 0 && cw_index < static_cast<int>(entry.cw_decoded.size())) {
        entry.cw_decoded[cw_index] = true;
        // Clear soft bits to free memory
        entry.cw_soft_bits[cw_index].clear();
        entry.cw_soft_bits[cw_index].shrink_to_fit();
    }

    // Check if all codewords are decoded - if so, remove entry
    bool all_decoded = std::all_of(entry.cw_decoded.begin(), entry.cw_decoded.end(),
                                    [](bool d) { return d; });
    if (all_decoded) {
        cache_.erase(it);
    }
}

void ChaseCache::removeEntry(const ChaseCacheKey& key) {
    std::lock_guard<std::mutex> lock(mutex_);
    cache_.erase(key);
}

void ChaseCache::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    cache_.clear();
}

size_t ChaseCache::size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return cache_.size();
}

ChaseCache::Stats ChaseCache::getStats() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return stats_;
}

void ChaseCache::resetStats() {
    std::lock_guard<std::mutex> lock(mutex_);
    stats_ = Stats{};
}

void ChaseCache::incrementRecoveries() {
    std::lock_guard<std::mutex> lock(mutex_);
    stats_.recoveries++;
}

void ChaseCache::evictIfNeeded() {
    // Called with mutex held
    while (cache_.size() >= config_.max_entries) {
        // Find oldest entry by last_access
        auto oldest = cache_.end();
        auto oldest_time = std::chrono::steady_clock::time_point::max();

        for (auto it = cache_.begin(); it != cache_.end(); ++it) {
            if (it->second.last_access < oldest_time) {
                oldest_time = it->second.last_access;
                oldest = it;
            }
        }

        if (oldest != cache_.end()) {
            cache_.erase(oldest);
            stats_.entries_evicted++;
        } else {
            break;  // Shouldn't happen
        }
    }
}

void ChaseCache::pruneExpired() {
    // Called with mutex held
    auto now = std::chrono::steady_clock::now();

    for (auto it = cache_.begin(); it != cache_.end(); ) {
        if (it->second.isExpired(config_.entry_ttl)) {
            it = cache_.erase(it);
            stats_.entries_expired++;
        } else {
            ++it;
        }
    }
}

} // namespace fec
} // namespace ultra
