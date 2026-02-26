// AES-256-CBC implementation for Ultra modem

#include "aes256.hpp"
#include <cstring>
#include <random>
#include <chrono>

namespace ultra {
namespace crypto {

// AES S-box
static const uint8_t SBOX[256] = {
    0x63, 0x7c, 0x77, 0x7b, 0xf2, 0x6b, 0x6f, 0xc5, 0x30, 0x01, 0x67, 0x2b, 0xfe, 0xd7, 0xab, 0x76,
    0xca, 0x82, 0xc9, 0x7d, 0xfa, 0x59, 0x47, 0xf0, 0xad, 0xd4, 0xa2, 0xaf, 0x9c, 0xa4, 0x72, 0xc0,
    0xb7, 0xfd, 0x93, 0x26, 0x36, 0x3f, 0xf7, 0xcc, 0x34, 0xa5, 0xe5, 0xf1, 0x71, 0xd8, 0x31, 0x15,
    0x04, 0xc7, 0x23, 0xc3, 0x18, 0x96, 0x05, 0x9a, 0x07, 0x12, 0x80, 0xe2, 0xeb, 0x27, 0xb2, 0x75,
    0x09, 0x83, 0x2c, 0x1a, 0x1b, 0x6e, 0x5a, 0xa0, 0x52, 0x3b, 0xd6, 0xb3, 0x29, 0xe3, 0x2f, 0x84,
    0x53, 0xd1, 0x00, 0xed, 0x20, 0xfc, 0xb1, 0x5b, 0x6a, 0xcb, 0xbe, 0x39, 0x4a, 0x4c, 0x58, 0xcf,
    0xd0, 0xef, 0xaa, 0xfb, 0x43, 0x4d, 0x33, 0x85, 0x45, 0xf9, 0x02, 0x7f, 0x50, 0x3c, 0x9f, 0xa8,
    0x51, 0xa3, 0x40, 0x8f, 0x92, 0x9d, 0x38, 0xf5, 0xbc, 0xb6, 0xda, 0x21, 0x10, 0xff, 0xf3, 0xd2,
    0xcd, 0x0c, 0x13, 0xec, 0x5f, 0x97, 0x44, 0x17, 0xc4, 0xa7, 0x7e, 0x3d, 0x64, 0x5d, 0x19, 0x73,
    0x60, 0x81, 0x4f, 0xdc, 0x22, 0x2a, 0x90, 0x88, 0x46, 0xee, 0xb8, 0x14, 0xde, 0x5e, 0x0b, 0xdb,
    0xe0, 0x32, 0x3a, 0x0a, 0x49, 0x06, 0x24, 0x5c, 0xc2, 0xd3, 0xac, 0x62, 0x91, 0x95, 0xe4, 0x79,
    0xe7, 0xc8, 0x37, 0x6d, 0x8d, 0xd5, 0x4e, 0xa9, 0x6c, 0x56, 0xf4, 0xea, 0x65, 0x7a, 0xae, 0x08,
    0xba, 0x78, 0x25, 0x2e, 0x1c, 0xa6, 0xb4, 0xc6, 0xe8, 0xdd, 0x74, 0x1f, 0x4b, 0xbd, 0x8b, 0x8a,
    0x70, 0x3e, 0xb5, 0x66, 0x48, 0x03, 0xf6, 0x0e, 0x61, 0x35, 0x57, 0xb9, 0x86, 0xc1, 0x1d, 0x9e,
    0xe1, 0xf8, 0x98, 0x11, 0x69, 0xd9, 0x8e, 0x94, 0x9b, 0x1e, 0x87, 0xe9, 0xce, 0x55, 0x28, 0xdf,
    0x8c, 0xa1, 0x89, 0x0d, 0xbf, 0xe6, 0x42, 0x68, 0x41, 0x99, 0x2d, 0x0f, 0xb0, 0x54, 0xbb, 0x16
};

// Inverse S-box
static const uint8_t INV_SBOX[256] = {
    0x52, 0x09, 0x6a, 0xd5, 0x30, 0x36, 0xa5, 0x38, 0xbf, 0x40, 0xa3, 0x9e, 0x81, 0xf3, 0xd7, 0xfb,
    0x7c, 0xe3, 0x39, 0x82, 0x9b, 0x2f, 0xff, 0x87, 0x34, 0x8e, 0x43, 0x44, 0xc4, 0xde, 0xe9, 0xcb,
    0x54, 0x7b, 0x94, 0x32, 0xa6, 0xc2, 0x23, 0x3d, 0xee, 0x4c, 0x95, 0x0b, 0x42, 0xfa, 0xc3, 0x4e,
    0x08, 0x2e, 0xa1, 0x66, 0x28, 0xd9, 0x24, 0xb2, 0x76, 0x5b, 0xa2, 0x49, 0x6d, 0x8b, 0xd1, 0x25,
    0x72, 0xf8, 0xf6, 0x64, 0x86, 0x68, 0x98, 0x16, 0xd4, 0xa4, 0x5c, 0xcc, 0x5d, 0x65, 0xb6, 0x92,
    0x6c, 0x70, 0x48, 0x50, 0xfd, 0xed, 0xb9, 0xda, 0x5e, 0x15, 0x46, 0x57, 0xa7, 0x8d, 0x9d, 0x84,
    0x90, 0xd8, 0xab, 0x00, 0x8c, 0xbc, 0xd3, 0x0a, 0xf7, 0xe4, 0x58, 0x05, 0xb8, 0xb3, 0x45, 0x06,
    0xd0, 0x2c, 0x1e, 0x8f, 0xca, 0x3f, 0x0f, 0x02, 0xc1, 0xaf, 0xbd, 0x03, 0x01, 0x13, 0x8a, 0x6b,
    0x3a, 0x91, 0x11, 0x41, 0x4f, 0x67, 0xdc, 0xea, 0x97, 0xf2, 0xcf, 0xce, 0xf0, 0xb4, 0xe6, 0x73,
    0x96, 0xac, 0x74, 0x22, 0xe7, 0xad, 0x35, 0x85, 0xe2, 0xf9, 0x37, 0xe8, 0x1c, 0x75, 0xdf, 0x6e,
    0x47, 0xf1, 0x1a, 0x71, 0x1d, 0x29, 0xc5, 0x89, 0x6f, 0xb7, 0x62, 0x0e, 0xaa, 0x18, 0xbe, 0x1b,
    0xfc, 0x56, 0x3e, 0x4b, 0xc6, 0xd2, 0x79, 0x20, 0x9a, 0xdb, 0xc0, 0xfe, 0x78, 0xcd, 0x5a, 0xf4,
    0x1f, 0xdd, 0xa8, 0x33, 0x88, 0x07, 0xc7, 0x31, 0xb1, 0x12, 0x10, 0x59, 0x27, 0x80, 0xec, 0x5f,
    0x60, 0x51, 0x7f, 0xa9, 0x19, 0xb5, 0x4a, 0x0d, 0x2d, 0xe5, 0x7a, 0x9f, 0x93, 0xc9, 0x9c, 0xef,
    0xa0, 0xe0, 0x3b, 0x4d, 0xae, 0x2a, 0xf5, 0xb0, 0xc8, 0xeb, 0xbb, 0x3c, 0x83, 0x53, 0x99, 0x61,
    0x17, 0x2b, 0x04, 0x7e, 0xba, 0x77, 0xd6, 0x26, 0xe1, 0x69, 0x14, 0x63, 0x55, 0x21, 0x0c, 0x7d
};

// Round constants
static const uint8_t RCON[11] = {
    0x00, 0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x1b, 0x36
};

// GF(2^8) multiplication
static uint8_t gmul(uint8_t a, uint8_t b) {
    uint8_t p = 0;
    for (int i = 0; i < 8; i++) {
        if (b & 1) p ^= a;
        bool hi = a & 0x80;
        a <<= 1;
        if (hi) a ^= 0x1b;  // x^8 + x^4 + x^3 + x + 1
        b >>= 1;
    }
    return p;
}

Aes256::Aes256() {
    key_.fill(0);
    expanded_key_.fill(0);
}

Aes256::~Aes256() {
    clearKey();
}

bool Aes256::setKey(const std::array<uint8_t, AES256_KEY_SIZE>& key) {
    key_ = key;
    key_set_ = true;
    keyExpansion();
    return true;
}

bool Aes256::setKeyFromPassphrase(const std::string& passphrase) {
    if (passphrase.empty()) {
        return false;
    }

    // Derive key using SHA-256
    auto hash = sha256(passphrase);
    std::array<uint8_t, AES256_KEY_SIZE> key;
    std::copy(hash.begin(), hash.end(), key.begin());
    return setKey(key);
}

void Aes256::clearKey() {
    // Secure clear
    volatile uint8_t* p = key_.data();
    for (size_t i = 0; i < key_.size(); i++) {
        p[i] = 0;
    }
    volatile uint32_t* e = expanded_key_.data();
    for (size_t i = 0; i < expanded_key_.size(); i++) {
        e[i] = 0;
    }
    key_set_ = false;
}

void Aes256::keyExpansion() {
    // Copy key to first 8 words
    for (int i = 0; i < 8; i++) {
        expanded_key_[i] = (key_[4*i] << 24) | (key_[4*i+1] << 16) |
                          (key_[4*i+2] << 8) | key_[4*i+3];
    }

    // Generate remaining words
    for (int i = 8; i < 60; i++) {
        uint32_t temp = expanded_key_[i - 1];

        if (i % 8 == 0) {
            // RotWord + SubWord + Rcon
            temp = ((SBOX[(temp >> 16) & 0xff] << 24) |
                    (SBOX[(temp >> 8) & 0xff] << 16) |
                    (SBOX[temp & 0xff] << 8) |
                    SBOX[(temp >> 24) & 0xff]);
            temp ^= static_cast<uint32_t>(RCON[i / 8]) << 24;
        } else if (i % 8 == 4) {
            // SubWord only
            temp = (SBOX[(temp >> 24) & 0xff] << 24) |
                   (SBOX[(temp >> 16) & 0xff] << 16) |
                   (SBOX[(temp >> 8) & 0xff] << 8) |
                   SBOX[temp & 0xff];
        }

        expanded_key_[i] = expanded_key_[i - 8] ^ temp;
    }
}

void Aes256::encryptBlock(const uint8_t* in, uint8_t* out) {
    uint8_t state[16];
    std::memcpy(state, in, 16);

    // AddRoundKey - initial
    for (int i = 0; i < 4; i++) {
        uint32_t k = expanded_key_[i];
        state[4*i] ^= (k >> 24) & 0xff;
        state[4*i+1] ^= (k >> 16) & 0xff;
        state[4*i+2] ^= (k >> 8) & 0xff;
        state[4*i+3] ^= k & 0xff;
    }

    // 14 rounds for AES-256
    for (int round = 1; round <= 14; round++) {
        // SubBytes
        for (int i = 0; i < 16; i++) {
            state[i] = SBOX[state[i]];
        }

        // ShiftRows
        uint8_t temp;
        // Row 1: shift left 1
        temp = state[1];
        state[1] = state[5]; state[5] = state[9]; state[9] = state[13]; state[13] = temp;
        // Row 2: shift left 2
        temp = state[2]; state[2] = state[10]; state[10] = temp;
        temp = state[6]; state[6] = state[14]; state[14] = temp;
        // Row 3: shift left 3 (= right 1)
        temp = state[15];
        state[15] = state[11]; state[11] = state[7]; state[7] = state[3]; state[3] = temp;

        // MixColumns (skip in final round)
        if (round < 14) {
            for (int c = 0; c < 4; c++) {
                uint8_t a0 = state[4*c], a1 = state[4*c+1], a2 = state[4*c+2], a3 = state[4*c+3];
                state[4*c]   = gmul(a0, 2) ^ gmul(a1, 3) ^ a2 ^ a3;
                state[4*c+1] = a0 ^ gmul(a1, 2) ^ gmul(a2, 3) ^ a3;
                state[4*c+2] = a0 ^ a1 ^ gmul(a2, 2) ^ gmul(a3, 3);
                state[4*c+3] = gmul(a0, 3) ^ a1 ^ a2 ^ gmul(a3, 2);
            }
        }

        // AddRoundKey
        for (int i = 0; i < 4; i++) {
            uint32_t k = expanded_key_[round * 4 + i];
            state[4*i] ^= (k >> 24) & 0xff;
            state[4*i+1] ^= (k >> 16) & 0xff;
            state[4*i+2] ^= (k >> 8) & 0xff;
            state[4*i+3] ^= k & 0xff;
        }
    }

    std::memcpy(out, state, 16);
}

void Aes256::decryptBlock(const uint8_t* in, uint8_t* out) {
    uint8_t state[16];
    std::memcpy(state, in, 16);

    // AddRoundKey - round 14
    for (int i = 0; i < 4; i++) {
        uint32_t k = expanded_key_[56 + i];
        state[4*i] ^= (k >> 24) & 0xff;
        state[4*i+1] ^= (k >> 16) & 0xff;
        state[4*i+2] ^= (k >> 8) & 0xff;
        state[4*i+3] ^= k & 0xff;
    }

    // 14 rounds for AES-256 (in reverse)
    for (int round = 13; round >= 0; round--) {
        // InvShiftRows
        uint8_t temp;
        // Row 1: shift right 1
        temp = state[13];
        state[13] = state[9]; state[9] = state[5]; state[5] = state[1]; state[1] = temp;
        // Row 2: shift right 2
        temp = state[2]; state[2] = state[10]; state[10] = temp;
        temp = state[6]; state[6] = state[14]; state[14] = temp;
        // Row 3: shift right 3 (= left 1)
        temp = state[3];
        state[3] = state[7]; state[7] = state[11]; state[11] = state[15]; state[15] = temp;

        // InvSubBytes
        for (int i = 0; i < 16; i++) {
            state[i] = INV_SBOX[state[i]];
        }

        // AddRoundKey
        for (int i = 0; i < 4; i++) {
            uint32_t k = expanded_key_[round * 4 + i];
            state[4*i] ^= (k >> 24) & 0xff;
            state[4*i+1] ^= (k >> 16) & 0xff;
            state[4*i+2] ^= (k >> 8) & 0xff;
            state[4*i+3] ^= k & 0xff;
        }

        // InvMixColumns (skip in round 0)
        if (round > 0) {
            for (int c = 0; c < 4; c++) {
                uint8_t a0 = state[4*c], a1 = state[4*c+1], a2 = state[4*c+2], a3 = state[4*c+3];
                state[4*c]   = gmul(a0, 14) ^ gmul(a1, 11) ^ gmul(a2, 13) ^ gmul(a3, 9);
                state[4*c+1] = gmul(a0, 9) ^ gmul(a1, 14) ^ gmul(a2, 11) ^ gmul(a3, 13);
                state[4*c+2] = gmul(a0, 13) ^ gmul(a1, 9) ^ gmul(a2, 14) ^ gmul(a3, 11);
                state[4*c+3] = gmul(a0, 11) ^ gmul(a1, 13) ^ gmul(a2, 9) ^ gmul(a3, 14);
            }
        }
    }

    std::memcpy(out, state, 16);
}

void Aes256::generateIV(uint8_t* iv) {
    // Use random device + time for entropy
    std::random_device rd;
    auto seed = rd() ^ static_cast<uint32_t>(
        std::chrono::high_resolution_clock::now().time_since_epoch().count());
    std::mt19937 gen(seed);
    std::uniform_int_distribution<uint32_t> dist(0, 255);

    for (size_t i = 0; i < AES_IV_SIZE; i++) {
        iv[i] = static_cast<uint8_t>(dist(gen));
    }
}

std::vector<uint8_t> Aes256::encrypt(const std::vector<uint8_t>& plaintext) {
    if (!key_set_ || plaintext.empty()) {
        return {};
    }

    // Calculate padded size (PKCS7)
    size_t pad_len = AES_BLOCK_SIZE - (plaintext.size() % AES_BLOCK_SIZE);
    size_t padded_size = plaintext.size() + pad_len;

    // Allocate output: IV + padded ciphertext
    std::vector<uint8_t> result(AES_IV_SIZE + padded_size);

    // Generate random IV
    generateIV(result.data());

    // Copy plaintext and add PKCS7 padding
    std::vector<uint8_t> padded(padded_size);
    std::memcpy(padded.data(), plaintext.data(), plaintext.size());
    for (size_t i = plaintext.size(); i < padded_size; i++) {
        padded[i] = static_cast<uint8_t>(pad_len);
    }

    // CBC encryption
    uint8_t* iv = result.data();
    uint8_t* out = result.data() + AES_IV_SIZE;
    uint8_t block[AES_BLOCK_SIZE];

    for (size_t i = 0; i < padded_size; i += AES_BLOCK_SIZE) {
        // XOR with previous ciphertext (or IV for first block)
        const uint8_t* prev = (i == 0) ? iv : (out + i - AES_BLOCK_SIZE);
        for (size_t j = 0; j < AES_BLOCK_SIZE; j++) {
            block[j] = padded[i + j] ^ prev[j];
        }
        encryptBlock(block, out + i);
    }

    return result;
}

std::vector<uint8_t> Aes256::decrypt(const std::vector<uint8_t>& ciphertext) {
    if (!key_set_ || ciphertext.size() < minCiphertextSize()) {
        return {};
    }

    // Verify size is valid (IV + N blocks)
    size_t ct_len = ciphertext.size() - AES_IV_SIZE;
    if (ct_len % AES_BLOCK_SIZE != 0) {
        return {};
    }

    // Extract IV
    const uint8_t* iv = ciphertext.data();
    const uint8_t* ct = ciphertext.data() + AES_IV_SIZE;

    // CBC decryption
    std::vector<uint8_t> plaintext(ct_len);
    uint8_t block[AES_BLOCK_SIZE];

    for (size_t i = 0; i < ct_len; i += AES_BLOCK_SIZE) {
        decryptBlock(ct + i, block);

        // XOR with previous ciphertext (or IV for first block)
        const uint8_t* prev = (i == 0) ? iv : (ct + i - AES_BLOCK_SIZE);
        for (size_t j = 0; j < AES_BLOCK_SIZE; j++) {
            plaintext[i + j] = block[j] ^ prev[j];
        }
    }

    // Remove PKCS7 padding
    uint8_t pad_len = plaintext.back();
    if (pad_len == 0 || pad_len > AES_BLOCK_SIZE) {
        return {};  // Invalid padding
    }

    // Verify padding
    for (size_t i = plaintext.size() - pad_len; i < plaintext.size(); i++) {
        if (plaintext[i] != pad_len) {
            return {};  // Invalid padding
        }
    }

    plaintext.resize(plaintext.size() - pad_len);
    return plaintext;
}

// SHA-256 implementation
namespace {

// SHA-256 constants
static const uint32_t SHA256_K[64] = {
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2
};

inline uint32_t rotr(uint32_t x, int n) { return (x >> n) | (x << (32 - n)); }
inline uint32_t ch(uint32_t x, uint32_t y, uint32_t z) { return (x & y) ^ (~x & z); }
inline uint32_t maj(uint32_t x, uint32_t y, uint32_t z) { return (x & y) ^ (x & z) ^ (y & z); }
inline uint32_t ep0(uint32_t x) { return rotr(x, 2) ^ rotr(x, 13) ^ rotr(x, 22); }
inline uint32_t ep1(uint32_t x) { return rotr(x, 6) ^ rotr(x, 11) ^ rotr(x, 25); }
inline uint32_t sig0(uint32_t x) { return rotr(x, 7) ^ rotr(x, 18) ^ (x >> 3); }
inline uint32_t sig1(uint32_t x) { return rotr(x, 17) ^ rotr(x, 19) ^ (x >> 10); }

} // anonymous namespace

std::array<uint8_t, 32> sha256(const std::vector<uint8_t>& data) {
    // Initial hash values
    uint32_t h[8] = {
        0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a,
        0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19
    };

    // Pre-processing: add padding
    size_t orig_len = data.size();
    size_t new_len = ((orig_len + 8) / 64 + 1) * 64;
    std::vector<uint8_t> msg(new_len, 0);
    std::memcpy(msg.data(), data.data(), orig_len);
    msg[orig_len] = 0x80;

    // Append length in bits (big-endian)
    uint64_t bit_len = orig_len * 8;
    for (int i = 0; i < 8; i++) {
        msg[new_len - 1 - i] = static_cast<uint8_t>(bit_len >> (i * 8));
    }

    // Process 512-bit blocks
    for (size_t chunk = 0; chunk < new_len; chunk += 64) {
        uint32_t w[64];

        // Prepare message schedule
        for (int i = 0; i < 16; i++) {
            w[i] = (msg[chunk + 4*i] << 24) | (msg[chunk + 4*i + 1] << 16) |
                   (msg[chunk + 4*i + 2] << 8) | msg[chunk + 4*i + 3];
        }
        for (int i = 16; i < 64; i++) {
            w[i] = sig1(w[i-2]) + w[i-7] + sig0(w[i-15]) + w[i-16];
        }

        // Working variables
        uint32_t a = h[0], b = h[1], c = h[2], d = h[3];
        uint32_t e = h[4], f = h[5], g = h[6], hh = h[7];

        // Compression
        for (int i = 0; i < 64; i++) {
            uint32_t t1 = hh + ep1(e) + ch(e, f, g) + SHA256_K[i] + w[i];
            uint32_t t2 = ep0(a) + maj(a, b, c);
            hh = g; g = f; f = e; e = d + t1;
            d = c; c = b; b = a; a = t1 + t2;
        }

        h[0] += a; h[1] += b; h[2] += c; h[3] += d;
        h[4] += e; h[5] += f; h[6] += g; h[7] += hh;
    }

    // Output hash
    std::array<uint8_t, 32> result;
    for (int i = 0; i < 8; i++) {
        result[4*i] = (h[i] >> 24) & 0xff;
        result[4*i + 1] = (h[i] >> 16) & 0xff;
        result[4*i + 2] = (h[i] >> 8) & 0xff;
        result[4*i + 3] = h[i] & 0xff;
    }
    return result;
}

std::array<uint8_t, 32> sha256(const std::string& str) {
    return sha256(std::vector<uint8_t>(str.begin(), str.end()));
}

} // namespace crypto
} // namespace ultra
