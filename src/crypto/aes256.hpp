// AES-256-CBC encryption for Ultra modem
// Provides optional encryption for data payloads

#pragma once

#include <vector>
#include <string>
#include <cstdint>
#include <array>

namespace ultra {
namespace crypto {

// AES-256 key size (32 bytes)
constexpr size_t AES256_KEY_SIZE = 32;
// AES block size (16 bytes)
constexpr size_t AES_BLOCK_SIZE = 16;
// IV size for CBC mode (16 bytes)
constexpr size_t AES_IV_SIZE = 16;

/**
 * AES-256-CBC Cipher
 *
 * Provides encryption and decryption using AES-256 in CBC mode.
 * Uses PKCS7 padding for data alignment.
 *
 * Wire format: [16-byte IV][encrypted data with padding]
 */
class Aes256 {
public:
    Aes256();
    ~Aes256();

    /**
     * Set encryption key from raw 32-byte key
     * @param key 32-byte AES-256 key
     * @return true if key is valid
     */
    bool setKey(const std::array<uint8_t, AES256_KEY_SIZE>& key);

    /**
     * Set encryption key from passphrase (SHA-256 hash)
     * @param passphrase User-provided passphrase
     * @return true if passphrase is non-empty
     */
    bool setKeyFromPassphrase(const std::string& passphrase);

    /**
     * Check if a valid key is set
     */
    bool hasKey() const { return key_set_; }

    /**
     * Clear the key from memory
     */
    void clearKey();

    /**
     * Encrypt data
     * @param plaintext Data to encrypt
     * @return Encrypted data with IV prepended, or empty on error
     */
    std::vector<uint8_t> encrypt(const std::vector<uint8_t>& plaintext);

    /**
     * Decrypt data
     * @param ciphertext Encrypted data (IV + ciphertext)
     * @return Decrypted plaintext, or empty on error
     */
    std::vector<uint8_t> decrypt(const std::vector<uint8_t>& ciphertext);

    /**
     * Get minimum ciphertext size (IV + 1 block for padding)
     */
    static constexpr size_t minCiphertextSize() {
        return AES_IV_SIZE + AES_BLOCK_SIZE;
    }

private:
    std::array<uint8_t, AES256_KEY_SIZE> key_;
    bool key_set_ = false;

    // Expanded key schedule (60 32-bit words for AES-256)
    std::array<uint32_t, 60> expanded_key_;

    // AES core operations
    void keyExpansion();
    void encryptBlock(const uint8_t* in, uint8_t* out);
    void decryptBlock(const uint8_t* in, uint8_t* out);

    // Generate random IV
    void generateIV(uint8_t* iv);
};

/**
 * SHA-256 hash for key derivation
 * @param data Input data
 * @return 32-byte hash
 */
std::array<uint8_t, 32> sha256(const std::vector<uint8_t>& data);
std::array<uint8_t, 32> sha256(const std::string& str);

} // namespace crypto
} // namespace ultra
