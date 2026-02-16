//! AES-256-GCM encryption for secure data transmission
//!
//! Optional encryption layer.
//! Uses PBKDF2-SHA256 for key derivation from shared passphrase.

use aes_gcm::{
    aead::{Aead, KeyInit, OsRng},
    Aes256Gcm, Nonce,
};
use aes_gcm::aead::rand_core::RngCore;
use sha2::Sha256;
use pbkdf2::pbkdf2_hmac;

/// Encryption configuration
#[derive(Debug, Clone)]
pub struct EncryptionConfig {
    /// Enable encryption
    pub enabled: bool,
    /// Shared passphrase for key derivation
    passphrase: String,
    /// Salt for PBKDF2 (derived from session)
    salt: [u8; 16],
    /// Number of PBKDF2 iterations
    iterations: u32,
}

impl Default for EncryptionConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            passphrase: String::new(),
            salt: [0u8; 16],
            iterations: 100_000,
        }
    }
}

impl EncryptionConfig {
    /// Create new encryption config
    pub fn new(passphrase: &str, enabled: bool) -> Self {
        Self {
            enabled,
            passphrase: passphrase.to_string(),
            salt: [0u8; 16],
            iterations: 100_000,
        }
    }

    /// Set session-specific salt
    /// Uses sorted callsigns to ensure both sides derive the same salt
    pub fn set_session_salt(&mut self, session_id: u16, local_call: &str, remote_call: &str) {
        use sha2::{Sha256, Digest};
        // Sort callsigns so both sides compute the same salt regardless of who initiated
        let (call1, call2) = if local_call <= remote_call {
            (local_call, remote_call)
        } else {
            (remote_call, local_call)
        };
        let mut hasher = Sha256::new();
        hasher.update(session_id.to_le_bytes());
        hasher.update(call1.as_bytes());
        hasher.update(call2.as_bytes());
        let hash = hasher.finalize();
        self.salt.copy_from_slice(&hash[..16]);
    }
}

/// AES-256-GCM encryptor/decryptor
pub struct Encryptor {
    cipher: Aes256Gcm,
    nonce_counter: u64,
    enabled: bool,
}

impl Encryptor {
    /// Create new encryptor from config
    pub fn new(config: &EncryptionConfig) -> Self {
        if !config.enabled || config.passphrase.is_empty() {
            return Self {
                cipher: Self::dummy_cipher(),
                nonce_counter: 0,
                enabled: false,
            };
        }

        // Derive 256-bit key using PBKDF2-SHA256
        let mut key = [0u8; 32];
        pbkdf2_hmac::<Sha256>(
            config.passphrase.as_bytes(),
            &config.salt,
            config.iterations,
            &mut key,
        );

        let cipher = Aes256Gcm::new_from_slice(&key)
            .expect("Key length should be valid");

        Self {
            cipher,
            nonce_counter: 0,
            enabled: true,
        }
    }

    /// Create disabled encryptor
    pub fn disabled() -> Self {
        Self {
            cipher: Self::dummy_cipher(),
            nonce_counter: 0,
            enabled: false,
        }
    }

    /// Create a placeholder cipher for disabled state.
    /// This cipher is never actually used - when encryption is disabled,
    /// encrypt/decrypt pass through data unchanged. We need a valid cipher
    /// instance to satisfy the struct, but it's never called.
    fn dummy_cipher() -> Aes256Gcm {
        // Use random key for the unused cipher as a defense-in-depth measure
        let mut key = [0u8; 32];
        OsRng.fill_bytes(&mut key);
        Aes256Gcm::new_from_slice(&key).unwrap()
    }

    /// Check if encryption is enabled
    pub fn is_enabled(&self) -> bool {
        self.enabled
    }

    /// Encrypt plaintext data
    /// Returns ciphertext with 12-byte nonce prepended
    pub fn encrypt(&mut self, plaintext: &[u8]) -> Result<Vec<u8>, EncryptionError> {
        if !self.enabled {
            return Ok(plaintext.to_vec());
        }

        // Generate nonce from counter
        let mut nonce_bytes = [0u8; 12];
        nonce_bytes[..8].copy_from_slice(&self.nonce_counter.to_le_bytes());

        // Add randomness to upper 4 bytes for extra security
        let mut rng_bytes = [0u8; 4];
        OsRng.fill_bytes(&mut rng_bytes);
        nonce_bytes[8..].copy_from_slice(&rng_bytes);

        self.nonce_counter += 1;

        let nonce = Nonce::from_slice(&nonce_bytes);

        let ciphertext = self.cipher
            .encrypt(nonce, plaintext)
            .map_err(|_| EncryptionError::EncryptionFailed)?;

        // Prepend nonce to ciphertext
        let mut output = Vec::with_capacity(12 + ciphertext.len());
        output.extend_from_slice(&nonce_bytes);
        output.extend(ciphertext);

        Ok(output)
    }

    /// Decrypt ciphertext data
    /// Expects 12-byte nonce prepended to ciphertext
    pub fn decrypt(&self, ciphertext: &[u8]) -> Result<Vec<u8>, EncryptionError> {
        if !self.enabled {
            return Ok(ciphertext.to_vec());
        }

        if ciphertext.len() < 12 + 16 { // nonce + minimum tag
            return Err(EncryptionError::InvalidCiphertext);
        }

        let nonce = Nonce::from_slice(&ciphertext[..12]);
        let encrypted = &ciphertext[12..];

        self.cipher
            .decrypt(nonce, encrypted)
            .map_err(|_| EncryptionError::DecryptionFailed)
    }

    /// Reset nonce counter (call on new session)
    pub fn reset(&mut self) {
        self.nonce_counter = 0;
    }
}

/// Encryption errors
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum EncryptionError {
    /// Encryption operation failed
    EncryptionFailed,
    /// Decryption operation failed (wrong key or corrupted data)
    DecryptionFailed,
    /// Invalid ciphertext format
    InvalidCiphertext,
}

impl std::fmt::Display for EncryptionError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            EncryptionError::EncryptionFailed => write!(f, "Encryption failed"),
            EncryptionError::DecryptionFailed => write!(f, "Decryption failed"),
            EncryptionError::InvalidCiphertext => write!(f, "Invalid ciphertext"),
        }
    }
}

impl std::error::Error for EncryptionError {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_encryption_disabled_by_default() {
        let config = EncryptionConfig::default();
        let encryptor = Encryptor::new(&config);
        assert!(!encryptor.is_enabled());
    }

    #[test]
    fn test_passthrough_when_disabled() {
        let mut encryptor = Encryptor::disabled();
        let plaintext = b"Hello, World!";

        let encrypted = encryptor.encrypt(plaintext).unwrap();
        assert_eq!(encrypted, plaintext);

        let decrypted = encryptor.decrypt(&encrypted).unwrap();
        assert_eq!(decrypted, plaintext);
    }

    #[test]
    fn test_encryption_round_trip() {
        let mut config = EncryptionConfig::new("test_passphrase_123", true);
        config.set_session_salt(12345, "W1AW", "K1ABC");

        let mut encryptor = Encryptor::new(&config);
        let plaintext = b"Hello, secure world! This is a test message.";

        let encrypted = encryptor.encrypt(plaintext).unwrap();

        // Encrypted should be longer (nonce + tag)
        assert!(encrypted.len() > plaintext.len());

        // Encrypted should be different from plaintext
        assert_ne!(&encrypted[12..12 + plaintext.len()], plaintext);

        let decryptor = Encryptor::new(&config);
        let decrypted = decryptor.decrypt(&encrypted).unwrap();

        assert_eq!(decrypted, plaintext);
    }

    #[test]
    fn test_wrong_key_fails() {
        let mut config1 = EncryptionConfig::new("correct_passphrase", true);
        config1.set_session_salt(12345, "W1AW", "K1ABC");

        let mut config2 = EncryptionConfig::new("wrong_passphrase", true);
        config2.set_session_salt(12345, "W1AW", "K1ABC");

        let mut encryptor1 = Encryptor::new(&config1);
        let plaintext = b"Secret message";

        let encrypted = encryptor1.encrypt(plaintext).unwrap();

        let decryptor2 = Encryptor::new(&config2);
        let result = decryptor2.decrypt(&encrypted);

        assert!(result.is_err());
    }

    #[test]
    fn test_unique_nonces() {
        let mut config = EncryptionConfig::new("passphrase", true);
        config.set_session_salt(1, "A", "B");

        let mut encryptor = Encryptor::new(&config);
        let plaintext = b"Test";

        let enc1 = encryptor.encrypt(plaintext).unwrap();
        let enc2 = encryptor.encrypt(plaintext).unwrap();

        // Same plaintext should produce different ciphertext due to unique nonces
        assert_ne!(enc1, enc2);

        // But both should decrypt correctly
        let decryptor = Encryptor::new(&config);
        assert_eq!(decryptor.decrypt(&enc1).unwrap(), plaintext);
        assert_eq!(decryptor.decrypt(&enc2).unwrap(), plaintext);
    }

    #[test]
    fn test_empty_plaintext() {
        let mut config = EncryptionConfig::new("passphrase", true);
        config.set_session_salt(1, "A", "B");

        let mut encryptor = Encryptor::new(&config);
        let plaintext = b"";

        let encrypted = encryptor.encrypt(plaintext).unwrap();
        let decryptor = Encryptor::new(&config);
        let decrypted = decryptor.decrypt(&encrypted).unwrap();

        assert_eq!(decrypted, plaintext);
    }

    #[test]
    fn test_large_plaintext() {
        let mut config = EncryptionConfig::new("passphrase", true);
        config.set_session_salt(1, "A", "B");

        let mut encryptor = Encryptor::new(&config);
        let plaintext: Vec<u8> = (0..10000).map(|i| (i % 256) as u8).collect();

        let encrypted = encryptor.encrypt(&plaintext).unwrap();
        let decryptor = Encryptor::new(&config);
        let decrypted = decryptor.decrypt(&encrypted).unwrap();

        assert_eq!(decrypted, plaintext);
    }
}
