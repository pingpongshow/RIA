//! CRC calculations for error detection
//!
//! Implements CRC-16 and CRC-32 for frame validation

/// CRC-16 CCITT calculator
pub struct Crc16 {
    table: [u16; 256],
    polynomial: u16,
    initial: u16,
}

impl Crc16 {
    /// Create CRC-16 CCITT (polynomial 0x1021)
    pub fn new() -> Self {
        Self::with_polynomial(0x1021, 0xFFFF)
    }

    /// Create CRC-16 with custom polynomial and initial value
    pub fn with_polynomial(polynomial: u16, initial: u16) -> Self {
        let table = Self::generate_table(polynomial);
        Self {
            table,
            polynomial,
            initial,
        }
    }

    fn generate_table(polynomial: u16) -> [u16; 256] {
        let mut table = [0u16; 256];

        for i in 0..256 {
            let mut crc = (i as u16) << 8;
            for _ in 0..8 {
                if crc & 0x8000 != 0 {
                    crc = (crc << 1) ^ polynomial;
                } else {
                    crc <<= 1;
                }
            }
            table[i] = crc;
        }

        table
    }

    /// Calculate CRC of data
    pub fn calculate(&self, data: &[u8]) -> u16 {
        let mut crc = self.initial;

        for &byte in data {
            let idx = ((crc >> 8) ^ (byte as u16)) as usize;
            crc = (crc << 8) ^ self.table[idx];
        }

        crc
    }

    /// Check if CRC matches
    pub fn verify(&self, data: &[u8], expected: u16) -> bool {
        self.calculate(data) == expected
    }

    /// Calculate and append CRC to data
    pub fn append(&self, data: &mut Vec<u8>) {
        let crc = self.calculate(data);
        data.push((crc >> 8) as u8);
        data.push((crc & 0xFF) as u8);
    }

    /// Extract and verify CRC from data (last 2 bytes)
    pub fn extract_and_verify(&self, data: &[u8]) -> Option<bool> {
        if data.len() < 2 {
            return None;
        }

        let crc_idx = data.len() - 2;
        let expected = ((data[crc_idx] as u16) << 8) | (data[crc_idx + 1] as u16);

        Some(self.calculate(&data[..crc_idx]) == expected)
    }
}

impl Default for Crc16 {
    fn default() -> Self {
        Self::new()
    }
}

/// CRC-32 calculator (IEEE 802.3 polynomial)
pub struct Crc32 {
    table: [u32; 256],
    polynomial: u32,
    initial: u32,
}

impl Crc32 {
    /// Create standard CRC-32 (IEEE 802.3)
    pub fn new() -> Self {
        Self::with_polynomial(0xEDB88320, 0xFFFFFFFF)
    }

    /// Create CRC-32 with custom polynomial and initial value
    pub fn with_polynomial(polynomial: u32, initial: u32) -> Self {
        let table = Self::generate_table(polynomial);
        Self {
            table,
            polynomial,
            initial,
        }
    }

    fn generate_table(polynomial: u32) -> [u32; 256] {
        let mut table = [0u32; 256];

        for i in 0..256 {
            let mut crc = i as u32;
            for _ in 0..8 {
                if crc & 1 != 0 {
                    crc = (crc >> 1) ^ polynomial;
                } else {
                    crc >>= 1;
                }
            }
            table[i] = crc;
        }

        table
    }

    /// Calculate CRC of data
    pub fn calculate(&self, data: &[u8]) -> u32 {
        let mut crc = self.initial;

        for &byte in data {
            let idx = ((crc ^ (byte as u32)) & 0xFF) as usize;
            crc = (crc >> 8) ^ self.table[idx];
        }

        crc ^ 0xFFFFFFFF
    }

    /// Check if CRC matches
    pub fn verify(&self, data: &[u8], expected: u32) -> bool {
        self.calculate(data) == expected
    }

    /// Calculate and append CRC to data (big-endian)
    pub fn append(&self, data: &mut Vec<u8>) {
        let crc = self.calculate(data);
        data.push((crc >> 24) as u8);
        data.push((crc >> 16) as u8);
        data.push((crc >> 8) as u8);
        data.push((crc & 0xFF) as u8);
    }

    /// Extract and verify CRC from data (last 4 bytes, big-endian)
    pub fn extract_and_verify(&self, data: &[u8]) -> Option<bool> {
        if data.len() < 4 {
            return None;
        }

        let crc_idx = data.len() - 4;
        let expected = ((data[crc_idx] as u32) << 24)
            | ((data[crc_idx + 1] as u32) << 16)
            | ((data[crc_idx + 2] as u32) << 8)
            | (data[crc_idx + 3] as u32);

        Some(self.calculate(&data[..crc_idx]) == expected)
    }
}

impl Default for Crc32 {
    fn default() -> Self {
        Self::new()
    }
}

/// Convenience function for CRC-16
pub fn crc16(data: &[u8]) -> u16 {
    Crc16::new().calculate(data)
}

/// Convenience function for CRC-32
pub fn crc32(data: &[u8]) -> u32 {
    Crc32::new().calculate(data)
}

/// Generate callsign hash using CRC-32
/// Verified: KD2NDR -> 0x48788FBC, TESTCS -> 0x2DF506D3
pub fn callsign_hash(callsign: &str) -> u32 {
    let normalized = callsign.to_uppercase();
    crc32(normalized.as_bytes())
}

/// Generate session seed from callsigns
///
/// Formula: session_seed = (src_hash XOR dst_hash XOR random_seed) & 0xFFFF
/// Returns 16-bit session ID for compact frame format
pub fn session_seed(src_callsign: &str, dst_callsign: &str, random_seed: u16) -> u16 {
    let src_hash = callsign_hash(src_callsign);
    let dst_hash = callsign_hash(dst_callsign);
    ((src_hash ^ dst_hash) as u16) ^ random_seed
}

/// Generate a random seed for session initialization
pub fn generate_random_seed() -> u16 {
    use std::time::{SystemTime, UNIX_EPOCH};
    let now = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default();
    // Mix time components for randomness
    let nanos = now.subsec_nanos();
    let secs = now.as_secs() as u32;
    (nanos.wrapping_mul(0x45D9F3B) ^ secs.wrapping_mul(0x119DE1F3)) as u16
}

/// Legacy session ID function (deprecated, use session_seed instead)
#[deprecated(note = "Use session_seed() for proper session handling")]
pub fn session_id(callsign1: &str, callsign2: &str, timestamp: u64) -> u64 {
    // Convert to session seed format
    let seed = session_seed(callsign1, callsign2, timestamp as u16);
    seed as u64
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_crc16_known_value() {
        let crc = Crc16::new();
        // Test with "123456789"
        let result = crc.calculate(b"123456789");
        assert_eq!(result, 0x29B1); // Known CRC-16 CCITT result
    }

    #[test]
    fn test_crc32_known_value() {
        let crc = Crc32::new();
        // Test with "123456789"
        let result = crc.calculate(b"123456789");
        assert_eq!(result, 0xCBF43926); // Known CRC-32 IEEE result
    }

    #[test]
    fn test_crc16_append_verify() {
        let crc = Crc16::new();
        let mut data = b"Hello, World!".to_vec();
        crc.append(&mut data);

        assert_eq!(data.len(), 13 + 2);
        assert_eq!(crc.extract_and_verify(&data), Some(true));
    }

    #[test]
    fn test_crc32_append_verify() {
        let crc = Crc32::new();
        let mut data = b"Hello, World!".to_vec();
        crc.append(&mut data);

        assert_eq!(data.len(), 13 + 4);
        assert_eq!(crc.extract_and_verify(&data), Some(true));
    }

    #[test]
    fn test_callsign_hash() {
        let hash1 = callsign_hash("W1AW");
        let hash2 = callsign_hash("w1aw"); // Should be same (case insensitive)
        let hash3 = callsign_hash("K1ABC");

        assert_eq!(hash1, hash2);
        assert_ne!(hash1, hash3);
    }

    #[test]
    fn test_callsign_hash_verified() {
        // These values are verified
        assert_eq!(callsign_hash("KD2NDR"), 0x48788FBC);
        assert_eq!(callsign_hash("TESTCS"), 0x2DF506D3);
    }

    #[test]
    fn test_session_seed() {
        // Same callsigns and random seed should produce same session seed
        let seed1 = session_seed("W1AW", "K1ABC", 12345);
        let seed2 = session_seed("W1AW", "K1ABC", 12345);
        assert_eq!(seed1, seed2);

        // Different random seed should produce different session seed
        let seed3 = session_seed("W1AW", "K1ABC", 12346);
        assert_ne!(seed1, seed3);

        // Order matters (src vs dst)
        let seed4 = session_seed("K1ABC", "W1AW", 12345);
        assert_eq!(seed1, seed4); // XOR is commutative, so these are equal
    }

    #[test]
    fn test_session_seed_formula() {
        // Verify the formula: seed = (src_hash XOR dst_hash) as u16 XOR random_seed
        let src = "W1AW";
        let dst = "K1ABC";
        let random = 0x5678u16;

        let expected = ((callsign_hash(src) ^ callsign_hash(dst)) as u16) ^ random;
        let actual = session_seed(src, dst, random);

        assert_eq!(actual, expected);
    }

    #[test]
    fn test_generate_random_seed() {
        // Random seeds should be different (with high probability)
        let seed1 = generate_random_seed();
        std::thread::sleep(std::time::Duration::from_millis(1));
        let seed2 = generate_random_seed();

        // They could theoretically be equal, but extremely unlikely
        assert_ne!(seed1, seed2);
    }
}
