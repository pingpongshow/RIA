//! Huffman compression for payload data
//!
//! Implements static Huffman coding optimized for typical ham radio text.
//! Uses a pre-computed codebook based on character frequency analysis.

use std::collections::HashMap;

/// Huffman code entry: (codeword bits, length)
#[derive(Debug, Clone, Copy)]
struct HuffmanCode {
    bits: u32,
    len: u8,
}

/// Static Huffman codebook optimized for ham radio messages
/// Canonical Huffman codes - prefix-free by construction
/// Code lengths derived from English letter frequencies
const HUFFMAN_TABLE: &[(u8, u32, u8)] = &[
    // 3-bit codes (most frequent) - use 0xx
    (b' ', 0b000, 3),           // Space
    (b'e', 0b001, 3),           // e
    (b't', 0b010, 3),           // t
    (b'a', 0b011, 3),           // a
    // 4-bit codes - use 10xx
    (b'o', 0b1000, 4),          // o
    (b'i', 0b1001, 4),          // i
    (b'n', 0b1010, 4),          // n
    (b's', 0b1011, 4),          // s
    // 5-bit codes - use 110xx
    (b'r', 0b11000, 5),         // r
    (b'h', 0b11001, 5),         // h
    (b'l', 0b11010, 5),         // l
    (b'd', 0b11011, 5),         // d
    // 6-bit codes - use 1110xx
    (b'c', 0b111000, 6),        // c
    (b'u', 0b111001, 6),        // u
    (b'm', 0b111010, 6),        // m
    (b'f', 0b111011, 6),        // f
    // 7-bit codes - use 11110xx
    (b'p', 0b1111000, 7),       // p
    (b'g', 0b1111001, 7),       // g
    (b'w', 0b1111010, 7),       // w
    (b'y', 0b1111011, 7),       // y
    // 8-bit codes - use 111110xx (escape uses 11111100 + 8 bits)
    (b'b', 0b11111000, 8),      // b
    (b'v', 0b11111001, 8),      // v
    (b'k', 0b11111010, 8),      // k
    (b'x', 0b11111011, 8),      // x
    // 9-bit codes for very rare letters
    (b'j', 0b111111010, 9),     // j
    (b'q', 0b111111011, 9),     // q
    (b'z', 0b111111100, 9),     // z
];

/// Escape code: 111111101 (9 bits) followed by 8-bit literal
const ESCAPE_PREFIX: u32 = 0b111111101;
const ESCAPE_LEN: u8 = 9;

/// Huffman encoder
pub struct HuffmanEncoder {
    encode_table: HashMap<u8, HuffmanCode>,
}

impl HuffmanEncoder {
    /// Create new Huffman encoder
    pub fn new() -> Self {
        let mut encode_table = HashMap::new();

        // Build encode table from static data
        for &(byte, _bits, len) in HUFFMAN_TABLE {
            encode_table.insert(byte, HuffmanCode { bits: _bits, len });
        }

        // Add uppercase versions with same codes as lowercase
        for &(byte, _bits, _len) in HUFFMAN_TABLE {
            if byte >= b'a' && byte <= b'z' {
                let _upper = byte - 32;
                // Uppercase uses escape + literal
            }
        }

        Self { encode_table }
    }

    /// Encode data using Huffman coding
    pub fn encode(&self, data: &[u8]) -> Vec<u8> {
        let mut bit_buffer: u64 = 0;
        let mut bits_in_buffer: u8 = 0;
        let mut output = Vec::with_capacity(data.len());

        for &byte in data {
            let lower = if byte >= b'A' && byte <= b'Z' {
                byte + 32 // Convert to lowercase for lookup
            } else {
                byte
            };

            if let Some(&code) = self.encode_table.get(&lower) {
                // Check if uppercase - need special handling
                if byte >= b'A' && byte <= b'Z' {
                    // Uppercase: use escape + literal
                    Self::write_bits(&mut bit_buffer, &mut bits_in_buffer, ESCAPE_PREFIX, ESCAPE_LEN, &mut output);
                    Self::write_bits(&mut bit_buffer, &mut bits_in_buffer, byte as u32, 8, &mut output);
                } else {
                    Self::write_bits(&mut bit_buffer, &mut bits_in_buffer, code.bits, code.len, &mut output);
                }
            } else {
                // Escape + literal byte
                Self::write_bits(&mut bit_buffer, &mut bits_in_buffer, ESCAPE_PREFIX, ESCAPE_LEN, &mut output);
                Self::write_bits(&mut bit_buffer, &mut bits_in_buffer, byte as u32, 8, &mut output);
            }
        }

        // Flush remaining bits with padding info
        let padding = if bits_in_buffer > 0 {
            let pad = 8 - bits_in_buffer;
            bit_buffer <<= pad;
            output.push((bit_buffer & 0xFF) as u8);
            pad
        } else {
            0
        };

        output.push(padding);
        output
    }

    fn write_bits(buffer: &mut u64, bits_count: &mut u8, bits: u32, len: u8, output: &mut Vec<u8>) {
        *buffer = (*buffer << len) | (bits as u64);
        *bits_count += len;

        while *bits_count >= 8 {
            *bits_count -= 8;
            let byte = (*buffer >> *bits_count) as u8;
            output.push(byte);
        }

        // Keep buffer clean
        *buffer &= (1u64 << *bits_count) - 1;
    }
}

impl Default for HuffmanEncoder {
    fn default() -> Self {
        Self::new()
    }
}

/// Huffman decoder using prefix table for fast lookup
pub struct HuffmanDecoder {
    // Map from (bits, len) -> decoded byte
    decode_map: HashMap<(u32, u8), u8>,
    max_code_len: u8,
}

impl HuffmanDecoder {
    /// Create new Huffman decoder
    pub fn new() -> Self {
        let mut decode_map = HashMap::new();
        let mut max_code_len = 0u8;

        for &(byte, bits, len) in HUFFMAN_TABLE {
            decode_map.insert((bits, len), byte);
            max_code_len = max_code_len.max(len);
        }

        Self {
            decode_map,
            max_code_len: max_code_len.max(ESCAPE_LEN),
        }
    }

    /// Decode Huffman-encoded data
    pub fn decode(&self, data: &[u8]) -> Result<Vec<u8>, HuffmanError> {
        if data.is_empty() {
            return Ok(Vec::new());
        }

        if data.len() == 1 {
            // Only padding byte, empty data
            return Ok(Vec::new());
        }

        // Last byte is padding count
        let padding = data[data.len() - 1] as usize;
        if padding > 7 {
            return Err(HuffmanError::InvalidPadding);
        }

        let data_bytes = &data[..data.len() - 1];
        let total_bits = if data_bytes.is_empty() {
            0
        } else {
            data_bytes.len() * 8 - padding
        };

        let mut output = Vec::new();
        let mut bit_pos = 0;

        while bit_pos < total_bits {
            // Try to match codes of increasing length
            let mut matched = false;

            for len in 3..=self.max_code_len {
                if bit_pos + len as usize > total_bits {
                    break;
                }

                let bits = self.read_bits(data_bytes, bit_pos, len as usize);

                // Check for escape sequence
                if len == ESCAPE_LEN && bits == ESCAPE_PREFIX {
                    // Read literal byte
                    if bit_pos + ESCAPE_LEN as usize + 8 > total_bits {
                        return Err(HuffmanError::UnexpectedEnd);
                    }
                    bit_pos += ESCAPE_LEN as usize;
                    let literal = self.read_bits(data_bytes, bit_pos, 8) as u8;
                    bit_pos += 8;
                    output.push(literal);
                    matched = true;
                    break;
                }

                if let Some(&byte) = self.decode_map.get(&(bits, len)) {
                    output.push(byte);
                    bit_pos += len as usize;
                    matched = true;
                    break;
                }
            }

            if !matched {
                return Err(HuffmanError::InvalidCode);
            }
        }

        Ok(output)
    }

    fn read_bits(&self, data: &[u8], start_bit: usize, len: usize) -> u32 {
        let mut result = 0u32;
        for i in 0..len {
            let byte_idx = (start_bit + i) / 8;
            let bit_idx = 7 - ((start_bit + i) % 8);

            if byte_idx < data.len() {
                let bit = (data[byte_idx] >> bit_idx) & 1;
                result = (result << 1) | (bit as u32);
            }
        }
        result
    }
}

impl Default for HuffmanDecoder {
    fn default() -> Self {
        Self::new()
    }
}

/// Huffman coding errors
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum HuffmanError {
    InvalidCode,
    InvalidPadding,
    UnexpectedEnd,
}

impl std::fmt::Display for HuffmanError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            HuffmanError::InvalidCode => write!(f, "Invalid Huffman code"),
            HuffmanError::InvalidPadding => write!(f, "Invalid padding"),
            HuffmanError::UnexpectedEnd => write!(f, "Unexpected end of data"),
        }
    }
}

impl std::error::Error for HuffmanError {}

/// Calculate compression ratio
pub fn compression_ratio(original: &[u8], compressed: &[u8]) -> f32 {
    if original.is_empty() {
        return 1.0;
    }
    compressed.len() as f32 / original.len() as f32
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_encode_decode_simple() {
        let encoder = HuffmanEncoder::new();
        let decoder = HuffmanDecoder::new();

        let original = b"the test";
        let encoded = encoder.encode(original);
        let decoded = decoder.decode(&encoded).unwrap();

        assert_eq!(decoded, original);
    }

    #[test]
    fn test_encode_decode_lowercase() {
        let encoder = HuffmanEncoder::new();
        let decoder = HuffmanDecoder::new();

        let original = b"hello world";
        let encoded = encoder.encode(original);
        let decoded = decoder.decode(&encoded).unwrap();

        assert_eq!(decoded, original);
    }

    #[test]
    fn test_encode_decode_with_spaces() {
        let encoder = HuffmanEncoder::new();
        let decoder = HuffmanDecoder::new();

        let original = b"cq cq de test";
        let encoded = encoder.encode(original);
        let decoded = decoder.decode(&encoded).unwrap();

        assert_eq!(decoded, original);
    }

    #[test]
    fn test_escape_sequence() {
        let encoder = HuffmanEncoder::new();
        let decoder = HuffmanDecoder::new();

        // Include bytes not in lowercase table (uppercase, numbers)
        let original = b"ABC 123";
        let encoded = encoder.encode(original);
        let decoded = decoder.decode(&encoded).unwrap();

        assert_eq!(decoded, original);
    }

    #[test]
    fn test_empty_input() {
        let encoder = HuffmanEncoder::new();
        let decoder = HuffmanDecoder::new();

        let original = b"";
        let encoded = encoder.encode(original);
        let decoded = decoder.decode(&encoded).unwrap();

        assert_eq!(decoded, original);
    }

    #[test]
    fn test_single_char() {
        let encoder = HuffmanEncoder::new();
        let decoder = HuffmanDecoder::new();

        let original = b"e";
        let encoded = encoder.encode(original);
        let decoded = decoder.decode(&encoded).unwrap();

        assert_eq!(decoded, original);
    }

    #[test]
    fn test_compression_achieved() {
        let encoder = HuffmanEncoder::new();

        // Typical ham radio text should compress
        let text = b"cq cq cq de test test test";
        let encoded = encoder.encode(text);

        // Should achieve some compression for repetitive lowercase text
        assert!(encoded.len() <= text.len(),
            "Expected compression: {} -> {}", text.len(), encoded.len());
    }
}
