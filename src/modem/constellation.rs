//! Constellation mapping for QAM/PSK modulation
//!
//! All constellation points are normalized at generation time to have unit average power.
//! This ensures consistent LLR calculations in soft demapping without requiring noise
//! variance scaling adjustments.

use super::Symbol;

/// Constellation type
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConstellationType {
    Bpsk,
    Qpsk,
    Psk8,
    Qam16,
    Qam32,
    Qam64,
    Qam256,
}

impl ConstellationType {
    /// Get bits per symbol
    pub fn bits_per_symbol(&self) -> usize {
        match self {
            ConstellationType::Bpsk => 1,
            ConstellationType::Qpsk => 2,
            ConstellationType::Psk8 => 3,
            ConstellationType::Qam16 => 4,
            ConstellationType::Qam32 => 5,
            ConstellationType::Qam64 => 6,
            ConstellationType::Qam256 => 8,
        }
    }

    /// Get number of constellation points
    pub fn num_points(&self) -> usize {
        1 << self.bits_per_symbol()
    }
}

/// Constellation mapper/demapper
///
/// Points are stored pre-normalized to unit average power, ensuring consistent
/// behavior between modulation and soft demodulation.
pub struct Constellation {
    constellation_type: ConstellationType,
    /// Pre-normalized constellation points (unit average power)
    points: Vec<Symbol>,
    gray_map: Vec<usize>,
    /// Normalization factor applied during generation (stored for reference)
    normalization: f32,
}

impl Constellation {
    /// Create a new constellation with pre-normalized points
    ///
    /// All points are normalized at construction time to have unit average power.
    /// This ensures that:
    /// - `map()` returns symbols with consistent power
    /// - `demap_soft()` LLR calculations work correctly with the noise variance
    pub fn new(constellation_type: ConstellationType) -> Self {
        let (raw_points, gray_map) = match constellation_type {
            ConstellationType::Bpsk => Self::generate_bpsk(),
            ConstellationType::Qpsk => Self::generate_qpsk(),
            ConstellationType::Psk8 => Self::generate_psk8(),
            ConstellationType::Qam16 => Self::generate_qam16(),
            ConstellationType::Qam32 => Self::generate_qam32(),
            ConstellationType::Qam64 => Self::generate_qam64(),
            ConstellationType::Qam256 => Self::generate_qam256(),
        };

        // Calculate normalization factor for unit average power
        // Guard against empty constellation or all-zero points to prevent infinity
        let normalization = if raw_points.is_empty() {
            1.0  // Default to unity if no points
        } else {
            let avg_power: f32 = raw_points.iter()
                .map(|p| p.i * p.i + p.q * p.q)
                .sum::<f32>() / raw_points.len() as f32;

            if avg_power <= 0.0 {
                1.0  // Default to unity for zero-power constellation
            } else {
                1.0 / avg_power.sqrt()
            }
        };

        // Apply normalization at generation time - points are now unit average power
        let points: Vec<Symbol> = raw_points.iter()
            .map(|p| Symbol::new(p.i * normalization, p.q * normalization))
            .collect();

        Self {
            constellation_type,
            points,
            gray_map,
            normalization,
        }
    }

    fn generate_bpsk() -> (Vec<Symbol>, Vec<usize>) {
        let points = vec![
            Symbol::new(1.0, 0.0),   // 0
            Symbol::new(-1.0, 0.0),  // 1
        ];
        let gray_map = vec![0, 1];
        (points, gray_map)
    }

    fn generate_qpsk() -> (Vec<Symbol>, Vec<usize>) {
        use std::f32::consts::FRAC_1_SQRT_2;
        let points = vec![
            Symbol::new(FRAC_1_SQRT_2, FRAC_1_SQRT_2),    // 00
            Symbol::new(-FRAC_1_SQRT_2, FRAC_1_SQRT_2),   // 01
            Symbol::new(FRAC_1_SQRT_2, -FRAC_1_SQRT_2),   // 10
            Symbol::new(-FRAC_1_SQRT_2, -FRAC_1_SQRT_2),  // 11
        ];
        // Gray mapping: 00->0, 01->1, 11->2, 10->3
        let gray_map = vec![0, 1, 3, 2];
        (points, gray_map)
    }

    fn generate_psk8() -> (Vec<Symbol>, Vec<usize>) {
        use std::f32::consts::PI;
        let points: Vec<Symbol> = (0..8)
            .map(|i| {
                let angle = PI / 4.0 + (i as f32) * PI / 4.0;
                Symbol::new(angle.cos(), angle.sin())
            })
            .collect();
        // Gray code for 8-PSK
        let gray_map = vec![0, 1, 3, 2, 6, 7, 5, 4];
        (points, gray_map)
    }

    fn generate_qam16() -> (Vec<Symbol>, Vec<usize>) {
        let levels = [-3.0, -1.0, 1.0, 3.0];
        let mut points = Vec::with_capacity(16);
        let mut gray_map = vec![0; 16];

        for (qi, &q) in levels.iter().enumerate() {
            for (ii, &i) in levels.iter().enumerate() {
                points.push(Symbol::new(i, q));
                // Gray code mapping
                let gray_i = ii ^ (ii >> 1);
                let gray_q = qi ^ (qi >> 1);
                let gray_idx = (gray_q << 2) | gray_i;
                gray_map[gray_idx] = qi * 4 + ii;
            }
        }

        (points, gray_map)
    }

    fn generate_qam32() -> (Vec<Symbol>, Vec<usize>) {
        // Rectangular 32-QAM: 4 rows x 8 columns = 32 points
        // I levels (8): -7, -5, -3, -1, 1, 3, 5, 7
        // Q levels (4): -3, -1, 1, 3
        let i_levels: [f32; 8] = [-7.0, -5.0, -3.0, -1.0, 1.0, 3.0, 5.0, 7.0];
        let q_levels: [f32; 4] = [-3.0, -1.0, 1.0, 3.0];
        let mut points = Vec::with_capacity(32);
        let mut gray_map = vec![0; 32];

        for (qi, &q) in q_levels.iter().enumerate() {
            for (ii, &i) in i_levels.iter().enumerate() {
                points.push(Symbol::new(i, q));
                // Gray code mapping: 2 bits for Q (MSB), 3 bits for I (LSB)
                let gray_i = ii ^ (ii >> 1);
                let gray_q = qi ^ (qi >> 1);
                let gray_idx = (gray_q << 3) | gray_i;
                gray_map[gray_idx] = qi * 8 + ii;
            }
        }

        (points, gray_map)
    }

    fn generate_qam64() -> (Vec<Symbol>, Vec<usize>) {
        let levels = [-7.0, -5.0, -3.0, -1.0, 1.0, 3.0, 5.0, 7.0];
        let mut points = Vec::with_capacity(64);
        let mut gray_map = vec![0; 64];

        for (qi, &q) in levels.iter().enumerate() {
            for (ii, &i) in levels.iter().enumerate() {
                points.push(Symbol::new(i, q));
                let gray_i = ii ^ (ii >> 1);
                let gray_q = qi ^ (qi >> 1);
                let gray_idx = (gray_q << 3) | gray_i;
                gray_map[gray_idx] = qi * 8 + ii;
            }
        }

        (points, gray_map)
    }

    fn generate_qam256() -> (Vec<Symbol>, Vec<usize>) {
        let levels: Vec<f32> = (-15..=15).step_by(2).map(|i| i as f32).collect();
        let mut points = Vec::with_capacity(256);
        let mut gray_map = vec![0; 256];

        for (qi, &q) in levels.iter().enumerate() {
            for (ii, &i) in levels.iter().enumerate() {
                points.push(Symbol::new(i, q));
                let gray_i = ii ^ (ii >> 1);
                let gray_q = qi ^ (qi >> 1);
                let gray_idx = (gray_q << 4) | gray_i;
                gray_map[gray_idx] = qi * 16 + ii;
            }
        }

        (points, gray_map)
    }

    /// Map bits to symbol
    ///
    /// Returns a pre-normalized symbol (unit average power across constellation).
    pub fn map(&self, bits: &[u8]) -> Symbol {
        let bits_per_sym = self.constellation_type.bits_per_symbol();
        assert!(bits.len() >= bits_per_sym);

        // Convert bits to index
        let mut idx = 0usize;
        for i in 0..bits_per_sym {
            idx = (idx << 1) | (bits[i] as usize & 1);
        }

        // Apply Gray mapping - points are already normalized
        let point_idx = self.gray_map[idx];
        self.points[point_idx]
    }

    /// Map integer value to symbol
    ///
    /// Returns a pre-normalized symbol (unit average power across constellation).
    pub fn map_value(&self, value: usize) -> Symbol {
        let point_idx = self.gray_map[value % self.points.len()];
        self.points[point_idx]
    }

    /// Demap symbol to bits (hard decision)
    ///
    /// Input symbol should be in the same normalized scale as the constellation.
    pub fn demap(&self, symbol: Symbol) -> Vec<u8> {
        let bits_per_sym = self.constellation_type.bits_per_symbol();

        // Find nearest constellation point (both symbol and points are normalized)
        let mut min_dist = f32::MAX;
        let mut nearest_idx = 0;

        for (gray_idx, &point_idx) in self.gray_map.iter().enumerate() {
            let p = &self.points[point_idx];
            let dist = (symbol.i - p.i).powi(2) + (symbol.q - p.q).powi(2);
            if dist < min_dist {
                min_dist = dist;
                nearest_idx = gray_idx;
            }
        }

        // Convert index to bits
        let mut bits = vec![0u8; bits_per_sym];
        for i in 0..bits_per_sym {
            bits[bits_per_sym - 1 - i] = ((nearest_idx >> i) & 1) as u8;
        }

        bits
    }

    /// Demap symbol to soft LLRs
    ///
    /// Input symbol and noise_var should both be in the normalized domain
    /// (unit average power). Since constellation points are pre-normalized,
    /// no scaling adjustment is needed.
    ///
    /// # Arguments
    /// * `symbol` - Received symbol (should be equalized/normalized)
    /// * `noise_var` - Noise variance measured on the normalized signal
    ///
    /// # Returns
    /// Vector of log-likelihood ratios, one per bit
    pub fn demap_soft(&self, symbol: Symbol, noise_var: f32) -> Vec<f32> {
        let bits_per_sym = self.constellation_type.bits_per_symbol();

        // Both symbol and constellation points are in normalized space
        let mut llrs = vec![0.0f32; bits_per_sym];
        let scale = 1.0 / noise_var;

        // For each bit position, find min distance for 0 and 1
        for bit_pos in 0..bits_per_sym {
            let bit_mask = 1 << (bits_per_sym - 1 - bit_pos);

            let mut min_dist_0 = f32::MAX;
            let mut min_dist_1 = f32::MAX;

            for (gray_idx, &point_idx) in self.gray_map.iter().enumerate() {
                let p = &self.points[point_idx];
                let dist = (symbol.i - p.i).powi(2) + (symbol.q - p.q).powi(2);

                if gray_idx & bit_mask == 0 {
                    min_dist_0 = min_dist_0.min(dist);
                } else {
                    min_dist_1 = min_dist_1.min(dist);
                }
            }

            // LLR = (d1^2 - d0^2) / (2 * noise_var)
            llrs[bit_pos] = (min_dist_1 - min_dist_0) * scale / 2.0;
        }

        llrs
    }

    /// Get constellation type
    pub fn constellation_type(&self) -> ConstellationType {
        self.constellation_type
    }

    /// Get all constellation points (pre-normalized to unit average power)
    pub fn points(&self) -> Vec<Symbol> {
        self.points.clone()
    }

    /// Get the normalization factor that was applied during construction
    ///
    /// This is the factor by which raw constellation coordinates were scaled
    /// to achieve unit average power.
    pub fn normalization_factor(&self) -> f32 {
        self.normalization
    }

    /// Get bits per symbol
    pub fn bits_per_symbol(&self) -> usize {
        self.constellation_type.bits_per_symbol()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_bpsk_mapping() {
        let const_map = Constellation::new(ConstellationType::Bpsk);

        let sym0 = const_map.map(&[0]);
        let sym1 = const_map.map(&[1]);

        assert!(sym0.i > 0.0);
        assert!(sym1.i < 0.0);
    }

    #[test]
    fn test_qpsk_round_trip() {
        let const_map = Constellation::new(ConstellationType::Qpsk);

        for bits in [[0, 0], [0, 1], [1, 0], [1, 1]] {
            let symbol = const_map.map(&bits);
            let recovered = const_map.demap(symbol);
            assert_eq!(recovered, bits.to_vec());
        }
    }

    #[test]
    fn test_qam16_round_trip() {
        let const_map = Constellation::new(ConstellationType::Qam16);

        for value in 0..16 {
            let bits: Vec<u8> = (0..4).rev().map(|i| ((value >> i) & 1) as u8).collect();
            let symbol = const_map.map(&bits);
            let recovered = const_map.demap(symbol);
            assert_eq!(recovered, bits, "Failed for value {}", value);
        }
    }

    #[test]
    fn test_normalization() {
        for ct in [ConstellationType::Bpsk, ConstellationType::Qpsk,
                   ConstellationType::Qam16, ConstellationType::Qam64] {
            let const_map = Constellation::new(ct);
            let points = const_map.points();

            let avg_power: f32 = points.iter()
                .map(|p| p.i * p.i + p.q * p.q)
                .sum::<f32>() / points.len() as f32;

            assert!((avg_power - 1.0).abs() < 0.01,
                "Average power for {:?} is {}", ct, avg_power);
        }
    }

    #[test]
    fn test_normalization_is_finite() {
        // Verify that all constellation types produce finite normalization values
        for ct in [ConstellationType::Bpsk, ConstellationType::Qpsk,
                   ConstellationType::Psk8, ConstellationType::Qam16,
                   ConstellationType::Qam32, ConstellationType::Qam64,
                   ConstellationType::Qam256] {
            let const_map = Constellation::new(ct);

            // Normalization should be finite (not infinity or NaN)
            assert!(const_map.normalization.is_finite(),
                "Normalization for {:?} is not finite: {}", ct, const_map.normalization);

            // Normalization should be positive
            assert!(const_map.normalization > 0.0,
                "Normalization for {:?} is not positive: {}", ct, const_map.normalization);

            // Test that mapping and demapping work without panics
            let bits: Vec<u8> = vec![0; ct.bits_per_symbol()];
            let symbol = const_map.map(&bits);

            // Symbol should have finite coordinates
            assert!(symbol.i.is_finite() && symbol.q.is_finite(),
                "Symbol for {:?} has non-finite coordinates", ct);

            // Demapping should work
            let recovered = const_map.demap(symbol);
            assert_eq!(recovered.len(), ct.bits_per_symbol());
        }
    }
}
