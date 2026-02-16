//! LDPC Encoder and Decoder
//!
//! Implements IEEE 802.11n QC-LDPC encoding and normalized min-sum decoding.
//! Provides the same interface as TurboEncoder/TurboDecoder for drop-in replacement.

use crate::fec::ldpc_matrices::{
    BLOCK_648, BLOCK_1296, BLOCK_1944,
    expansion_factor, get_base_matrix, info_bits, parity_bits,
};
use crate::fec::{CodeRate, Llr};

/// LDPC Encoder using IEEE 802.11n QC-LDPC codes
pub struct LdpcEncoder {
    /// Input block size (information bits)
    block_size: usize,
    /// LDPC codeword length (n)
    codeword_size: usize,
    /// Code rate
    rate: CodeRate,
    /// Expansion factor (Z)
    z: usize,
    /// Generator matrix (sparse representation)
    /// For systematic encoding, we only need the parity generation part
    parity_check_info: ParityCheckInfo,
}

/// LDPC Decoder using normalized min-sum algorithm
pub struct LdpcDecoder {
    /// Input block size (information bits)
    block_size: usize,
    /// LDPC codeword length (n)
    codeword_size: usize,
    /// Code rate
    rate: CodeRate,
    /// Expansion factor (Z)
    z: usize,
    /// Number of decoding iterations
    iterations: usize,
    /// Parity check matrix info
    parity_check_info: ParityCheckInfo,
    /// Normalization factor for min-sum (typically 0.75-0.875)
    normalization: f32,
}

/// Sparse representation of parity check matrix H
struct ParityCheckInfo {
    /// Number of rows in H (parity bits)
    num_rows: usize,
    /// Number of columns in H (codeword bits)
    num_cols: usize,
    /// Expansion factor Z
    z: usize,
    /// Base matrix (rows x cols), -1 = zero, 0..Z-1 = shift
    base_matrix: Vec<i16>,
    /// Number of base matrix rows
    base_rows: usize,
    /// Number of base matrix columns
    base_cols: usize,
}

impl ParityCheckInfo {
    fn new(block_size: usize, rate: CodeRate) -> Self {
        let ldpc_block = select_ldpc_block(block_size);
        let z = expansion_factor(ldpc_block);
        let (base_rows, base_cols, matrix_data) = get_base_matrix(ldpc_block, rate);

        Self {
            num_rows: base_rows * z,
            num_cols: base_cols * z,
            z,
            base_matrix: matrix_data.to_vec(),
            base_rows,
            base_cols,
        }
    }

    /// Get shift value for base matrix position (row, col)
    fn get_shift(&self, row: usize, col: usize) -> i16 {
        self.base_matrix[row * self.base_cols + col]
    }

    /// Iterator over non-zero entries in a row of H
    /// Returns (column_index, shift_value) for each non-zero ZxZ block
    fn row_entries(&self, base_row: usize) -> impl Iterator<Item = (usize, i16)> + '_ {
        (0..self.base_cols)
            .filter_map(move |col| {
                let shift = self.get_shift(base_row, col);
                if shift >= 0 {
                    Some((col, shift))
                } else {
                    None
                }
            })
    }

    /// Iterator over non-zero entries in a column of H
    fn col_entries(&self, base_col: usize) -> impl Iterator<Item = (usize, i16)> + '_ {
        (0..self.base_rows)
            .filter_map(move |row| {
                let shift = self.get_shift(row, base_col);
                if shift >= 0 {
                    Some((row, shift))
                } else {
                    None
                }
            })
    }
}

/// Select appropriate LDPC block size for given input
fn select_ldpc_block(input_bits: usize) -> usize {
    // Map input block size to nearest LDPC block
    // We need to fit input_bits into the information portion (k bits)
    // For rate 1/2: k = n/2, so n = 2*input_bits
    // But we use fixed block sizes, so find smallest that fits

    if input_bits <= 324 {
        BLOCK_648
    } else if input_bits <= 648 {
        BLOCK_1296
    } else {
        BLOCK_1944
    }
}

impl LdpcEncoder {
    /// Create a new LDPC encoder with default rate 1/2
    pub fn new(block_size: usize) -> Self {
        Self::new_with_rate(block_size, CodeRate::Rate1_2)
    }

    /// Create a new LDPC encoder with specified rate
    /// Uses LDPC block 1944 for multi-block encoding of larger frames
    pub fn new_with_rate(block_size: usize, rate: CodeRate) -> Self {
        // Always use largest block for consistency
        let ldpc_block = BLOCK_1944;
        let z = expansion_factor(ldpc_block);
        let _k = info_bits(ldpc_block, rate);
        let parity_check_info = ParityCheckInfo::new(ldpc_block, rate);

        Self {
            block_size,
            codeword_size: ldpc_block,
            rate,
            z,
            parity_check_info,
        }
    }

    /// Get the block size (information bits)
    pub fn block_size(&self) -> usize {
        self.block_size
    }

    /// Get the codeword size per LDPC block
    pub fn codeword_size(&self) -> usize {
        self.codeword_size
    }

    /// Get the total encoded output size for this block_size
    pub fn total_encoded_bits(&self) -> usize {
        let num_codewords = self.rate.ldpc_codeword_count(self.block_size);
        num_codewords * self.codeword_size
    }

    /// Encode information bits to LDPC codewords
    /// Input: block_size bits (will be zero-padded if needed)
    /// Output: LDPC codewords sized to match OFDM frame capacity
    pub fn encode(&self, bits: &[u8]) -> Vec<u8> {
        let k = info_bits(self.codeword_size, self.rate);
        let n = self.codeword_size;

        // Calculate number of codewords based on OFDM frame capacity
        // (matches Turbo output size for frame compatibility)
        let num_codewords = self.rate.ldpc_codeword_count(self.block_size);

        let mut output = Vec::with_capacity(num_codewords * n);

        // Encode each block
        for cw_idx in 0..num_codewords {
            let start = cw_idx * k;
            let end = ((cw_idx + 1) * k).min(bits.len());

            // Extract and pad this chunk
            let mut info_chunk = vec![0u8; k];
            if start < bits.len() {
                let copy_len = (end - start).min(k);
                info_chunk[..copy_len].copy_from_slice(&bits[start..start + copy_len]);
            }

            // Encode this chunk
            let parity = self.compute_parity(&info_chunk);

            // Append [info | parity]
            output.extend_from_slice(&info_chunk);
            output.extend_from_slice(&parity);
        }

        output
    }

    /// Compute parity bits using Richardson-Urbanke algorithm
    ///
    /// The parity-check matrix H is partitioned as:
    /// H = [A | B | T]   where T is lower triangular
    ///     [C | D | E]
    ///
    /// For a codeword c = [s | p1 | p2]:
    /// - s is the systematic (information) part
    /// - p1 has length g (gap)
    /// - p2 has length m-g
    ///
    /// Encoding steps:
    /// 1. Compute phi = -E*T^(-1)*B + D
    /// 2. p1^T = phi^(-1) * (-E*T^(-1)*A + C) * s^T
    /// 3. p2^T = -T^(-1) * (A*s^T + B*p1^T)
    fn compute_parity(&self, info: &[u8]) -> Vec<u8> {
        let k = info_bits(self.codeword_size, self.rate);
        let m = parity_bits(self.codeword_size, self.rate);
        let z = self.z;
        let h = &self.parity_check_info;
        let base_rows = h.base_rows;
        let _base_cols = h.base_cols;
        let info_base_cols = k / z;
        let parity_base_cols = base_rows; // m/z

        // For 802.11n codes, the parity sub-matrix has approximately lower triangular
        // structure. We'll use direct Gaussian elimination on the expanded matrix.

        // Step 1: Build the expanded parity sub-matrix H_p (m x m)
        // For QC-LDPC, entry (base_row, base_col) with shift s means:
        // H_expanded[base_row*z + (j+s)%z, base_col*z + j] = 1 for j = 0..z-1
        let mut h_parity = vec![vec![0u8; m]; m];

        for base_row in 0..base_rows {
            for parity_idx in 0..parity_base_cols {
                let base_col = info_base_cols + parity_idx;
                let shift = h.get_shift(base_row, base_col);
                if shift >= 0 {
                    let shift = shift as usize;
                    for j in 0..z {
                        let row = base_row * z + ((j + shift) % z);
                        let col = parity_idx * z + j;
                        if row < m && col < m {
                            h_parity[row][col] = 1;
                        }
                    }
                }
            }
        }

        // Step 2: Compute syndrome = H_info * info
        let mut syndrome = vec![0u8; m];
        for base_row in 0..base_rows {
            for base_col in 0..info_base_cols {
                let shift = h.get_shift(base_row, base_col);
                if shift >= 0 {
                    let shift = shift as usize;
                    for j in 0..z {
                        let info_idx = base_col * z + j;
                        let syndrome_idx = base_row * z + ((j + shift) % z);
                        if info_idx < info.len() && syndrome_idx < m {
                            syndrome[syndrome_idx] ^= info[info_idx];
                        }
                    }
                }
            }
        }

        // Step 3: Solve H_parity * parity = syndrome using Gaussian elimination
        // Build augmented matrix [H_parity | syndrome]
        let mut aug = vec![vec![0u8; m + 1]; m];
        for i in 0..m {
            for j in 0..m {
                aug[i][j] = h_parity[i][j];
            }
            aug[i][m] = syndrome[i];
        }

        // Forward elimination with partial pivoting
        let mut pivot_col = vec![0usize; m]; // Track which column each row pivots on
        let mut row_order = vec![0usize; m]; // Track row permutation
        for i in 0..m {
            row_order[i] = i;
        }

        for col in 0..m {
            // Find pivot row (first row with 1 in this column, starting from col)
            let mut pivot_row = None;
            for row in col..m {
                if aug[row][col] == 1 {
                    pivot_row = Some(row);
                    break;
                }
            }

            if let Some(pr) = pivot_row {
                // Swap rows if needed
                if pr != col {
                    aug.swap(col, pr);
                    row_order.swap(col, pr);
                }
                pivot_col[col] = col;

                // Eliminate all other rows (both above and below for reduced form)
                for row in 0..m {
                    if row != col && aug[row][col] == 1 {
                        for c in 0..=m {
                            aug[row][c] ^= aug[col][c];
                        }
                    }
                }
            }
        }

        // Extract solution (parity bits)
        let mut parity = vec![0u8; m];
        for row in 0..m {
            // Find the pivot column for this row
            for col in 0..m {
                if aug[row][col] == 1 {
                    // Check if this is the only 1 in the row (pivot)
                    let mut is_pivot = true;
                    for c in 0..m {
                        if c != col && aug[row][c] == 1 {
                            is_pivot = false;
                            break;
                        }
                    }
                    if is_pivot {
                        parity[col] = aug[row][m];
                    }
                    break;
                }
            }
        }

        parity
    }
}

impl LdpcDecoder {
    /// Create a new LDPC decoder with default rate 1/2
    pub fn new(block_size: usize) -> Self {
        Self::new_with_rate(block_size, CodeRate::Rate1_2)
    }

    /// Create a new LDPC decoder with specified rate
    /// Uses LDPC block 1944 for multi-block decoding of larger frames
    pub fn new_with_rate(block_size: usize, rate: CodeRate) -> Self {
        // Always use largest block for consistency
        let ldpc_block = BLOCK_1944;
        let z = expansion_factor(ldpc_block);
        let parity_check_info = ParityCheckInfo::new(ldpc_block, rate);

        Self {
            block_size,
            codeword_size: ldpc_block,
            rate,
            z,
            iterations: 30, // Default iterations for LDPC
            parity_check_info,
            normalization: 0.75, // Typical normalization factor
        }
    }

    /// Get the block size (information bits to output)
    pub fn block_size(&self) -> usize {
        self.block_size
    }

    /// Get the codeword size per LDPC block
    pub fn codeword_size(&self) -> usize {
        self.codeword_size
    }

    /// Get the total expected input LLRs for this block_size
    pub fn total_encoded_bits(&self) -> usize {
        // Use same calculation as encoder for consistency
        let num_codewords = self.rate.ldpc_codeword_count(self.block_size);
        num_codewords * self.codeword_size
    }

    /// Set number of decoding iterations
    pub fn set_iterations(&mut self, iterations: usize) {
        self.iterations = iterations.clamp(1, 100);
    }

    /// Set normalization factor for min-sum
    pub fn set_normalization(&mut self, factor: f32) {
        self.normalization = factor.clamp(0.5, 1.0);
    }

    /// Decode received LLRs to information bits
    /// Input: Multiple concatenated LDPC codewords as LLRs
    /// Output: block_size bits
    pub fn decode(&self, llrs: &[Llr]) -> Vec<u8> {
        let n = self.codeword_size;
        let k = info_bits(n, self.rate);

        // Calculate number of codewords (matches encoder)
        let num_codewords = self.rate.ldpc_codeword_count(self.block_size);

        let mut output = Vec::with_capacity(self.block_size);

        // Decode each codeword
        for cw_idx in 0..num_codewords {
            let start = cw_idx * n;
            let end = start + n;

            // Extract LLRs for this codeword
            let mut cw_llrs = vec![0.0f32; n];
            if start < llrs.len() {
                let copy_len = (end - start).min(llrs.len() - start);
                cw_llrs[..copy_len].copy_from_slice(&llrs[start..start + copy_len]);
            }

            // Decode this codeword
            let decoded = self.decode_single_block(&cw_llrs);

            // Append info bits (first k bits of decoded)
            let take = (self.block_size - output.len()).min(k);
            output.extend_from_slice(&decoded[..take]);
        }

        output
    }

    /// Decode a single LDPC codeword
    fn decode_single_block(&self, llrs: &[Llr]) -> Vec<u8> {
        let n = self.codeword_size;
        let k = info_bits(n, self.rate);
        let _m = parity_bits(n, self.rate);
        let z = self.z;
        let h = &self.parity_check_info;

        // Copy LLRs
        let mut channel_llrs = vec![0.0f32; n];
        let copy_len = llrs.len().min(n);
        channel_llrs[..copy_len].copy_from_slice(&llrs[..copy_len]);

        // Initialize variable-to-check messages (Q) with channel LLRs
        // Q[var][check] = message from variable var to check check
        // For efficiency, we store as: q_vc[base_col][base_row][z_idx]

        // Initialize check-to-variable messages (R) to 0
        // R[check][var] = message from check check to variable var

        // Using sparse representation based on H structure
        let mut q_vc: Vec<Vec<Vec<f32>>> = vec![vec![vec![0.0; z]; h.base_rows]; h.base_cols];
        let mut r_cv: Vec<Vec<Vec<f32>>> = vec![vec![vec![0.0; z]; h.base_cols]; h.base_rows];

        // Initialize Q messages with channel LLRs
        for base_col in 0..h.base_cols {
            for &(base_row, _shift) in &h.col_entries(base_col).collect::<Vec<_>>() {
                for zi in 0..z {
                    let var_idx = base_col * z + zi;
                    q_vc[base_col][base_row][zi] = channel_llrs[var_idx];
                }
            }
        }

        // Iterative decoding
        for _iter in 0..self.iterations {
            // Check node update (horizontal step)
            // R[c][v] = sign(product of Q) * min(|Q[v'][c]| for v' != v)
            for base_row in 0..h.base_rows {
                let row_entries: Vec<(usize, i16)> = h.row_entries(base_row).collect();

                for zi in 0..z {
                    // Compute product of signs and minimum magnitudes
                    let mut signs: Vec<f32> = Vec::with_capacity(row_entries.len());
                    let mut mags: Vec<f32> = Vec::with_capacity(row_entries.len());

                    for &(base_col, shift) in &row_entries {
                        let shifted_zi = (zi + z - shift as usize) % z;
                        let q = q_vc[base_col][base_row][shifted_zi];
                        signs.push(q.signum());
                        mags.push(q.abs());
                    }

                    // Compute output for each variable
                    for (idx, &(base_col, shift)) in row_entries.iter().enumerate() {
                        // Product of all signs except this one
                        let mut sign_prod = 1.0f32;
                        for (j, &s) in signs.iter().enumerate() {
                            if j != idx {
                                sign_prod *= s;
                            }
                        }

                        // Minimum of all magnitudes except this one
                        let mut min_mag = f32::MAX;
                        for (j, &m) in mags.iter().enumerate() {
                            if j != idx && m < min_mag {
                                min_mag = m;
                            }
                        }
                        if min_mag == f32::MAX {
                            min_mag = 0.0;
                        }

                        // Normalized min-sum
                        let r = sign_prod * min_mag * self.normalization;
                        let shifted_zi = (zi + z - shift as usize) % z;
                        r_cv[base_row][base_col][shifted_zi] = r;
                    }
                }
            }

            // Variable node update (vertical step)
            // Q[v][c] = channel_llr[v] + sum(R[c'][v] for c' != c)
            // r_cv is indexed by variable position: r_cv[base_row][base_col][var_idx_in_block]
            for base_col in 0..h.base_cols {
                let col_entries: Vec<(usize, i16)> = h.col_entries(base_col).collect();

                for zi in 0..z {
                    let var_idx = base_col * z + zi;
                    let channel = channel_llrs[var_idx];

                    // Sum of all R messages to this variable (variable index is zi)
                    let mut r_sum = 0.0f32;
                    for &(base_row, _shift) in &col_entries {
                        r_sum += r_cv[base_row][base_col][zi];
                    }

                    // Update Q messages (excluding each check's own R)
                    for &(base_row, _shift) in &col_entries {
                        let r_self = r_cv[base_row][base_col][zi];
                        q_vc[base_col][base_row][zi] = channel + r_sum - r_self;
                    }
                }
            }

            // Early termination: check syndrome
            if self.check_syndrome(&channel_llrs, &r_cv, h, z) {
                break;
            }
        }

        // Final decision - extract k info bits from this codeword
        let mut decisions = Vec::with_capacity(k);
        for base_col in 0..(k / z) {
            let col_entries: Vec<(usize, i16)> = h.col_entries(base_col).collect();

            for zi in 0..z {
                let var_idx = base_col * z + zi;
                if var_idx >= k {
                    break;
                }

                // Sum channel LLR + all R messages (r_cv indexed by variable position)
                let mut total_llr = channel_llrs[var_idx];
                for &(base_row, _shift) in &col_entries {
                    total_llr += r_cv[base_row][base_col][zi];
                }

                decisions.push(if total_llr >= 0.0 { 0 } else { 1 });
            }
        }

        // Ensure exactly k bits
        decisions.resize(k, 0);
        decisions
    }

    /// Check if syndrome is zero (valid codeword)
    fn check_syndrome(
        &self,
        channel_llrs: &[f32],
        r_cv: &[Vec<Vec<f32>>],
        h: &ParityCheckInfo,
        z: usize,
    ) -> bool {
        let n = self.codeword_size;

        // Compute hard decisions
        let mut hard = vec![0u8; n];
        for base_col in 0..h.base_cols {
            let col_entries: Vec<(usize, i16)> = h.col_entries(base_col).collect();

            for zi in 0..z {
                let var_idx = base_col * z + zi;
                if var_idx >= n {
                    break;
                }

                let mut total = channel_llrs[var_idx];
                for &(base_row, _shift) in &col_entries {
                    total += r_cv[base_row][base_col][zi];
                }

                hard[var_idx] = if total >= 0.0 { 0 } else { 1 };
            }
        }

        // Check syndrome: H * c^T = 0
        for base_row in 0..h.base_rows {
            for zi in 0..z {
                let mut sum = 0u8;
                for (base_col, shift) in h.row_entries(base_row) {
                    let shifted_zi = (zi + z - shift as usize) % z;
                    let var_idx = base_col * z + shifted_zi;
                    if var_idx < n {
                        sum ^= hard[var_idx];
                    }
                }
                if sum != 0 {
                    return false;
                }
            }
        }

        true
    }
}

/// Verify that a codeword has zero syndrome (H*c = 0)
fn verify_codeword(codeword: &[u8], h: &ParityCheckInfo, z: usize) -> bool {
    let base_rows = h.base_rows;
    let base_cols = h.base_cols;

    for base_row in 0..base_rows {
        for zi in 0..z {
            let mut sum = 0u8;
            for base_col in 0..base_cols {
                let shift = h.get_shift(base_row, base_col);
                if shift >= 0 {
                    let shift = shift as usize;
                    let var_idx = base_col * z + ((zi + z - shift) % z);
                    if var_idx < codeword.len() {
                        sum ^= codeword[var_idx];
                    }
                }
            }
            if sum != 0 {
                return false;
            }
        }
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_encoder_creation() {
        let encoder = LdpcEncoder::new(288);
        assert_eq!(encoder.block_size(), 288);
        // Always uses largest block for multi-block support
        assert_eq!(encoder.codeword_size(), BLOCK_1944);

        let encoder = LdpcEncoder::new_with_rate(500, CodeRate::Rate3_4);
        assert_eq!(encoder.block_size(), 500);
        assert_eq!(encoder.codeword_size(), BLOCK_1944);
    }

    #[test]
    fn test_decoder_creation() {
        let decoder = LdpcDecoder::new(288);
        assert_eq!(decoder.block_size(), 288);
        // Always uses largest block for multi-block support
        assert_eq!(decoder.codeword_size(), BLOCK_1944);

        let mut decoder = LdpcDecoder::new_with_rate(500, CodeRate::Rate2_3);
        decoder.set_iterations(50);
        assert_eq!(decoder.iterations, 50);
    }

    #[test]
    fn test_syndrome_check() {
        // Test that encoded codewords have zero syndrome
        let block_size = 288;
        let encoder = LdpcEncoder::new_with_rate(block_size, CodeRate::Rate1_2);

        // All zeros
        let zeros = vec![0u8; block_size];
        let encoded = encoder.encode(&zeros);

        let valid = verify_codeword(&encoded, &encoder.parity_check_info, encoder.z);
        assert!(valid, "All-zeros codeword should have zero syndrome");

        // Pattern
        let pattern: Vec<u8> = (0..block_size).map(|i| ((i * 7) % 2) as u8).collect();
        let encoded = encoder.encode(&pattern);
        let valid = verify_codeword(&encoded, &encoder.parity_check_info, encoder.z);
        assert!(valid, "Pattern codeword should have zero syndrome");
    }

    #[test]
    fn test_encode_decode_roundtrip() {
        let block_size = 288;
        let encoder = LdpcEncoder::new_with_rate(block_size, CodeRate::Rate1_2);
        let mut decoder = LdpcDecoder::new_with_rate(block_size, CodeRate::Rate1_2);
        decoder.set_iterations(30);

        // Create test data
        let mut input = vec![0u8; block_size];
        for i in 0..block_size {
            input[i] = ((i * 7 + 3) % 2) as u8;
        }

        // Encode
        let encoded = encoder.encode(&input);
        assert_eq!(encoded.len(), encoder.codeword_size());

        // Verify syndrome is zero
        assert!(
            verify_codeword(&encoded, &encoder.parity_check_info, encoder.z),
            "Encoded codeword should have zero syndrome"
        );

        // Convert to LLRs (no noise, perfect channel)
        let llrs: Vec<f32> = encoded
            .iter()
            .map(|&b| if b == 0 { 10.0 } else { -10.0 })
            .collect();

        // Decode
        let decoded = decoder.decode(&llrs);

        // Compare (first block_size bits should match)
        for i in 0..block_size {
            assert_eq!(
                decoded[i], input[i],
                "Mismatch at bit {}: expected {}, got {}",
                i, input[i], decoded[i]
            );
        }
    }

    #[test]
    fn test_all_rates() {
        let rates = [
            CodeRate::Rate1_2,
            CodeRate::Rate2_3,
            CodeRate::Rate3_4,
            CodeRate::Rate5_6,
        ];

        for rate in rates {
            let block_size = 288;
            let encoder = LdpcEncoder::new_with_rate(block_size, rate);
            let mut decoder = LdpcDecoder::new_with_rate(block_size, rate);
            decoder.set_iterations(30);

            // Random-ish test data
            let input: Vec<u8> = (0..block_size).map(|i| ((i * 13) % 2) as u8).collect();

            let encoded = encoder.encode(&input);
            let llrs: Vec<f32> = encoded
                .iter()
                .map(|&b| if b == 0 { 8.0 } else { -8.0 })
                .collect();

            let decoded = decoder.decode(&llrs);

            for i in 0..block_size {
                assert_eq!(
                    decoded[i], input[i],
                    "Rate {:?}: mismatch at bit {}",
                    rate, i
                );
            }
        }
    }

    #[test]
    fn test_all_block_sizes() {
        let block_sizes = [100, 288, 400, 648, 800, 972];

        for &block_size in &block_sizes {
            let encoder = LdpcEncoder::new_with_rate(block_size, CodeRate::Rate1_2);
            let mut decoder = LdpcDecoder::new_with_rate(block_size, CodeRate::Rate1_2);
            decoder.set_iterations(30);

            let input: Vec<u8> = (0..block_size).map(|i| ((i * 11) % 2) as u8).collect();

            let encoded = encoder.encode(&input);
            let llrs: Vec<f32> = encoded
                .iter()
                .map(|&b| if b == 0 { 8.0 } else { -8.0 })
                .collect();

            let decoded = decoder.decode(&llrs);

            for i in 0..block_size {
                assert_eq!(
                    decoded[i], input[i],
                    "Block size {}: mismatch at bit {}",
                    block_size, i
                );
            }
        }
    }

    #[test]
    fn test_noisy_channel() {
        let block_size = 288;
        let encoder = LdpcEncoder::new_with_rate(block_size, CodeRate::Rate1_2);
        let mut decoder = LdpcDecoder::new_with_rate(block_size, CodeRate::Rate1_2);
        decoder.set_iterations(50);

        // All zeros
        let input = vec![0u8; block_size];
        let encoded = encoder.encode(&input);

        // Add some noise (flip some LLRs to weak values)
        let mut llrs: Vec<f32> = encoded
            .iter()
            .map(|&b| if b == 0 { 3.0 } else { -3.0 })
            .collect();

        // Corrupt a few bits (weak LLRs)
        for i in (0..llrs.len()).step_by(50) {
            llrs[i] *= -0.5; // Flip sign and weaken
        }

        let decoded = decoder.decode(&llrs);

        // Should still decode correctly due to error correction
        let errors: usize = (0..block_size)
            .filter(|&i| decoded[i] != input[i])
            .count();

        assert!(
            errors < block_size / 10,
            "Too many errors: {} out of {}",
            errors,
            block_size
        );
    }

    #[test]
    fn test_multi_block_encoding() {
        // Test large block sizes that require multiple LDPC codewords
        // Mode 12 at 2300Hz with LDPC has fec_block_bits = 2916 (2 codewords × k=1458)
        let block_size = 2916;
        let encoder = LdpcEncoder::new_with_rate(block_size, CodeRate::Rate3_4);
        let mut decoder = LdpcDecoder::new_with_rate(block_size, CodeRate::Rate3_4);
        decoder.set_iterations(30);

        // Pattern input
        let input: Vec<u8> = (0..block_size).map(|i| ((i * 13) % 2) as u8).collect();
        let encoded = encoder.encode(&input);

        // Convert to LLRs (clean channel)
        let llrs: Vec<f32> = encoded
            .iter()
            .map(|&b| if b == 0 { 10.0 } else { -10.0 })
            .collect();

        let decoded = decoder.decode(&llrs);

        // Verify all bits match
        for i in 0..block_size {
            assert_eq!(
                decoded[i], input[i],
                "Multi-block: mismatch at bit {} (expected {}, got {})",
                i, input[i], decoded[i]
            );
        }
    }
}
