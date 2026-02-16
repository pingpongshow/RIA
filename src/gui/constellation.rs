//! Constellation/Eye Pattern display for signal quality visualization
//!
//! Supports two modes:
//! - Traditional IQ constellation (shows modulation symbol positions)
//! - Eye pattern (shows correlation, diagonal = good signal)

use eframe::egui::{self, Color32, Pos2, Rect, Stroke, Ui};

use super::colors;

/// Display mode for constellation
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConstellationMode {
    /// Traditional IQ constellation (symbol positions)
    IqConstellation,
    /// Eye pattern (correlation plot)
    EyePattern,
}

/// Constellation/Eye Pattern display
pub struct ConstellationDisplay {
    points: Vec<(f32, f32)>,
    max_points: usize,
    point_size: f32,
    mode: ConstellationMode,
    // For eye pattern: store previous sample for correlation
    prev_sample: f32,
    // Signal quality metric (0.0 = noise, 1.0 = perfect)
    signal_quality: f32,
    // Detected cluster centers (for IQ mode)
    cluster_centers: Vec<(f32, f32)>,
    // Expected constellation size (4=QPSK, 8=8PSK, 16=16QAM, etc.)
    expected_constellation_size: usize,
}

impl ConstellationDisplay {
    /// Create new constellation display
    pub fn new() -> Self {
        Self {
            points: Vec::new(),
            max_points: 500,
            point_size: 2.0,
            mode: ConstellationMode::EyePattern, // Default to eye pattern
            prev_sample: 0.0,
            signal_quality: 0.0,
            cluster_centers: Vec::new(),
            expected_constellation_size: 4, // Default to QPSK
        }
    }

    /// Set expected constellation size (4=QPSK, 8=8PSK, 16=16QAM, 64=64QAM)
    pub fn set_constellation_size(&mut self, size: usize) {
        self.expected_constellation_size = size.clamp(2, 64);
        self.cluster_centers.clear();
    }

    /// Set display mode
    pub fn set_mode(&mut self, mode: ConstellationMode) {
        self.mode = mode;
        self.points.clear();
    }

    /// Set constellation points (for IQ mode)
    pub fn set_points(&mut self, points: &[(f32, f32)]) {
        self.points.clear();
        self.points.extend_from_slice(points);

        if self.points.len() > self.max_points {
            let start = self.points.len() - self.max_points;
            self.points = self.points[start..].to_vec();
        }

        self.update_signal_quality();
    }

    /// Add a single IQ point
    pub fn add_point(&mut self, i: f32, q: f32) {
        self.points.push((i, q));

        if self.points.len() > self.max_points {
            self.points.remove(0);
        }

        self.update_signal_quality();
    }

    /// Add sample for eye pattern mode (correlates consecutive samples)
    pub fn add_sample(&mut self, sample: f32) {
        if self.mode == ConstellationMode::EyePattern {
            // Eye pattern plots current vs previous sample
            // Good signal: strong correlation = diagonal line
            // Noise: no correlation = scattered
            let x = self.prev_sample;
            let y = sample;
            self.points.push((x, y));
            self.prev_sample = sample;

            if self.points.len() > self.max_points {
                self.points.remove(0);
            }

            self.update_signal_quality();
        }
    }

    /// Add batch of samples for eye pattern
    pub fn add_samples(&mut self, samples: &[f32]) {
        for &sample in samples {
            self.add_sample(sample);
        }
    }

    /// Calculate signal quality from point clustering
    fn update_signal_quality(&mut self) {
        if self.points.len() < 10 {
            self.signal_quality = 0.0;
            return;
        }

        match self.mode {
            ConstellationMode::EyePattern => {
                // For eye pattern: measure correlation coefficient
                // Perfect diagonal = correlation of 1.0
                let n = self.points.len() as f32;
                let sum_x: f32 = self.points.iter().map(|(x, _)| x).sum();
                let sum_y: f32 = self.points.iter().map(|(_, y)| y).sum();
                let sum_xy: f32 = self.points.iter().map(|(x, y)| x * y).sum();
                let sum_x2: f32 = self.points.iter().map(|(x, _)| x * x).sum();
                let sum_y2: f32 = self.points.iter().map(|(_, y)| y * y).sum();

                let numerator = n * sum_xy - sum_x * sum_y;
                let denominator = ((n * sum_x2 - sum_x * sum_x) * (n * sum_y2 - sum_y * sum_y)).sqrt();

                if denominator > 0.0001 {
                    self.signal_quality = (numerator / denominator).abs().clamp(0.0, 1.0);
                } else {
                    self.signal_quality = 0.0;
                }
            }
            ConstellationMode::IqConstellation => {
                // For IQ: use k-means clustering to find actual cluster centers
                // then measure average distance to nearest cluster
                self.cluster_centers = self.find_cluster_centers(self.expected_constellation_size);

                if self.cluster_centers.is_empty() {
                    self.signal_quality = 0.0;
                    return;
                }

                // Calculate average distance to nearest cluster center
                let avg_distance: f32 = self.points.iter()
                    .map(|(x, y)| {
                        self.cluster_centers.iter()
                            .map(|(cx, cy)| ((x - cx).powi(2) + (y - cy).powi(2)).sqrt())
                            .fold(f32::MAX, f32::min)
                    })
                    .sum::<f32>() / self.points.len() as f32;

                // Calculate expected distance for normalization based on constellation size
                // For QPSK (4 points), ideal spacing is ~1.41 (sqrt(2))
                // Error Magnitude Vector (EVM) style: smaller distance = better
                let expected_spacing = match self.expected_constellation_size {
                    2 => 2.0,      // BPSK
                    4 => 1.41,     // QPSK
                    8 => 0.77,     // 8PSK
                    16 => 0.63,    // 16QAM
                    64 => 0.29,    // 64QAM
                    _ => 1.0,
                };

                // Normalize: 0 distance = 1.0 quality, expected_spacing/2 = 0.0 quality
                let max_acceptable_error = expected_spacing / 2.0;
                self.signal_quality = (1.0 - avg_distance / max_acceptable_error).clamp(0.0, 1.0);
            }
        }
    }

    /// Get current signal quality estimate (0.0-1.0)
    pub fn signal_quality(&self) -> f32 {
        self.signal_quality
    }

    /// Get detected cluster centers
    pub fn cluster_centers(&self) -> &[(f32, f32)] {
        &self.cluster_centers
    }

    /// Find cluster centers using k-means clustering
    fn find_cluster_centers(&self, k: usize) -> Vec<(f32, f32)> {
        if self.points.len() < k * 2 {
            return Vec::new();
        }

        // Initialize cluster centers using k-means++ initialization
        let mut centers = self.kmeans_plus_plus_init(k);

        // Run k-means iterations (max 20 iterations)
        for _ in 0..20 {
            // Assign points to nearest cluster
            let mut cluster_sums: Vec<(f32, f32, usize)> = vec![(0.0, 0.0, 0); k];

            for &(x, y) in &self.points {
                // Find nearest cluster
                let mut min_dist = f32::MAX;
                let mut nearest = 0;
                for (i, &(cx, cy)) in centers.iter().enumerate() {
                    let dist = (x - cx).powi(2) + (y - cy).powi(2);
                    if dist < min_dist {
                        min_dist = dist;
                        nearest = i;
                    }
                }

                // Add to cluster sum
                cluster_sums[nearest].0 += x;
                cluster_sums[nearest].1 += y;
                cluster_sums[nearest].2 += 1;
            }

            // Update centers
            let mut converged = true;
            for (i, (sum_x, sum_y, count)) in cluster_sums.iter().enumerate() {
                if *count > 0 {
                    let new_x = sum_x / *count as f32;
                    let new_y = sum_y / *count as f32;

                    // Check for convergence (centers moved less than 0.01)
                    let dx = new_x - centers[i].0;
                    let dy = new_y - centers[i].1;
                    if dx.abs() > 0.01 || dy.abs() > 0.01 {
                        converged = false;
                    }

                    centers[i] = (new_x, new_y);
                }
            }

            if converged {
                break;
            }
        }

        // Remove empty clusters (centers that have no points)
        centers.retain(|&(cx, cy)| {
            self.points.iter().any(|&(x, y)| {
                let dist = (x - cx).powi(2) + (y - cy).powi(2);
                dist < 1.0 // At least one point within distance 1.0
            })
        });

        centers
    }

    /// K-means++ initialization for better cluster center selection
    fn kmeans_plus_plus_init(&self, k: usize) -> Vec<(f32, f32)> {
        if self.points.is_empty() {
            return Vec::new();
        }

        let mut centers = Vec::with_capacity(k);

        // First center: random point (use deterministic selection based on point count)
        let first_idx = self.points.len() / 2;
        centers.push(self.points[first_idx]);

        // Remaining centers: choose points with probability proportional to squared distance
        for _ in 1..k {
            // Calculate squared distances to nearest center for all points
            let distances: Vec<f32> = self.points.iter()
                .map(|&(x, y)| {
                    centers.iter()
                        .map(|&(cx, cy)| (x - cx).powi(2) + (y - cy).powi(2))
                        .fold(f32::MAX, f32::min)
                })
                .collect();

            let total_dist: f32 = distances.iter().sum();
            if total_dist < 0.0001 {
                break; // All points are at existing centers
            }

            // Select point with probability proportional to distance squared
            // Use a deterministic approach: pick the point with maximum distance
            let max_idx = distances.iter()
                .enumerate()
                .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal))
                .map(|(i, _)| i)
                .unwrap_or(0);

            centers.push(self.points[max_idx]);
        }

        centers
    }

    /// Paint the display
    pub fn paint(&self, ui: &Ui, rect: Rect) {
        let painter = ui.painter_at(rect);

        // Navy blue background
        painter.rect_filled(rect, 0.0, Color32::from_rgb(0, 0, 80));

        // Draw grid
        let grid_color = Color32::from_rgb(180, 180, 180);
        let num_divisions = 10;

        for i in 1..num_divisions {
            let t = i as f32 / num_divisions as f32;

            // Vertical lines
            let x = rect.left() + t * rect.width();
            painter.line_segment(
                [Pos2::new(x, rect.top()), Pos2::new(x, rect.bottom())],
                Stroke::new(1.0, grid_color),
            );

            // Horizontal lines
            let y = rect.top() + t * rect.height();
            painter.line_segment(
                [Pos2::new(rect.left(), y), Pos2::new(rect.right(), y)],
                Stroke::new(1.0, grid_color),
            );
        }

        // Draw axis labels (0.0, 0.5, 1.0)
        let label_color = Color32::from_rgb(200, 200, 200);
        for (i, label) in ["0.0", "0.5", "1.0"].iter().enumerate() {
            let t = i as f32 / 2.0;

            // Top labels (X axis)
            painter.text(
                Pos2::new(rect.left() + t * rect.width(), rect.top() + 3.0),
                egui::Align2::CENTER_TOP,
                *label,
                egui::FontId::proportional(10.0),
                label_color,
            );

            // Right labels (Y axis)
            painter.text(
                Pos2::new(rect.right() - 3.0, rect.bottom() - t * rect.height()),
                egui::Align2::RIGHT_CENTER,
                *label,
                egui::FontId::proportional(10.0),
                label_color,
            );
        }

        // Draw points
        if !self.points.is_empty() {
            for (idx, &(x, y)) in self.points.iter().enumerate() {
                // Normalize to 0-1 range
                let norm_x = match self.mode {
                    ConstellationMode::EyePattern => (x + 1.0) / 2.0, // Assume -1 to 1 range
                    ConstellationMode::IqConstellation => (x + 1.5) / 3.0, // Wider range for constellation
                };
                let norm_y = match self.mode {
                    ConstellationMode::EyePattern => (y + 1.0) / 2.0,
                    ConstellationMode::IqConstellation => (y + 1.5) / 3.0,
                };

                let pos = Pos2::new(
                    rect.left() + norm_x.clamp(0.0, 1.0) * rect.width(),
                    rect.bottom() - norm_y.clamp(0.0, 1.0) * rect.height(),
                );

                // Fade older points, newer points are brighter (white)
                let age = idx as f32 / self.points.len() as f32;
                let alpha = (age * 200.0) as u8 + 55;

                // White dots
                let color = Color32::from_rgba_unmultiplied(255, 255, 255, alpha);

                painter.circle_filled(pos, self.point_size, color);
            }
        }

        // Mode-specific overlays
        match self.mode {
            ConstellationMode::EyePattern => {
                // Draw diagonal reference line
                // Perfect correlation would be along this diagonal
                painter.line_segment(
                    [
                        Pos2::new(rect.left(), rect.bottom()),
                        Pos2::new(rect.right(), rect.top()),
                    ],
                    Stroke::new(1.0, Color32::from_rgba_unmultiplied(200, 200, 200, 100)),
                );
            }
            ConstellationMode::IqConstellation => {
                // Draw detected cluster centers as crosses
                for &(cx, cy) in &self.cluster_centers {
                    let norm_x = (cx + 1.5) / 3.0;
                    let norm_y = (cy + 1.5) / 3.0;

                    let pos = Pos2::new(
                        rect.left() + norm_x.clamp(0.0, 1.0) * rect.width(),
                        rect.bottom() - norm_y.clamp(0.0, 1.0) * rect.height(),
                    );

                    // Draw cross marker for cluster center
                    let cross_size = 5.0;
                    let cross_color = Color32::from_rgb(255, 100, 100);
                    painter.line_segment(
                        [
                            Pos2::new(pos.x - cross_size, pos.y),
                            Pos2::new(pos.x + cross_size, pos.y),
                        ],
                        Stroke::new(2.0, cross_color),
                    );
                    painter.line_segment(
                        [
                            Pos2::new(pos.x, pos.y - cross_size),
                            Pos2::new(pos.x, pos.y + cross_size),
                        ],
                        Stroke::new(2.0, cross_color),
                    );
                }

                // Draw crosshairs at center for reference
                let center_x = rect.left() + rect.width() / 2.0;
                let center_y = rect.top() + rect.height() / 2.0;
                painter.line_segment(
                    [Pos2::new(center_x - 10.0, center_y), Pos2::new(center_x + 10.0, center_y)],
                    Stroke::new(1.0, Color32::from_rgba_unmultiplied(200, 200, 200, 100)),
                );
                painter.line_segment(
                    [Pos2::new(center_x, center_y - 10.0), Pos2::new(center_x, center_y + 10.0)],
                    Stroke::new(1.0, Color32::from_rgba_unmultiplied(200, 200, 200, 100)),
                );
            }
        }

        // Border
        painter.rect_stroke(rect, 0.0, Stroke::new(1.0, Color32::from_rgb(180, 180, 180)));

        // Signal quality indicator in corner
        let quality_text = format!("Q: {:.0}%", self.signal_quality * 100.0);
        painter.text(
            Pos2::new(rect.right() - 5.0, rect.top() + 5.0),
            egui::Align2::RIGHT_TOP,
            quality_text,
            egui::FontId::proportional(10.0),
            if self.signal_quality > 0.7 {
                colors::SUCCESS
            } else if self.signal_quality > 0.4 {
                colors::WARNING
            } else {
                colors::ERROR
            },
        );
    }

    /// Clear all points
    pub fn clear(&mut self) {
        self.points.clear();
        self.cluster_centers.clear();
        self.prev_sample = 0.0;
        self.signal_quality = 0.0;
    }

    /// Set maximum number of points to display
    pub fn set_max_points(&mut self, max: usize) {
        self.max_points = max;
    }

    /// Set point size
    pub fn set_point_size(&mut self, size: f32) {
        self.point_size = size;
    }

    /// Generate demo data for testing
    pub fn generate_demo_data(&mut self, noise_level: f32) {
        self.clear();

        match self.mode {
            ConstellationMode::EyePattern => {
                // Generate correlated samples with noise
                // Good signal: samples[n] ≈ samples[n-1] (strong correlation)
                let mut prev = 0.5f32;
                for _ in 0..self.max_points {
                    // Correlated component (diagonal pattern)
                    let correlated = prev * 0.9;
                    // Noise component
                    let noise = (rand_simple() - 0.5) * noise_level;
                    let sample = (correlated + noise).clamp(-1.0, 1.0);

                    self.add_sample(sample);
                    prev = sample;
                }
            }
            ConstellationMode::IqConstellation => {
                // Generate QPSK constellation points with noise
                let s = 0.707f32;
                let ideal = [(s, s), (-s, s), (s, -s), (-s, -s)];

                for _ in 0..self.max_points {
                    // Safe index calculation: clamp rand_simple to [0, 1) and use modulo
                    let rand_val = rand_simple().clamp(0.0, 0.9999);
                    let idx = (rand_val * 4.0) as usize;
                    let (ix, iy) = ideal[idx.min(3)]; // Ensure index is always valid
                    let x = ix + (rand_simple() - 0.5) * noise_level;
                    let y = iy + (rand_simple() - 0.5) * noise_level;
                    self.add_point(x, y);
                }
            }
        }
    }
}

impl Default for ConstellationDisplay {
    fn default() -> Self {
        Self::new()
    }
}

/// Simple pseudo-random number generator for demo data
fn rand_simple() -> f32 {
    use std::cell::Cell;
    thread_local! {
        static SEED: Cell<u32> = Cell::new(12345);
    }
    SEED.with(|s| {
        let val = s.get().wrapping_mul(1103515245).wrapping_add(12345);
        s.set(val);
        val as f32 / u32::MAX as f32
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_eye_pattern_quality() {
        let mut display = ConstellationDisplay::new();
        display.set_mode(ConstellationMode::EyePattern);

        // Perfect correlation (diagonal line)
        for i in 0..100 {
            let v = (i as f32) / 100.0;
            display.add_sample(v);
        }

        // Should have high quality (strong correlation)
        assert!(display.signal_quality() > 0.9, "Expected high quality for diagonal pattern");
    }

    #[test]
    fn test_constellation_mode() {
        let mut display = ConstellationDisplay::new();
        display.set_mode(ConstellationMode::IqConstellation);

        // Add QPSK points with no noise
        let s = 0.707f32;
        for _ in 0..25 {
            display.add_point(s, s);
            display.add_point(-s, s);
            display.add_point(s, -s);
            display.add_point(-s, -s);
        }

        // Should have high quality (tight clusters)
        assert!(display.signal_quality() > 0.8, "Expected high quality for clean constellation");
    }
}
