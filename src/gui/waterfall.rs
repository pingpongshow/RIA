//! Waterfall display for spectrum visualization

use eframe::egui::{self, Color32, Pos2, Rect, Stroke, Ui};
use std::collections::VecDeque;

/// Waterfall display - centered on 1500Hz with zoom
pub struct Waterfall {
    width: usize,
    height: usize,
    lines: VecDeque<Vec<f32>>,
    min_db: f32,
    max_db: f32,
    /// Center frequency in Hz
    center_freq: f32,
    /// Sample rate in Hz
    sample_rate: f32,
    /// Zoom factor (1.0 = full spectrum, 2.0 = show half, etc.)
    zoom: f32,
}

impl Waterfall {
    /// Create new waterfall display centered on 1500Hz
    pub fn new(width: usize, height: usize) -> Self {
        Self {
            width,
            height,
            lines: VecDeque::with_capacity(height),
            min_db: -80.0,
            max_db: -20.0,
            center_freq: 1500.0,
            sample_rate: 48000.0,
            zoom: 3.0, // Show ~1/3 of spectrum centered on 1500Hz
        }
    }

    /// Set center frequency
    pub fn set_center_freq(&mut self, freq: f32) {
        self.center_freq = freq;
    }

    /// Set zoom level (1.0 = full, 2.0 = 2x zoom, etc.)
    pub fn set_zoom(&mut self, zoom: f32) {
        self.zoom = zoom.max(1.0);
    }

    /// Add a new spectrum line
    pub fn add_line(&mut self, spectrum: &[f32]) {
        // Fixed frequency range: 0 to 3500 Hz
        let nyquist = self.sample_rate / 2.0;
        let freq_start = 0.0;
        let freq_end = 3500.0_f32.min(nyquist);

        // Convert frequencies to spectrum indices
        let bin_start = ((freq_start / nyquist) * spectrum.len() as f32) as usize;
        let bin_end = ((freq_end / nyquist) * spectrum.len() as f32) as usize;
        let bin_range = bin_end.saturating_sub(bin_start).max(1);

        // Resample the zoomed portion to match display width
        let mut line = Vec::with_capacity(self.width);

        for i in 0..self.width {
            let idx = bin_start + (i * bin_range / self.width);
            let value = spectrum.get(idx).copied().unwrap_or(-80.0);
            line.push(value);
        }

        self.lines.push_front(line);

        // Trim to height
        while self.lines.len() > self.height {
            self.lines.pop_back();
        }
    }

    /// Set dB range
    pub fn set_range(&mut self, min_db: f32, max_db: f32) {
        self.min_db = min_db;
        self.max_db = max_db;
    }

    /// Convert dB value to color (gray/white palette on navy background)
    fn db_to_color(&self, db: f32) -> Color32 {
        let normalized = (db - self.min_db) / (self.max_db - self.min_db);
        let normalized = normalized.clamp(0.0, 1.0);

        // Gray gradient: navy blue -> gray -> white (strong signals are white)
        // Start from slightly brighter than background so weak signals are visible
        let gray_value = (normalized * 255.0) as u8;
        Color32::from_rgb(gray_value, gray_value, gray_value)
    }

    /// Linear interpolation between colors
    fn lerp_color(a: Color32, b: Color32, t: f32) -> Color32 {
        Color32::from_rgb(
            (a.r() as f32 * (1.0 - t) + b.r() as f32 * t) as u8,
            (a.g() as f32 * (1.0 - t) + b.g() as f32 * t) as u8,
            (a.b() as f32 * (1.0 - t) + b.b() as f32 * t) as u8,
        )
    }

    /// Paint the waterfall (gray/white on navy blue)
    pub fn paint(&self, ui: &Ui, rect: Rect) {
        let painter = ui.painter_at(rect);

        // Reserve space for frequency labels at bottom
        let label_height = 16.0;
        let waterfall_rect = Rect::from_min_max(
            rect.min,
            Pos2::new(rect.max.x, rect.max.y - label_height),
        );

        // Navy blue background
        painter.rect_filled(waterfall_rect, 0.0, Color32::from_rgb(0, 0, 80));

        if !self.lines.is_empty() {
            let pixel_width = waterfall_rect.width() / self.width as f32;
            let pixel_height = waterfall_rect.height() / self.height as f32;

            // Draw each line
            for (y, line) in self.lines.iter().enumerate() {
                let y_pos = waterfall_rect.top() + y as f32 * pixel_height;

                for (x, &db) in line.iter().enumerate() {
                    let x_pos = waterfall_rect.left() + x as f32 * pixel_width;
                    let color = self.db_to_color(db);

                    painter.rect_filled(
                        Rect::from_min_size(
                            egui::pos2(x_pos, y_pos),
                            egui::vec2(pixel_width + 1.0, pixel_height + 1.0),
                        ),
                        0.0,
                        color,
                    );
                }
            }
        }

        // Fixed frequency range: 0 to 3500 Hz
        let nyquist = self.sample_rate / 2.0;
        let freq_start = 0.0;
        let freq_end = 3500.0_f32.min(nyquist);

        // Draw center frequency marker (vertical line at 1500Hz position)
        let center_freq_normalized = (self.center_freq - freq_start) / (freq_end - freq_start);
        let center_x = waterfall_rect.left() + center_freq_normalized * waterfall_rect.width();
        painter.line_segment(
            [Pos2::new(center_x, waterfall_rect.top()), Pos2::new(center_x, waterfall_rect.bottom())],
            Stroke::new(1.0, Color32::from_rgba_unmultiplied(255, 255, 0, 128)),
        );

        let label_y = rect.bottom() - 4.0;
        let label_color = Color32::from_rgb(200, 200, 200);

        // Left frequency
        painter.text(
            Pos2::new(rect.left() + 5.0, label_y),
            egui::Align2::LEFT_CENTER,
            format!("{:.0}", freq_start),
            egui::FontId::proportional(10.0),
            label_color,
        );

        // Center frequency
        painter.text(
            Pos2::new(center_x, label_y),
            egui::Align2::CENTER_CENTER,
            format!("{:.0} Hz", self.center_freq),
            egui::FontId::proportional(10.0),
            Color32::YELLOW,
        );

        // Right frequency
        painter.text(
            Pos2::new(rect.right() - 5.0, label_y),
            egui::Align2::RIGHT_CENTER,
            format!("{:.0}", freq_end),
            egui::FontId::proportional(10.0),
            label_color,
        );
    }

    /// Clear the waterfall
    pub fn clear(&mut self) {
        self.lines.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_waterfall_add_line() {
        let mut waterfall = Waterfall::new(100, 50);

        let spectrum: Vec<f32> = (0..200).map(|i| -60.0 + (i as f32) * 0.1).collect();
        waterfall.add_line(&spectrum);

        assert_eq!(waterfall.lines.len(), 1);
        assert_eq!(waterfall.lines[0].len(), 100);
    }

    #[test]
    fn test_waterfall_trim() {
        let mut waterfall = Waterfall::new(100, 10);

        for _ in 0..20 {
            waterfall.add_line(&[-50.0; 100]);
        }

        assert_eq!(waterfall.lines.len(), 10);
    }
}
