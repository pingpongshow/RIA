//! Meter displays - analog gauges and status indicators

use eframe::egui::{self, Color32, Pos2, Rect, Stroke, Ui, Vec2};
use std::f32::consts::PI;

use super::colors;
use super::GuiState;

/// Analog gauge display with smoothing
pub struct AnalogGauge {
    label: String,
    min_value: f32,
    max_value: f32,
    unit: String,
    // Gauge colors (green-yellow-red zones)
    green_end: f32,   // Percentage where green ends
    yellow_end: f32,  // Percentage where yellow ends
    // Smoothing
    smoothed_value: f32,
    smoothing_factor: f32, // 0.0 = no smoothing, 0.95 = heavy smoothing
}

impl AnalogGauge {
    /// Create new analog gauge
    pub fn new(label: &str, min: f32, max: f32, unit: &str) -> Self {
        Self {
            label: label.to_string(),
            min_value: min,
            max_value: max,
            unit: unit.to_string(),
            green_end: 0.6,
            yellow_end: 0.8,
            smoothed_value: min,
            smoothing_factor: 0.85, // Smooth needle movement
        }
    }

    /// Set color zone thresholds (as percentages 0.0-1.0)
    pub fn set_zones(&mut self, green_end: f32, yellow_end: f32) {
        self.green_end = green_end;
        self.yellow_end = yellow_end;
    }

    /// Set smoothing factor (0.0 = instant, 0.95 = very smooth)
    pub fn set_smoothing(&mut self, factor: f32) {
        self.smoothing_factor = factor.clamp(0.0, 0.99);
    }

    /// Display the gauge
    pub fn show(&mut self, ui: &mut Ui, value: f32, width: f32, height: f32, subtitle: &str) {
        // Apply exponential smoothing to the value
        self.smoothed_value = self.smoothing_factor * self.smoothed_value
                            + (1.0 - self.smoothing_factor) * value;
        let (rect, _response) = ui.allocate_exact_size(
            Vec2::new(width, height),
            egui::Sense::hover(),
        );

        let painter = ui.painter_at(rect);

        // Background
        painter.rect_filled(rect, 4.0, colors::PANEL);

        // Calculate gauge center and radius (increased by 20%)
        let center = Pos2::new(rect.center().x, rect.bottom() - 22.0);
        let radius = (width.min(height - 30.0) * 0.54).max(36.0);

        // Draw gauge arc background (gray)
        self.draw_arc(&painter, center, radius, 0.0, 1.0, Color32::from_gray(60), 8.0);

        // Draw colored zones
        self.draw_arc(&painter, center, radius, 0.0, self.green_end, colors::SUCCESS, 8.0);
        self.draw_arc(&painter, center, radius, self.green_end, self.yellow_end, colors::WARNING, 8.0);
        self.draw_arc(&painter, center, radius, self.yellow_end, 1.0, colors::ERROR, 8.0);

        // Draw tick marks
        for i in 0..=10 {
            let pct = i as f32 / 10.0;
            let angle = self.pct_to_angle(pct);
            let inner = radius - 12.0;
            let outer = radius - 4.0;

            let cos_a = angle.cos();
            let sin_a = angle.sin();

            let p1 = Pos2::new(center.x + cos_a * inner, center.y - sin_a * inner);
            let p2 = Pos2::new(center.x + cos_a * outer, center.y - sin_a * outer);

            painter.line_segment([p1, p2], Stroke::new(1.0, colors::TEXT_DIM));
        }

        // Calculate needle position using smoothed value
        let normalized = ((self.smoothed_value - self.min_value) / (self.max_value - self.min_value)).clamp(0.0, 1.0);
        let needle_angle = self.pct_to_angle(normalized);
        let needle_length = radius - 15.0;

        // Draw needle
        let needle_tip = Pos2::new(
            center.x + needle_angle.cos() * needle_length,
            center.y - needle_angle.sin() * needle_length,
        );
        painter.line_segment([center, needle_tip], Stroke::new(2.0, colors::TEXT));

        // Draw center circle
        painter.circle_filled(center, 5.0, colors::TEXT);

        // Draw label at top
        painter.text(
            Pos2::new(rect.center().x, rect.top() + 12.0),
            egui::Align2::CENTER_CENTER,
            &self.label,
            egui::FontId::proportional(14.0),
            colors::TEXT,
        );

        // Draw subtitle at bottom
        painter.text(
            Pos2::new(rect.center().x, rect.bottom() - 5.0),
            egui::Align2::CENTER_CENTER,
            subtitle,
            egui::FontId::proportional(10.0),
            colors::TEXT_DIM,
        );
    }

    fn pct_to_angle(&self, pct: f32) -> f32 {
        // Gauge spans from 150° to 30° (sweeping through top)
        // 150° = 5π/6, 30° = π/6
        let start_angle = 5.0 * PI / 6.0;
        let end_angle = PI / 6.0;
        let sweep = start_angle - end_angle; // Clockwise sweep

        start_angle - pct * sweep
    }

    fn draw_arc(&self, painter: &egui::Painter, center: Pos2, radius: f32,
                start_pct: f32, end_pct: f32, color: Color32, width: f32) {
        let segments = 32;
        let start = (start_pct * segments as f32) as usize;
        let end = (end_pct * segments as f32) as usize;

        for i in start..end {
            let pct1 = i as f32 / segments as f32;
            let pct2 = (i + 1) as f32 / segments as f32;

            let angle1 = self.pct_to_angle(pct1);
            let angle2 = self.pct_to_angle(pct2);

            let p1 = Pos2::new(center.x + angle1.cos() * radius, center.y - angle1.sin() * radius);
            let p2 = Pos2::new(center.x + angle2.cos() * radius, center.y - angle2.sin() * radius);

            painter.line_segment([p1, p2], Stroke::new(width, color));
        }
    }
}

/// Status indicator panel (DATA, ACK, IDLE, NACK, BREAK, REQ, QRT)
pub struct StatusIndicators {
    // No state needed - uses GuiState
}

impl StatusIndicators {
    pub fn new() -> Self {
        Self {}
    }

    pub fn paint(&self, ui: &Ui, rect: Rect, state: &GuiState) {
        let painter = ui.painter_at(rect);

        // Background
        painter.rect_filled(rect, 4.0, colors::PANEL);

        // Layout: 2 columns, 4 rows
        // Column 1: DATA, ACK, IDLE, REQ
        // Column 2: (empty), NACK, BREAK, QRT

        let indicators = [
            ("DATA", 0, 0, state.status_data),
            ("ACK", 0, 1, state.status_ack),
            ("IDLE", 0, 2, state.status_idle),
            ("REQ", 0, 3, state.status_req),
            ("START", 1, 0, state.status_start),
            ("NACK", 1, 1, state.status_nack),
            ("BREAK", 1, 2, state.status_break),
            ("QRT", 1, 3, state.status_qrt),
        ];

        let col_width = rect.width() / 2.0;
        let row_height = rect.height() / 4.0;
        let indicator_size = 10.0;

        for (label, col, row, active) in indicators {
            let x = rect.left() + col as f32 * col_width + col_width / 2.0;
            let y = rect.top() + row as f32 * row_height + row_height / 2.0;

            // Draw indicator light
            let light_color = if active { colors::SUCCESS } else { colors::IDLE };
            painter.circle_filled(Pos2::new(x - 20.0, y), indicator_size / 2.0, light_color);
            painter.circle_stroke(Pos2::new(x - 20.0, y), indicator_size / 2.0,
                                  Stroke::new(1.0, colors::TEXT_DIM));

            // Draw label
            painter.text(
                Pos2::new(x + 5.0, y),
                egui::Align2::LEFT_CENTER,
                label,
                egui::FontId::proportional(11.0),
                colors::TEXT,
            );
        }
    }
}

impl Default for StatusIndicators {
    fn default() -> Self {
        Self::new()
    }
}

/// Bitrate bar graph display - shows historical peak bitrates over 4 minutes
pub struct BitrateGraph {
    /// Peak bitrate for each bar (indexed by bar position, 0 = oldest)
    bar_peaks: Vec<f32>,
    /// Timestamp when each bar started
    bar_start_times: Vec<f64>,
    /// Duration to display in seconds (4 minutes)
    display_duration_secs: f64,
    /// Number of bars to display
    num_bars: usize,
    /// Duration of each bar in seconds
    bar_duration_secs: f64,
    /// Current bar's peak bitrate (for the rightmost/newest bar)
    current_peak: f32,
    /// Timestamp when current bar started
    current_bar_start: f64,
    /// Maximum bitrate attained during entire session
    max_bitrate: f32,
}

impl BitrateGraph {
    /// Create new bitrate graph (4 minute history, 24 bars = 10 sec each)
    pub fn new() -> Self {
        let num_bars = 24;
        let display_duration = 240.0; // 4 minutes
        Self {
            bar_peaks: Vec::with_capacity(num_bars),
            bar_start_times: Vec::with_capacity(num_bars),
            display_duration_secs: display_duration,
            num_bars,
            bar_duration_secs: display_duration / num_bars as f64, // 10 seconds per bar
            current_peak: 0.0,
            current_bar_start: 0.0,
            max_bitrate: 0.0,
        }
    }

    /// Add bitrate sample with current timestamp
    pub fn add_sample(&mut self, bps: f32) {
        use std::time::{SystemTime, UNIX_EPOCH};
        let now = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_secs_f64())
            .unwrap_or(0.0);

        // Track maximum bitrate for entire session
        if bps > self.max_bitrate {
            self.max_bitrate = bps;
        }

        // Initialize current bar if needed
        if self.current_bar_start == 0.0 {
            self.current_bar_start = now;
        }

        // Update current bar's peak
        if bps > self.current_peak {
            self.current_peak = bps;
        }

        // Check if we need to move to a new bar
        if now >= self.current_bar_start + self.bar_duration_secs {
            // Save current bar's peak
            self.bar_peaks.push(self.current_peak);
            self.bar_start_times.push(self.current_bar_start);

            // Trim old bars that have scrolled off
            while self.bar_peaks.len() > self.num_bars {
                self.bar_peaks.remove(0);
                self.bar_start_times.remove(0);
            }

            // Start new bar
            self.current_bar_start = now;
            self.current_peak = bps;
        }
    }

    /// Clear history (call when session ends)
    pub fn clear(&mut self) {
        self.bar_peaks.clear();
        self.bar_start_times.clear();
        self.current_peak = 0.0;
        self.current_bar_start = 0.0;
        self.max_bitrate = 0.0;
    }

    /// Paint the bar graph
    pub fn paint(&self, ui: &Ui, rect: Rect) {
        use std::time::{SystemTime, UNIX_EPOCH};
        let now = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_secs_f64())
            .unwrap_or(0.0);

        let painter = ui.painter_at(rect);

        // Background with grid
        painter.rect_filled(rect, 0.0, Color32::from_rgb(0, 0, 80)); // Navy blue background

        // Draw grid lines
        let grid_color = Color32::from_rgb(180, 180, 180); // Light gray

        // Horizontal grid lines (bps markers)
        for i in 1..6 {
            let y = rect.top() + (i as f32 / 6.0) * rect.height();
            painter.line_segment(
                [Pos2::new(rect.left(), y), Pos2::new(rect.right(), y)],
                Stroke::new(1.0, grid_color),
            );
        }

        // Vertical grid lines (time markers - every minute for 4 minutes)
        for i in 1..4 {
            let x = rect.left() + 35.0 + (i as f32 / 4.0) * (rect.width() - 40.0);
            painter.line_segment(
                [Pos2::new(x, rect.top()), Pos2::new(x, rect.bottom())],
                Stroke::new(1.0, grid_color),
            );
        }

        // Draw Y-axis labels
        let label_color = Color32::from_rgb(200, 200, 200); // Light gray
        let y_labels = ["8500", "7000", "5500", "4000", "2500", "1000", "0"];
        for (i, label) in y_labels.iter().enumerate() {
            let y = rect.top() + (i as f32 / 6.0) * rect.height();
            painter.text(
                Pos2::new(rect.left() + 5.0, y + 2.0),
                egui::Align2::LEFT_TOP,
                *label,
                egui::FontId::proportional(9.0),
                label_color,
            );
        }

        // Draw "bps" label and time range
        painter.text(
            Pos2::new(rect.left() + 5.0, rect.top() + 5.0),
            egui::Align2::LEFT_TOP,
            "bps",
            egui::FontId::proportional(10.0),
            label_color,
        );

        // Draw max bitrate in top right
        painter.text(
            Pos2::new(rect.right() - 5.0, rect.top() + 5.0),
            egui::Align2::RIGHT_TOP,
            format!("Max: {:.0}", self.max_bitrate),
            egui::FontId::proportional(10.0),
            Color32::from_rgb(255, 255, 100), // Yellow for visibility
        );


        // Draw bars showing peak bitrates
        let bar_width = (rect.width() - 40.0) / self.num_bars as f32;
        let bar_area_left = rect.left() + 35.0;

        // Draw historical bars (oldest on left, newest on right)
        let total_bars = self.bar_peaks.len();
        let start_pos = self.num_bars.saturating_sub(total_bars + 1); // +1 for current bar

        for (i, &peak_bps) in self.bar_peaks.iter().enumerate() {
            if peak_bps <= 0.0 {
                continue;
            }

            let bar_pos = start_pos + i;
            let normalized = (peak_bps / 8500.0).clamp(0.0, 1.0);
            let bar_height = normalized * (rect.height() - 20.0);

            let x = bar_area_left + bar_pos as f32 * bar_width;
            let bar_rect = Rect::from_min_size(
                Pos2::new(x, rect.bottom() - 15.0 - bar_height),
                Vec2::new(bar_width - 1.0, bar_height),
            );

            painter.rect_filled(bar_rect, 0.0, Color32::WHITE);
        }

        // Draw current bar (rightmost)
        if self.current_peak > 0.0 {
            let bar_pos = self.num_bars - 1;
            let normalized = (self.current_peak / 8500.0).clamp(0.0, 1.0);
            let bar_height = normalized * (rect.height() - 20.0);

            let x = bar_area_left + bar_pos as f32 * bar_width;
            let bar_rect = Rect::from_min_size(
                Pos2::new(x, rect.bottom() - 15.0 - bar_height),
                Vec2::new(bar_width - 1.0, bar_height),
            );

            painter.rect_filled(bar_rect, 0.0, Color32::WHITE);
        }

        // Border
        painter.rect_stroke(rect, 0.0, Stroke::new(1.0, Color32::from_rgb(180, 180, 180)));
    }
}

impl Default for BitrateGraph {
    fn default() -> Self {
        Self::new()
    }
}

/// SNR meter display (legacy - kept for compatibility)
pub struct SnrMeter {
    min_db: f32,
    max_db: f32,
    history: Vec<f32>,
    max_history: usize,
}

impl SnrMeter {
    pub fn new() -> Self {
        Self {
            min_db: -10.0,
            max_db: 40.0,
            history: Vec::new(),
            max_history: 50,
        }
    }

    pub fn show(&mut self, ui: &mut Ui, snr_db: f32) {
        self.history.push(snr_db);
        if self.history.len() > self.max_history {
            self.history.remove(0);
        }

        let normalized = (snr_db - self.min_db) / (self.max_db - self.min_db);
        let normalized = normalized.clamp(0.0, 1.0);

        let color = if snr_db < 5.0 {
            colors::ERROR
        } else if snr_db < 15.0 {
            colors::WARNING
        } else {
            colors::SUCCESS
        };

        let desired_size = egui::vec2(ui.available_width(), 20.0);
        let (rect, _response) = ui.allocate_exact_size(desired_size, egui::Sense::hover());

        let painter = ui.painter_at(rect);
        painter.rect_filled(rect, 2.0, colors::PANEL);

        let fill_rect = Rect::from_min_size(
            rect.min,
            egui::vec2(rect.width() * normalized, rect.height()),
        );
        painter.rect_filled(fill_rect, 2.0, color);
        painter.rect_stroke(rect, 2.0, Stroke::new(1.0, colors::TEXT_DIM));

        painter.text(
            rect.center(),
            egui::Align2::CENTER_CENTER,
            format!("{:.1} dB", snr_db),
            egui::FontId::default(),
            colors::TEXT,
        );
    }

    pub fn set_range(&mut self, min_db: f32, max_db: f32) {
        self.min_db = min_db;
        self.max_db = max_db;
    }
}

impl Default for SnrMeter {
    fn default() -> Self {
        Self::new()
    }
}

/// VU meter display (legacy)
pub struct VuMeter {
    min_db: f32,
    max_db: f32,
    peak: f32,
    peak_hold_frames: usize,
    peak_counter: usize,
}

impl VuMeter {
    pub fn new() -> Self {
        Self {
            min_db: -60.0,
            max_db: 0.0,
            peak: -60.0,
            peak_hold_frames: 30,
            peak_counter: 0,
        }
    }

    pub fn show(&mut self, ui: &mut Ui, level_db: f32) {
        if level_db > self.peak {
            self.peak = level_db;
            self.peak_counter = 0;
        } else {
            self.peak_counter += 1;
            if self.peak_counter > self.peak_hold_frames {
                self.peak = (self.peak - 0.5).max(level_db);
            }
        }

        let normalized = (level_db - self.min_db) / (self.max_db - self.min_db);
        let normalized = normalized.clamp(0.0, 1.0);

        let desired_size = egui::vec2(ui.available_width(), 20.0);
        let (rect, _response) = ui.allocate_exact_size(desired_size, egui::Sense::hover());

        let painter = ui.painter_at(rect);
        painter.rect_filled(rect, 2.0, colors::PANEL);

        let num_segments = 20;
        let segment_width = rect.width() / num_segments as f32;

        for i in 0..num_segments {
            let segment_fill = ((i + 1) as f32 / num_segments as f32) <= normalized;
            if !segment_fill {
                continue;
            }

            let x = rect.left() + i as f32 * segment_width;
            let segment_rect = Rect::from_min_size(
                Pos2::new(x + 1.0, rect.top() + 2.0),
                Vec2::new(segment_width - 2.0, rect.height() - 4.0),
            );

            let color = if i < 12 {
                colors::SUCCESS
            } else if i < 16 {
                colors::WARNING
            } else {
                colors::ERROR
            };

            painter.rect_filled(segment_rect, 1.0, color);
        }

        painter.rect_stroke(rect, 2.0, Stroke::new(1.0, colors::TEXT_DIM));
    }
}

impl Default for VuMeter {
    fn default() -> Self {
        Self::new()
    }
}

/// CPU usage meter (legacy)
pub struct CpuMeter {
    history: Vec<f32>,
    max_history: usize,
}

impl CpuMeter {
    pub fn new() -> Self {
        Self {
            history: Vec::new(),
            max_history: 100,
        }
    }

    pub fn show(&mut self, ui: &mut Ui, cpu_percent: f32) {
        self.history.push(cpu_percent);
        if self.history.len() > self.max_history {
            self.history.remove(0);
        }

        let normalized = (cpu_percent / 100.0).clamp(0.0, 1.0);

        let color = if cpu_percent < 50.0 {
            colors::SUCCESS
        } else if cpu_percent < 80.0 {
            colors::WARNING
        } else {
            colors::ERROR
        };

        let desired_size = egui::vec2(ui.available_width(), 20.0);
        let (rect, _response) = ui.allocate_exact_size(desired_size, egui::Sense::hover());

        let painter = ui.painter_at(rect);
        painter.rect_filled(rect, 2.0, colors::PANEL);

        let fill_rect = Rect::from_min_size(
            rect.min,
            egui::vec2(rect.width() * normalized, rect.height()),
        );
        painter.rect_filled(fill_rect, 2.0, color);
        painter.rect_stroke(rect, 2.0, Stroke::new(1.0, colors::TEXT_DIM));

        painter.text(
            rect.center(),
            egui::Align2::CENTER_CENTER,
            format!("{:.0}%", cpu_percent),
            egui::FontId::default(),
            colors::TEXT,
        );
    }
}

impl Default for CpuMeter {
    fn default() -> Self {
        Self::new()
    }
}
