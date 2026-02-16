//! Main application window

use eframe::egui::{self, Rounding, Vec2};
use std::sync::{Arc, Mutex};

use super::{GuiState, GuiEvent, colors};
use super::{Waterfall, ConstellationDisplay, AnalogGauge, StatusIndicators, BitrateGraph};
use crate::audio::AudioEngine;

/// Main RIA application
pub struct RiaApp {
    state: Arc<Mutex<GuiState>>,
    events: Arc<Mutex<Vec<GuiEvent>>>,

    // UI components
    waterfall: Waterfall,
    constellation: ConstellationDisplay,
    bitrate_graph: BitrateGraph,
    vu_gauge: AnalogGauge,
    cpu_gauge: AnalogGauge,
    afc_gauge: AnalogGauge,
    sn_gauge: AnalogGauge,
    status_indicators: StatusIndicators,

    // UI state
    show_settings: bool,
    show_soundcard: bool,
    show_log: bool,
    show_about: bool,
    connect_callsign: String,
    my_callsign: String,

    // Audio device state
    input_devices: Vec<String>,
    output_devices: Vec<String>,
    selected_input: String,
    selected_output: String,
    drive_level: f32,
    audio_channel: usize,  // 0=L, 1=R, 2=L+R

    // Settings panel state
    tcp_command_port: String,
    tcp_data_port: String,
    tcp_kiss_port: String,
    accept_500hz: bool,
    kiss_interface: bool,
    cw_id_enabled: bool,
    retries: u8,

    // Tune mode state
    tune_active: bool,

    // Log buffer for Show Log window
    log_buffer: Vec<String>,

    // Track previous connection state for session end detection
    was_connected: bool,
}

impl RiaApp {
    /// Create new application
    pub fn new(
        _cc: &eframe::CreationContext<'_>,
        state: Arc<Mutex<GuiState>>,
        events: Arc<Mutex<Vec<GuiEvent>>>,
    ) -> Self {
        // Get available audio devices
        let input_devices = AudioEngine::list_input_devices();
        let output_devices = AudioEngine::list_output_devices();

        Self {
            state,
            events,
            waterfall: Waterfall::new(768, 120),
            constellation: {
                let mut c = ConstellationDisplay::new();
                c.set_mode(super::ConstellationMode::IqConstellation);
                c
            },
            bitrate_graph: BitrateGraph::new(),
            vu_gauge: AnalogGauge::new("VU", -60.0, 0.0, "dB"),
            cpu_gauge: AnalogGauge::new("CPU", 0.0, 100.0, "%"),
            afc_gauge: AnalogGauge::new("AFC", -100.0, 100.0, "Hz"),
            sn_gauge: AnalogGauge::new("S/N", -10.0, 40.0, "dB"),
            status_indicators: StatusIndicators::new(),
            show_settings: false,
            show_soundcard: false,
            show_log: false,
            show_about: false,
            connect_callsign: String::new(),
            my_callsign: String::new(),
            input_devices,
            output_devices,
            selected_input: "Default".to_string(),
            selected_output: "Default".to_string(),
            drive_level: 0.8,
            audio_channel: 2, // L+R
            // Settings panel defaults
            tcp_command_port: "8300".to_string(),
            tcp_data_port: "8301".to_string(),
            tcp_kiss_port: "8100".to_string(),
            accept_500hz: false,
            kiss_interface: true,
            cw_id_enabled: false,
            retries: 15,
            tune_active: false,
            log_buffer: Vec::new(),
            was_connected: false,
        }
    }

    /// Refresh audio device lists
    fn refresh_devices(&mut self) {
        self.input_devices = AudioEngine::list_input_devices();
        self.output_devices = AudioEngine::list_output_devices();
    }

    /// Send event to modem core
    fn send_event(&self, event: GuiEvent) {
        if let Ok(mut events) = self.events.lock() {
            events.push(event);
        }
    }

    /// Update waterfall with new spectrum data
    pub fn update_waterfall(&mut self, spectrum: &[f32]) {
        self.waterfall.add_line(spectrum);
    }

    /// Update constellation with new symbols
    pub fn update_constellation(&mut self, symbols: &[(f32, f32)]) {
        self.constellation.set_points(symbols);
    }

    /// Update bitrate graph
    pub fn update_bitrate(&mut self, bps: f32) {
        self.bitrate_graph.add_sample(bps);
    }
}

impl eframe::App for RiaApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // Use ok() to gracefully handle poisoned mutex instead of panicking
        let state = match self.state.lock() {
            Ok(guard) => guard.clone(),
            Err(poisoned) => {
                // Recover from poisoned mutex by taking the inner value
                poisoned.into_inner().clone()
            }
        };

        // Detect session end (transition from connected to disconnected)
        if self.was_connected && !state.connected {
            // Session ended - clear bitrate history
            self.bitrate_graph.clear();
        }
        self.was_connected = state.connected;

        // Update bitrate graph with current value (only when connected)
        if state.connected {
            self.bitrate_graph.add_sample(state.bitrate);
        }

        // Update constellation display with received symbols
        if !state.constellation_points.is_empty() {
            self.constellation.set_points(&state.constellation_points);
        }

        // Update waterfall with spectrum data
        if !state.spectrum_data.is_empty() {
            self.waterfall.add_line(&state.spectrum_data);
        }

        // Menu bar
        egui::TopBottomPanel::top("menu_bar").show(ctx, |ui| {
            egui::menu::bar(ui, |ui| {
                ui.menu_button("Settings", |ui| {
                    if ui.button("Setup...").clicked() {
                        self.show_settings = true;
                        ui.close_menu();
                    }
                    if ui.button("SoundCard...").clicked() {
                        self.show_soundcard = true;
                        ui.close_menu();
                    }
                    ui.separator();
                    if ui.button("Exit").clicked() {
                        self.send_event(GuiEvent::Quit);
                    }
                });

                ui.menu_button("View", |ui| {
                    if ui.button("Reset Layout").clicked() {
                        // Reset gauge and graph states to defaults
                        self.waterfall = Waterfall::new(512, 100);
                        self.constellation = ConstellationDisplay::new();
                        self.bitrate_graph = BitrateGraph::new();
                        ui.close_menu();
                    }
                });

                ui.menu_button("Monitor", |ui| {
                    if ui.button("Show Log").clicked() {
                        self.show_log = true;
                        ui.close_menu();
                    }
                });

                ui.menu_button("Help", |ui| {
                    if ui.button("About RIA").clicked() {
                        self.show_about = true;
                        ui.close_menu();
                    }
                });

                // Right-aligned title
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if state.connected {
                        ui.colored_label(colors::SUCCESS,
                            format!("Connected: {} <-> {}", state.local_call, state.remote_call));
                    } else {
                        ui.label("Disconnected");
                    }
                });
            });
        });

        // Bottom status bar
        egui::TopBottomPanel::bottom("status_bar")
            .exact_height(24.0)
            .show(ctx, |ui| {
                ui.horizontal_centered(|ui| {
                    // RX/TX indicator
                    let rx_color = if state.rx_active { colors::RX_ACTIVE } else { colors::IDLE };
                    let tx_color = if state.tx_active { colors::TX_ACTIVE } else { colors::IDLE };

                    ui.colored_label(rx_color, "RX");
                    ui.colored_label(tx_color, "TX");

                    ui.separator();

                    // Callsigns
                    if state.connected {
                        ui.label(format!("{} <-> {}", state.local_call, state.remote_call));
                    } else {
                        ui.colored_label(colors::TEXT_DIM, "Disconnected");
                    }

                    ui.separator();

                    // Bitrate
                    ui.label(format!("{:.0} bps", state.bitrate));

                    ui.separator();

                    // Bytes
                    ui.label(format!("{} Bytes Rcvd", state.bytes_received));

                    // Right side indicators
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        // DCD indicator
                        let dcd_color = if state.busy { colors::SUCCESS } else { colors::IDLE };
                        ui.colored_label(dcd_color, "DCD");

                        // TCP indicator (shows if any TCP client is connected)
                        let tcp_color = if state.tcp_connected { colors::SUCCESS } else { colors::IDLE };
                        ui.colored_label(tcp_color, "TCP");

                        // Listen indicator
                        let listen_color = if state.listen { colors::SUCCESS } else { colors::IDLE };
                        ui.colored_label(listen_color, "LISTEN");

                        // Bandwidth
                        ui.label(&state.bandwidth);
                    });
                });
            });

        // Main content area
        egui::CentralPanel::default().show(ctx, |ui| {
            ui.style_mut().spacing.item_spacing = Vec2::new(4.0, 4.0);

            // Calculate layout sizes
            let available = ui.available_size();
            let graph_height = (available.y * 0.35).min(200.0);
            let gauge_height = (available.y * 0.25).min(120.0);
            let waterfall_height = available.y - graph_height - gauge_height - 20.0;

            // ROW 1: Bitrate graph (left) + Constellation plot (right)
            ui.horizontal(|ui| {
                let half_width = (available.x - 8.0) / 2.0;

                // Bitrate bar graph
                egui::Frame::dark_canvas(ui.style())
                    .rounding(Rounding::same(4.0))
                    .show(ui, |ui| {
                        let (rect, _) = ui.allocate_exact_size(
                            Vec2::new(half_width, graph_height),
                            egui::Sense::hover(),
                        );
                        self.bitrate_graph.paint(ui, rect);
                    });

                // Constellation plot
                egui::Frame::dark_canvas(ui.style())
                    .rounding(Rounding::same(4.0))
                    .show(ui, |ui| {
                        let (rect, _) = ui.allocate_exact_size(
                            Vec2::new(half_width, graph_height),
                            egui::Sense::hover(),
                        );
                        self.constellation.paint(ui, rect);
                    });
            });

            ui.add_space(4.0);

            // ROW 2: Analog gauges with status indicators in center
            ui.horizontal(|ui| {
                let gauge_width = (available.x - 165.0) / 4.0; // Leave 165px for status indicators

                // VU Gauge
                self.vu_gauge.show(ui, state.signal_level, gauge_width, gauge_height,
                    &format!("Audio Input: {:.0} dB", state.signal_level));

                // CPU Gauge
                self.cpu_gauge.show(ui, state.cpu_percent, gauge_width, gauge_height,
                    &format!("CPU Usage: {:.0} %", state.cpu_percent));

                // Status indicators (center)
                egui::Frame::none()
                    .show(ui, |ui| {
                        let (rect, _) = ui.allocate_exact_size(
                            Vec2::new(165.0, gauge_height),
                            egui::Sense::hover(),
                        );
                        self.status_indicators.paint(ui, rect, &state);
                    });

                // AFC Gauge
                self.afc_gauge.show(ui, state.afc_offset, gauge_width, gauge_height,
                    &format!("AFC: {:+.1} Hz", state.afc_offset));

                // S/N Gauge
                self.sn_gauge.show(ui, state.snr_db, gauge_width, gauge_height,
                    &format!("S/N: {:+.1} dB", state.snr_db));
            });

            ui.add_space(4.0);

            // ROW 3: Waterfall (full width)
            egui::Frame::dark_canvas(ui.style())
                .rounding(Rounding::same(4.0))
                .show(ui, |ui| {
                    let (rect, _) = ui.allocate_exact_size(
                        Vec2::new(available.x, waterfall_height.max(80.0)),
                        egui::Sense::hover(),
                    );
                    self.waterfall.paint(ui, rect);
                });
        });

        // Settings window
        let mut show_settings = self.show_settings;
        let mut settings_changed = false;
        if show_settings {
            egui::Window::new("RIA Setup")
                .open(&mut show_settings)
                .resizable(false)
                .show(ctx, |ui| {
                    // TCP Ports section
                    ui.group(|ui| {
                        ui.label("TCP Ports:");
                        ui.horizontal(|ui| {
                            ui.label("Command");
                            ui.label("Data");
                            ui.label("KISS");
                        });
                        ui.horizontal(|ui| {
                            if ui.add(egui::TextEdit::singleline(&mut self.tcp_command_port).desired_width(60.0)).changed() {
                                settings_changed = true;
                            }
                            if ui.add(egui::TextEdit::singleline(&mut self.tcp_data_port).desired_width(60.0)).changed() {
                                settings_changed = true;
                            }
                            if ui.add(egui::TextEdit::singleline(&mut self.tcp_kiss_port).desired_width(60.0)).changed() {
                                settings_changed = true;
                            }
                        });
                    });

                    ui.add_space(8.0);

                    // Options
                    if ui.checkbox(&mut self.accept_500hz, "Accept 500 Hz connections").changed() {
                        self.send_event(GuiEvent::SetAccept500Hz { enabled: self.accept_500hz });
                    }
                    if ui.checkbox(&mut self.kiss_interface, "KISS Interface").changed() {
                        self.send_event(GuiEvent::SetKissInterface { enabled: self.kiss_interface });
                    }
                    if ui.checkbox(&mut self.cw_id_enabled, "CW ID").changed() {
                        self.send_event(GuiEvent::SetCwId { enabled: self.cw_id_enabled });
                    }

                    ui.add_space(8.0);

                    ui.horizontal(|ui| {
                        ui.label("Retries:");
                        egui::ComboBox::from_id_salt("retries")
                            .selected_text(format!("{}", self.retries))
                            .show_ui(ui, |ui| {
                                for i in [5u8, 10, 15, 20, 25] {
                                    if ui.selectable_value(&mut self.retries, i, format!("{}", i)).changed() {
                                        self.send_event(GuiEvent::SetRetries { count: self.retries });
                                    }
                                }
                            });
                    });

                    ui.add_space(8.0);

                    // Apply button for TCP ports
                    if ui.button("Apply TCP Ports").clicked() {
                        if let (Ok(cmd), Ok(data), Ok(kiss)) = (
                            self.tcp_command_port.parse::<u16>(),
                            self.tcp_data_port.parse::<u16>(),
                            self.tcp_kiss_port.parse::<u16>(),
                        ) {
                            self.send_event(GuiEvent::SetTcpPorts { command: cmd, data, kiss });
                        }
                    }
                });
        }
        self.show_settings = show_settings;

        // SoundCard window
        let mut show_soundcard = self.show_soundcard;
        let mut tune_clicked = false;
        let mut refresh_clicked = false;
        let mut input_changed = false;
        let mut output_changed = false;
        let mut channel_changed = false;
        let mut drive_changed = false;
        let prev_input = self.selected_input.clone();
        let prev_output = self.selected_output.clone();
        let prev_channel = self.audio_channel;
        let prev_drive = self.drive_level;

        if show_soundcard {
            egui::Window::new("SoundCard")
                .open(&mut show_soundcard)
                .resizable(false)
                .show(ctx, |ui| {
                    ui.horizontal(|ui| {
                        ui.label("Device Input:");
                        if ui.button("⟳").on_hover_text("Refresh devices").clicked() {
                            refresh_clicked = true;
                        }
                    });
                    egui::ComboBox::from_id_salt("input_device")
                        .selected_text(&self.selected_input)
                        .width(300.0)
                        .show_ui(ui, |ui| {
                            for device in &self.input_devices {
                                if ui.selectable_value(&mut self.selected_input, device.clone(), device).changed() {
                                    input_changed = true;
                                }
                            }
                        });

                    ui.add_space(8.0);

                    ui.horizontal(|ui| {
                        ui.label("Device Output:");
                    });
                    egui::ComboBox::from_id_salt("output_device")
                        .selected_text(&self.selected_output)
                        .width(300.0)
                        .show_ui(ui, |ui| {
                            for device in &self.output_devices {
                                if ui.selectable_value(&mut self.selected_output, device.clone(), device).changed() {
                                    output_changed = true;
                                }
                            }
                        });

                    ui.add_space(8.0);

                    // Channel selection
                    ui.group(|ui| {
                        ui.label("Channel");
                        ui.horizontal(|ui| {
                            if ui.radio_value(&mut self.audio_channel, 0, "L").changed() {
                                channel_changed = true;
                            }
                            if ui.radio_value(&mut self.audio_channel, 1, "R").changed() {
                                channel_changed = true;
                            }
                            if ui.radio_value(&mut self.audio_channel, 2, "L+R").changed() {
                                channel_changed = true;
                            }
                        });
                    });

                    ui.add_space(8.0);

                    // Drive level with Tune button
                    ui.horizontal(|ui| {
                        let tune_button_color = if self.tune_active { colors::TX_ACTIVE } else { colors::IDLE };
                        let tune_label = if self.tune_active { "Stop Tune" } else { "Tune" };
                        if ui.add(egui::Button::new(tune_label).fill(tune_button_color)).clicked() {
                            tune_clicked = true;
                        }

                        ui.label("Drive level:");
                        if ui.add(egui::Slider::new(&mut self.drive_level, 0.0..=1.0).show_value(false)).changed() {
                            drive_changed = true;
                        }
                        ui.label(format!("{:.0} dB", (self.drive_level * 60.0) - 60.0));
                    });

                    ui.add_space(4.0);
                    ui.label("Press Tune and set the Drive Level for ALC=1/3");
                });
        }
        // Stop tune if window was closed while tune was active
        if !show_soundcard && self.show_soundcard && self.tune_active {
            self.tune_active = false;
            self.send_event(GuiEvent::StopTune);
        }
        self.show_soundcard = show_soundcard;

        // Send events for changed settings
        if refresh_clicked {
            self.refresh_devices();
        }
        if tune_clicked {
            self.tune_active = !self.tune_active;
            if self.tune_active {
                self.send_event(GuiEvent::StartTune);
            } else {
                self.send_event(GuiEvent::StopTune);
            }
        }
        if input_changed || self.selected_input != prev_input {
            self.send_event(GuiEvent::SetAudioInput { device: self.selected_input.clone() });
        }
        if output_changed || self.selected_output != prev_output {
            self.send_event(GuiEvent::SetAudioOutput { device: self.selected_output.clone() });
        }
        if channel_changed || self.audio_channel != prev_channel {
            self.send_event(GuiEvent::SetAudioChannel { channel: self.audio_channel });
        }
        if drive_changed || (self.drive_level - prev_drive).abs() > 0.001 {
            self.send_event(GuiEvent::SetDriveLevel { level: self.drive_level });
        }

        // Show Log window
        let mut show_log = self.show_log;
        if show_log {
            egui::Window::new("Event Log")
                .open(&mut show_log)
                .default_size([500.0, 300.0])
                .resizable(true)
                .show(ctx, |ui| {
                    egui::ScrollArea::vertical()
                        .auto_shrink([false, false])
                        .stick_to_bottom(true)
                        .show(ui, |ui| {
                            if self.log_buffer.is_empty() {
                                ui.label("No log entries yet.");
                            } else {
                                for entry in &self.log_buffer {
                                    ui.label(entry);
                                }
                            }
                        });

                    ui.separator();
                    ui.horizontal(|ui| {
                        if ui.button("Clear").clicked() {
                            self.log_buffer.clear();
                        }
                        ui.label(format!("{} entries", self.log_buffer.len()));
                    });
                });
        }
        self.show_log = show_log;

        // About RIA window
        let mut show_about = self.show_about;
        if show_about {
            egui::Window::new("About RIA")
                .open(&mut show_about)
                .resizable(false)
                .collapsible(false)
                .show(ctx, |ui| {
                    ui.vertical_centered(|ui| {
                        ui.heading("RIA - STANAG HF Modem");
                        ui.add_space(8.0);
                        ui.label(format!("Version {}", env!("CARGO_PKG_VERSION")));
                        ui.add_space(8.0);
                        ui.label("A STANAG 4197-style 39-tone modem for HF radio");
                        ui.label("communications with LDPC FEC coding.");
                        ui.add_space(16.0);
                        ui.separator();
                        ui.add_space(8.0);
                        ui.label("Features:");
                        ui.label("• STANAG 39-tone DQPSK modulation");
                        ui.label("• IEEE 802.11n LDPC FEC");
                        ui.label("• Adaptive rate control");
                        ui.label("• ARQ protocol for reliable delivery");
                        ui.add_space(16.0);
                        ui.label("Open source HF modem implementation.");
                        ui.add_space(8.0);
                        if ui.button("OK").clicked() {
                            self.show_about = false;
                        }
                    });
                });
        }
        self.show_about = show_about;

        // Request repaint for animation
        ctx.request_repaint();
    }
}

impl RiaApp {
    /// Add a log entry (can be called from main loop)
    pub fn add_log(&mut self, message: String) {
        // Keep last 1000 entries
        if self.log_buffer.len() >= 1000 {
            self.log_buffer.remove(0);
        }
        self.log_buffer.push(message);
    }
}

