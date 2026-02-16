//! Settings panel

use eframe::egui::{self, Ui};

/// Settings panel events
#[derive(Debug, Clone)]
pub enum SettingsEvent {
    /// Request to refresh audio device lists
    RefreshDevices,
    /// Apply current settings
    ApplySettings {
        input_device: String,
        output_device: String,
        drive_level: f32,
        ptt_method: String,
        ptt_port: String,
        ptt_delay_ms: u32,
        cmd_port: u16,
        data_port: u16,
    },
}

/// Settings panel
pub struct SettingsPanel {
    // Audio settings
    pub input_device: String,
    pub output_device: String,
    pub input_devices: Vec<String>,
    pub output_devices: Vec<String>,

    // Drive level
    pub drive_level: f32,

    // PTT settings
    pub ptt_method: String,
    pub ptt_port: String,
    pub ptt_delay_ms: u32,

    // Modem settings
    pub auto_tune: bool,
    pub cwid_enabled: bool,
    pub cwid_callsign: String,

    // Network settings
    pub cmd_port: u16,
    pub data_port: u16,

    // Pending events
    pending_events: Vec<SettingsEvent>,
}

impl SettingsPanel {
    /// Create new settings panel
    pub fn new() -> Self {
        Self {
            input_device: "Default".to_string(),
            output_device: "Default".to_string(),
            input_devices: vec!["Default".to_string()],
            output_devices: vec!["Default".to_string()],
            drive_level: 0.8,
            ptt_method: "VOX".to_string(),
            ptt_port: String::new(),
            ptt_delay_ms: 50,
            auto_tune: true,
            cwid_enabled: false,
            cwid_callsign: String::new(),
            cmd_port: 8300,
            data_port: 8301,
            pending_events: Vec::new(),
        }
    }

    /// Get and clear pending events
    pub fn take_events(&mut self) -> Vec<SettingsEvent> {
        std::mem::take(&mut self.pending_events)
    }

    /// Display the settings panel
    pub fn show(&mut self, ui: &mut Ui) {
        egui::ScrollArea::vertical().show(ui, |ui| {
            // Audio settings
            ui.collapsing("Audio Settings", |ui| {
                ui.horizontal(|ui| {
                    ui.label("Input Device:");
                    egui::ComboBox::from_id_salt("input_device")
                        .selected_text(&self.input_device)
                        .show_ui(ui, |ui: &mut egui::Ui| {
                            for device in &self.input_devices {
                                ui.selectable_value(&mut self.input_device, device.clone(), device);
                            }
                        });
                });

                ui.horizontal(|ui| {
                    ui.label("Output Device:");
                    egui::ComboBox::from_id_salt("output_device")
                        .selected_text(&self.output_device)
                        .show_ui(ui, |ui: &mut egui::Ui| {
                            for device in &self.output_devices {
                                ui.selectable_value(&mut self.output_device, device.clone(), device);
                            }
                        });
                });

                ui.horizontal(|ui| {
                    ui.label("Drive Level:");
                    ui.add(egui::Slider::new(&mut self.drive_level, 0.0..=1.0)
                        .show_value(true));
                });

                if ui.button("Refresh Devices").clicked() {
                    self.pending_events.push(SettingsEvent::RefreshDevices);
                }
            });

            ui.separator();

            // PTT settings
            ui.collapsing("PTT Settings", |ui| {
                ui.horizontal(|ui| {
                    ui.label("Method:");
                    egui::ComboBox::from_id_salt("ptt_method")
                        .selected_text(&self.ptt_method)
                        .show_ui(ui, |ui: &mut egui::Ui| {
                            ui.selectable_value(&mut self.ptt_method, "VOX".to_string(), "VOX");
                            ui.selectable_value(&mut self.ptt_method, "RTS".to_string(), "RTS");
                            ui.selectable_value(&mut self.ptt_method, "DTR".to_string(), "DTR");
                        });
                });

                if self.ptt_method != "VOX" {
                    ui.horizontal(|ui| {
                        ui.label("Port:");
                        ui.text_edit_singleline(&mut self.ptt_port);
                    });
                }

                ui.horizontal(|ui| {
                    ui.label("Delay (ms):");
                    ui.add(egui::DragValue::new(&mut self.ptt_delay_ms)
                        .range(0..=500));
                });
            });

            ui.separator();

            // Modem settings
            ui.collapsing("Modem Settings", |ui| {
                ui.checkbox(&mut self.auto_tune, "Auto Tune");

                ui.horizontal(|ui| {
                    ui.checkbox(&mut self.cwid_enabled, "CW ID");
                    if self.cwid_enabled {
                        ui.text_edit_singleline(&mut self.cwid_callsign);
                    }
                });
            });

            ui.separator();

            // Network settings
            ui.collapsing("Network Settings", |ui| {
                ui.horizontal(|ui| {
                    ui.label("Command Port:");
                    ui.add(egui::DragValue::new(&mut self.cmd_port)
                        .range(1024..=65535));
                });

                ui.horizontal(|ui| {
                    ui.label("Data Port:");
                    ui.add(egui::DragValue::new(&mut self.data_port)
                        .range(1024..=65535));
                });
            });

            ui.separator();

            ui.horizontal(|ui| {
                if ui.button("Apply").clicked() {
                    self.pending_events.push(SettingsEvent::ApplySettings {
                        input_device: self.input_device.clone(),
                        output_device: self.output_device.clone(),
                        drive_level: self.drive_level,
                        ptt_method: self.ptt_method.clone(),
                        ptt_port: self.ptt_port.clone(),
                        ptt_delay_ms: self.ptt_delay_ms,
                        cmd_port: self.cmd_port,
                        data_port: self.data_port,
                    });
                }
                if ui.button("Reset").clicked() {
                    let events = std::mem::take(&mut self.pending_events);
                    *self = Self::new();
                    self.pending_events = events;
                }
            });
        });
    }

    /// Update device lists
    pub fn update_devices(&mut self, inputs: Vec<String>, outputs: Vec<String>) {
        self.input_devices = inputs;
        self.output_devices = outputs;

        if !self.input_devices.contains(&self.input_device) {
            self.input_device = self.input_devices.first()
                .cloned()
                .unwrap_or_else(|| "Default".to_string());
        }

        if !self.output_devices.contains(&self.output_device) {
            self.output_device = self.output_devices.first()
                .cloned()
                .unwrap_or_else(|| "Default".to_string());
        }
    }
}

impl Default for SettingsPanel {
    fn default() -> Self {
        Self::new()
    }
}
