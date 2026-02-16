//! PTT (Push-to-Talk) control

use anyhow::Result;
use log::debug;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

/// PTT control method
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PttMethod {
    /// No PTT control (always transmit when audio present)
    None,
    /// Voice-operated transmit
    Vox,
    /// RTS line on serial port
    Rts,
    /// DTR line on serial port
    Dtr,
}

/// PTT controller
pub struct PttController {
    method: PttMethod,
    active: Arc<AtomicBool>,

    // VOX settings
    vox_threshold: f32,
    vox_delay_ms: u64,
    vox_hangtime: Option<Instant>,

    // Serial port for RTS/DTR
    #[cfg(feature = "serial")]
    serial_port: Option<Box<dyn serialport::SerialPort>>,
    serial_port_name: Option<String>,
}

impl PttController {
    /// Create a new PTT controller
    pub fn new(method: PttMethod) -> Self {
        Self {
            method,
            active: Arc::new(AtomicBool::new(false)),
            vox_threshold: -30.0, // dB
            vox_delay_ms: 200,
            vox_hangtime: None,
            #[cfg(feature = "serial")]
            serial_port: None,
            serial_port_name: None,
        }
    }

    /// Set VOX threshold in dB
    pub fn set_vox_threshold(&mut self, threshold_db: f32) {
        self.vox_threshold = threshold_db;
    }

    /// Set VOX delay in milliseconds
    pub fn set_vox_delay(&mut self, delay_ms: u64) {
        self.vox_delay_ms = delay_ms;
    }

    /// Configure serial port for PTT
    pub fn set_serial_port(&mut self, port_name: &str) -> Result<()> {
        self.serial_port_name = Some(port_name.to_string());

        #[cfg(feature = "serial")]
        {
            let port = serialport::new(port_name, 9600)
                .timeout(Duration::from_millis(100))
                .open()?;
            self.serial_port = Some(port);
            info!("PTT serial port opened: {}", port_name);
        }

        Ok(())
    }

    /// Activate PTT (key transmitter)
    pub fn key(&mut self) -> Result<()> {
        if self.active.load(Ordering::SeqCst) {
            return Ok(()); // Already keyed
        }

        debug!("PTT: Keying transmitter");

        match self.method {
            PttMethod::None => {}
            PttMethod::Vox => {
                // VOX doesn't need explicit keying
            }
            PttMethod::Rts => {
                #[cfg(feature = "serial")]
                if let Some(ref mut port) = self.serial_port {
                    port.write_request_to_send(true)?;
                }
            }
            PttMethod::Dtr => {
                #[cfg(feature = "serial")]
                if let Some(ref mut port) = self.serial_port {
                    port.write_data_terminal_ready(true)?;
                }
            }
        }

        self.active.store(true, Ordering::SeqCst);
        Ok(())
    }

    /// Deactivate PTT (unkey transmitter)
    pub fn unkey(&mut self) -> Result<()> {
        if !self.active.load(Ordering::SeqCst) {
            return Ok(()); // Already unkeyed
        }

        debug!("PTT: Unkeying transmitter");

        match self.method {
            PttMethod::None => {}
            PttMethod::Vox => {}
            PttMethod::Rts => {
                #[cfg(feature = "serial")]
                if let Some(ref mut port) = self.serial_port {
                    port.write_request_to_send(false)?;
                }
            }
            PttMethod::Dtr => {
                #[cfg(feature = "serial")]
                if let Some(ref mut port) = self.serial_port {
                    port.write_data_terminal_ready(false)?;
                }
            }
        }

        self.active.store(false, Ordering::SeqCst);
        Ok(())
    }

    /// Update VOX state based on audio level
    pub fn update_vox(&mut self, level_db: f32) {
        if self.method != PttMethod::Vox {
            return;
        }

        if level_db > self.vox_threshold {
            // Audio detected
            self.vox_hangtime = Some(Instant::now() + Duration::from_millis(self.vox_delay_ms));
            if !self.active.load(Ordering::SeqCst) {
                let _ = self.key();
            }
        } else {
            // No audio
            if let Some(hangtime) = self.vox_hangtime {
                if Instant::now() > hangtime {
                    let _ = self.unkey();
                    self.vox_hangtime = None;
                }
            }
        }
    }

    /// Check if PTT is currently active
    pub fn is_active(&self) -> bool {
        self.active.load(Ordering::SeqCst)
    }

    /// Get shared active state
    pub fn active_state(&self) -> Arc<AtomicBool> {
        self.active.clone()
    }
}

impl Drop for PttController {
    fn drop(&mut self) {
        // Ensure PTT is released on drop
        let _ = self.unkey();
    }
}
