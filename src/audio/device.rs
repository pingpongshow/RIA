//! Audio device management

use super::{AudioConfig, AudioStats, RingBuffer, Sample};
use anyhow::Result;
use cpal::traits::{DeviceTrait, HostTrait, StreamTrait};
use cpal::{Device, Host, Stream, StreamConfig};
use crossbeam_channel::{bounded, Receiver, Sender};
use log::{error, info, warn};
use parking_lot::Mutex;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

/// Represents an audio device
#[derive(Debug, Clone)]
pub struct AudioDevice {
    pub name: String,
    pub is_input: bool,
    pub is_output: bool,
    pub sample_rates: Vec<u32>,
}

/// Main audio engine handling input/output streams
pub struct AudioEngine {
    host: Host,
    input_device: Option<Device>,
    output_device: Option<Device>,
    input_stream: Option<Stream>,
    output_stream: Option<Stream>,
    config: AudioConfig,

    // Ring buffers for audio data
    rx_buffer: Arc<Mutex<RingBuffer<Sample>>>,
    tx_buffer: Arc<Mutex<RingBuffer<Sample>>>,

    // Channels for inter-thread communication
    rx_sender: Sender<Vec<Sample>>,
    rx_receiver: Receiver<Vec<Sample>>,
    tx_sender: Sender<Vec<Sample>>,
    tx_receiver: Receiver<Vec<Sample>>,

    // State
    running: Arc<AtomicBool>,
    stats: Arc<Mutex<AudioStats>>,
}

impl AudioEngine {
    /// Create a new audio engine with the given configuration
    pub fn new(config: &crate::Config) -> Result<Self> {
        let host = cpal::default_host();
        info!("Audio host: {}", host.id().name());

        let audio_config = AudioConfig {
            sample_rate: config.sample_rate,
            buffer_size: 1024,
            channels: 1,
        };

        // Find devices
        let input_device = Self::find_input_device(&host, config.audio_input.as_deref())?;
        let output_device = Self::find_output_device(&host, config.audio_output.as_deref())?;

        if let Some(ref dev) = input_device {
            info!("Input device: {}", dev.name().unwrap_or_default());
        }
        if let Some(ref dev) = output_device {
            info!("Output device: {}", dev.name().unwrap_or_default());
        }

        // Create ring buffers (1 second of audio at sample rate)
        let buffer_capacity = config.sample_rate as usize;
        let rx_buffer = Arc::new(Mutex::new(RingBuffer::new(buffer_capacity)));
        let tx_buffer = Arc::new(Mutex::new(RingBuffer::new(buffer_capacity)));

        // Create channels
        // TX channel needs capacity for longest frame: FSK mode 1 = 264618 samples / 1024 = ~259 chunks
        // Use 300 for headroom
        let (rx_sender, rx_receiver) = bounded(16);
        let (tx_sender, tx_receiver) = bounded(300);

        Ok(Self {
            host,
            input_device,
            output_device,
            input_stream: None,
            output_stream: None,
            config: audio_config,
            rx_buffer,
            tx_buffer,
            rx_sender,
            rx_receiver,
            tx_sender,
            tx_receiver,
            running: Arc::new(AtomicBool::new(false)),
            stats: Arc::new(Mutex::new(AudioStats::default())),
        })
    }

    /// List available audio devices
    pub fn list_devices() -> Result<Vec<AudioDevice>> {
        let host = cpal::default_host();
        let mut devices = Vec::new();

        for device in host.devices()? {
            let name = device.name().unwrap_or_else(|_| "Unknown".to_string());

            let is_input = device.default_input_config().is_ok();
            let is_output = device.default_output_config().is_ok();

            let mut sample_rates = Vec::new();
            for rate in [8000, 16000, 22050, 44100, 48000, 96000] {
                if is_input {
                    if device.supported_input_configs().map(|mut c| {
                        c.any(|c| c.min_sample_rate().0 <= rate && c.max_sample_rate().0 >= rate)
                    }).unwrap_or(false) {
                        sample_rates.push(rate);
                    }
                }
            }

            devices.push(AudioDevice {
                name,
                is_input,
                is_output,
                sample_rates,
            });
        }

        Ok(devices)
    }

    /// List input devices only
    pub fn list_input_devices() -> Vec<String> {
        let mut devices = vec!["Default".to_string()];
        if let Ok(all_devices) = Self::list_devices() {
            for device in all_devices {
                if device.is_input {
                    devices.push(device.name);
                }
            }
        }
        devices
    }

    /// List output devices only
    pub fn list_output_devices() -> Vec<String> {
        let mut devices = vec!["Default".to_string()];
        if let Ok(all_devices) = Self::list_devices() {
            for device in all_devices {
                if device.is_output {
                    devices.push(device.name);
                }
            }
        }
        devices
    }

    fn find_input_device(host: &Host, name: Option<&str>) -> Result<Option<Device>> {
        if let Some(name) = name {
            for device in host.input_devices()? {
                if device.name().map(|n| n.contains(name)).unwrap_or(false) {
                    return Ok(Some(device));
                }
            }
            warn!("Requested input device '{}' not found, using default", name);
        }
        Ok(host.default_input_device())
    }

    fn find_output_device(host: &Host, name: Option<&str>) -> Result<Option<Device>> {
        if let Some(name) = name {
            for device in host.output_devices()? {
                if device.name().map(|n| n.contains(name)).unwrap_or(false) {
                    return Ok(Some(device));
                }
            }
            warn!("Requested output device '{}' not found, using default", name);
        }
        Ok(host.default_output_device())
    }

    /// Start audio streams
    pub fn start(&mut self) -> Result<()> {
        if self.running.load(Ordering::SeqCst) {
            return Ok(());
        }

        self.running.store(true, Ordering::SeqCst);

        // Start input stream
        self.start_input_stream()?;

        // Start output stream
        self.start_output_stream()?;

        Ok(())
    }

    /// Stop audio streams
    pub fn stop(&mut self) {
        self.running.store(false, Ordering::SeqCst);
        self.input_stream = None;
        self.output_stream = None;
        info!("Audio streams stopped");
    }

    /// Get receiver channel for RX samples
    pub fn rx_channel(&self) -> Receiver<Vec<Sample>> {
        self.rx_receiver.clone()
    }

    /// Get sender channel for TX samples
    pub fn tx_channel(&self) -> Sender<Vec<Sample>> {
        self.tx_sender.clone()
    }

    /// Queue samples for transmission
    pub fn queue_tx(&self, samples: &[Sample]) {
        self.tx_buffer.lock().write(samples);
    }

    /// Read received samples
    pub fn read_rx(&self, buffer: &mut [Sample]) -> usize {
        self.rx_buffer.lock().read(buffer)
    }

    /// Get current audio statistics
    pub fn stats(&self) -> AudioStats {
        self.stats.lock().clone()
    }

    /// Check if audio engine is running
    pub fn is_running(&self) -> bool {
        self.running.load(Ordering::SeqCst)
    }

    /// Get current sample rate
    pub fn sample_rate(&self) -> u32 {
        self.config.sample_rate
    }

    /// Change input device (restarts input stream)
    pub fn set_input_device(&mut self, name: Option<&str>) -> Result<()> {
        info!("set_input_device called with: {:?}", name);

        // Stop current input stream explicitly - pause first, then drop
        if let Some(stream) = self.input_stream.take() {
            info!("Pausing and dropping current input stream");
            // Pause the stream before dropping to ensure callbacks stop
            let _ = stream.pause();
            // Small delay to ensure stream is fully stopped
            std::thread::sleep(std::time::Duration::from_millis(50));
            drop(stream);
            info!("Old input stream dropped");
        }

        // Clear the ring buffer to avoid stale data
        self.rx_buffer.lock().clear();

        // Find new device - list all available devices for debugging
        info!("Available input devices:");
        if let Ok(devices) = self.host.input_devices() {
            for dev in devices {
                info!("  - {}", dev.name().unwrap_or_else(|_| "Unknown".to_string()));
            }
        }

        // Find the requested device
        self.input_device = Self::find_input_device(&self.host, name)?;

        match &self.input_device {
            Some(dev) => {
                let dev_name = dev.name().unwrap_or_else(|_| "Unknown".to_string());
                info!("Input device set to: {}", dev_name);
                // Verify this is the device we requested
                if let Some(requested) = name {
                    if !dev_name.contains(requested) {
                        warn!("WARNING: Device '{}' does not match requested '{}'", dev_name, requested);
                    }
                }
            },
            None => warn!("No input device found for: {:?}", name),
        }

        // Restart if we were running
        if self.running.load(Ordering::SeqCst) {
            info!("Restarting input stream on new device");
            self.start_input_stream()?;
        }

        Ok(())
    }

    /// Change output device (restarts output stream)
    pub fn set_output_device(&mut self, name: Option<&str>) -> Result<()> {
        info!("set_output_device called with: {:?}", name);

        // Stop current output stream explicitly
        if let Some(stream) = self.output_stream.take() {
            info!("Stopping current output stream");
            drop(stream);
        }

        // Clear the ring buffer
        self.tx_buffer.lock().clear();

        // Find new device
        self.output_device = Self::find_output_device(&self.host, name)?;

        match &self.output_device {
            Some(dev) => info!("Output device set to: {}", dev.name().unwrap_or_default()),
            None => warn!("No output device found for: {:?}", name),
        }

        // Restart if we were running
        if self.running.load(Ordering::SeqCst) {
            info!("Restarting output stream");
            self.start_output_stream()?;
        }

        Ok(())
    }

    /// Start input stream only
    fn start_input_stream(&mut self) -> Result<()> {
        if let Some(ref device) = self.input_device {
            let device_name = device.name().unwrap_or_else(|_| "Unknown".to_string());
            info!("Building input stream on device: {}", device_name);

            let config = StreamConfig {
                channels: self.config.channels,
                sample_rate: cpal::SampleRate(self.config.sample_rate),
                buffer_size: cpal::BufferSize::Fixed(self.config.buffer_size as u32),
            };
            info!("Stream config: {:?}", config);

            let rx_buffer = self.rx_buffer.clone();
            let rx_sender = self.rx_sender.clone();
            let stats = self.stats.clone();
            let dev_name_for_log = device_name.clone();

            let stream = device.build_input_stream(
                &config,
                move |data: &[f32], _: &cpal::InputCallbackInfo| {
                    // Log first callback to confirm stream is active
                    static LOGGED: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);
                    if !LOGGED.swap(true, std::sync::atomic::Ordering::SeqCst) {
                        log::info!("First audio callback from device: {}", dev_name_for_log);
                    }

                    // Calculate level
                    let level = data.iter()
                        .map(|s| s.abs())
                        .max_by(|a, b| a.partial_cmp(b).unwrap())
                        .unwrap_or(0.0);
                    let level_db = if level > 0.0 { 20.0 * level.log10() } else { -60.0 };
                    stats.lock().rx_level_db = level_db;

                    // Store in ring buffer
                    rx_buffer.lock().write(data);

                    // Send to processing channel
                    let _ = rx_sender.try_send(data.to_vec());
                },
                move |err| {
                    error!("Input stream error: {}", err);
                },
                None,
            )?;

            stream.play()?;
            self.input_stream = Some(stream);
            info!("Input stream started and playing on: {}", device_name);
        } else {
            warn!("No input device available to start stream");
        }
        Ok(())
    }

    /// Start output stream only
    fn start_output_stream(&mut self) -> Result<()> {
        if let Some(ref device) = self.output_device {
            let config = StreamConfig {
                channels: self.config.channels,
                sample_rate: cpal::SampleRate(self.config.sample_rate),
                buffer_size: cpal::BufferSize::Fixed(self.config.buffer_size as u32),
            };

            let tx_buffer = self.tx_buffer.clone();
            let tx_receiver = self.tx_receiver.clone();
            let stats = self.stats.clone();

            // Track samples output for debugging
            let total_samples_output = std::sync::Arc::new(std::sync::atomic::AtomicUsize::new(0));
            let total_samples_clone = total_samples_output.clone();
            let callback_count = std::sync::Arc::new(std::sync::atomic::AtomicUsize::new(0));
            let callback_count_clone = callback_count.clone();

            let stream = device.build_output_stream(
                &config,
                move |data: &mut [f32], _: &cpal::OutputCallbackInfo| {
                    let cb_num = callback_count_clone.fetch_add(1, std::sync::atomic::Ordering::Relaxed);

                    // Try to get data from channel first - drain ALL available chunks
                    let mut filled = 0;
                    let mut chunks_used = 0;
                    while filled < data.len() {
                        if let Ok(samples) = tx_receiver.try_recv() {
                            let copy_len = samples.len().min(data.len() - filled);
                            data[filled..filled + copy_len].copy_from_slice(&samples[..copy_len]);
                            filled += copy_len;
                            chunks_used += 1;
                        } else {
                            break;
                        }
                    }

                    let prev_total = total_samples_clone.fetch_add(filled, std::sync::atomic::Ordering::Relaxed);

                    // Log every 10th callback when there's actual audio
                    if filled > 0 && cb_num % 10 == 0 {
                        let rms: f32 = (data[..filled].iter().map(|s| s * s).sum::<f32>() / filled as f32).sqrt();
                        eprintln!("[TX-AUDIO] callback={}, filled={}, chunks={}, total={}, rms={:.4}",
                            cb_num, filled, chunks_used, prev_total + filled, rms);
                    }

                    // Zero any remaining samples
                    for sample in &mut data[filled..] {
                        *sample = 0.0;
                    }

                    // Calculate level
                    let level = data.iter()
                        .map(|s| s.abs())
                        .max_by(|a, b| a.partial_cmp(b).unwrap())
                        .unwrap_or(0.0);
                    let level_db = if level > 0.0 { 20.0 * level.log10() } else { -60.0 };
                    stats.lock().tx_level_db = level_db;
                },
                move |err| {
                    error!("Output stream error: {}", err);
                },
                None,
            )?;

            stream.play()?;
            self.output_stream = Some(stream);
            info!("Output stream started");
        }
        Ok(())
    }
}

impl Drop for AudioEngine {
    fn drop(&mut self) {
        self.stop();
    }
}
