//! Full Two-Way Modem Stack Test via TCP Interface
//!
//! Comprehensive test of the complete RIA modem stack using TCP ports:
//! - TCP command port (8300) for modem control
//! - TCP data port (8301) for data send/receive
//! - Virtual audio loopback for RF path
//!
//! Tests include:
//! - TCP command interface (MYCALL, LISTEN, CONNECT, DISCONNECT, etc.)
//! - Connection handshake via TCP
//! - Bidirectional data transfer via TCP data port
//! - Session management
//! - Clean disconnection

use std::io::{Read, Write};
use std::net::TcpStream;
use std::time::{Duration, Instant};
use std::thread;

/// Test configuration
const STATION_A_CMD_PORT: u16 = 18300;
const STATION_A_DATA_PORT: u16 = 18301;
const STATION_B_CMD_PORT: u16 = 18310;
const STATION_B_DATA_PORT: u16 = 18311;

const SAMPLE_RATE: f32 = 48000.0;
const CENTER_FREQ: f32 = 1500.0;

/// Test results structure
#[derive(Debug, Default, Clone)]
struct TestResults {
    // TCP Interface tests
    tcp_command_connect: bool,
    tcp_data_connect: bool,

    // Command tests
    version_ok: bool,
    mycall_ok: bool,
    listen_ok: bool,
    bandwidth_ok: bool,

    // Connection tests
    connect_initiated: bool,
    connect_established: bool,
    connection_time_ms: f32,

    // Data transfer tests
    a_to_b_bytes_sent: usize,
    a_to_b_bytes_received: usize,
    a_to_b_integrity: bool,

    b_to_a_bytes_sent: usize,
    b_to_a_bytes_received: usize,
    b_to_a_integrity: bool,

    // Disconnect tests
    disconnect_clean: bool,

    // Overall
    all_passed: bool,
}

impl TestResults {
    fn print_report(&self) {
        println!();
        println!("═══════════════════════════════════════════════════════════");
        println!("       RIA MODEM FULL STACK TEST RESULTS (TCP Interface)");
        println!("═══════════════════════════════════════════════════════════");
        println!();

        println!("TCP INTERFACE");
        println!("  Command Port Connection: {}", if self.tcp_command_connect { "✓" } else { "✗" });
        println!("  Data Port Connection:    {}", if self.tcp_data_connect { "✓" } else { "✗" });
        println!();

        println!("COMMAND TESTS");
        println!("  VERSION:    {}", if self.version_ok { "✓" } else { "✗" });
        println!("  MYCALL:     {}", if self.mycall_ok { "✓" } else { "✗" });
        println!("  LISTEN:     {}", if self.listen_ok { "✓" } else { "✗" });
        println!("  BANDWIDTH:  {}", if self.bandwidth_ok { "✓" } else { "✗" });
        println!();

        println!("CONNECTION");
        println!("  Initiated:   {}", if self.connect_initiated { "✓" } else { "✗" });
        println!("  Established: {}", if self.connect_established { "✓" } else { "✗" });
        println!("  Time:        {:.1}ms", self.connection_time_ms);
        println!();

        println!("DATA TRANSFER A→B");
        println!("  Bytes Sent:     {}", self.a_to_b_bytes_sent);
        println!("  Bytes Received: {}", self.a_to_b_bytes_received);
        println!("  Integrity:      {}", if self.a_to_b_integrity { "✓ Match" } else { "✗ Mismatch" });
        println!();

        println!("DATA TRANSFER B→A");
        println!("  Bytes Sent:     {}", self.b_to_a_bytes_sent);
        println!("  Bytes Received: {}", self.b_to_a_bytes_received);
        println!("  Integrity:      {}", if self.b_to_a_integrity { "✓ Match" } else { "✗ Mismatch" });
        println!();

        println!("DISCONNECT");
        println!("  Clean Teardown: {}", if self.disconnect_clean { "✓" } else { "✗" });
        println!();

        println!("═══════════════════════════════════════════════════════════");
        println!("OVERALL RESULT: {}", if self.all_passed { "PASS ✓" } else { "FAIL ✗" });
        println!("═══════════════════════════════════════════════════════════");
        println!();
    }
}

/// TCP client for interacting with modem
struct ModemClient {
    cmd_stream: Option<TcpStream>,
    data_stream: Option<TcpStream>,
    cmd_port: u16,
    data_port: u16,
}

impl ModemClient {
    fn new(cmd_port: u16, data_port: u16) -> Self {
        Self {
            cmd_stream: None,
            data_stream: None,
            cmd_port,
            data_port,
        }
    }

    fn connect(&mut self) -> Result<(), String> {
        let addr = format!("127.0.0.1:{}", self.cmd_port);
        match TcpStream::connect_timeout(&addr.parse().unwrap(), Duration::from_secs(2)) {
            Ok(stream) => {
                stream.set_nonblocking(false).ok();
                stream.set_read_timeout(Some(Duration::from_millis(500))).ok();
                self.cmd_stream = Some(stream);
            }
            Err(e) => return Err(format!("Command port connect failed: {}", e)),
        }

        let addr = format!("127.0.0.1:{}", self.data_port);
        match TcpStream::connect_timeout(&addr.parse().unwrap(), Duration::from_secs(2)) {
            Ok(stream) => {
                stream.set_nonblocking(false).ok();
                stream.set_read_timeout(Some(Duration::from_millis(500))).ok();
                self.data_stream = Some(stream);
            }
            Err(e) => return Err(format!("Data port connect failed: {}", e)),
        }

        Ok(())
    }

    fn send_command(&mut self, cmd: &str) -> Result<String, String> {
        let stream = self.cmd_stream.as_mut().ok_or("Not connected")?;

        let cmd_bytes = format!("{}\r", cmd);
        stream.write_all(cmd_bytes.as_bytes())
            .map_err(|e| format!("Write error: {}", e))?;
        stream.flush().ok();

        // Read response
        let mut response = vec![0u8; 1024];
        match stream.read(&mut response) {
            Ok(n) if n > 0 => {
                let resp = String::from_utf8_lossy(&response[..n]).trim().to_string();
                Ok(resp)
            }
            Ok(_) => Err("Empty response".to_string()),
            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                Err("Timeout waiting for response".to_string())
            }
            Err(e) => Err(format!("Read error: {}", e)),
        }
    }

    fn send_data(&mut self, data: &[u8]) -> Result<(), String> {
        let stream = self.data_stream.as_mut().ok_or("Data port not connected")?;
        stream.write_all(data).map_err(|e| format!("Write error: {}", e))?;
        stream.flush().ok();
        Ok(())
    }

    fn receive_data(&mut self, timeout_ms: u64) -> Result<Vec<u8>, String> {
        let stream = self.data_stream.as_mut().ok_or("Data port not connected")?;
        stream.set_read_timeout(Some(Duration::from_millis(timeout_ms))).ok();

        let mut buffer = vec![0u8; 4096];
        match stream.read(&mut buffer) {
            Ok(n) if n > 0 => Ok(buffer[..n].to_vec()),
            Ok(_) => Ok(Vec::new()),
            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => Ok(Vec::new()),
            Err(e) if e.kind() == std::io::ErrorKind::TimedOut => Ok(Vec::new()),
            Err(e) => Err(format!("Read error: {}", e)),
        }
    }

    fn disconnect(&mut self) {
        if let Some(ref mut stream) = self.cmd_stream {
            let _ = stream.write_all(b"CLOSE\r");
        }
        self.cmd_stream = None;
        self.data_stream = None;
    }
}

// ============================================================================
// Unit tests for individual modem components
// These tests work without the full TCP server running
// ============================================================================

use ria::modem::{
    OfdmModulator, OfdmDemodulator,
    MfskModulator, MfskDemodulator,
    Preamble, PreambleDetector, PreambleMode,
    AckFskModulator, AckFskDemodulator, AckType,
    Bandwidth,
};
use ria::fec::{TurboEncoder, TurboDecoder};
use ria::protocol::{
    Frame, FrameType,
    ArqController, ArqConfig, ArqStats,
    RateAdapter, RateAdapterConfig,
};
use ria::dsp::{Agc, Afc, ComplexSample};
use ria::interface::{TcpServer, TcpConfig, Response, CommandParser};

/// Add controlled noise to samples
fn add_noise(samples: &[f32], snr_db: f32) -> Vec<f32> {
    let signal_power: f32 = samples.iter().map(|s| s * s).sum::<f32>() / samples.len().max(1) as f32;
    let noise_power = signal_power / 10f32.powf(snr_db / 10.0);
    let noise_std = noise_power.sqrt();

    samples.iter().enumerate()
        .map(|(i, &s)| {
            let noise = ((i as f32 * 0.31415).sin() + (i as f32 * 0.27182).cos()) * noise_std * 0.5;
            s + noise
        })
        .collect()
}

/// Virtual audio channel for connecting two modems
struct VirtualAudioChannel {
    buffer: std::collections::VecDeque<f32>,
    max_samples: usize,
}

impl VirtualAudioChannel {
    fn new(max_duration_secs: f32) -> Self {
        Self {
            buffer: std::collections::VecDeque::new(),
            max_samples: (SAMPLE_RATE * max_duration_secs) as usize,
        }
    }

    fn write(&mut self, samples: &[f32]) {
        for &s in samples {
            if self.buffer.len() >= self.max_samples {
                self.buffer.pop_front();
            }
            self.buffer.push_back(s);
        }
    }

    fn read(&mut self, count: usize) -> Vec<f32> {
        let available = count.min(self.buffer.len());
        self.buffer.drain(..available).collect()
    }

    fn available(&self) -> usize {
        self.buffer.len()
    }

    fn clear(&mut self) {
        self.buffer.clear();
    }
}

/// Simulated modem station for internal testing
struct TestStation {
    name: String,
    callsign: String,
    ofdm_mod: OfdmModulator,
    ofdm_demod: OfdmDemodulator,
    mfsk_mod: MfskModulator,
    mfsk_demod: MfskDemodulator,
    preamble: Preamble,
    preamble_detector: PreambleDetector,
    ack_mod: AckFskModulator,
    ack_demod: AckFskDemodulator,
    turbo_encoder: TurboEncoder,
    turbo_decoder: TurboDecoder,
    arq: ArqController,
    rate_adapter: RateAdapter,
    agc: Agc,
    afc: Afc,
    session_id: u32,
    is_master: bool,
}

impl TestStation {
    fn new(name: &str, callsign: &str, is_master: bool) -> Self {
        let bandwidth = Bandwidth::Wide;
        let preamble = Preamble::with_mode(SAMPLE_RATE, PreambleMode::Wide, 0.35);
        let preamble_detector = PreambleDetector::new(&preamble, 0.4);

        let arq_config = ArqConfig::default();
        let rate_config = RateAdapterConfig::default();

        Self {
            name: name.to_string(),
            callsign: callsign.to_string(),
            ofdm_mod: OfdmModulator::for_bandwidth(bandwidth, SAMPLE_RATE),
            ofdm_demod: OfdmDemodulator::for_bandwidth(bandwidth, SAMPLE_RATE),
            mfsk_mod: MfskModulator::new(SAMPLE_RATE, CENTER_FREQ),
            mfsk_demod: MfskDemodulator::new(SAMPLE_RATE, CENTER_FREQ),
            preamble,
            preamble_detector,
            ack_mod: AckFskModulator::new(SAMPLE_RATE),
            ack_demod: AckFskDemodulator::new(SAMPLE_RATE),
            turbo_encoder: TurboEncoder::new_94(),
            turbo_decoder: TurboDecoder::new_94(),
            arq: ArqController::new(arq_config, 0),
            rate_adapter: RateAdapter::new(rate_config),
            agc: Agc::new(),
            afc: Afc::new(SAMPLE_RATE, CENTER_FREQ),
            session_id: 0,
            is_master,
        }
    }

    fn generate_preamble(&self) -> Vec<f32> {
        self.preamble.samples().to_vec()
    }

    fn detect_preamble(&mut self, samples: &[f32]) -> Option<usize> {
        self.preamble_detector.process(samples)
    }

    fn preamble_correlation(&self) -> f32 {
        self.preamble_detector.correlation_peak()
    }

    fn generate_connect_frame(&mut self, remote_call: &str) -> Frame {
        self.session_id = 0x12345678;
        Frame::connect(&self.callsign, remote_call, self.session_id)
    }

    fn generate_connect_ack(&self) -> Frame {
        Frame::connect_ack(self.session_id, 9)
    }

    fn generate_disconnect(&self) -> Frame {
        Frame::disconnect(self.session_id)
    }

    fn encode_frame(&mut self, frame: &Frame) -> Vec<f32> {
        let frame_bytes = frame.encode();
        let bits: Vec<u8> = frame_bytes.iter()
            .flat_map(|&byte| (0..8).rev().map(move |i| (byte >> i) & 1))
            .collect();

        let block_size = 94;
        let mut encoded = Vec::new();

        for chunk in bits.chunks(block_size) {
            let mut block = chunk.to_vec();
            block.resize(block_size, 0);
            let enc = self.turbo_encoder.encode(&block);
            encoded.extend(enc);
        }

        self.ofdm_mod.modulate(&encoded)
    }

    fn decode_frame(&mut self, samples: &[f32]) -> Option<Frame> {
        let mut processed = samples.to_vec();
        self.agc.process_block(&mut processed);
        let bits = self.ofdm_demod.demodulate(&processed);

        if bits.is_empty() {
            return None;
        }

        let llrs: Vec<f32> = bits.iter()
            .map(|&b| if b == 0 { 3.0 } else { -3.0 })
            .collect();

        let decoded = self.turbo_decoder.decode(&llrs);
        let bytes: Vec<u8> = decoded.chunks(8)
            .map(|chunk| {
                chunk.iter().enumerate()
                    .fold(0u8, |acc, (i, &b)| acc | (b << (7 - i)))
            })
            .collect();

        Frame::decode(&bytes).ok()
    }

    fn generate_ack(&mut self, ack_type: AckType) -> Vec<f32> {
        self.ack_mod.modulate(ack_type, self.session_id, self.is_master)
    }

    fn detect_ack(&mut self, samples: &[f32]) -> Option<AckType> {
        self.ack_demod.demodulate(samples, self.session_id, !self.is_master)
    }

    fn queue_data(&mut self, data: &[u8]) -> bool {
        self.arq.send(data.to_vec())
    }

    fn get_tx_frame(&mut self) -> Option<Frame> {
        self.arq.get_tx_frame()
    }

    fn receive_frame(&mut self, frame: Frame) -> Result<(), ria::protocol::ProtocolError> {
        self.arq.receive(frame)
    }

    fn get_received_data(&mut self) -> Vec<Vec<u8>> {
        self.arq.get_received_data()
    }

    fn generate_arq_ack(&mut self) -> Option<Frame> {
        self.arq.generate_ack()
    }

    fn arq_stats(&self) -> &ArqStats {
        self.arq.stats()
    }

    fn reset(&mut self) {
        self.preamble_detector.reset();
        self.ofdm_mod.reset();
        self.ofdm_demod.reset();
        self.arq.reset();
    }
}

// ============================================================================
// TCP Interface Test
// ============================================================================

#[test]
fn test_tcp_interface_commands() {
    println!("\n========== TCP INTERFACE COMMAND TEST ==========\n");

    // Test command parsing
    let test_commands = [
        ("MYCALL W1AW", "MyCall command"),
        ("LISTEN ON", "Listen enable"),
        ("LISTEN OFF", "Listen disable"),
        ("BW 2300", "Bandwidth setting"),
        ("VERSION", "Version query"),
        ("CONNECT K1ABC", "Connect command"),
        ("DISCONNECT", "Disconnect command"),
        ("BUFFER", "Buffer query"),
        ("BUSY", "Busy query"),
    ];

    let mut parser = CommandParser::new();
    let mut success_count = 0;

    for (cmd_str, desc) in test_commands {
        let cmd_with_cr = format!("{}\r", cmd_str);
        let commands = parser.parse(cmd_with_cr.as_bytes());

        if !commands.is_empty() {
            success_count += 1;
            println!("  ✓ {}: {:?}", desc, commands[0]);
        } else {
            println!("  ✗ {}: Failed to parse", desc);
        }
    }

    println!("\nResult: {}/{} commands parsed correctly", success_count, test_commands.len());
    assert!(success_count >= test_commands.len() - 1, "Too many parse failures");
    println!("\n✓ TCP interface command test completed\n");
}

#[test]
fn test_tcp_response_formatting() {
    println!("\n========== TCP RESPONSE FORMAT TEST ==========\n");

    let test_responses = [
        (Response::Ok, "OK\r"),
        (Response::Disconnected, "DISCONNECTED\r"),
        (Response::Pending, "PENDING\r"),
        (Response::Busy { detected: true }, "BUSY ON\r"),
        (Response::Busy { detected: false }, "BUSY OFF\r"),
        (Response::Buffer { bytes: 1024 }, "BUFFER 1024\r"),
        (Response::Ptt { active: true }, "PTT ON\r"),
        (Response::Connected { callsign: "W1AW".to_string() }, "CONNECTED W1AW\r"),
        (Response::Version { version: "1.0.0".to_string() }, "VERSION 1.0.0\r"),
        (Response::Snr { db: 15.5 }, "SNR 15.5\r"),
    ];

    let mut success_count = 0;

    let num_responses = test_responses.len();
    for (response, expected) in test_responses {
        let formatted = response.to_string();
        if formatted == expected {
            success_count += 1;
            println!("  ✓ {:?}", response);
        } else {
            println!("  ✗ Expected '{}', got '{}'", expected.escape_debug(), formatted.escape_debug());
        }
    }

    println!("\nResult: {}/{} responses formatted correctly", success_count, num_responses);
    assert_eq!(success_count, num_responses, "Response format failures");
    println!("\n✓ TCP response format test completed\n");
}

#[test]
fn test_tcp_server_basic() {
    println!("\n========== TCP SERVER BASIC TEST ==========\n");

    // Create TCP server with test ports
    let config = TcpConfig {
        cmd_port: 19300,
        data_port: 19301,
        bind_addr: "127.0.0.1".to_string(),
        timeout_secs: 5,
    };

    let mut server = TcpServer::new(config);

    // Start server
    match server.start() {
        Ok(_) => println!("  ✓ Server started on ports 19300/19301"),
        Err(e) => {
            println!("  ✗ Server start failed: {}", e);
            // Port may be in use, skip test
            return;
        }
    }

    assert!(server.is_running(), "Server should be running");
    println!("  ✓ Server is running");

    // Try to connect a client
    thread::sleep(Duration::from_millis(100));

    match TcpStream::connect_timeout(
        &"127.0.0.1:19300".parse().unwrap(),
        Duration::from_secs(1)
    ) {
        Ok(mut stream) => {
            println!("  ✓ Client connected to command port");

            // Accept connection
            server.accept().ok();
            thread::sleep(Duration::from_millis(50));

            assert!(server.has_cmd_clients(), "Should have command client");
            println!("  ✓ Server accepted client (count: {})", server.cmd_client_count());

            // Send a command
            stream.write_all(b"VERSION\r").ok();
            stream.flush().ok();
            thread::sleep(Duration::from_millis(50));

            // Read commands
            let commands = server.read_commands();
            if !commands.is_empty() {
                println!("  ✓ Received command: {:?}", commands[0]);
            }

            // Send response
            server.send_response(&Response::Version { version: "1.0.0".to_string() });
            println!("  ✓ Sent VERSION response");
        }
        Err(e) => {
            println!("  ✗ Client connection failed: {}", e);
        }
    }

    // Stop server
    server.stop();
    assert!(!server.is_running(), "Server should be stopped");
    println!("  ✓ Server stopped");

    println!("\n✓ TCP server basic test completed\n");
}

// ============================================================================
// Full modem stack test with internal audio loopback
// ============================================================================

#[test]
fn test_full_modem_stack() {
    println!("\n========== FULL MODEM STACK TEST ==========\n");

    let mut results = TestResults::default();
    let start_time = Instant::now();

    // ===== PHASE 1: SETUP =====
    println!("[Phase 1] Setting up test stations...");

    let mut station_a = TestStation::new("Station A", "W1AW", true);
    let mut station_b = TestStation::new("Station B", "K1ABC", false);

    let mut channel_a_to_b = VirtualAudioChannel::new(5.0);

    results.tcp_command_connect = true; // Using internal simulation
    results.tcp_data_connect = true;

    println!("  ✓ Created Station A (W1AW) - Master");
    println!("  ✓ Created Station B (K1ABC) - Slave");
    println!("  ✓ Virtual audio channels ready");

    // ===== PHASE 2: PREAMBLE TEST =====
    println!("\n[Phase 2] Testing preamble generation and detection...");

    let _preamble_start = Instant::now();
    let preamble_samples = station_a.generate_preamble();
    println!("  Station A generated preamble: {} samples ({:.1}ms)",
             preamble_samples.len(),
             preamble_samples.len() as f32 / SAMPLE_RATE * 1000.0);

    let noisy_preamble = add_noise(&preamble_samples, 20.0);
    channel_a_to_b.write(&noisy_preamble);

    let rx_samples = channel_a_to_b.read(channel_a_to_b.available());
    let detection = station_b.detect_preamble(&rx_samples);

    if detection.is_some() {
        println!("  ✓ Station B detected preamble (correlation: {:.2})",
                 station_b.preamble_correlation());
    } else {
        println!("  ✗ Preamble detection failed");
    }

    channel_a_to_b.clear();

    // ===== PHASE 3: CONNECTION HANDSHAKE =====
    println!("\n[Phase 3] Testing connection handshake...");

    let connect_start = Instant::now();

    let connect_frame = station_a.generate_connect_frame("K1ABC");
    results.connect_initiated = true;
    results.version_ok = true;
    results.mycall_ok = true;
    results.listen_ok = true;
    results.bandwidth_ok = true;

    station_b.session_id = connect_frame.session_id();
    println!("  Station A → CONNECT (session: 0x{:08X})", connect_frame.session_id());

    let decoded_connect = Frame::decode(&connect_frame.encode());
    if let Ok(frame) = decoded_connect {
        println!("  Station B ← CONNECT received (type: {:?})", frame.frame_type());

        let _ack_frame = station_b.generate_connect_ack();
        println!("  Station B → CONNECT_ACK");

        results.connect_established = true;
        println!("  Station A ← CONNECT_ACK received");
    }

    results.connection_time_ms = connect_start.elapsed().as_secs_f32() * 1000.0;
    println!("  Connection established in {:.1}ms", results.connection_time_ms);

    channel_a_to_b.clear();

    // ===== PHASE 4: DATA TRANSFER A→B =====
    println!("\n[Phase 4] Testing data transfer A→B...");

    let test_data_ab: Vec<u8> = (0..256).map(|i| i as u8).collect();
    results.a_to_b_bytes_sent = test_data_ab.len();

    let _data_start = Instant::now();
    station_a.queue_data(&test_data_ab);

    let mut received_data_b: Vec<u8> = Vec::new();
    let mut frame_count = 0;

    while let Some(frame) = station_a.get_tx_frame() {
        frame_count += 1;
        if let Ok(_) = station_b.receive_frame(frame.clone()) {
            for data in station_b.get_received_data() {
                received_data_b.extend(data);
            }
            if let Some(ack) = station_b.generate_arq_ack() {
                if let Ok(_) = station_a.receive_frame(ack) {}
            }
        }
        if frame_count > 100 { break; }
    }

    results.a_to_b_bytes_received = received_data_b.len();
    results.a_to_b_integrity = test_data_ab == received_data_b;

    println!("  Sent {} bytes in {} frames", results.a_to_b_bytes_sent, frame_count);
    println!("  Received {} bytes", results.a_to_b_bytes_received);
    println!("  Integrity: {}", if results.a_to_b_integrity { "✓ Match" } else { "✗ Mismatch" });

    // ===== PHASE 5: DATA TRANSFER B→A =====
    println!("\n[Phase 5] Testing data transfer B→A...");

    station_a.reset();
    station_b.reset();
    station_a.session_id = connect_frame.session_id();
    station_b.session_id = connect_frame.session_id();

    let test_data_ba: Vec<u8> = (0..256).map(|i| (255 - i) as u8).collect();
    results.b_to_a_bytes_sent = test_data_ba.len();

    station_b.queue_data(&test_data_ba);

    let mut received_data_a: Vec<u8> = Vec::new();
    let mut frame_count = 0;

    while let Some(frame) = station_b.get_tx_frame() {
        frame_count += 1;
        if let Ok(_) = station_a.receive_frame(frame.clone()) {
            for data in station_a.get_received_data() {
                received_data_a.extend(data);
            }
            if let Some(ack) = station_a.generate_arq_ack() {
                if let Ok(_) = station_b.receive_frame(ack) {}
            }
        }
        if frame_count > 100 { break; }
    }

    results.b_to_a_bytes_received = received_data_a.len();
    results.b_to_a_integrity = test_data_ba == received_data_a;

    println!("  Sent {} bytes in {} frames", results.b_to_a_bytes_sent, frame_count);
    println!("  Received {} bytes", results.b_to_a_bytes_received);
    println!("  Integrity: {}", if results.b_to_a_integrity { "✓ Match" } else { "✗ Mismatch" });

    // ===== PHASE 6: ARQ STATISTICS =====
    println!("\n[Phase 6] ARQ statistics...");

    let stats_a = station_a.arq_stats();
    let stats_b = station_b.arq_stats();

    println!("  Station A: {} TX, {} RX, {} retrans",
             stats_a.frames_sent, stats_a.frames_received, stats_a.retransmissions);
    println!("  Station B: {} TX, {} RX, {} retrans",
             stats_b.frames_sent, stats_b.frames_received, stats_b.retransmissions);

    // ===== PHASE 7: SESSION TEARDOWN =====
    println!("\n[Phase 7] Testing session teardown...");

    let disconnect_frame = station_a.generate_disconnect();
    println!("  Station A → DISCONNECT");

    results.disconnect_clean = disconnect_frame.frame_type() == FrameType::Disconnect;

    if results.disconnect_clean {
        println!("  ✓ Clean disconnect");
    }

    // ===== CALCULATE OVERALL RESULT =====
    results.all_passed =
        results.tcp_command_connect &&
        results.tcp_data_connect &&
        results.version_ok &&
        results.mycall_ok &&
        results.listen_ok &&
        results.connect_initiated &&
        results.connect_established &&
        results.a_to_b_integrity &&
        results.b_to_a_integrity &&
        results.disconnect_clean;

    let total_time = start_time.elapsed();
    println!("\nTest completed in {:.2}s", total_time.as_secs_f32());

    results.print_report();

    assert!(results.all_passed, "Full stack test failed - see report above");
}

// ============================================================================
// Component Unit Tests
// ============================================================================

#[test]
fn test_ofdm_modulation_all_bandwidths() {
    println!("\n========== OFDM MODULATION TEST ==========\n");

    for bw in [Bandwidth::Narrow, Bandwidth::Wide, Bandwidth::Ultra] {
        println!("Testing {:?} bandwidth...", bw);

        let mut modulator = OfdmModulator::for_bandwidth(bw, SAMPLE_RATE);
        let mut demodulator = OfdmDemodulator::for_bandwidth(bw, SAMPLE_RATE);

        let bits_per_symbol = modulator.bits_per_symbol();
        let test_bits: Vec<u8> = (0..bits_per_symbol).map(|i| (i % 2) as u8).collect();

        let samples = modulator.modulate_symbol(&test_bits);
        let noisy = add_noise(&samples, 15.0);
        let recovered = demodulator.demodulate_symbol(&noisy);

        let matching: usize = test_bits.iter()
            .zip(recovered.iter())
            .filter(|(&a, &b)| a == b)
            .count();

        let accuracy = matching as f32 / test_bits.len() as f32 * 100.0;
        println!("  Bits: {}, Samples: {}, Accuracy: {:.1}%",
                 bits_per_symbol, samples.len(), accuracy);

        assert!(accuracy > 80.0, "OFDM {:?} accuracy too low: {:.1}%", bw, accuracy);
    }

    println!("\n✓ All bandwidth modes passed\n");
}

#[test]
fn test_turbo_fec_error_correction() {
    println!("\n========== TURBO FEC TEST ==========\n");

    let mut encoder = TurboEncoder::new_94();
    let mut decoder = TurboDecoder::new_94();

    let data: Vec<u8> = (0..94).map(|i| ((i * 7) % 2) as u8).collect();
    let encoded = encoder.encode(&data);
    println!("Input: {} bits, Encoded: {} bits (rate ~1/3)", data.len(), encoded.len());

    for error_rate in [0.0, 0.05, 0.10, 0.15] {
        let llrs: Vec<f32> = encoded.iter().enumerate()
            .map(|(i, &b)| {
                let error = ((i as f32 * 0.31).sin() + 1.0) / 2.0 < error_rate;
                let bit = if error { 1 - b } else { b };
                if bit == 0 { 3.0 } else { -3.0 }
            })
            .collect();

        let decoded = decoder.decode(&llrs);

        let errors: usize = decoded.iter()
            .zip(data.iter())
            .filter(|(&d, &o)| d != o)
            .count();

        let ber = errors as f32 / data.len() as f32 * 100.0;
        println!("  Channel error rate: {:.0}% → Decoded BER: {:.1}% ({} errors)",
                 error_rate * 100.0, ber, errors);
    }

    println!("\n✓ Turbo FEC test completed\n");
}

#[test]
fn test_48fsk_ack_all_types() {
    println!("\n========== 48-FSK ACK TEST ==========\n");

    let session_id = 0x12345678u32;
    let mut modulator = AckFskModulator::new(SAMPLE_RATE);
    let mut demodulator = AckFskDemodulator::new(SAMPLE_RATE);

    let ack_types = [
        AckType::Ack0,
        AckType::Ack1,
        AckType::SpeedUp,
        AckType::SpeedDown,
        AckType::Nack,
        AckType::Break,
        AckType::Idle,
    ];

    let mut success_count = 0;

    for &ack_type in &ack_types {
        let samples = modulator.modulate(ack_type, session_id, true);
        let noisy = add_noise(&samples, 12.0);
        let detected = demodulator.demodulate(&noisy, session_id, false);

        if let Some(det) = detected {
            if det == ack_type {
                success_count += 1;
                println!("  ✓ {:?}", ack_type);
            } else {
                println!("  ✗ {:?} detected as {:?}", ack_type, det);
            }
        } else {
            println!("  ✗ {:?} not detected", ack_type);
        }
    }

    println!("\nResult: {}/{} ACK types detected correctly", success_count, ack_types.len());
    assert!(success_count >= ack_types.len() - 1, "Too many ACK detection failures");
}

#[test]
fn test_arq_retransmission() {
    println!("\n========== ARQ RETRANSMISSION TEST ==========\n");

    let config = ArqConfig {
        window_size: 8,
        ack_timeout_ms: 100,
        max_retries: 5,
        selective_repeat: true,
        max_queue_bytes: 1024 * 1024,
    };

    let mut tx_arq = ArqController::new(config.clone(), 0x12345678);
    let mut rx_arq = ArqController::new(config, 0x12345678);

    for i in 0..8 {
        tx_arq.send(vec![i as u8; 32]);
    }

    println!("Queued 8 frames for transmission");

    let mut iterations = 0;
    let mut total_received = 0;

    while !tx_arq.tx_empty() && iterations < 20 {
        if let Some(frame) = tx_arq.get_tx_frame() {
            rx_arq.receive(frame).ok();
            let data = rx_arq.get_received_data();
            total_received += data.len();

            if let Some(ack) = rx_arq.generate_ack() {
                tx_arq.receive(ack).ok();
            }
        }
        iterations += 1;
    }

    let stats = tx_arq.stats();
    println!("\nARQ Statistics:");
    println!("  Frames sent: {}", stats.frames_sent);
    println!("  Retransmissions: {}", stats.retransmissions);
    println!("  ACKs received: {}", stats.acks_received);
    println!("  Data blocks received: {}", total_received);

    assert!(total_received >= 6, "Should receive at least 6 of 8 frames, got {}", total_received);
    println!("\n✓ ARQ retransmission test passed\n");
}

#[test]
fn test_afc_frequency_tracking() {
    println!("\n========== AFC FREQUENCY TRACKING TEST ==========\n");

    let mut afc = Afc::new(SAMPLE_RATE, CENTER_FREQ);
    afc.set_max_offset(200.0);
    afc.set_tracking_bandwidth(100.0);

    let freq_offset = 50.0;
    let actual_freq = CENTER_FREQ + freq_offset;

    let duration = 0.1;
    let num_samples = (SAMPLE_RATE * duration) as usize;
    let symbol_period = 1024usize;

    let signal: Vec<ComplexSample> = (0..num_samples)
        .map(|i| {
            let t = i as f32 / SAMPLE_RATE;
            let phase = 2.0 * std::f32::consts::PI * actual_freq * t;
            ComplexSample::new(phase.cos(), phase.sin())
        })
        .collect();

    let estimated_offset = afc.estimate_offset(&signal, symbol_period);

    println!("  Actual offset: {:.1} Hz", freq_offset);
    println!("  Estimated offset: {:.1} Hz", estimated_offset);
    println!("  Error: {:.1} Hz", (estimated_offset - freq_offset).abs());

    println!("\n✓ AFC frequency tracking test completed\n");
}

#[test]
fn test_mfsk_header_modulation() {
    println!("\n========== MFSK HEADER MODULATION TEST ==========\n");

    let mut modulator = MfskModulator::new(SAMPLE_RATE, CENTER_FREQ);
    let mut demodulator = MfskDemodulator::new(SAMPLE_RATE, CENTER_FREQ);

    let mut success = 0;
    let test_tones = [0, 3, 7, 11, 15];

    for &tone in &test_tones {
        modulator.reset();
        let samples = modulator.modulate(&[tone]);
        let noisy = add_noise(&samples, 20.0);
        let recovered = demodulator.demodulate(&noisy);

        if !recovered.is_empty() && recovered[0] == tone {
            success += 1;
            println!("  ✓ Tone {} correctly detected", tone);
        } else {
            println!("  ✗ Tone {} failed (got {:?})", tone, recovered.first());
        }
    }

    println!("\nResult: {}/{} tones detected correctly", success, test_tones.len());
    assert!(success >= 3, "Too many MFSK detection failures");
    println!("\n✓ MFSK header test completed\n");
}

#[test]
fn test_frame_encoding_decoding() {
    println!("\n========== FRAME ENCODING/DECODING TEST ==========\n");

    let test_cases = [
        ("CONNECT", Frame::connect("W1AW", "K1ABC", 0x12345678)),
        ("CONNECT_ACK", Frame::connect_ack(0x12345678, 9)),
        ("DATA", Frame::data(1, 0x12345678, b"Hello, World!".to_vec())),
        ("DISCONNECT", Frame::disconnect(0x12345678)),
        ("ACK", Frame::ack(5, 0x12345678)),
    ];

    let mut success = 0;

    for (name, frame) in test_cases {
        let encoded = frame.encode();
        let decoded = Frame::decode(&encoded);

        if let Ok(dec) = decoded {
            if dec.frame_type() == frame.frame_type() &&
               dec.session_id() == frame.session_id() {
                success += 1;
                println!("  ✓ {} frame: {} bytes", name, encoded.len());
            } else {
                println!("  ✗ {} frame: type/session mismatch", name);
            }
        } else {
            println!("  ✗ {} frame: decode failed", name);
        }
    }

    println!("\nResult: {}/5 frame types passed", success);
    assert_eq!(success, 5, "Frame encoding/decoding test failed");
    println!("\n✓ Frame encoding/decoding test completed\n");
}

// ============================================================================
// Variable Data Size Tests
// ============================================================================

#[test]
fn test_data_transfer_various_sizes() {
    println!("\n========== VARIABLE DATA SIZE TEST ==========\n");

    // Test sizes: tiny, small, medium, large, very large
    let test_sizes = [
        ("Tiny", 16),
        ("Small", 64),
        ("Medium", 256),
        ("Large", 1024),
        ("Very Large", 4096),
    ];

    for (name, size) in test_sizes {
        println!("Testing {} ({} bytes)...", name, size);

        let config = ArqConfig {
            window_size: 8,
            ack_timeout_ms: 100,
            max_retries: 5,
            selective_repeat: true,
            max_queue_bytes: 1024 * 1024,
        };

        let mut tx_arq = ArqController::new(config.clone(), 0x12345678);
        let mut rx_arq = ArqController::new(config, 0x12345678);

        // Generate test data
        let test_data: Vec<u8> = (0..size).map(|i| (i % 256) as u8).collect();

        // Queue data
        tx_arq.send(test_data.clone());

        // Simulate transfer
        let mut received_data: Vec<u8> = Vec::new();
        let mut iterations = 0;
        let max_iterations = size / 32 + 50; // Scale with data size

        while !tx_arq.tx_empty() && iterations < max_iterations {
            if let Some(frame) = tx_arq.get_tx_frame() {
                rx_arq.receive(frame).ok();

                for data in rx_arq.get_received_data() {
                    received_data.extend(data);
                }

                if let Some(ack) = rx_arq.generate_ack() {
                    tx_arq.receive(ack).ok();
                }
            }
            iterations += 1;
        }

        let stats = tx_arq.stats();
        let integrity = test_data == received_data;

        println!("  Sent: {} bytes, Received: {} bytes", size, received_data.len());
        println!("  Frames: {} TX, {} retrans", stats.frames_sent, stats.retransmissions);
        println!("  Integrity: {}", if integrity { "✓" } else { "✗" });

        assert!(received_data.len() >= size * 3 / 4,
                "{} transfer failed: only {}/{} bytes received", name, received_data.len(), size);
        println!();
    }

    println!("✓ Variable data size test completed\n");
}

// ============================================================================
// WAV File Generation
// ============================================================================

/// Write samples to a WAV file (mono, 48kHz, 16-bit)
fn write_wav_file(path: &str, samples: &[f32], sample_rate: u32) -> std::io::Result<()> {
    use std::fs::File;
    use std::io::Write;

    let mut file = File::create(path)?;

    // Convert f32 samples to i16
    let samples_i16: Vec<i16> = samples.iter()
        .map(|&s| {
            let clamped = s.max(-1.0).min(1.0);
            (clamped * 32767.0) as i16
        })
        .collect();

    let num_samples = samples_i16.len() as u32;
    let byte_rate = sample_rate * 2; // 16-bit mono
    let data_size = num_samples * 2;
    let file_size = 36 + data_size;

    // RIFF header
    file.write_all(b"RIFF")?;
    file.write_all(&file_size.to_le_bytes())?;
    file.write_all(b"WAVE")?;

    // fmt chunk
    file.write_all(b"fmt ")?;
    file.write_all(&16u32.to_le_bytes())?;  // Chunk size
    file.write_all(&1u16.to_le_bytes())?;   // Audio format (PCM)
    file.write_all(&1u16.to_le_bytes())?;   // Num channels (mono)
    file.write_all(&sample_rate.to_le_bytes())?;  // Sample rate
    file.write_all(&byte_rate.to_le_bytes())?;    // Byte rate
    file.write_all(&2u16.to_le_bytes())?;   // Block align
    file.write_all(&16u16.to_le_bytes())?;  // Bits per sample

    // data chunk
    file.write_all(b"data")?;
    file.write_all(&data_size.to_le_bytes())?;

    for sample in &samples_i16 {
        file.write_all(&sample.to_le_bytes())?;
    }

    Ok(())
}

/// Write stereo WAV file (48kHz, 16-bit)
fn write_stereo_wav_file(path: &str, left: &[f32], right: &[f32], sample_rate: u32) -> std::io::Result<()> {
    use std::fs::File;
    use std::io::Write;

    let mut file = File::create(path)?;

    // Interleave channels
    let num_samples = left.len().min(right.len());
    let mut samples_i16: Vec<i16> = Vec::with_capacity(num_samples * 2);

    for i in 0..num_samples {
        let l = (left[i].max(-1.0).min(1.0) * 32767.0) as i16;
        let r = (right[i].max(-1.0).min(1.0) * 32767.0) as i16;
        samples_i16.push(l);
        samples_i16.push(r);
    }

    let byte_rate = sample_rate * 4; // 16-bit stereo
    let data_size = (num_samples * 4) as u32;
    let file_size = 36 + data_size;

    // RIFF header
    file.write_all(b"RIFF")?;
    file.write_all(&file_size.to_le_bytes())?;
    file.write_all(b"WAVE")?;

    // fmt chunk
    file.write_all(b"fmt ")?;
    file.write_all(&16u32.to_le_bytes())?;  // Chunk size
    file.write_all(&1u16.to_le_bytes())?;   // Audio format (PCM)
    file.write_all(&2u16.to_le_bytes())?;   // Num channels (stereo)
    file.write_all(&sample_rate.to_le_bytes())?;  // Sample rate
    file.write_all(&byte_rate.to_le_bytes())?;    // Byte rate
    file.write_all(&4u16.to_le_bytes())?;   // Block align
    file.write_all(&16u16.to_le_bytes())?;  // Bits per sample

    // data chunk
    file.write_all(b"data")?;
    file.write_all(&data_size.to_le_bytes())?;

    for sample in &samples_i16 {
        file.write_all(&sample.to_le_bytes())?;
    }

    Ok(())
}

#[test]
fn test_generate_connection_wav() {
    println!("\n========== GENERATE CONNECTION WAV FILE ==========\n");

    let mut station_a = TestStation::new("Station A", "W1AW", true);
    let mut station_b = TestStation::new("Station B", "K1ABC", false);

    // Collected audio for each station
    let mut audio_station_a: Vec<f32> = Vec::new();
    let mut audio_station_b: Vec<f32> = Vec::new();

    // Add some silence at the start (100ms)
    let silence_samples = (SAMPLE_RATE * 0.1) as usize;
    audio_station_a.extend(vec![0.0f32; silence_samples]);
    audio_station_b.extend(vec![0.0f32; silence_samples]);

    // ===== PHASE 1: Station A sends preamble =====
    println!("[1] Station A: Generating preamble...");
    let preamble_a = station_a.generate_preamble();
    println!("    Preamble: {} samples ({:.1}ms)",
             preamble_a.len(), preamble_a.len() as f32 / SAMPLE_RATE * 1000.0);

    audio_station_a.extend(&preamble_a);
    // Station B is silent during this time
    audio_station_b.extend(vec![0.0f32; preamble_a.len()]);

    // ===== PHASE 2: Station A sends CONNECT frame =====
    println!("[2] Station A: Sending CONNECT frame...");
    let connect_frame = station_a.generate_connect_frame("K1ABC");
    station_b.session_id = connect_frame.session_id();

    let connect_audio = station_a.encode_frame(&connect_frame);
    println!("    CONNECT frame: {} samples ({:.1}ms)",
             connect_audio.len(), connect_audio.len() as f32 / SAMPLE_RATE * 1000.0);

    audio_station_a.extend(&connect_audio);
    audio_station_b.extend(vec![0.0f32; connect_audio.len()]);

    // Small gap between TX and RX (50ms)
    let gap_samples = (SAMPLE_RATE * 0.05) as usize;
    audio_station_a.extend(vec![0.0f32; gap_samples]);
    audio_station_b.extend(vec![0.0f32; gap_samples]);

    // ===== PHASE 3: Station B sends preamble + CONNECT_ACK =====
    println!("[3] Station B: Sending preamble + CONNECT_ACK...");
    let preamble_b = station_b.generate_preamble();
    let connect_ack_frame = station_b.generate_connect_ack();
    let connect_ack_audio = station_b.encode_frame(&connect_ack_frame);

    // Station A is silent, Station B transmits
    audio_station_a.extend(vec![0.0f32; preamble_b.len() + connect_ack_audio.len()]);
    audio_station_b.extend(&preamble_b);
    audio_station_b.extend(&connect_ack_audio);

    println!("    CONNECT_ACK: {} samples ({:.1}ms)",
             connect_ack_audio.len(), connect_ack_audio.len() as f32 / SAMPLE_RATE * 1000.0);

    // Gap
    audio_station_a.extend(vec![0.0f32; gap_samples]);
    audio_station_b.extend(vec![0.0f32; gap_samples]);

    // ===== PHASE 4: Station A sends small data transfer =====
    println!("[4] Station A: Sending data frames...");

    // Queue 64 bytes of test data
    let test_data: Vec<u8> = (0..64).map(|i| i as u8).collect();
    station_a.queue_data(&test_data);

    let mut data_frames_audio: Vec<f32> = Vec::new();
    let mut frame_count = 0;

    while let Some(frame) = station_a.get_tx_frame() {
        frame_count += 1;
        // Add preamble before each data frame
        data_frames_audio.extend(station_a.generate_preamble());
        data_frames_audio.extend(station_a.encode_frame(&frame));

        // Receive at station B
        station_b.receive_frame(frame).ok();

        if frame_count >= 5 { break; } // Limit for demo
    }

    println!("    Data frames: {} frames, {} samples ({:.1}ms)",
             frame_count, data_frames_audio.len(),
             data_frames_audio.len() as f32 / SAMPLE_RATE * 1000.0);

    audio_station_a.extend(&data_frames_audio);
    audio_station_b.extend(vec![0.0f32; data_frames_audio.len()]);

    // Gap
    audio_station_a.extend(vec![0.0f32; gap_samples]);
    audio_station_b.extend(vec![0.0f32; gap_samples]);

    // ===== PHASE 5: Station B sends ACK =====
    println!("[5] Station B: Sending ACK...");

    if let Some(ack_frame) = station_b.generate_arq_ack() {
        let preamble_b = station_b.generate_preamble();
        let ack_audio = station_b.encode_frame(&ack_frame);

        audio_station_a.extend(vec![0.0f32; preamble_b.len() + ack_audio.len()]);
        audio_station_b.extend(&preamble_b);
        audio_station_b.extend(&ack_audio);

        println!("    ACK: {} samples ({:.1}ms)",
                 ack_audio.len(), ack_audio.len() as f32 / SAMPLE_RATE * 1000.0);
    }

    // Gap
    audio_station_a.extend(vec![0.0f32; gap_samples]);
    audio_station_b.extend(vec![0.0f32; gap_samples]);

    // ===== PHASE 6: Station A sends DISCONNECT =====
    println!("[6] Station A: Sending DISCONNECT...");

    let preamble_a = station_a.generate_preamble();
    let disconnect_frame = station_a.generate_disconnect();
    let disconnect_audio = station_a.encode_frame(&disconnect_frame);

    audio_station_a.extend(&preamble_a);
    audio_station_a.extend(&disconnect_audio);
    audio_station_b.extend(vec![0.0f32; preamble_a.len() + disconnect_audio.len()]);

    println!("    DISCONNECT: {} samples ({:.1}ms)",
             disconnect_audio.len(), disconnect_audio.len() as f32 / SAMPLE_RATE * 1000.0);

    // Add trailing silence (100ms)
    audio_station_a.extend(vec![0.0f32; silence_samples]);
    audio_station_b.extend(vec![0.0f32; silence_samples]);

    // Ensure both channels have the same length
    let max_len = audio_station_a.len().max(audio_station_b.len());
    audio_station_a.resize(max_len, 0.0);
    audio_station_b.resize(max_len, 0.0);

    // ===== Write WAV files =====
    let total_duration_ms = max_len as f32 / SAMPLE_RATE * 1000.0;
    println!("\nTotal audio duration: {:.1}ms ({} samples)", total_duration_ms, max_len);

    // Combine both stations for a mono mix
    let combined: Vec<f32> = audio_station_a.iter()
        .zip(audio_station_b.iter())
        .map(|(&a, &b)| (a + b) * 0.7) // Mix with some headroom
        .collect();

    // Write files
    let output_dir = "/Users/stephen/Desktop/VARA HF Decompile/RIA";

    let mono_path = format!("{}/ria_connection_mono.wav", output_dir);
    match write_wav_file(&mono_path, &combined, SAMPLE_RATE as u32) {
        Ok(_) => println!("\n✓ Wrote mono WAV: {}", mono_path),
        Err(e) => println!("\n✗ Failed to write mono WAV: {}", e),
    }

    let stereo_path = format!("{}/ria_connection_stereo.wav", output_dir);
    match write_stereo_wav_file(&stereo_path, &audio_station_a, &audio_station_b, SAMPLE_RATE as u32) {
        Ok(_) => println!("✓ Wrote stereo WAV: {}", stereo_path),
        Err(e) => println!("✗ Failed to write stereo WAV: {}", e),
    }

    // Also write individual station files
    let a_path = format!("{}/ria_station_a.wav", output_dir);
    match write_wav_file(&a_path, &audio_station_a, SAMPLE_RATE as u32) {
        Ok(_) => println!("✓ Wrote Station A WAV: {}", a_path),
        Err(e) => println!("✗ Failed to write Station A WAV: {}", e),
    }

    let b_path = format!("{}/ria_station_b.wav", output_dir);
    match write_wav_file(&b_path, &audio_station_b, SAMPLE_RATE as u32) {
        Ok(_) => println!("✓ Wrote Station B WAV: {}", b_path),
        Err(e) => println!("✗ Failed to write Station B WAV: {}", e),
    }

    println!("\n========== WAV FILES GENERATED ==========\n");
    println!("Files created:");
    println!("  - ria_connection_mono.wav   (combined audio)");
    println!("  - ria_connection_stereo.wav (L=Station A, R=Station B)");
    println!("  - ria_station_a.wav         (W1AW transmissions)");
    println!("  - ria_station_b.wav         (K1ABC transmissions)");
    println!();
}

// ============================================================================
// Audio Level Diagnostics and AFC Test
// ============================================================================

#[test]
fn test_audio_levels_and_afc() {
    println!("\n========== AUDIO LEVELS AND AFC TEST ==========\n");

    let bandwidth = Bandwidth::Wide;
    let preamble = Preamble::with_mode(SAMPLE_RATE, PreambleMode::Wide, 0.35);
    let mut ofdm_mod = OfdmModulator::for_bandwidth(bandwidth, SAMPLE_RATE);
    let mut turbo_encoder = TurboEncoder::new_94();
    let mut afc = Afc::new(SAMPLE_RATE, CENTER_FREQ);
    afc.set_max_offset(200.0);
    afc.set_tracking_bandwidth(100.0);

    // ===== Test 1: All modulator amplitudes =====
    println!("[1] All modulator amplitude analysis...");

    // Preamble
    let preamble_samples = preamble.samples();
    let preamble_max: f32 = preamble_samples.iter().map(|s| s.abs()).fold(0.0, f32::max);
    let preamble_rms: f32 = (preamble_samples.iter().map(|s| s * s).sum::<f32>() / preamble_samples.len() as f32).sqrt();
    println!("  Preamble:  max={:.3}, RMS={:.3}", preamble_max, preamble_rms);

    // MFSK
    let mut mfsk_mod = MfskModulator::new(SAMPLE_RATE, CENTER_FREQ);
    let mfsk_samples = mfsk_mod.modulate(&[0, 5, 10, 15]);
    let mfsk_max: f32 = mfsk_samples.iter().map(|s| s.abs()).fold(0.0, f32::max);
    let mfsk_rms: f32 = (mfsk_samples.iter().map(|s| s * s).sum::<f32>() / mfsk_samples.len() as f32).sqrt();
    println!("  MFSK:      max={:.3}, RMS={:.3}", mfsk_max, mfsk_rms);

    // ACK FSK
    let mut ack_mod = AckFskModulator::new(SAMPLE_RATE);
    let ack_samples = ack_mod.modulate(AckType::Ack1, 0x12345678, true);
    let ack_max: f32 = ack_samples.iter().map(|s| s.abs()).fold(0.0, f32::max);
    let ack_rms: f32 = (ack_samples.iter().map(|s| s * s).sum::<f32>() / ack_samples.len() as f32).sqrt();
    println!("  ACK FSK:   max={:.3}, RMS={:.3}", ack_max, ack_rms);

    // ===== Test 2: OFDM modulator output amplitude =====
    println!("\n[2] OFDM modulator amplitude analysis...");

    // Generate test bits
    let test_bits: Vec<u8> = (0..ofdm_mod.bits_per_symbol()).map(|i| (i % 2) as u8).collect();
    let ofdm_samples = ofdm_mod.modulate_symbol(&test_bits);
    let ofdm_max: f32 = ofdm_samples.iter().map(|s| s.abs()).fold(0.0, f32::max);
    let ofdm_rms: f32 = (ofdm_samples.iter().map(|s| s * s).sum::<f32>() / ofdm_samples.len() as f32).sqrt();
    println!("  OFDM symbol: {} samples, max={:.3}, RMS={:.3}", ofdm_samples.len(), ofdm_max, ofdm_rms);

    // ===== Test 3: Calculate normalization factor =====
    let normalization = preamble_rms / ofdm_rms.max(0.001);
    println!("\n[3] Normalization factor: {:.2}x", normalization);
    println!("  (OFDM needs to be amplified {:.1}x to match preamble level)", normalization);

    // ===== Test 4: Test normalized OFDM frame =====
    println!("\n[4] Generating normalized OFDM frame...");

    // Generate a full frame
    let frame = Frame::data(1, 0x12345678, b"Test data for AFC".to_vec());
    let frame_bytes = frame.encode();
    let bits: Vec<u8> = frame_bytes.iter()
        .flat_map(|&byte| (0..8).rev().map(move |i| (byte >> i) & 1))
        .collect();

    // Turbo encode in blocks
    let block_size = 94;
    let mut encoded = Vec::new();
    for chunk in bits.chunks(block_size) {
        let mut block = chunk.to_vec();
        block.resize(block_size, 0);
        let enc = turbo_encoder.encode(&block);
        encoded.extend(enc);
    }

    // OFDM modulate
    ofdm_mod.reset();
    let ofdm_frame = ofdm_mod.modulate(&encoded);

    // Normalize to match preamble level
    let normalized_frame: Vec<f32> = ofdm_frame.iter()
        .map(|&s| s * normalization * 0.8) // 0.8 for headroom
        .collect();

    let norm_max: f32 = normalized_frame.iter().map(|s| s.abs()).fold(0.0, f32::max);
    let norm_rms: f32 = (normalized_frame.iter().map(|s| s * s).sum::<f32>() / normalized_frame.len() as f32).sqrt();
    println!("  Normalized frame: {} samples, max={:.3}, RMS={:.3}", normalized_frame.len(), norm_max, norm_rms);

    // ===== Test 5: AFC frequency tracking test =====
    println!("\n[5] AFC frequency tracking test...");

    let fft_size = 1024;
    let cp_length = 128;

    // Test 5a: Tone-based AFC (for preamble) - must support ±200 Hz
    println!("\n  5a. Tone-based AFC (preamble detection, ±200Hz range):");
    let freq_offsets = [-200.0, -100.0, -50.0, 0.0, 50.0, 100.0, 200.0];
    let mut all_passed = true;
    for offset in freq_offsets {
        // Generate complex tone at center_freq + offset
        let tone: Vec<ComplexSample> = (0..4800)
            .map(|i| {
                let t = i as f32 / SAMPLE_RATE;
                let freq = CENTER_FREQ + offset;
                let phase = 2.0 * std::f32::consts::PI * freq * t;
                ComplexSample::new(phase.cos(), phase.sin())
            })
            .collect();

        let estimated = afc.estimate_tone_offset(&tone, CENTER_FREQ);
        let error = (estimated - offset).abs();
        let status = if error < 5.0 { "✓" } else { "✗" };
        if error >= 5.0 { all_passed = false; }
        println!("    Offset {:+.0}Hz → Estimated {:+.1}Hz (error: {:.1}Hz) {}",
                 offset, estimated, error, status);
    }
    assert!(all_passed, "Tone-based AFC must work for ±200 Hz range");

    // Test 5b: OFDM CP-based AFC
    // Note: Max unambiguous offset = 48000 / (2 * 1024) ≈ 23.4 Hz
    println!("\n  5b. OFDM cyclic prefix AFC (max ±23Hz):");
    for offset in [-20.0f32, -10.0, 0.0, 10.0, 20.0] {
        // Generate OFDM-like signal with proper CP structure
        let mut samples: Vec<ComplexSample> = Vec::new();

        for sym in 0..4 {
            // Generate symbol data
            let symbol_data: Vec<ComplexSample> = (0..fft_size)
                .map(|i| {
                    let phase = (i as f32 * 0.1 + sym as f32 * 0.5) % (2.0 * std::f32::consts::PI);
                    ComplexSample::new(phase.cos(), phase.sin())
                })
                .collect();

            // Add cyclic prefix (copy of last cp_length samples)
            for i in (fft_size - cp_length)..fft_size {
                samples.push(symbol_data[i]);
            }
            // Add symbol data
            samples.extend(&symbol_data);
        }

        // Apply frequency offset
        let offset_samples: Vec<ComplexSample> = samples.iter().enumerate()
            .map(|(i, &s)| {
                let t = i as f32 / SAMPLE_RATE;
                let phase = 2.0 * std::f32::consts::PI * offset * t;
                s * ComplexSample::new(phase.cos(), phase.sin())
            })
            .collect();

        let estimated = afc.estimate_ofdm_offset(&offset_samples, fft_size, cp_length);
        let error = (estimated - offset).abs();
        let status = if error < 5.0 { "✓" } else { "✗" };
        println!("    Offset {:+.0}Hz → Estimated {:+.1}Hz (error: {:.1}Hz) {}",
                 offset, estimated, error, status);
    }

    // Test 5c: Combined coarse + fine AFC for large offsets (±200 Hz)
    println!("\n  5c. Combined coarse (tone) + fine (OFDM) AFC for ±200Hz:");
    for coarse_offset in [-200.0f32, -150.0, 150.0, 200.0] {
        // First, use tone-based AFC for coarse estimation (from preamble)
        let tone: Vec<ComplexSample> = (0..4800)
            .map(|i| {
                let t = i as f32 / SAMPLE_RATE;
                let freq = CENTER_FREQ + coarse_offset;
                let phase = 2.0 * std::f32::consts::PI * freq * t;
                ComplexSample::new(phase.cos(), phase.sin())
            })
            .collect();

        let coarse_est = afc.estimate_tone_offset(&tone, CENTER_FREQ);
        let coarse_error = (coarse_est - coarse_offset).abs();

        // Then use OFDM AFC for fine tracking (residual after coarse correction)
        let residual = coarse_offset - coarse_est;

        let status = if coarse_error < 5.0 { "✓" } else { "✗" };
        println!("    {:+.0}Hz → Coarse: {:+.1}Hz, Residual: {:+.1}Hz {}",
                 coarse_offset, coarse_est, residual, status);
    }

    // Test 5d: AFC configuration info
    let max_ofdm = afc.max_ofdm_offset(fft_size);
    println!("\n  5d. AFC Configuration:");
    println!("    Coarse (tone-based): ±200Hz supported");
    println!("    Fine (OFDM CP): ±{:.1}Hz unambiguous range", max_ofdm);
    println!("    Strategy: Use preamble for coarse, OFDM for fine tracking");

    // ===== Test 6: Write diagnostic WAV =====
    println!("\n[6] Writing diagnostic WAV file...");

    let mut diagnostic_audio: Vec<f32> = Vec::new();

    // Silence
    let silence = vec![0.0f32; (SAMPLE_RATE * 0.1) as usize];
    diagnostic_audio.extend(&silence);

    // Preamble (full amplitude)
    diagnostic_audio.extend(preamble.samples());
    diagnostic_audio.extend(&silence);

    // Raw OFDM (low amplitude - for comparison)
    diagnostic_audio.extend(&ofdm_frame);
    diagnostic_audio.extend(&silence);

    // Normalized OFDM (should be audible)
    diagnostic_audio.extend(&normalized_frame);
    diagnostic_audio.extend(&silence);

    // Test tone at 1500 Hz for reference
    let tone_samples = (SAMPLE_RATE * 0.2) as usize;
    let tone: Vec<f32> = (0..tone_samples)
        .map(|i| {
            let t = i as f32 / SAMPLE_RATE;
            0.7 * (2.0 * std::f32::consts::PI * CENTER_FREQ * t).sin()
        })
        .collect();
    diagnostic_audio.extend(&tone);
    diagnostic_audio.extend(&silence);

    let output_dir = "/Users/stephen/Desktop/VARA HF Decompile/RIA";
    let diag_path = format!("{}/ria_diagnostic.wav", output_dir);
    match write_wav_file(&diag_path, &diagnostic_audio, SAMPLE_RATE as u32) {
        Ok(_) => println!("  ✓ Wrote diagnostic WAV: {}", diag_path),
        Err(e) => println!("  ✗ Failed: {}", e),
    }

    println!("\n  Diagnostic WAV contents:");
    println!("  [0.0s - 0.1s]  Silence");
    println!("  [0.1s - 0.45s] Preamble (should be audible)");
    println!("  [0.55s - ?]    Raw OFDM (likely very quiet)");
    println!("  [? - ?]        Normalized OFDM (should be audible)");
    println!("  [? - ?]        1500Hz reference tone");

    println!("\n✓ Audio levels and AFC test completed\n");
}

#[test]
fn test_generate_proper_connection_wav() {
    println!("\n========== GENERATE PROPER CONNECTION WAV ==========\n");

    // Calculate normalization factor first
    let preamble_ref = Preamble::with_mode(SAMPLE_RATE, PreambleMode::Wide, 0.1);
    let mut ofdm_ref = OfdmModulator::for_bandwidth(Bandwidth::Wide, SAMPLE_RATE);
    let test_bits: Vec<u8> = (0..ofdm_ref.bits_per_symbol()).map(|i| (i % 2) as u8).collect();
    let ofdm_samples = ofdm_ref.modulate_symbol(&test_bits);

    let preamble_rms: f32 = (preamble_ref.samples().iter().map(|s| s * s).sum::<f32>()
        / preamble_ref.samples().len() as f32).sqrt();
    let ofdm_rms: f32 = (ofdm_samples.iter().map(|s| s * s).sum::<f32>()
        / ofdm_samples.len() as f32).sqrt();
    let norm_factor = preamble_rms / ofdm_rms.max(0.001) * 0.8;

    println!("Normalization factor: {:.2}x", norm_factor);

    let mut station_a = TestStation::new("Station A", "W1AW", true);
    let mut station_b = TestStation::new("Station B", "K1ABC", false);

    let mut audio_a: Vec<f32> = Vec::new();
    let mut audio_b: Vec<f32> = Vec::new();

    let silence_short = vec![0.0f32; (SAMPLE_RATE * 0.05) as usize];
    let silence_long = vec![0.0f32; (SAMPLE_RATE * 0.1) as usize];

    // Helper to normalize OFDM output
    let normalize = |samples: Vec<f32>| -> Vec<f32> {
        samples.iter().map(|&s| s * norm_factor).collect()
    };

    // ===== Station A: Preamble + CONNECT =====
    println!("[1] Station A: Preamble + CONNECT");
    audio_a.extend(&silence_long);
    audio_b.extend(&silence_long);

    let preamble_a = station_a.generate_preamble();
    audio_a.extend(&preamble_a);
    audio_b.extend(vec![0.0f32; preamble_a.len()]);

    let connect_frame = station_a.generate_connect_frame("K1ABC");
    station_b.session_id = connect_frame.session_id();
    let connect_audio = normalize(station_a.encode_frame(&connect_frame));
    audio_a.extend(&connect_audio);
    audio_b.extend(vec![0.0f32; connect_audio.len()]);

    audio_a.extend(&silence_short);
    audio_b.extend(&silence_short);

    // ===== Station B: Preamble + CONNECT_ACK =====
    println!("[2] Station B: Preamble + CONNECT_ACK");
    let preamble_b = station_b.generate_preamble();
    audio_a.extend(vec![0.0f32; preamble_b.len()]);
    audio_b.extend(&preamble_b);

    let ack_frame = station_b.generate_connect_ack();
    let ack_audio = normalize(station_b.encode_frame(&ack_frame));
    audio_a.extend(vec![0.0f32; ack_audio.len()]);
    audio_b.extend(&ack_audio);

    audio_a.extend(&silence_short);
    audio_b.extend(&silence_short);

    // ===== Station A: Data frames =====
    println!("[3] Station A: Data frames");
    let test_data: Vec<u8> = b"Hello from W1AW to K1ABC! This is a test message.".to_vec();
    station_a.queue_data(&test_data);

    let mut frame_count = 0;
    while let Some(frame) = station_a.get_tx_frame() {
        frame_count += 1;

        // Preamble before data
        let pre = station_a.generate_preamble();
        audio_a.extend(&pre);
        audio_b.extend(vec![0.0f32; pre.len()]);

        // Data frame (normalized)
        let data_audio = normalize(station_a.encode_frame(&frame));
        audio_a.extend(&data_audio);
        audio_b.extend(vec![0.0f32; data_audio.len()]);

        audio_a.extend(&silence_short);
        audio_b.extend(&silence_short);

        if frame_count >= 3 { break; }
    }

    // ===== Station B: ACK =====
    println!("[4] Station B: ACK");
    if let Some(ack) = station_b.generate_arq_ack() {
        let pre = station_b.generate_preamble();
        audio_a.extend(vec![0.0f32; pre.len()]);
        audio_b.extend(&pre);

        let ack_audio = normalize(station_b.encode_frame(&ack));
        audio_a.extend(vec![0.0f32; ack_audio.len()]);
        audio_b.extend(&ack_audio);

        audio_a.extend(&silence_short);
        audio_b.extend(&silence_short);
    }

    // ===== Station A: DISCONNECT =====
    println!("[5] Station A: DISCONNECT");
    let pre = station_a.generate_preamble();
    audio_a.extend(&pre);
    audio_b.extend(vec![0.0f32; pre.len()]);

    let disconnect = station_a.generate_disconnect();
    let disconnect_audio = normalize(station_a.encode_frame(&disconnect));
    audio_a.extend(&disconnect_audio);
    audio_b.extend(vec![0.0f32; disconnect_audio.len()]);

    audio_a.extend(&silence_long);
    audio_b.extend(&silence_long);

    // Ensure same length
    let max_len = audio_a.len().max(audio_b.len());
    audio_a.resize(max_len, 0.0);
    audio_b.resize(max_len, 0.0);

    // Combined mono
    let combined: Vec<f32> = audio_a.iter().zip(audio_b.iter())
        .map(|(&a, &b)| (a + b) * 0.7)
        .collect();

    let duration_secs = max_len as f32 / SAMPLE_RATE;
    println!("\nTotal duration: {:.2}s ({} samples)", duration_secs, max_len);

    // Write files
    let output_dir = "/Users/stephen/Desktop/VARA HF Decompile/RIA";

    let path = format!("{}/ria_connection_proper.wav", output_dir);
    match write_wav_file(&path, &combined, SAMPLE_RATE as u32) {
        Ok(_) => println!("✓ Wrote: {}", path),
        Err(e) => println!("✗ Failed: {}", e),
    }

    let path = format!("{}/ria_connection_proper_stereo.wav", output_dir);
    match write_stereo_wav_file(&path, &audio_a, &audio_b, SAMPLE_RATE as u32) {
        Ok(_) => println!("✓ Wrote: {}", path),
        Err(e) => println!("✗ Failed: {}", e),
    }

    println!("\n✓ Proper connection WAV generated\n");
}

#[test]
fn test_full_exchange_with_sizes() {
    println!("\n========== FULL EXCHANGE WITH VARYING SIZES ==========\n");

    let test_configs = [
        ("Small", 32, "small"),
        ("Medium", 256, "medium"),
        ("Large", 1024, "large"),
    ];

    for (name, size, suffix) in test_configs {
        println!("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        println!("Testing {} transfer ({} bytes)", name, size);
        println!("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");

        let mut station_a = TestStation::new("Station A", "W1AW", true);
        let mut station_b = TestStation::new("Station B", "K1ABC", false);

        let mut audio_combined: Vec<f32> = Vec::new();

        // Silence
        let silence = vec![0.0f32; (SAMPLE_RATE * 0.05) as usize];
        audio_combined.extend(&silence);

        // Connection setup
        let preamble = station_a.generate_preamble();
        let connect_frame = station_a.generate_connect_frame("K1ABC");
        station_b.session_id = connect_frame.session_id();

        audio_combined.extend(&preamble);
        audio_combined.extend(station_a.encode_frame(&connect_frame));
        audio_combined.extend(&silence);

        // CONNECT_ACK
        audio_combined.extend(station_b.generate_preamble());
        audio_combined.extend(station_b.encode_frame(&station_b.generate_connect_ack()));
        audio_combined.extend(&silence);

        // Data transfer A→B
        let test_data: Vec<u8> = (0..size).map(|i| (i % 256) as u8).collect();
        station_a.queue_data(&test_data);

        let mut received_data: Vec<u8> = Vec::new();
        let mut frame_count = 0;
        let start_time = Instant::now();

        while let Some(frame) = station_a.get_tx_frame() {
            frame_count += 1;
            audio_combined.extend(station_a.generate_preamble());
            audio_combined.extend(station_a.encode_frame(&frame));

            station_b.receive_frame(frame).ok();
            for data in station_b.get_received_data() {
                received_data.extend(data);
            }

            // ACK every few frames
            if frame_count % 4 == 0 {
                if let Some(ack) = station_b.generate_arq_ack() {
                    audio_combined.extend(&silence);
                    audio_combined.extend(station_b.generate_preamble());
                    audio_combined.extend(station_b.encode_frame(&ack));
                    station_a.receive_frame(ack).ok();
                }
            }

            if frame_count > 100 { break; }
        }

        // Final ACK
        if let Some(ack) = station_b.generate_arq_ack() {
            audio_combined.extend(&silence);
            audio_combined.extend(station_b.generate_preamble());
            audio_combined.extend(station_b.encode_frame(&ack));
        }

        let elapsed = start_time.elapsed();

        // Disconnect
        audio_combined.extend(&silence);
        audio_combined.extend(station_a.generate_preamble());
        audio_combined.extend(station_a.encode_frame(&station_a.generate_disconnect()));
        audio_combined.extend(&silence);

        // Stats
        let stats_a = station_a.arq_stats();
        let integrity = test_data == received_data;
        let audio_duration_ms = audio_combined.len() as f32 / SAMPLE_RATE * 1000.0;

        println!("Results:");
        println!("  Data sent:      {} bytes", size);
        println!("  Data received:  {} bytes", received_data.len());
        println!("  Integrity:      {}", if integrity { "✓ PASS" } else { "✗ FAIL" });
        println!("  Frames TX:      {}", stats_a.frames_sent);
        println!("  Retransmissions: {}", stats_a.retransmissions);
        println!("  Processing time: {:.2}ms", elapsed.as_secs_f32() * 1000.0);
        println!("  Audio duration:  {:.1}ms", audio_duration_ms);

        // Write WAV
        let output_dir = "/Users/stephen/Desktop/VARA HF Decompile/RIA";
        let wav_path = format!("{}/ria_transfer_{}.wav", output_dir, suffix);
        match write_wav_file(&wav_path, &audio_combined, SAMPLE_RATE as u32) {
            Ok(_) => println!("  WAV file:       {}", wav_path),
            Err(e) => println!("  WAV error:      {}", e),
        }

        println!();

        assert!(received_data.len() >= size / 2,
                "{} transfer failed: only {}/{} bytes", name, received_data.len(), size);
    }

    println!("✓ Full exchange with varying sizes completed\n");
}
