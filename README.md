# RIA - High-Performance OFDM HF Modem

A cross-platform amateur radio modem for HF (High Frequency) communications supporting multiple modulation modes, turbo FEC coding, and ARQ protocol for reliable data transfer.

## Features

- **Multi-mode Modulation**: FSK, BPSK, QPSK, 8-PSK, 16-QAM, 32-QAM
- **Turbo FEC**: Variable code rates (1/2, 2/3, 3/4, 5/6) with soft-decision decoding
- **Automatic Rate Adaptation**: SNR-based mode selection with 3dB hysteresis
- **ARQ Protocol**: Selective repeat with session management
- **Optional Encryption**: AES-256-GCM with passphrase-based key derivation
- **TCP Interface**: Dual-port architecture for command/control and data
- **Real-time Audio**: 48 kHz sample rate, 1500 Hz center frequency
- **GUI Interface**: Optional egui-based graphical interface

## Supported Bandwidths

| Bandwidth | Carriers | Max Mode | Use Case |
|-----------|----------|----------|----------|
| 500 Hz    | 11       | 13       | QRP, weak signal |
| 2300 Hz   | 49       | 16       | Standard HF operations |
| 2750 Hz   | 59       | 17       | Tactical, high throughput |

## Speed Levels

RIA supports 17 speed levels with automatic rate negotiation:

| Mode | Modulation | Code Rate | Data Rate (2300Hz) | Min SNR |
|------|------------|-----------|-------------------|---------|
| 2    | FSK        | 1/2       | ~41 bps           | -3 dB   |
| 5-10 | QPSK       | 1/2       | 175-2011 bps      | 6-19 dB |
| 11   | QPSK       | 2/3       | ~2500 bps         | 21 dB   |
| 12   | QPSK       | 3/4       | ~2800 bps         | 23 dB   |
| 13   | 8-PSK      | 2/3       | ~3700 bps         | 25 dB   |
| 14   | 8-PSK      | 3/4       | ~4200 bps         | 27 dB   |
| 15   | 16-QAM     | 3/4       | ~5600 bps         | 29 dB   |
| 16   | 32-QAM     | 5/6       | ~7050 bps         | 31 dB   |

*Mode 1 (FSK) is disabled pending soft-decision decoder implementation.*

## Building

### Requirements

- Rust 1.70 or later
- Audio libraries (platform-specific, handled by cpal)

### Build Commands

```bash
# Debug build
cargo build

# Optimized release build
cargo build --release

# Headless build (no GUI)
cargo build --release --no-default-features --features headless
```

### Run

```bash
./target/release/ria
```

## Configuration

RIA uses TOML configuration files. Location priority:

1. `$RIA_CONFIG` environment variable
2. `~/.config/ria/config.toml` (default)

### Configuration Options

```toml
# Station identification
callsign = "N0CALL"              # Amateur radio callsign (required)

# Network ports
tcp_command_port = 8300          # Control port for commands
tcp_data_port = 8301             # Data port for binary I/O

# Audio devices (omit for system default)
audio_input = "default"
audio_output = "default"
sample_rate = 48000              # Fixed at 48000 Hz

# Modulation settings
bandwidth = "2300"               # "500", "2300", or "2750"
default_speed_level = 9          # Speed level 2-17

# PTT control
ptt_method = "vox"               # vox, rts, dtr, or none

# Optional encryption
encryption_passphrase = ""       # Empty = disabled

# Advanced
preamble_threshold = 0.50        # Detection threshold (0.1-0.9)
```

### Environment Variables

```bash
# Override config file location
export RIA_CONFIG=/path/to/config.toml

# Run multiple instances (offsets ports by N×10)
export RIA_INSTANCE=1   # Uses ports 8310/8311
```

## TCP Interface

RIA provides a dual-port TCP interface for host application integration.

### Command Port (default: 8300)

Text-based commands with CR-terminated responses.

| Command | Arguments | Description |
|---------|-----------|-------------|
| `CONNECT` | `<callsign>` | Initiate connection |
| `DISCONNECT` | — | End current session |
| `LISTEN` | `ON/OFF` | Enable/disable listen mode |
| `MYCALL` | `<callsign>` | Set local callsign |
| `BWMODE` | `500/2300/2750` | Set bandwidth |
| `VERSION` | — | Get modem version |
| `BUFFER` | — | Get TX buffer size |
| `BUSY` | — | Check channel busy state |

**Example session:**
```bash
$ nc localhost 8300
MYCALL N0TEST
OK
LISTEN ON
OK
CONNECT W5ABC
PENDING
# ... wait for connection ...
CONNECTED W5ABC
```

### Data Port (default: 8301)

Binary data stream for frame transmission/reception.

- Single client connection enforced
- Raw binary frames (bidirectional)
- Data is automatically framed with headers and CRC

**Sending data:**
```bash
cat message.bin | nc localhost 8301
```

**Receiving data:**
```bash
nc localhost 8301 > received.bin
```

## Two-Modem Testing

For local testing with virtual audio cables (e.g., BlackHole on macOS):

**Terminal 1 - Modem 1:**
```bash
cat > /tmp/modem1.toml << EOF
callsign = "TEST1"
tcp_command_port = 8300
tcp_data_port = 8301
audio_input = "BlackHole 2ch"
audio_output = "BlackHole 2ch"
bandwidth = "2300"
default_speed_level = 12
ptt_method = "vox"
EOF

RIA_CONFIG=/tmp/modem1.toml ./target/release/ria
```

**Terminal 2 - Modem 2:**
```bash
cat > /tmp/modem2.toml << EOF
callsign = "TEST2"
tcp_command_port = 8310
tcp_data_port = 8311
audio_input = "BlackHole 2ch"
audio_output = "BlackHole 2ch"
bandwidth = "2300"
default_speed_level = 12
ptt_method = "vox"
EOF

RIA_CONFIG=/tmp/modem2.toml ./target/release/ria
```

**Test connection:**
```bash
# Enable listening on modem2
echo "LISTEN ON" | nc -w 1 localhost 8310

# Connect from modem1
echo "CONNECT TEST2" | nc -w 1 localhost 8300

# Send data through modem1
echo "Hello, World!" | nc localhost 8301

# Receive on modem2
nc localhost 8311
```

## Protocol Details

### Frame Structure

```
┌─────────┬──────┬──────────┬───────────┬────────────┬───────┬─────────┬─────┐
│ Version │ Type │ Sequence │ SessionID │ SpeedLevel │ Flags │ Payload │ CRC │
│  (1B)   │ (1B) │   (2B)   │   (2B)    │    (1B)    │ (1B)  │ (var)   │(4B) │
└─────────┴──────┴──────────┴───────────┴────────────┴───────┴─────────┴─────┘
```

- **Header**: 8 bytes
- **Payload**: Variable (20-1176 bytes depending on mode)
- **CRC-32**: 4 bytes
- **Overhead**: 16 bytes total

### Frame Types

| Type | Value | Description |
|------|-------|-------------|
| CONNECT | 0x01 | Connection request |
| CONNECT_ACK | 0x02 | Connection accepted |
| DISCONNECT | 0x03 | Disconnect request |
| DATA | 0x10 | Data frame |
| DATA_ACK | 0x11 | Data acknowledgment |
| IDLE | 0x20 | Keepalive |
| SPEED_CHANGE | 0x30 | Rate change request |

### Rate Negotiation

1. Connections start at mode 2 (most robust OFDM)
2. Each frame carries measured SNR and proposed mode
3. Effective SNR = min(local_snr, remote_snr)
4. Agreed mode = min(my_proposed, peer_proposed)
5. Fast downshift (immediate), slow upshift (3 consecutive readings)
6. 500ms debounce between mode changes prevents oscillation

### ARQ Protocol

- Selective repeat (individual frame retransmission)
- Maximum retries: 10
- Default timeout: 4000 ms
- Keepalive interval: 15 seconds

## Project Structure

```
RIA/
├── src/
│   ├── main.rs          # Application entry point
│   ├── lib.rs           # Library exports
│   ├── audio/           # Audio I/O (cpal)
│   ├── dsp/             # DSP (FFT, AGC, AFC, filters)
│   ├── fec/             # FEC (Turbo, convolutional, CRC)
│   ├── modem/           # Modulation (OFDM, FSK, preamble)
│   ├── protocol/        # Protocol (frames, ARQ, sessions)
│   ├── interface/       # TCP/KISS interfaces
│   └── gui/             # egui GUI
├── tests/               # Integration tests
├── test_debug/          # Test configurations
└── Cargo.toml           # Dependencies
```

## Dependencies

| Category | Libraries |
|----------|-----------|
| Audio | cpal, rodio |
| DSP | rustfft, realfft, num-complex |
| Networking | tokio (async runtime) |
| GUI | eframe, egui |
| Crypto | aes-gcm, pbkdf2, sha2 |
| Serialization | serde, toml |

## License

MIT License

## Contributing

Contributions are welcome. Please ensure all tests pass before submitting PRs:

```bash
cargo test --release
```

## Known Limitations

- Mode 1 (FSK) disabled - requires soft-decision decoder
- TCP binds to localhost only (127.0.0.1)
- Sample rate fixed at 48 kHz
- Per-bandwidth mode limits:
  - 500 Hz: modes 2-13
  - 2300 Hz: modes 2-16
  - 2750 Hz: modes 2-17

