# RIA Modem - Build and Usage Instructions

## System Requirements

- **Operating System**: macOS, Linux, or Windows
- **Rust**: Version 1.70 or later
- **Audio**: Working audio input/output devices (or virtual audio cable for testing)

## Building from Source

### Install Rust

If you don't have Rust installed:

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
source $HOME/.cargo/env
```

### Clone and Build

```bash
# Clone the repository
git clone <repository-url>
cd RIA

# Build release version (recommended)
cargo build --release

# The binary will be at ./target/release/ria
```

### Build Options

```bash
# Standard build with GUI
cargo build --release

# Headless build (no GUI, smaller binary)
cargo build --release --no-default-features --features headless

# Debug build (slower, more logging)
cargo build
```

## Configuration

### Create Configuration File

Create a configuration file at `~/.config/ria/config.toml`:

```bash
mkdir -p ~/.config/ria
```

Example configuration:

```toml
# Your amateur radio callsign (required)
callsign = "N0CALL"

# TCP ports for host application interface
tcp_command_port = 8300
tcp_data_port = 8301

# Audio devices (omit or use "default" for system default)
# audio_input = "USB Audio Device"
# audio_output = "USB Audio Device"

# Bandwidth mode: "500", "2300", or "2750"
bandwidth = "2300"

# Starting speed level (2-17, default 9)
default_speed_level = 9

# PTT method: "vox", "rts", "dtr", or "none"
ptt_method = "vox"

# Optional encryption passphrase (leave empty to disable)
encryption_passphrase = ""
```

### Using a Custom Config Location

```bash
export RIA_CONFIG=/path/to/your/config.toml
./target/release/ria
```

## Running the Modem

### Basic Startup

```bash
./target/release/ria
```

The modem will:
1. Load configuration from default location or `$RIA_CONFIG`
2. Initialize audio devices
3. Start TCP servers on configured ports
4. Display the GUI (if built with GUI support)

### Running Multiple Instances

Use the `RIA_INSTANCE` environment variable to offset ports:

```bash
# Instance 0: ports 8300/8301
RIA_INSTANCE=0 ./target/release/ria

# Instance 1: ports 8310/8311
RIA_INSTANCE=1 ./target/release/ria

# Instance 2: ports 8320/8321
RIA_INSTANCE=2 ./target/release/ria
```

## Basic Usage

### Connecting to a Remote Station

1. Start the modem
2. Connect to the command port:
   ```bash
   nc localhost 8300
   ```
3. Set your callsign (if not in config):
   ```
   MYCALL N0CALL
   ```
4. Enable listening on the remote station:
   ```
   LISTEN ON
   ```
5. Initiate connection:
   ```
   CONNECT W5ABC
   ```
6. Wait for `CONNECTED W5ABC` response

### Sending Data

Once connected, send data through the data port:

```bash
# Send a file
cat message.txt | nc localhost 8301

# Send text
echo "Hello, World!" | nc localhost 8301
```

### Receiving Data

```bash
# Receive to terminal
nc localhost 8301

# Receive to file
nc localhost 8301 > received.bin
```

### Disconnecting

```bash
echo "DISCONNECT" | nc -w 1 localhost 8300
```

## Audio Setup

### Finding Audio Devices

The modem will list available audio devices in its startup log. Look for lines like:

```
[INFO] Input device: USB Audio Device
[INFO] Output device: USB Audio Device
```

### Virtual Audio Cable (for Testing)

For local testing without radio hardware:

**macOS**: Install [BlackHole](https://github.com/ExistentialAudio/BlackHole)
```bash
brew install blackhole-2ch
```

**Linux**: Use PulseAudio null sink or JACK
```bash
pactl load-module module-null-sink sink_name=virtual_cable
```

**Windows**: Install [VB-Cable](https://vb-audio.com/Cable/)

Then configure both modem instances to use the virtual cable:
```toml
audio_input = "BlackHole 2ch"
audio_output = "BlackHole 2ch"
```

## Bandwidth Selection

Choose bandwidth based on your operating conditions:

| Bandwidth | Best For | Modes |
|-----------|----------|-------|
| 500 Hz | Weak signals, QRP, crowded bands | 2-13 |
| 2300 Hz | General HF operations | 2-16 |
| 2750 Hz | Maximum throughput, good conditions | 2-17 |

Set in config file:
```toml
bandwidth = "2300"
```

Or via TCP command:
```
BWMODE 2300
```

## Speed Level Selection

The modem automatically adapts speed based on SNR, but you can set a starting level:

| Level | Best For |
|-------|----------|
| 2-4 | Very weak signals, high noise |
| 5-9 | Moderate conditions |
| 10-12 | Good conditions |
| 13-17 | Excellent conditions, high throughput |

## Troubleshooting

### No Audio Devices Found

- Check that audio devices are connected and recognized by OS
- Try specifying device names explicitly in config
- Run with debug logging: `RUST_LOG=debug ./target/release/ria`

### Connection Timeouts

- Verify both stations are on the same bandwidth setting
- Check that audio levels are appropriate (not clipping, not too quiet)
- Ensure PTT is activating correctly

### High Error Rates

- Reduce speed level: `default_speed_level = 5`
- Use narrower bandwidth: `bandwidth = "500"`
- Check for interference or noise on the channel

### Port Already in Use

- Another instance may be running: `pkill ria`
- Use different ports via `RIA_INSTANCE` or config file

## Logging

Enable detailed logging:

```bash
# Info level (default)
RUST_LOG=info ./target/release/ria

# Debug level (verbose)
RUST_LOG=debug ./target/release/ria

# Trace level (very verbose)
RUST_LOG=trace ./target/release/ria
```

## Testing Your Setup

### Loopback Test

With a virtual audio cable configured:

```bash
# Terminal 1 - Start modem 1
RIA_CONFIG=/tmp/m1.toml ./target/release/ria

# Terminal 2 - Start modem 2
RIA_CONFIG=/tmp/m2.toml ./target/release/ria

# Terminal 3 - Enable listening
echo "LISTEN ON" | nc -w 1 localhost 8310

# Terminal 3 - Connect
echo "CONNECT TEST2" | nc -w 1 localhost 8300

# Terminal 3 - Send test data
head -c 1000 /dev/urandom > /tmp/test.bin
cat /tmp/test.bin | nc localhost 8301

# Terminal 4 - Receive and verify
nc localhost 8311 > /tmp/received.bin
cmp /tmp/test.bin /tmp/received.bin && echo "SUCCESS"
```
