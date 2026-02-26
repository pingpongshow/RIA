# CAT Control System

Ultra implements integrated CAT (Computer Aided Transceiver) control, allowing the modem to directly manage radio PTT, frequency, and mode. This eliminates the need for external applications to coordinate PTT timing and enables seamless radio integration.

## Overview

The CAT system provides:
- **Direct PTT control** with configurable lead/tail timing
- **Frequency and mode control** for supported radios
- **TX watchdog** to prevent stuck transmitters
- **Multiple backends** for different radio types

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                      CatController                          │
│  - PTT timing (lead/tail delays)                           │
│  - TX watchdog (auto-release after timeout)                │
│  - Backend lifecycle management                            │
└─────────────────────────────────────────────────────────────┘
                              │
         ┌────────────────────┼────────────────────┐
         │                    │                    │
         ▼                    ▼                    ▼
┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐
│ SerialPttBackend│  │  HamlibBackend  │  │KenwoodTcpBackend│
│ (DTR/RTS only)  │  │ (200+ radios)   │  │ (FlexRadio)     │
└─────────────────┘  └─────────────────┘  └─────────────────┘
```

## Backends

### Serial PTT Backend

The simplest backend - controls PTT via serial port DTR or RTS lines.

**Features:**
- PTT control via DTR or RTS
- Optional inversion
- No frequency/mode control

**Configuration:**
| Setting | Description | Default |
|---------|-------------|---------|
| Port | Serial port (e.g., `/dev/ttyUSB0`, `COM3`) | - |
| PTT Line | DTR (0) or RTS (1) | DTR |
| Invert | Invert PTT signal | No |

**Use Cases:**
- Simple radio interfaces (SignaLink, homebrew)
- Radios without CAT capability
- VOX-based setups with PTT override

### Hamlib Backend

Full-featured backend using the Hamlib library for broad radio support.

**Features:**
- PTT control
- Frequency get/set
- Mode get/set (USB, LSB, AM, FM, CW, DATA, etc.)
- Support for 200+ radio models

**Configuration:**
| Setting | Description | Default |
|---------|-------------|---------|
| Model | Hamlib model ID (see table below) | - |
| Port | Serial port | - |
| Baud | Serial baud rate | 9600 |

**Common Hamlib Model IDs:**
| ID | Radio |
|----|-------|
| 1 | Dummy/Test rig |
| 2014 | Kenwood TS-2000 |
| 3073 | Icom IC-7300 |
| 3081 | Icom IC-7610 |
| 3085 | Icom IC-705 |
| 229 | Yaesu FT-991A |
| 241 | Yaesu FT-DX10 |
| 1035 | Elecraft K3/K3S |
| 1037 | Elecraft KX3 |

For a complete list, see: https://hamlib.github.io/

**Requirements:**
- Hamlib library must be installed (`brew install hamlib` or `apt install libhamlib-dev`)
- Ultra must be built with Hamlib support (`ULTRA_ENABLE_HAMLIB=ON`)

### Kenwood TCP Backend

TCP-based backend using Kenwood ASCII protocol over TCP. Primarily for FlexRadio SmartSDR.

**Features:**
- PTT control
- Frequency get/set
- Mode get/set
- Slice selection (FlexRadio)

**Configuration:**
| Setting | Description | Default |
|---------|-------------|---------|
| Host:Port | TCP endpoint | localhost:4532 |
| Slice | FlexRadio slice number (0-7) | 0 |

**Protocol:**
Commands follow Kenwood ASCII format:
- `FA` - Get/set frequency (VFO A)
- `MD` - Get/set mode
- `TX0`/`TX1` - Set PTT off/on
- `RX` - Set PTT off

**FlexRadio Setup:**
1. Enable SmartSDR CAT in Flex settings
2. Configure TCP port (default 4532)
3. Select appropriate slice

## PTT Timing

PTT timing is critical for proper radio operation. The modem handles timing automatically:

```
         ┌────────────────────────────────────────┐
PTT:   ──┘                                        └──
         │←lead→│←─── Audio Transmission ───→│←tail→│

Timeline: PTT ON → Lead Delay → Audio Starts → Audio Ends → Tail Delay → PTT OFF
```

| Parameter | Description | Default | Typical Range |
|-----------|-------------|---------|---------------|
| Lead Time | Delay after PTT before audio | 50 ms | 20-200 ms |
| Tail Time | Delay after audio before PTT release | 50 ms | 20-100 ms |

**Tuning Tips:**
- **Too short lead time**: Audio clipped at beginning, missed preamble
- **Too long lead time**: Wasted airtime, slower throughput
- **Too short tail time**: Audio clipped at end, CRC failures
- **Too long tail time**: Wasted airtime

Typical values:
- Modern transceivers (IC-7300, FT-991A): 50ms lead, 30ms tail
- Older transceivers: 100ms lead, 50ms tail
- External amplifiers: Add 50-100ms to lead time

## TX Watchdog

The TX watchdog is a safety feature that automatically releases PTT if transmission exceeds a timeout.

**Configuration:**
- Default timeout: 120 seconds (2 minutes)
- Range: 0 (disabled) to 3600 seconds
- Set via GUI or TCP command: `CATWATCHDOG <seconds>`

**Behavior:**
1. Watchdog timer starts when PTT is asserted
2. If transmission completes normally, timer resets
3. If timeout exceeded, PTT is forced off and transmission aborted
4. Warning logged to message log

**Use Cases:**
- Prevent stuck transmitter due to software bugs
- Comply with regulatory TX time limits
- Protect equipment from overheating

## GUI Configuration

CAT settings are configured in Settings -> CAT tab:

### Enable CAT
Master enable/disable for CAT control. When disabled, legacy serial PTT (Settings -> PTT) is used.

### Backend Selection
- **None**: CAT disabled
- **Serial PTT**: DTR/RTS only
- **Hamlib**: Full CAT via Hamlib
- **Kenwood TCP**: TCP-based for FlexRadio

### Backend-Specific Options

**Serial PTT:**
- Serial Port
- PTT Line (DTR/RTS)
- Invert

**Hamlib:**
- Model ID
- Serial Port
- Baud Rate

**Kenwood TCP:**
- Host
- Port
- Slice (FlexRadio)

### Common Options
- PTT Lead Time (ms)
- PTT Tail Time (ms)
- Watchdog Timeout (seconds)

## TCP Commands

See [TCPCommands.md](TCPCommands.md) for complete command reference. Summary:

| Command | Description |
|---------|-------------|
| `CATENABLE ON/OFF` | Enable/disable CAT |
| `CATBACKEND type` | Select backend |
| `CATMODEL id` | Set Hamlib model |
| `CATPORT path` | Set serial port or host:port |
| `CATBAUD rate` | Set serial baud rate |
| `CATSLICE n` | Set FlexRadio slice |
| `CATCONNECT` | Connect to radio |
| `CATDISCONNECT` | Disconnect from radio |
| `CATPTT ON/OFF` | Manual PTT control |
| `CATFREQ hz` | Set frequency |
| `CATGETFREQ` | Get frequency |
| `CATMODE mode` | Set mode (USB/LSB/etc.) |
| `CATGETMODE` | Get mode |
| `CATWATCHDOG sec` | Set TX timeout |
| `CATPTTLEAD ms` | Set PTT lead time |
| `CATPTTTAIL ms` | Set PTT tail time |
| `CATSTATUS` | Get CAT status |

## Example: FlexRadio Setup

```bash
# Connect to modem
nc localhost 8300

# Configure CAT for FlexRadio
CATENABLE ON
CATBACKEND KENWOOD
CATPORT 192.168.1.100:4532
CATSLICE 0
CATCONNECT

# Set frequency and mode
CATFREQ 14074000
CATMODE USB

# Ready for operation
STATE
```

## Example: IC-7300 Setup

```bash
# Connect to modem
nc localhost 8300

# Configure CAT for IC-7300 via Hamlib
CATENABLE ON
CATBACKEND HAMLIB
CATMODEL 3073
CATPORT /dev/ttyUSB0
CATBAUD 19200
CATCONNECT

# Set frequency and mode
CATFREQ 7074000
CATMODE USB

# Check status
CATSTATUS
```

## Troubleshooting

### Connection Fails

1. **Check serial port permissions** (Linux):
   ```bash
   sudo usermod -a -G dialout $USER
   # Log out and back in
   ```

2. **Verify port name**:
   - Linux: `/dev/ttyUSB0`, `/dev/ttyACM0`
   - macOS: `/dev/tty.usbserial-*`
   - Windows: `COM3`, `COM4`

3. **Check baud rate** matches radio settings

4. **Test with rigctl** (Hamlib):
   ```bash
   rigctl -m 3073 -r /dev/ttyUSB0 -s 19200 f
   ```

### PTT Not Working

1. **Check PTT line** setting (DTR vs RTS)
2. **Check invert** setting
3. **Verify cable wiring**
4. **Check radio VOX** is disabled

### Frequency/Mode Not Changing

1. **Backend must be Hamlib or Kenwood TCP** (Serial PTT doesn't support)
2. **Verify radio is in remote control mode**
3. **Check radio manual** for CAT enable settings
4. **Some radios require specific CI-V address** (Hamlib config)

### Watchdog Triggering

1. **Increase watchdog timeout** if legitimate long transmissions
2. **Check for software bugs** causing stuck TX
3. **Set to 0 to disable** (not recommended)

## Integration with External Applications

Ultra's CAT system allows external applications to control the radio through the modem:

1. **Connect to command port** (8300)
2. **Use CAT commands** to set frequency, mode, etc.
3. **Modem handles PTT** automatically during transmission

This enables scenarios like:
- Remote frequency scanning
- Band/mode switching via TCP
- Integration with logging software
- Automated antenna switching (via frequency-based logic)

## Files

| File | Description |
|------|-------------|
| `src/cat/cat_backend.hpp` | Abstract backend interface |
| `src/cat/cat_backend.cpp` | String conversions, factory |
| `src/cat/cat_controller.hpp` | Main controller header |
| `src/cat/cat_controller.cpp` | Controller implementation |
| `src/cat/serial_ptt_backend.hpp/cpp` | Serial PTT backend |
| `src/cat/hamlib_backend.hpp/cpp` | Hamlib backend |
| `src/cat/kenwood_tcp_backend.hpp/cpp` | Kenwood TCP backend |

## Building with Hamlib

Hamlib support is optional. To enable:

```bash
# Install Hamlib
# macOS:
brew install hamlib

# Ubuntu/Debian:
sudo apt install libhamlib-dev

# Build Ultra with Hamlib
cmake -DULTRA_ENABLE_HAMLIB=ON ..
make
```

Without Hamlib, the Hamlib backend is unavailable but Serial PTT and Kenwood TCP still work.
