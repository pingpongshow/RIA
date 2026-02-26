# RIA Modem

**High-performance HF modem for amateur radio**

*Last updated: 2026-02-24*

> **NON-COMMERCIAL USE ONLY**
>
> This project is under active development and is **not ready for production use**.
> Features may be incomplete, untested, or broken. APIs and protocols may change
> without notice. Performance numbers are from simulation only - real-world HF
> testing is ongoing. **Use at your own risk for experimentation and development only.**
>
> **See [LICENSE](LICENSE) for terms of use, encryption notices, and disclaimers.**

RIA Modem is a software modem that achieves reliable, high-speed data transfer over HF radio. It uses adaptive waveform selection to maintain communication across varying ionospheric conditions - from quiet bands to disturbed polar paths.

[![License: Non-Commercial](https://img.shields.io/badge/License-Non--Commercial-red.svg)](LICENSE)
[![Status: Experimental](https://img.shields.io/badge/Status-Experimental-orange.svg)]()
[![Platform](https://img.shields.io/badge/Platform-Linux%20%7C%20macOS%20%7C%20Windows-lightgrey.svg)]()

---

## Current Status (v0.3.0-alpha)

- **Extreme low-SNR capability**: MC-DPSK with 4x spreading verified at -8 dB SNR (floor -14 dB)
- **HARQ chase combining**: Soft-bit caching provides ~3 dB gain per combine on retransmissions
- **Hybrid preamble**: CHIRP for handshake (±50 Hz), Zadoff-Chu for data (23x faster sync)
- **QAM auto-selection**: QAM16/32/64 automatically enabled on high-SNR AWGN channels
- **Selective Repeat ARQ**: Window=4 for OFDM, window=1 with chase combining for MC-DPSK
- **Stable baseline**: MC-DPSK DBPSK is now default for all connections (most robust)
- **Still experimental**: High-order QAM on fading channels needs further work
- **OTA status**: On-air testing is active, but this is still alpha software and not production-ready

---

## Features

- **Adaptive Waveforms**: Automatically selects optimal waveform based on channel conditions
- **Extreme Low-SNR Operation**: Down to -14 dB with 4x time-domain spreading (MC-DPSK)
- **Wide SNR Range**: Operates from -14 dB to 30+ dB across all modes
- **Strong FEC**: LDPC codes with rates from R1/4 to R3/4
- **HARQ Chase Combining**: Soft-bit combining on retransmissions (~3 dB gain per doubling)
- **Hybrid Preamble**: CHIRP (±50 Hz CFO) for handshake, Zadoff-Chu (52ms) for data
- **ARQ Protocol**: Stop-and-wait for MC-DPSK, Selective Repeat (window=4) for OFDM
- **AES-256 Encryption**: Optional payload encryption (see license for legal notices)
- **GUI Application**: Real-time waterfall, constellation display, and message log
- **CLI Tools**: Frame-level transmit/receive for testing and debugging
- **CAT Control**: Integrated PTT via Hamlib, Kenwood TCP, or serial

---

## Performance

**MC-DPSK (Low SNR, 10 carriers)** - Default for challenging conditions:
| SNR | Mode | Spreading | Throughput | Notes |
|-----|------|-----------|------------|-------|
| **< -7 dB** | DBPSK | **4x** | ~117 bps | Floor -14 dB, verified at -8 dB |
| **-7 to -3 dB** | DBPSK | **2x** | ~235 bps | Floor -8 dB, verified |
| **-3 to 5 dB** | DBPSK | None | ~469 bps | Floor -4 dB, verified |
| **5 to 10 dB** | DQPSK | None | ~938 bps | Floor +5 dB, verified |

**OFDM (Higher SNR, 59 carriers)** - 1024 FFT, CP=96, 42.9 sym/s:
| SNR | Mode | Data carriers | Throughput | Notes |
|-----|------|---------------|------------|-------|
| 10+ dB | OFDM DQPSK R1/4 | 59 (no pilots) | 1264 bps | 100% verified, fading OK |
| 15+ dB | OFDM DQPSK R1/2 | 53 (6 pilots) | 2271 bps | 100% verified, good + moderate fading |
| 20+ dB | OFDM DQPSK R2/3 | 53 (6 pilots) | 3028 bps | 100% verified, good fading |
| 20+ dB | OFDM DQPSK R3/4 | 55 (4 pilots) | 3536 bps | 100% verified, AWGN only |
| 18+ dB | OFDM QAM16 R1/2 | 53 (6 pilots) | ~4800 bps | Auto-selected on AWGN |
| 22+ dB | OFDM QAM32 R3/4 | 53 (6 pilots) | ~6000 bps | Auto-selected on AWGN |
| 25+ dB | OFDM QAM64 R3/4 | 53 (6 pilots) | ~7200 bps | Auto-selected on AWGN |

**Coherent modes (stable channels only):**
| SNR | Mode | Data carriers | Throughput | Notes |
|-----|------|---------------|------------|-------|
| 20+ dB | OFDM QPSK R1/2 | 47 (12 pilots) | 2014 bps | Coherent path, sensitive to phase/fading |
| 25+ dB | OFDM 16QAM R3/4 | 44 (15 pilots) | 5657 bps | Stable paths (NVIS, ground wave) |
| 30+ dB | OFDM 32QAM R3/4 | 44 (15 pilots) | **7071 bps** | Maximum throughput |
|        | OFDM 64QAM      |                |              | Unlikely to be used on HF |

**When to use coherent modes (16QAM/32QAM):** Stable propagation paths like
ground wave or direct cable connection. These paths have stable phase, allowing
coherent demodulation with pilot-assisted channel estimation.

### Waveform Strategy

RIA Modem uses adaptive waveform selection based on channel conditions:

```
SNR Range           Waveform                    Why
─────────────────────────────────────────────────────────────────────────
< -7 dB             MC-DPSK DBPSK 4x spread     Extreme low-SNR, ~117 bps
-7 to -3 dB         MC-DPSK DBPSK 2x spread     Very low SNR, ~235 bps
-3 to 5 dB          MC-DPSK DBPSK               Low SNR default, ~469 bps
5-10 dB             MC-DPSK DQPSK               Moderate SNR, ~938 bps
10-17 dB            OFDM-CHIRP DQPSK            Good SNR, 1.2-3.5 kbps
17+ dB              OFDM-CHIRP QAM16/32/64      Excellent SNR, 4.8-7.2 kbps
25+ dB (NVIS)       OFDM QAM64 R3/4             Maximum throughput
```

**MC-DPSK (Multi-Carrier DPSK)**: 10 carriers with differential encoding. Default mode for connection establishment and low-SNR operation. Features:
- **Time-domain spreading**: 2x or 4x symbol repetition for extreme low-SNR
- **Hybrid preamble**: CHIRP (1200ms) for handshake, ZC (52ms) for data frames
- **HARQ Chase Combining**: Caches soft bits from failed decodes, combines on retransmission
- Works down to -14 dB with 4x spreading (floor verified at -8 dB)

**D8PSK (8-Phase DPSK)**: +50% throughput over DQPSK (3 bits/symbol vs 2). This mode is currently treated as opportunistic. It performs well in AWGN and some strong fading runs, but remains more variable than DQPSK in sustained HF fading. A two-pass decoder uses the embedded DQPSK grid to estimate and correct phase drift before D8PSK decoding.

**Two OFDM sync modes**:
- **OFDM-CHIRP**: Dual chirp preamble for robust sync, works at 10+ dB
- **OFDM-COX**: Schmidl-Cox sync for faster acquisition, requires 17+ dB

**NVIS/Local optimization**: For stable paths like NVIS (Near Vertical Incidence Skywave), 16QAM with pilots provides better performance than differential modes. The pilots track slow phase drift and frequency-selective fading. Maximum throughput ~7.1 kbps with 32QAM R3/4.

---

## Getting Started

### Requirements

- Linux, macOS, or Windows
- CMake 3.16+
- SDL2 (for GUI)
- **FFTW3** (required for fast chirp detection)
- C++20 compiler (GCC 10+, Clang 12+, MSVC 2019+)

### Building

```bash
# Install dependencies
# Ubuntu/Debian: sudo apt install libsdl2-dev libfftw3-dev cmake build-essential
# macOS: brew install sdl2 fftw cmake
# Windows: vcpkg install sdl2 fftw3

git clone https://github.com/mfrigerio/RIAModem.git
cd RIAModem
mkdir build && cd build
cmake ..
make -j4
```

### Running

**GUI Application:**
```bash
./ria_gui                # Normal mode
./ria_gui -sim           # Simulator mode (no radio needed)
```

**CLI Tools** (individual frames, not full protocol):
```bash
# Transmit single frame
./ria ptx "Hello World" -s MYCALL -d THEIRCALL | aplay -f FLOAT_LE -r 48000

# Decode frames from audio
arecord -f FLOAT_LE -r 48000 | ./ria prx

# Loopback test
./ria ptx "Test message" -s A -d B | ./ria prx
```

> **Note**: The CLI generates/decodes individual frames. It does not support
> full protocol sessions (PING→CONNECT→DATA→DISCONNECT). For interactive
> operation, use the GUI application.

---

## How It Works

### Protocol

1. **PING/PONG** - Fast presence probe (~1 sec each) to check if remote station is listening
2. **CONNECT** - Full callsign exchange after successful probe (FCC Part 97.119 compliance)
3. **MODE_CHANGE** - Negotiates optimal modulation/coding based on measured SNR
4. **DATA** - Transfers payload with per-frame acknowledgment
5. **DISCONNECT** - Graceful termination with callsign ID (regulatory compliance)

The PING/PONG probe allows quick "anyone home?" detection before committing to the
full CONNECT sequence. If no response after 5 PINGs (15 seconds), connection fails fast.

### Signal Parameters

| Parameter | MC-DPSK | OFDM |
|-----------|---------|------|
| Sample Rate | 48 kHz | 48 kHz |
| Bandwidth | ~2.4 kHz | ~2.8 kHz |
| Center Frequency | 1500 Hz | 1500 Hz |
| Carriers | 8 | 59 |
| FFT Size | N/A | 1024 |
| Symbol Rate | ~94 baud | ~42.9 baud |
| Cyclic Prefix | N/A | 96 samples (2ms) |
| Sync Method | Dual Chirp | Dual Chirp or Schmidl-Cox |
| LDPC Codeword | 648 bits | 648 bits |

### LDPC Codes

| Rate | Info Bytes | Use Case |
|------|------------|----------|
| R1/4 | 20 | Low SNR, maximum reliability |
| R1/2 | 40 | Moderate SNR, good balance |
| R2/3 | 54 | Good SNR, higher throughput |
| R3/4 | 60 | Very good SNR |
| R5/6 | 67 | Excellent SNR, maximum throughput |

---

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│              GUI Application (ImGui + SDL2)             │
├─────────────────────────────────────────────────────────┤
│     Protocol Engine (Connection, ARQ, File Transfer)    │
├─────────────────────────────────────────────────────────┤
│        Modem Engine (TX/RX coordination, SNR est)       │
├──────────────────────────┬────────────────────────────┤
│       OFDM Mod/Dem       │       DPSK Mod/Dem        │
├──────────────────────────┴────────────────────────────┤
│  LDPC Encoder/Decoder  │  Interleaver  │  Sync/CFO     │
├─────────────────────────────────────────────────────────┤
│                    Audio I/O (SDL2)                     │
└─────────────────────────────────────────────────────────┘
```

---

## Testing

```bash
cd build

# Primary test - full protocol (PING/CONNECT/MODE_CHANGE/DATA/DISCONNECT)
./cli_simulator --snr 15 --fading good --rate r1_4 --test

# Test specific modulation/rate combinations
./cli_simulator --snr 20 --fading good --mod d8psk --rate r1_2 --test
./cli_simulator --snr 20 --mod dqpsk --rate r2_3 --test

# Quick single-frame sanity check (not full protocol)
./test_waveform_simple -w ofdm_chirp --snr 15

# Run unit tests
ctest
```

**Adaptive Advisory Smoke Test (log-only, no live MODE_CHANGE yet):**

```bash
./build/cli_simulator --adpt-test --snr 20 --channel good --waveform ofdm_chirp --seed 42
```

This test runs two message phases across two channel conditions and prints `[ADPT]` lines:
- Peer-reported advisory from CONNECT/CONNECT_ACK metrics.
- Local rolling advisory with hysteresis behavior:
  - fast downgrade path
  - delayed upgrade path (hold timer before promotion)

Useful flags:
- `--hop-snr <dB>` sets phase-B SNR (default: `12`)
- `--hop-channel <awgn|good|moderate|poor|flutter>` sets phase-B channel

**Manual Modulation Selection:**

The `--mod` flag allows testing specific modulations:
- `dqpsk` - 4-phase differential (default, 2 bits/symbol)
- `d8psk` - 8-phase differential (+50% throughput, 3 bits/symbol)
- `dbpsk` - 2-phase differential (most robust, 1 bit/symbol)

The `--rate` flag selects LDPC code rate:
- `r1_4` - Most robust (required for fading channels)
- `r1_2` - Balanced (default for AWGN)
- `r2_3`, `r3_4`, `r5_6` - Higher throughput, needs better SNR

**GUI Expert Settings:**

To force specific modulation in the GUI application:
1. Open **Settings** (gear icon)
2. Go to **Expert** tab
3. Set **Forced Modulation** to desired mode (D8PSK, DQPSK, etc.)
4. Optionally set **Forced Code Rate** (R1/4 recommended for fading)

When D8PSK is forced on a fading channel, the two-pass decoder automatically
activates to improve reliability (uses embedded DQPSK grid for phase correction).

**Test Tools:**
| Tool | Purpose |
|------|---------|
| `cli_simulator` | Primary - full protocol with two-station interaction |
| `test_waveform_simple` | Quick single-frame sanity checks |
| `test_chase_cache` | HARQ chase combining verification |
| `test_spreading` | MC-DPSK time-domain spreading modes |
| `test_zc_dbpsk` | Zadoff-Chu sync + DBPSK SNR sweep |

---

## Radio Setup

### Requirements
- SSB transceiver with 2.8+ kHz filter bandwidth
- Audio interface (SignaLink, RigBlaster, or direct soundcard connection)
- PTT control (VOX, CAT, or hardware)

### Audio Levels
- TX: Adjust for clean signal without ALC compression
- RX: Set for comfortable listening level (avoid clipping)

### Recommended Operating Frequencies

RIA Modem uses **~2.8 kHz bandwidth**. Operate in wideband digital segments,
not narrow-band FT8/PSK31 areas.

| Band | Frequency (USB) | Notes |
|------|-----------------|-------|
| 80m | 3.590 MHz | Above narrow digital, below voice |
| 40m | 7.102 MHz | Common for wideband digital |
| 30m | 10.145 MHz | Check for WSPR at 10.140 |
| 20m | 14.108 MHz | Above FT8 crowd, wideband digital |
| 15m | 21.110 MHz | Above narrow digital segment |
| 10m | 28.120 MHz | Plenty of room |

**Avoid:**
- 14.070-14.095 MHz (FT8/PSK31)
- Any .074 MHz frequencies (FT8)
- 14.100 MHz (NCDXF beacons)

**Best practice:** Listen for 10-15 seconds before transmitting. Use minimum
power necessary. Be ready to QSY if causing interference.

---

## Current Status

> **Note**: Core waveforms have been verified in audio loopback.
> On-air HF testing is in progress.

### What Is Solid Today

| Waveform | SNR Range | CFO Tolerance | Status |
|----------|-----------|---------------|--------|
| **MC-DPSK DBPSK 4x spread** | **-14 to -7 dB** | ±50 Hz | **Verified at -8 dB** |
| **MC-DPSK DBPSK 2x spread** | -7 to -3 dB | ±50 Hz | Verified |
| **MC-DPSK DBPSK** (10 carriers) | -3 to 5 dB | ±50 Hz | Stable baseline, default |
| **MC-DPSK DQPSK** (10 carriers) | 5 to 10 dB | ±50 Hz | Stable baseline |
| **OFDM-CHIRP DQPSK** (59 carriers) | 10+ dB | ±50 Hz | Stable baseline |
| **OFDM-CHIRP QAM16/32/64** | 18+ dB | ±50 Hz | Auto-selected on AWGN |
| **OFDM-COX** (Schmidl-Cox sync) | 17+ dB | TBD | Working, OTA characterization ongoing |

**Recent verification (2026-02-24):**
- MC-DPSK 4x spreading at -8 dB: 7/7 messages, 100% frame success
- MC-DPSK DBPSK at SNR 0: 7/7 messages, 0 retransmissions
- HARQ chase combining: 25% → 99% decode success with 2 combines at marginal SNR
- QAM64 at SNR 25 AWGN: 100% frame success, auto-selected

### Use With Caution (Experimental)

- Coherent modes (QPSK/16QAM/32QAM): suitable for stable high-SNR paths, not the default for difficult HF fading.
- High-rate operation (`R2/3+`) still depends on channel quality and control-path stability.

### Active Work

OTA testing ang bug fixes

## Research Roadmap

**Goal: Exceed industry-leading HF modem speeds (currently ~8.5 kbps in 2.8 kHz bandwidth)**

RIA Modem is a research platform for investigating advanced modulation and coding techniques for HF channels. Our target is **10+ kbps** throughput while maintaining robustness.

### Techniques Under Investigation

| Technique | Expected Gain | Status | References |
|-----------|---------------|--------|------------|
| **OTFS** | 20% SE over OFDM, full diversity | Implemented, needs HF testing | [arxiv.org/pdf/2302.14224](https://arxiv.org/pdf/2302.14224) |
| **AFDM** | >1 dB over OTFS, lower pilots | Not yet implemented | [arxiv.org/html/2507.21704v3](https://arxiv.org/html/2507.21704v3) |
| **Turbo DPSK** | 4.65 dB on Rayleigh fading | Planned | [Turbo DPSK paper](https://www.academia.edu/81703561/Turbo_DPSK_iterative_differential_PSK_demodulation_and_channel_decoding) |
| **Faster-than-Nyquist** | 25% more bits (Mazo limit) | Planned | [IEEE ComSoc overview](https://www.comsoc.org/publications/ctn/running-faster-nyquist-idea-whose-time-may-have-come) |
| **SC-LDPC** | 0.07 dB from capacity | Planned | [SC-LDPC tutorial](https://www.itsoc.org/sites/default/files/2021-03/laurent.schmalen@kit.edu%20-%20ESIT_20_Schmalen.pdf) |
| **OFDM-IM** | 2x SE via index modulation | Planned | [arxiv.org/html/2501.15437v1](https://arxiv.org/html/2501.15437v1) |

### Why These Techniques?

**OTFS/AFDM**: Traditional OFDM suffers from inter-carrier interference (ICI) when Doppler spread is high. OTFS and AFDM work in the delay-Doppler domain, achieving full diversity over doubly-selective channels. Literature shows AFDM outperforms OTFS by >1 dB with simpler channel estimation.

**Turbo DPSK**: Our single-carrier DPSK already works to -11 dB SNR. Adding iterative (turbo) processing between the DPSK demodulator and LDPC decoder could push this to -15 dB or improve throughput at higher SNR.

**Faster-than-Nyquist**: By accepting controlled intersymbol interference and using iterative detection, FTN can transmit 25% more symbols in the same bandwidth (Mazo limit: τ=0.8).

**Spatially-Coupled LDPC**: Our current LDPC codes are from IEEE 802.11n. SC-LDPC codes achieve threshold saturation, approaching capacity within 0.07 dB on fading channels.

### Current Benchmarks

| Mode | RIA Modem | Industry Leader | Gap |
|------|-----------|-----------------|-----|
| Max throughput (verified, QAM64) | 7.2 kbps | 8.5 kbps | -15% |
| Max throughput (differential DQPSK) | 3.5 kbps | 8.5 kbps | -59% |
| **Low-SNR floor (verified)** | **-14 dB** | ~0 dB | **+14 dB better** |
| Low-SNR throughput | 117 bps @ -8 dB | N/A | Unique capability |
| CFO tolerance | ±50 Hz | ±100 Hz | Sufficient for HF |

### How to Contribute

TESTING

---

## Configuration Notes

### TCP Interface

The TCP interface allows external programs to control the modem.

- **Command Port**: 8300 (default) - Send ASCII commands
- **Data Port**: 8301 (default) - Send/receive binary data

Connect to the command port: `nc localhost 8300`

Common commands: `CONNECT`, `DISCONNECT`, `MYCALL`, `VERSION`, `WAVEFORM`, `MODULATION`, `CODERATE`, etc.

See [docs/TCPCommands.md](docs/TCPCommands.md) for full documentation.

### KISS TNC

Standard TNC protocol for packet applications.

- **KISS Port**: 8302 (default)
- Standard KISS framing (FEND, FESC escaping)
- Configuration commands: TxDelay, Persistence, SlotTime, TxTail, FullDuplex

### Encryption

- Both stations must use the same passphrase for successful communication
- The passphrase is not saved to disk for security
- All data payloads are encrypted with AES-256-CBC
- Control frames (ACK/NACK) remain unencrypted for protocol operation

TCP Commands:
- `ENCRYPT ON|OFF` - Enable/disable encryption
- `ENCRYPTKEY <passphrase>` - Set encryption key

### Data Compression

- Uses deflate/zlib compression
- Compresses data payloads to improve throughput
- Automatically skips compression for already-compressed data

TCP Command: `COMPRESSION ON|OFF`

---

## License

**Non-Commercial Use Only.** Copyright (c) 2026 Stephen Packer.

This software is provided for personal, educational, and amateur radio experimentation purposes only. Commercial use is prohibited.

See [LICENSE](LICENSE) for full terms, including:
- Non-commercial use restrictions
- Third-party technology attributions
- Encryption legal notices
- Disclaimers of warranty

Third-party attributions: See [LICENSE_THIRDPARTY.md](LICENSE_THIRDPARTY.md)

---

## Acknowledgments

- ProjectUltra
- [Dear ImGui](https://github.com/ocornut/imgui) - GUI framework
- [SDL2](https://libsdl.org/) - Audio and windowing
- [FFTW3](https://www.fftw.org/) - Fast Fourier Transform (required for chirp detection)
- [miniz](https://github.com/richgel999/miniz) - Compression
