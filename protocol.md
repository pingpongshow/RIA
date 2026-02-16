# RIA Protocol Specification

## Overview

RIA is a high-performance HF modem protocol designed for reliable data transfer over amateur radio channels. It combines OFDM modulation, turbo forward error correction (FEC), and automatic repeat request (ARQ) to achieve robust communication in challenging conditions.

## Physical Layer

### Center Frequency

- **1500 Hz** (standard USB dial frequency offset)

### Sample Rate

- **48000 Hz** (fixed)

### Bandwidths

| Mode | Bandwidth | FFT Size | Carrier Spacing | Max Carriers |
|------|-----------|----------|-----------------|--------------|
| Narrow | 500 Hz | 1024/2048 | 46.88/23.44 Hz | 11 |
| Wide | 2300 Hz | 512/1024 | 93.75/46.88 Hz | 49 |
| Ultra | 2750 Hz | 512/1024 | 93.75/46.88 Hz | 59 |

### Modulation Types

| Type | Bits/Symbol | Usage |
|------|-------------|-------|
| FSK (MFSK) | 1-5 | Modes 1-4 (preamble, robust) |
| BPSK | 1 | Mode 4 |
| QPSK | 2 | Modes 5-12 |
| 8-PSK | 3 | Modes 13-14 |
| 16-QAM | 4 | Mode 15 |
| 32-QAM | 5 | Modes 16-17 |

## Preamble Structure

### MFSK Preamble

Each frame begins with an MFSK tone-hopping preamble for synchronization:

- **Symbol Duration**: FFT size / sample rate
  - FFT=512: 10.67 ms
  - FFT=1024: 21.33 ms
  - FFT=2048: 42.67 ms
- **Number of Symbols**: 16 (Wide/Ultra), 14 (Narrow)
- **Tone Selection**: Pseudo-random sequence from bandwidth-specific tone bins

### Preamble Tone Bins

| Bandwidth | FFT Size | Tone Bins | Frequency Range |
|-----------|----------|-----------|-----------------|
| 500 Hz | 2048 | 50-76 | ~1172-1781 Hz |
| 2300 Hz | 1024 | 15-38 | ~703-1781 Hz |
| 2750 Hz | 512 | 8-23 | ~750-2156 Hz |

### Detection

- Correlation-based detection with configurable threshold (default 0.50)
- Fingerprint verification using symbol confidence scores
- Minimum 6 symbols required for valid detection

## Frame Structure

### Header Format (8 bytes)

```
Byte 0: Protocol Version (0x01)
Byte 1: Frame Type
Byte 2-3: Sequence Number (little-endian)
Byte 4-5: Session ID (little-endian)
Byte 6: Speed Level (1-17)
Byte 7: Flags / Measured SNR (i8)
```

### Complete Frame

```
┌─────────────────────────────────────────────────────────────────┐
│                         PREAMBLE                                │
│                    (MFSK tone sequence)                         │
├─────────┬──────┬──────────┬───────────┬────────────┬───────────┤
│ Version │ Type │ Sequence │ SessionID │ SpeedLevel │ Flags/SNR │
│  (1B)   │ (1B) │   (2B)   │   (2B)    │    (1B)    │   (1B)    │
├─────────┴──────┴──────────┴───────────┴────────────┴───────────┤
│                          PAYLOAD                                │
│                    (variable length)                            │
├─────────────────────────────────────────────────────────────────┤
│                         CRC-32                                  │
│                          (4B)                                   │
└─────────────────────────────────────────────────────────────────┘
```

### Frame Types

| Type | Value | Direction | Description |
|------|-------|-----------|-------------|
| CONNECT | 0x01 | TX→RX | Connection request |
| CONNECT_ACK | 0x02 | RX→TX | Connection accepted |
| DISCONNECT | 0x03 | Both | Disconnect request |
| DISCONNECT_ACK | 0x04 | Both | Disconnect acknowledged |
| DATA | 0x10 | Both | Data frame |
| DATA_ACK | 0x11 | Both | Data acknowledgment |
| NACK | 0x12 | Both | Negative acknowledgment |
| IDLE | 0x20 | Both | Keepalive |
| BREAK | 0x21 | Both | Abort transmission |
| REQUEST | 0x22 | Both | Retransmission request |
| QUIT | 0x23 | Both | Session termination |
| SPEED_CHANGE | 0x30 | Both | Rate change request |
| SPEED_CHANGE_ACK | 0x31 | Both | Rate change acknowledgment |
| PING | 0x40 | Both | RTT measurement |
| PONG | 0x41 | Both | Ping response |

### CONNECT Frame Payload

```
Bytes 0-9: Source Callsign (10 bytes, space-padded)
Bytes 10-19: Destination Callsign (10 bytes, space-padded)
```

## Forward Error Correction

### Turbo Coding

- **Encoder**: Parallel concatenated convolutional code
- **Constituent Code**: K=7 RSC (Recursive Systematic Convolutional)
- **Interleaver**: S-random interleaver
- **Decoder**: Max-Log-MAP iterative decoding (8 iterations)

### Code Rates

| Code Rate | Puncturing | Usage |
|-----------|------------|-------|
| 1/2 | None | Modes 2-10 (most robust) |
| 2/3 | 1 in 2 parity bits | Modes 11, 13 |
| 3/4 | 1 in 3 parity bits | Modes 12, 14, 15 |
| 5/6 | 2 in 3 parity bits | Modes 16, 17 |

### FEC Block Sizes

Block sizes vary by mode and bandwidth. Examples for 2300 Hz:

| Mode | FEC Block (bits) | Max Payload (bytes) |
|------|------------------|---------------------|
| 2 | 288 | 20 |
| 10 | 2016 | 236 |
| 12 | 3024 | 362 |
| 16 | 7560 | 929 |

### CRC

- **CRC-32**: IEEE 802.3 polynomial on complete frame
- **Verification**: Frame rejected if CRC fails

## Speed Levels

### Mode Parameters

| Level | Modulation | Code Rate | Min SNR | Enter SNR | Stay SNR |
|-------|------------|-----------|---------|-----------|----------|
| 1 | FSK | 1/8 | -6 dB | -3 dB | -6 dB |
| 2 | FSK | 1/6 | -3 dB | 0 dB | -3 dB |
| 3 | FSK | 1/4 | 0 dB | 3 dB | 0 dB |
| 4 | BPSK | 1/3 | 3 dB | 6 dB | 3 dB |
| 5 | QPSK | 1/2 | 6 dB | 9 dB | 6 dB |
| 6 | QPSK | 1/2 | 9 dB | 12 dB | 9 dB |
| 7 | QPSK | 1/2 | 12 dB | 15 dB | 12 dB |
| 8 | QPSK | 1/2 | 15 dB | 18 dB | 15 dB |
| 9 | QPSK | 1/2 | 17 dB | 20 dB | 17 dB |
| 10 | QPSK | 1/2 | 19 dB | 22 dB | 19 dB |
| 11 | QPSK | 2/3 | 21 dB | 24 dB | 21 dB |
| 12 | QPSK | 3/4 | 23 dB | 26 dB | 23 dB |
| 13 | 8-PSK | 2/3 | 25 dB | 28 dB | 25 dB |
| 14 | 8-PSK | 3/4 | 27 dB | 30 dB | 27 dB |
| 15 | 16-QAM | 3/4 | 29 dB | 32 dB | 29 dB |
| 16 | 32-QAM | 5/6 | 31 dB | 34 dB | 31 dB |
| 17 | 32-QAM | 5/6 | 33 dB | 36 dB | 33 dB |

**Note**: Mode 1 is disabled (requires soft-decision FSK decoder).

### Per-Bandwidth Limits

| Bandwidth | Minimum Mode | Maximum Mode |
|-----------|--------------|--------------|
| 500 Hz | 2 | 13 |
| 2300 Hz | 2 | 16 |
| 2750 Hz | 2 | 17 |

## Rate Negotiation

### Algorithm

1. **Initial State**: Connection starts at mode 2 (most robust OFDM)

2. **SNR Measurement**: Each station measures received SNR and includes it in frame headers (Flags byte as i8)

3. **Effective SNR**: min(local_snr, remote_snr) - uses worse direction

4. **Mode Proposal**: Each frame includes proposed mode in SpeedLevel field

5. **Agreement**: Agreed mode = min(my_proposed, peer_proposed) - conservative

### State Transitions

**Downshift (Fast)**:
- Immediate if SNR < stay_snr threshold
- 500ms debounce between changes

**Upshift (Slow)**:
- Requires 3 consecutive frames with SNR >= enter_snr
- 500ms debounce between changes

**Initial Upshift (Rapid)**:
- After 2 frames at mode 2, can jump directly to SNR-supported mode
- Only happens once per connection

**Timeout Fallback**:
- If no keepalive received for 3× keepalive interval, fall back to mode 2

### Speed Change Handshake

```
Initiator                    Responder
    |                            |
    |------ SPEED_CHANGE ------->|
    |                            |
    |<----- SPEED_CHANGE_ACK ----|
    |                            |
    |  (both switch to new mode) |
```

### Collision Resolution

If both sides send SPEED_CHANGE simultaneously:
- Connection initiator has priority
- Non-initiator cancels its request and ACKs the initiator's request

## ARQ Protocol

### Selective Repeat

- Individual frame retransmission (not go-back-N)
- Sequence numbers: 16-bit (0-65535)
- Maximum retries: 10

### Timeouts

| Parameter | Value |
|-----------|-------|
| Frame timeout | 4000 ms (adjusted per mode) |
| Keepalive interval | 15 seconds |
| Connection timeout | 45 seconds (3× keepalive) |
| Speed change ACK timeout | 10 seconds |

### Flow Control

- Transmit window limited by mode payload capacity
- Receiver sends ACK/NACK after each frame
- NACK triggers immediate retransmission

## Session Management

### Session States

```
┌──────────────┐
│ Disconnected │
└──────┬───────┘
       │ CONNECT sent
       ▼
┌──────────────┐
│  Connecting  │──────────────────┐
└──────┬───────┘                  │
       │ CONNECT_ACK received     │ Timeout
       ▼                          │
┌──────────────┐                  │
│  Connected   │                  │
└──────┬───────┘                  │
       │ DISCONNECT sent          │
       ▼                          │
┌──────────────┐                  │
│Disconnecting │──────────────────┤
└──────┬───────┘                  │
       │ DISCONNECT_ACK received  │
       ▼                          ▼
┌──────────────┐         ┌──────────────┐
│ Disconnected │         │ Disconnected │
└──────────────┘         └──────────────┘
```

### Session ID

- 16-bit random value generated by connection initiator
- Used to identify frames belonging to a session
- Prevents confusion from stale frames

### Keepalives

- IDLE frames sent every 15 seconds during idle periods
- Contains current SNR measurement and proposed mode
- Timeout after 45 seconds triggers connection drop

## Encryption (Optional)

### Algorithm

- **Cipher**: AES-256-GCM
- **Key Derivation**: PBKDF2-SHA256
- **Iterations**: 100,000
- **Salt**: Derived from session ID + callsigns

### Encrypted Frame

```
┌─────────────────────────────────────────────────────────────────┐
│                     UNENCRYPTED HEADER                          │
│        (Version, Type, Sequence, SessionID, Speed, Flags)       │
├─────────────────────────────────────────────────────────────────┤
│                          NONCE                                  │
│                          (12B)                                  │
├─────────────────────────────────────────────────────────────────┤
│                     ENCRYPTED PAYLOAD                           │
│                    (variable length)                            │
├─────────────────────────────────────────────────────────────────┤
│                       AUTH TAG                                  │
│                         (16B)                                   │
├─────────────────────────────────────────────────────────────────┤
│                         CRC-32                                  │
│                          (4B)                                   │
└─────────────────────────────────────────────────────────────────┘
```

### Key Exchange

- Passphrase must be pre-shared between stations
- No in-band key exchange protocol

## Signal Processing

### Automatic Gain Control (AGC)

- Attack rate: Fast (tracks signal increases quickly)
- Decay rate: Slow (maintains gain during fades)
- Target level: Normalized for consistent demodulation

### Automatic Frequency Control (AFC)

- Tracks frequency offset from nominal 1500 Hz
- Phase-locked loop for carrier recovery
- Compensates for radio tuning errors

### Soft Decision Decoding

- Log-likelihood ratios (LLR) computed per bit
- LLR clamped to [-100, 100]
- Noise variance estimated from pilots
- Previous frame's noise estimate used for current frame

### Channel Estimation

- Pilot symbols inserted in OFDM carriers
- Linear interpolation between pilots
- Pilot positions: Edge carriers for >8 carriers, center for ≤8

## Compatibility Notes

### Mode 1 Disabled

Mode 1 (soft FSK) is not currently implemented. The minimum operational mode is 2.

### Callsign Format

- 1-10 characters
- Alphanumeric only
- Case-insensitive (stored uppercase)
- Space-padded in frames

### Frame Overhead

Total overhead per frame:
- Header: 8 bytes
- CRC: 4 bytes
- Length fields: 4 bytes
- **Total: 16 bytes**

Effective throughput = raw_bps × (payload / (payload + 16))
