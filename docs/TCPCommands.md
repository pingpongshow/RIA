# TCP Commands Reference

Ultra provides a TCP command interface for controlling the modem from external programs. By default:
- **Command Port**: 8300
- **Data Port**: 8301
- **KISS Port**: 8302 (optional)

Commands are sent as ASCII text terminated with `\r` (carriage return) or `\n` (newline).

---

## Connection Commands

### CONNECT

Initiate connection to a remote station.

```
CONNECT <callsign>
```

**Parameters:**
- `callsign` - Remote station callsign (1-10 characters)

**Response:**
- `PENDING` - Connection attempt initiated
- `ERROR Already connected` - Already in a session
- `ERROR Invalid callsign` - Callsign too long or empty

**Example:**
```
CONNECT W1AW
PENDING
```

---

### DISCONNECT

Disconnect from current session.

```
DISCONNECT
```

**Response:**
- `OK` - Disconnect initiated

---

### ABORT

Abort a pending connection attempt.

```
ABORT
```

**Response:**
- `OK` - Connection attempt aborted

---

## Configuration Commands

### MYCALL

Set local station callsign.

```
MYCALL <callsign>
```

**Parameters:**
- `callsign` - Your callsign (1-10 characters)

**Response:**
- `OK` - Callsign set
- `ERROR Invalid callsign` - Invalid format

**Example:**
```
MYCALL K1ABC
OK
```

---

### MYAUX

Set auxiliary callsigns (for multi-call operation).

```
MYAUX <callsign1>,<callsign2>,...
```

**Parameters:**
- Comma-separated list of auxiliary callsigns

**Example:**
```
MYAUX K1ABC-1,K1ABC-2
OK
```

---

### WAVEFORM

Set the waveform mode for transmission.

```
WAVEFORM <mode>
```

**Parameters:**
- `mode` - Waveform type:
  - `AUTO` - Automatic selection based on conditions (default)
  - `MC_DPSK` or `DPSK` - Multi-Carrier DPSK (robust, low SNR)
  - `OFDM_CHIRP` or `OFDM` - OFDM with chirp sync (balanced)
  - `OFDM_COX` or `COX` - OFDM with Schmidl-Cox sync (high throughput)
  - `OTFS` - OTFS experimental mode

**Waveform Selection Guide:**
| Condition | Recommended Waveform |
|-----------|---------------------|
| SNR ≥ 20 dB, good channel | OFDM_COX (max throughput) |
| SNR ≥ 15 dB, good fading | OFDM_CHIRP (proven) |
| SNR 5-10 dB OR moderate/poor fading | MC_DPSK DQPSK R1/4 (~938 bps) |
| SNR < 5 dB | MC_DPSK DBPSK R1/4 (~469 bps, floor -4 dB) |

**Example:**
```
WAVEFORM OFDM_CHIRP
OK
```

---

### MODULATION / MOD

Set the modulation scheme.

```
MODULATION <scheme>
MOD <scheme>
```

**Parameters:**
- `scheme` - Modulation type:
  - `AUTO` - Automatic selection based on SNR (default)
  - `DQPSK` - Differential QPSK (2 bits/symbol, robust)
  - `QPSK` - Coherent QPSK (2 bits/symbol)
  - `D8PSK` or `8PSK` - Differential 8PSK (3 bits/symbol)
  - `QAM16` or `16QAM` - 16-QAM (4 bits/symbol)
  - `QAM32` or `32QAM` - 32-QAM (5 bits/symbol)
  - `QAM64` or `64QAM` - 64-QAM (6 bits/symbol)

**Example:**
```
MODULATION DQPSK
OK
```

**Auto Selection (AUTOMODE ON):**
When AUTO mode is enabled, modulation is selected based on SNR and fading:

| Channel | SNR | Auto-Selected Modulation |
|---------|-----|--------------------------|
| AWGN | >= 25 dB | QAM64 R3/4 (~7200 bps) |
| AWGN | >= 22 dB | QAM32 R3/4 (~6000 bps) |
| AWGN | >= 18 dB | QAM16 R3/4 (~4800 bps) |
| Good fading | >= 22 dB | QAM16 R2/3 (~4000 bps) |
| Good fading | >= 20 dB | DQPSK R2/3 (~3200 bps) |
| All | >= 15 dB | DQPSK R1/2 (~2300 bps) |
| All | >= 10 dB | DQPSK R1/4 (~1150 bps) |

**Note:** QAM64/32 are only recommended for AWGN channels (e.g., VHF/UHF line-of-sight).
On HF fading channels, QAM16 is the maximum recommended, and only at high SNR.

---

### CODERATE / RATE / FEC

Set the FEC code rate.

```
CODERATE <rate>
RATE <rate>
FEC <rate>
```

**Parameters:**
- `rate` - Code rate:
  - `AUTO` - Automatic selection based on conditions (default)
  - `R1/4` or `R1_4` or `1/4` - Rate 1/4 (most robust)
  - `R1/3` or `R1_3` or `1/3` - Rate 1/3
  - `R1/2` or `R1_2` or `1/2` - Rate 1/2 (balanced)
  - `R2/3` or `R2_3` or `2/3` - Rate 2/3
  - `R3/4` or `R3_4` or `3/4` - Rate 3/4
  - `R5/6` or `R5_6` or `5/6` - Rate 5/6
  - `R7/8` or `R7_8` or `7/8` - Rate 7/8 (highest throughput)

**Example:**
```
CODERATE R1/2
OK
```

---

### AUTOMODE / AUTO

Enable/disable automatic mode selection. When enabled, resets all forced settings.

```
AUTOMODE <ON|OFF|1|0>
```

**Example:**
```
AUTOMODE ON
OK
```

---

### COMPRESSION

Enable/disable data compression.

```
COMPRESSION <ON|OFF|1|0>
```

**Example:**
```
COMPRESSION ON
OK
```

---

### LISTEN

Enable/disable listen mode (monitor without transmitting).

```
LISTEN <ON|OFF|1|0>
```

**Example:**
```
LISTEN ON
OK
```

---

### CHATMODE

Enable/disable chat mode.

```
CHATMODE <ON|OFF|1|0>
```

---

### WINLINK / WINLINKSESSION

Enable/disable Winlink gateway mode.

```
WINLINK <ON|OFF|1|0>
```

---

## Status Commands

### VERSION

Get modem version.

```
VERSION
```

**Response:**
```
VERSION 1.0.0
```

---

### CODEC

Get current codec/mode information.

```
CODEC
```

**Response:**
```
Ultra OFDM 2300 Hz, LDPC FEC
```

---

### STATE

Get current connection state.

```
STATE
```

**Response:**
```
STATE IDLE
```

Possible states: `IDLE`, `CONNECTING`, `CONNECTED`, `DISCONNECTING`

---

### PTT / PTTSTATE

Get PTT (Push-To-Talk) state.

```
PTT
PTTSTATE
```

**Response:**
- `PTT ON` - Transmitting
- `PTT OFF` - Receiving

---

### BUSY / BUSYSTATE

Get busy channel state.

```
BUSY
```

**Response:**
- `BUSY ON` - Channel busy detected
- `BUSY OFF` - Channel clear

---

### BUFFER

Get transmit buffer status.

```
BUFFER
```

**Response:**
```
BUFFER 1234
```
(Number of bytes waiting to transmit)

---

## Control Commands

### TUNE

Enable/disable tune output (continuous carrier for tuning).

```
TUNE <ON|OFF|1|0>
```

---

### CWID

Get or set CW ID callsign.

```
CWID [callsign]
```

Without argument: returns current CW ID callsign
With argument: sets CW ID callsign

---

### PTTLEAD / TXDELAY

Set PTT lead time (delay before TX after PTT asserted).

```
PTTLEAD <ms>
TXDELAY <ms>
```

**Parameters:**
- `ms` - Delay in milliseconds (default: 50)

**Example:**
```
PTTLEAD 100
OK
```

---

### PTTTAIL

Set PTT tail time (delay after TX before releasing PTT).

```
PTTTAIL <ms>
```

**Parameters:**
- `ms` - Delay in milliseconds (default: 50)

**Example:**
```
PTTTAIL 50
OK
```

---

### TXDRIVE

Set TX audio drive level.

```
TXDRIVE <level>
```

**Parameters:**
- `level` - Audio level 0.0 to 1.0 (default: 0.8)

**Example:**
```
TXDRIVE 0.7
OK
```

---

## Broadcast Commands

All disconnected TX commands (`BEACON`, `CQ`, `PING`, `RAWTX`) are staged:
1. Send command on command port (modem replies `PENDING`)
2. Send optional payload on data port

The staged TX is emitted when one of these conditions is met:
- payload reaches the command's byte limit
- at least 40 ms has elapsed since the last payload bytes arrived
- 300 ms has elapsed since command staging (empty payload is sent if none arrived)

### BEACON

Transmit a beacon frame without requiring a connection. Beacons are one-way informational broadcasts (callsign, grid, status message).

```
BEACON
```

**Flow:**
1. Send `BEACON` command on command port
2. Send beacon payload data on data port (optional; empty payload sent after timeout)
3. Modem transmits a disconnected broadcast `DATA` frame using robust MC-DPSK control path
4. Modem returns to IDLE state

**Response:**
- `PENDING` - Beacon TX staged (waiting for data-port payload/timer)
- `ERROR Cannot beacon while connected or connecting`

**Blocking Conditions:**
- Blocked when modem is CONNECTED, CONNECTING, or PROBING
- Only allowed when IDLE or DISCONNECTED

**Payload Limit:**
- Up to 2048 bytes from data port (excess bytes are truncated).

**Example:**
```
BEACON
PENDING
```

Then send beacon data (callsign, grid, message) on data port.

---

### CQ

Transmit a CQ frame and enter listening mode for responses. CQ is a call requesting any station to respond.

```
CQ
```

**Flow:**
1. Send `CQ` command on command port
2. Send CQ payload data on data port (optional; empty payload sent after timeout)
3. Modem transmits a disconnected broadcast `DATA` frame using robust MC-DPSK control path
4. Modem enters LISTENING state with 10-second timeout
5. If a PING is received addressed to us → begin connection handshake
6. If timeout expires → return to IDLE

**Response:**
- `PENDING` - CQ TX staged (waiting for data-port payload/timer)
- `ERROR Cannot CQ while connected or connecting`

**Blocking Conditions:**
- Blocked when modem is CONNECTED, CONNECTING, or PROBING
- Only allowed when IDLE or DISCONNECTED

**Payload Limit:**
- Up to 2048 bytes from data port (excess bytes are truncated).

**Example:**
```
CQ
PENDING
```

Then send CQ data (callsign, grid, message) on data port.

---

### PING

Send a directed ping to a specific station (status check without full connection).

```
PING <callsign>
```

**Parameters:**
- `callsign` - Target station callsign to ping

**Flow:**
1. Send `PING <callsign>` command on command port
2. Send ping payload data on data port (e.g., 0xE0 marker + grid square)
3. Modem transmits a disconnected directed `DATA` frame (dst hash = target callsign hash)
4. Target station receives and can respond

**Response:**
- `PENDING` - Ping TX initiated
- `ERROR No callsign specified` - Missing target callsign
- `ERROR Cannot ping while connected or connecting` - Blocked by connection state

**Blocking Conditions:**
- Blocked when modem is CONNECTED, CONNECTING, or PROBING
- Only allowed when IDLE or DISCONNECTED

**Payload Limit:**
- Up to 2048 bytes from data port (excess bytes are truncated).

**Example:**
```
PING W1AW
PENDING
```

Then send ping data on data port.

**Difference from BEACON/CQ:**
- BEACON: Broadcast to all stations (dst_hash = 0xFFFFFF)
- CQ: Broadcast + enter listening state for responses
- PING: Directed to specific station (dst_hash = hash of target callsign)

---

### RAWTX

Transmit arbitrary payload bytes using a caller-selected PHY profile while disconnected.
This bypasses connection/ARQ framing and sends the provided payload as one raw modem frame.

```
RAWTX [waveform] [modulation] [coderate]
```

**Parameters (optional, positional):**
- `waveform` - `MC_DPSK`, `OFDM_CHIRP`, `OFDM_COX`, `OTFS`, `MFSK` (or `AUTO`)
- `modulation` - `DBPSK`, `DQPSK`, `QPSK`, `D8PSK`, `QAM16`, `QAM32`, `QAM64` (or `AUTO`)
- `coderate` - `R1/4`, `R1/2`, `R2/3`, `R3/4`, `R5/6`, `R7/8` (or `AUTO`)

**Flow:**
1. Send `RAWTX ...` on command port
2. Send payload bytes on data port
3. Modem transmits one preamble+payload burst with the requested PHY

**Defaults when omitted/AUTO:**
- waveform: `MC_DPSK`
- modulation: `DBPSK` for `MC_DPSK`, otherwise `DQPSK`
- coderate: `R1/4` for `MC_DPSK`, otherwise `R1/2`

**Response:**
- `PENDING` - RAWTX staged; waiting for data-port payload (or timeout)
- `ERROR Cannot RAWTX while connected or connecting`
- `ERROR Invalid RAWTX waveform/modulation/coderate`

**Payload Limit:**
- Up to 4096 bytes from data port (excess bytes are truncated).

**Blocking Conditions:**
- Allowed only in `IDLE` / `DISCONNECTED`

**Example:**
```
RAWTX OFDM_CHIRP DQPSK R1/2
PENDING
```

Then send raw bytes on the data port.

---

### Receiving Beacons

When the modem receives a beacon or CQ frame from another station, the data is delivered to the data port with a beacon marker prefix:

**Data Port Output Format:**
```
[0xFA][callsign_length (1 byte)][callsign (N bytes)][payload]
```

- `0xFA` - Beacon marker byte
- `callsign_length` - Length of source callsign (1 byte)
- `callsign` - Source station callsign (ASCII)
- `payload` - Beacon payload data

**Example Reception:**
If station W1AW sends a beacon with "CQ DE W1AW FM19", the data port receives:
```
0xFA 0x04 W 1 A W C Q   D E   W 1 A W   F M 1 9
     ^--- callsign length (4 bytes)
          ^------- callsign
                   ^------------------------- payload
```

---

### Beacon/CQ/Ping Waveform

Beacon, CQ, and TCP `PING` transmissions use the same robust disconnected control TX path:

| Parameter | Value |
|-----------|-------|
| Waveform | MC-DPSK |
| Modulation | DBPSK (10 carriers) |
| Spreading | 4x (extreme low-SNR) |
| Preamble | CHIRP (1200ms, ±50 Hz CFO) |
| Frame Type | `DATA` (v2 data frame) |
| Destination | `0xFFFFFF` (BEACON/CQ) or target hash (PING) |

This ensures these frames can be decoded at very low SNR (-14 dB floor).

---

### PTT Behavior (BEACON/CQ/PING/RAWTX)

For GUI modem TX paths, these commands run through the standard TX audio queue and:
- **do** assert/release **CAT PTT** automatically when CAT is enabled and currently connected.
- otherwise, **do** assert/release **Serial PTT** when serial PTT is enabled in settings (`Enable Serial PTT (DTR/RTS)`).
- `CATPTT` remains available for manual testing/override use cases.

## Encryption Commands

### ENCRYPT / ENCRYPTION

Enable or disable AES-256 encryption.

```
ENCRYPT <ON|OFF|1|0>
ENCRYPTION <ON|OFF|1|0>
```

**Note:** Both stations must use the same passphrase for successful communication.

**Example:**
```
ENCRYPT ON
OK (warning: no key set - use ENCRYPTKEY)
```

---

### ENCRYPTKEY / KEY

Set the encryption passphrase. The passphrase is hashed with SHA-256 to derive the AES-256 key.

```
ENCRYPTKEY <passphrase>
KEY <passphrase>
```

**Parameters:**
- `passphrase` - Any string (8+ characters recommended)

**Example:**
```
ENCRYPTKEY MySecretPassphrase123
OK
ENCRYPT ON
OK
```

**Security Notes:**
- The passphrase is not saved to disk for security
- Use a strong passphrase (16+ characters recommended)
- Share the passphrase securely with the other station
- Control frames (ACK/NACK) remain unencrypted for protocol operation

---

## File Transfer Commands

### SENDFILE / SEND

Send a file to the connected station.

```
SENDFILE <filepath>
SEND <filepath>
```

**Parameters:**
- `filepath` - Full path to the file to send

**Response:**
- `OK` - File transfer started
- `ERROR No filepath specified` - Missing filepath
- `ERROR Not connected` - No active connection
- `ERROR Failed to start file transfer` - File not found or busy

**Example:**
```
SENDFILE /home/user/document.txt
OK
```

**Notes:**
- File is automatically encrypted if encryption is enabled
- Received files are saved to the configured receive directory (default: Downloads)
- Large files are chunked and sent reliably via ARQ

---

### CLOSE

Close the TCP command connection.

```
CLOSE
```

**Response:**
```
OK
```
(Connection then closes)

---

## CAT Control Commands

CAT (Computer Aided Transceiver) commands provide direct radio control through the modem. This eliminates the need for external PTT coordination and enables frequency/mode control.

### CATENABLE

Enable or disable CAT control.

```
CATENABLE <ON|OFF|1|0>
```

**Response:**
- `OK` - CAT state changed
- `OK (already enabled/disabled)` - No change needed

**Example:**
```
CATENABLE ON
OK
```

---

### CATBACKEND

Select the CAT backend type.

```
CATBACKEND <type>
```

**Parameters:**
- `type` - Backend selection:
  - `NONE` or `OFF` - Disable CAT (PTT via serial DTR/RTS only)
  - `SERIAL` or `SERIALPTT` - Serial PTT only (DTR/RTS)
  - `HAMLIB` - Hamlib library (200+ radio models)
  - `KENWOOD` or `KENWOODTCP` or `FLEX` - Kenwood ASCII over TCP (FlexRadio)

**Response:**
- `OK` - Backend type set
- `ERROR Invalid backend` - Unknown backend type

**Example:**
```
CATBACKEND HAMLIB
OK
```

---

### CATMODEL

Set the Hamlib radio model ID.

```
CATMODEL <model_id>
```

**Parameters:**
- `model_id` - Hamlib model number (see Hamlib documentation)

**Common Model IDs:**
| ID | Radio |
|----|-------|
| 1 | Dummy/Test rig |
| 2014 | Kenwood TS-2000 |
| 3073 | Icom IC-7300 |
| 3081 | Icom IC-7610 |
| 3085 | Icom IC-705 |
| 229 | Yaesu FT-991A |
| 241 | Yaesu FT-DX10 |

**Response:**
- `OK` - Model set
- `ERROR Invalid model` - Model ID not a valid number

**Example:**
```
CATMODEL 3073
OK
```

---

### CATPORT

Set the CAT serial port or TCP host:port.

```
CATPORT <path|host:port>
```

**Parameters:**
- For serial: `/dev/ttyUSB0`, `COM3`, etc.
- For TCP: `hostname:port` (e.g., `192.168.1.100:4532`)

**Response:**
- `OK` - Port set
- `ERROR No port specified` - Missing parameter

**Example:**
```
CATPORT /dev/ttyUSB0
OK
CATPORT 192.168.1.200:4992
OK
```

---

### CATBAUD

Set the serial baud rate for CAT communication.

```
CATBAUD <rate>
```

**Parameters:**
- `rate` - Baud rate (4800, 9600, 19200, 38400, 57600, 115200)

**Response:**
- `OK` - Baud rate set
- `ERROR Invalid baud rate` - Invalid number

**Example:**
```
CATBAUD 38400
OK
```

---

### CATSLICE

Set the FlexRadio slice number (for Kenwood TCP backend).

```
CATSLICE <n>
```

**Parameters:**
- `n` - Slice number (0-7)

**Response:**
- `OK` - Slice set
- `ERROR Invalid slice` - Invalid number

**Example:**
```
CATSLICE 0
OK
```

---

### CATCONNECT

Connect to the radio using the configured backend.

```
CATCONNECT
```

**Response:**
- `OK` - Connected successfully
- `ERROR CAT not enabled` - CAT is disabled
- `ERROR No backend configured` - Backend type is NONE
- `ERROR Connection failed` - Failed to connect to radio

**Example:**
```
CATENABLE ON
OK
CATBACKEND HAMLIB
OK
CATMODEL 3073
OK
CATPORT /dev/ttyUSB0
OK
CATCONNECT
OK
```

---

### CATDISCONNECT

Disconnect from the radio.

```
CATDISCONNECT
```

**Response:**
- `OK` - Disconnected

**Example:**
```
CATDISCONNECT
OK
```

---

### CATPTT

Manually control PTT (Push-To-Talk).

```
CATPTT <ON|OFF|1|0>
```

**Response:**
- `OK` - PTT state changed
- `ERROR Not connected` - CAT not connected to radio
- `ERROR PTT failed` - Radio did not respond

**Example:**
```
CATPTT ON
OK
CATPTT OFF
OK
```

**Note:** During normal modem operation, PTT is controlled automatically. This command is for manual testing or special applications.

---

### CATFREQ

Set the radio frequency.

```
CATFREQ <hz>
```

**Parameters:**
- `hz` - Frequency in Hz

**Response:**
- `OK` - Frequency set
- `ERROR Not connected` - CAT not connected
- `ERROR Frequency not supported` - Backend doesn't support frequency control
- `ERROR Set frequency failed` - Radio did not respond

**Example:**
```
CATFREQ 14074000
OK
```

---

### CATGETFREQ

Get the current radio frequency.

```
CATGETFREQ
```

**Response:**
- `CATFREQ <hz>` - Current frequency in Hz
- `ERROR Not connected` - CAT not connected
- `ERROR Frequency not supported` - Backend doesn't support frequency control

**Example:**
```
CATGETFREQ
CATFREQ 14074000
```

---

### CATMODE

Set the radio operating mode.

```
CATMODE <mode>
```

**Parameters:**
- `mode` - Operating mode:
  - `USB` - Upper sideband
  - `LSB` - Lower sideband
  - `AM` - Amplitude modulation
  - `FM` - Frequency modulation
  - `CW` - Continuous wave
  - `CW_R` or `CWR` - CW reverse
  - `RTTY` - RTTY
  - `RTTY_R` or `RTTYR` - RTTY reverse
  - `DATA` - Data mode (USB-D)
  - `DATA_R` or `DATAR` - Data mode reverse (LSB-D)

**Response:**
- `OK` - Mode set
- `ERROR Not connected` - CAT not connected
- `ERROR Mode not supported` - Backend doesn't support mode control
- `ERROR Invalid mode` - Unknown mode
- `ERROR Set mode failed` - Radio did not respond

**Example:**
```
CATMODE USB
OK
```

---

### CATGETMODE

Get the current radio operating mode.

```
CATGETMODE
```

**Response:**
- `CATMODE <mode>` - Current mode
- `ERROR Not connected` - CAT not connected
- `ERROR Mode not supported` - Backend doesn't support mode control

**Example:**
```
CATGETMODE
CATMODE USB
```

---

### CATWATCHDOG

Set the TX watchdog timeout. Automatically releases PTT if TX exceeds this duration.

```
CATWATCHDOG <seconds>
```

**Parameters:**
- `seconds` - Timeout in seconds (0 = disabled, default: 120)

**Response:**
- `OK` - Watchdog timeout set
- `ERROR Invalid timeout` - Invalid number

**Example:**
```
CATWATCHDOG 180
OK
```

**Note:** The watchdog is a safety feature to prevent stuck transmitters. If TX exceeds the timeout, PTT is automatically released and the transmission is aborted.

---

### CATPTTLEAD

Set PTT lead time (delay before audio starts after PTT is asserted).

```
CATPTTLEAD <ms>
```

**Parameters:**
- `ms` - Lead time in milliseconds (default: 50)

**Response:**
- `OK` - Lead time set
- `ERROR Invalid lead time` - Invalid number

**Example:**
```
CATPTTLEAD 75
OK
```

---

### CATPTTTAIL

Set PTT tail time (delay after audio ends before PTT is released).

```
CATPTTTAIL <ms>
```

**Parameters:**
- `ms` - Tail time in milliseconds (default: 50)

**Response:**
- `OK` - Tail time set
- `ERROR Invalid tail time` - Invalid number

**Example:**
```
CATPTTTAIL 30
OK
```

---

### CATSTATUS

Get current CAT status.

```
CATSTATUS
```

**Response:**
```
CAT <ENABLED|DISABLED> <backend> <CONNECTED|DISCONNECTED> PTT <ON|OFF>
```

**Example:**
```
CATSTATUS
CAT ENABLED HAMLIB CONNECTED PTT OFF
```

---

## Asynchronous Responses

The modem may send unsolicited responses on the command port:

| Response | Description |
|----------|-------------|
| `CONNECTED <callsign>` | Connection established |
| `DISCONNECTED` | Session ended |
| `LINK BROKEN` | Connection lost (timeout/failure) |
| `BUSY ON` / `BUSY OFF` | Channel busy state changed |
| `SNR <value>` | Signal-to-noise ratio report |
| `DATA <bytes>` | Data received notification |
| `PTT ON` / `PTT OFF` | Transmit state changed |
| `STATE <state>` | Connection state changed |
| `BEACON <callsign>` | Beacon received from station |
| `CQ <callsign>` | CQ received from station |

---

## Data Port

The data port (default 8301) is used for sending and receiving user data:

- **Sending**: Write data to the port; modem will transmit when connected
- **Receiving**: Read from the port to receive incoming data

Data is raw binary (not text-encoded).

---

## KISS TNC Port

The optional KISS port (default 8302) provides standard KISS TNC protocol:

- Standard KISS framing (FEND, FESC escaping)
- Data frames transmitted via modem
- Configuration commands: TxDelay, Persistence, SlotTime, TxTail, FullDuplex

---

## Example Session

```
$ nc localhost 8300
MYCALL K1ABC
OK
WAVEFORM OFDM_CHIRP
OK
MODULATION DQPSK
OK
CODERATE R1/2
OK
CONNECT W1AW
PENDING
CONNECTED W1AW
VERSION
VERSION 1.0.0
DISCONNECT
OK
DISCONNECTED
CLOSE
OK
```

---

## Error Responses

All error responses follow the format:
```
ERROR <message>
```

Common errors:
- `ERROR Already connected` - Attempting connect while in session
- `ERROR Invalid callsign` - Callsign format error
- `ERROR Not initialized` - Component not ready
- `ERROR Invalid waveform` - Unknown waveform type
- `ERROR Invalid modulation` - Unknown modulation scheme
- `ERROR Invalid code rate` - Unknown code rate

---

## GUI Configuration

TCP settings can be configured in Settings -> Network tab:

- **Enable TCP Interface**: Enable/disable TCP server
- **Bind Address**: Network interface to bind (0.0.0.0 = all)
- **Command Port**: TCP port for commands (default 8300)
- **Data Port**: TCP port for data (default 8301)
- **Enable KISS TNC**: Enable KISS protocol support
- **KISS Port**: TCP port for KISS (default 8302)

---

## Notes

1. Commands are case-insensitive (`connect` = `CONNECT`)
2. Responses are terminated with `\r`
3. Multiple commands can be pipelined
4. The data port only handles binary data, not commands
5. Only one data client connection allowed at a time
6. Up to 10 command client connections allowed
