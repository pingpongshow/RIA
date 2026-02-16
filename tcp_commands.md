# RIA Modem - TCP Interface Reference

## Overview

RIA provides a dual-port TCP interface for integration with host applications:

- **Command Port** (default 8300): Text-based control commands
- **Data Port** (default 8301): Binary data transmission/reception

Both ports bind to `127.0.0.1` (localhost only).

## Connecting to the Command Port

### Using netcat (nc)

```bash
nc localhost 8300
```

### Using telnet

```bash
telnet localhost 8300
```

### Programmatic Connection (Python example)

```python
import socket

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect(('127.0.0.1', 8300))

# Send command
sock.send(b'VERSION\r\n')

# Receive response
response = sock.recv(1024)
print(response.decode())
```

## Command Format

- Commands are case-insensitive
- Commands are terminated with CR (`\r`) or CRLF (`\r\n`)
- Responses are terminated with CRLF
- One command per line

## Command Reference

### Connection Management

#### CONNECT

Initiate a connection to a remote station.

```
CONNECT <callsign>
```

**Arguments:**
- `callsign`: Remote station callsign (1-10 characters)

**Responses:**
- `PENDING` - Connection request sent
- `CONNECTED <callsign>` - Connection established
- `ERROR <message>` - Connection failed

**Example:**
```
> CONNECT W5ABC
< PENDING
< CONNECTED W5ABC
```

#### DISCONNECT

End the current session.

```
DISCONNECT
```

**Responses:**
- `OK` - Disconnect initiated
- `DISCONNECTED` - Session ended
- `ERROR Not connected` - No active session

**Example:**
```
> DISCONNECT
< OK
< DISCONNECTED
```

#### ABORT

Abort a pending connection attempt.

```
ABORT
```

**Responses:**
- `OK` - Connection attempt aborted
- `ERROR <message>` - Nothing to abort

#### LISTEN

Enable or disable listen mode (auto-accept incoming connections).

```
LISTEN ON
LISTEN OFF
```

**Responses:**
- `OK` - Mode changed

**Example:**
```
> LISTEN ON
< OK
```

### Station Configuration

#### MYCALL

Set the local station callsign.

```
MYCALL <callsign>
```

**Arguments:**
- `callsign`: Your callsign (1-10 characters, alphanumeric)

**Responses:**
- `OK` - Callsign set
- `ERROR Invalid callsign` - Invalid format

**Example:**
```
> MYCALL N0TEST
< OK
```

#### MYAUX

Set auxiliary callsigns (aliases).

```
MYAUX <call1,call2,...>
```

**Arguments:**
- Comma-separated list of auxiliary callsigns

**Responses:**
- `OK` - Callsigns set

**Example:**
```
> MYAUX N0TEST-1,N0TEST-2
< OK
```

### Bandwidth and Mode

#### BWMODE

Set the bandwidth mode.

```
BWMODE <bandwidth>
```

**Arguments:**
- `500` - 500 Hz narrow bandwidth
- `2300` - 2300 Hz standard bandwidth
- `2750` - 2750 Hz wide bandwidth

**Responses:**
- `OK` - Bandwidth changed
- `ERROR Invalid bandwidth` - Unknown mode

**Example:**
```
> BWMODE 2300
< OK
```

#### BANDWIDTH

Alternative bandwidth command (same as BWMODE).

```
BANDWIDTH <mode>
```

### Features

#### COMPRESSION

Enable or disable Huffman compression.

```
COMPRESSION ON
COMPRESSION OFF
```

**Responses:**
- `OK` - Setting changed

#### CHATMODE

Enable or disable chat mode (optimized for short messages).

```
CHATMODE ON
CHATMODE OFF
```

**Responses:**
- `OK` - Mode changed

#### WINLINK

Enable or disable Winlink gateway mode.

```
WINLINK ON
WINLINK OFF
```

**Responses:**
- `OK` - Mode changed

### Transmitter Control

#### TUNE

Enable or disable continuous tune signal.

```
TUNE ON
TUNE OFF
```

**Responses:**
- `OK` - Tune mode changed

**Note:** Tune mode transmits a continuous carrier for antenna tuning.

#### AUTOTUNE

Enable or disable automatic tuning.

```
AUTOTUNE ON
AUTOTUNE OFF
```

**Responses:**
- `OK` - Setting changed

#### CWID

Get or set CW identification callsign.

```
CWID
CWID <callsign>
```

**Responses:**
- `<callsign>` - Current CW ID (when getting)
- `OK` - CW ID set (when setting)

### Status Queries

#### VERSION

Get modem version information.

```
VERSION
```

**Response:**
```
RIA 0.1.0
```

#### CODEC

Get current codec/modulation information.

```
CODEC
```

**Response:**
```
QPSK 3/4 49 carriers
```

#### PTTSTATE

Get current PTT (Push-To-Talk) state.

```
PTTSTATE
```

**Responses:**
- `PTT ON` - Transmitting
- `PTT OFF` - Not transmitting

#### BUSY

Check if the channel is busy.

```
BUSY
```

**Responses:**
- `BUSY ON` - Channel busy (signal detected)
- `BUSY OFF` - Channel clear

#### BUFFER

Get transmit buffer size in bytes.

```
BUFFER
```

**Response:**
```
BUFFER 1234
```

### Session Management

#### CLOSE

Close the TCP command connection.

```
CLOSE
```

**Note:** This closes the TCP socket, not the radio session.

## Asynchronous Notifications

The command port may send unsolicited notifications:

| Notification | Description |
|--------------|-------------|
| `CONNECTED <call>` | Connection established |
| `DISCONNECTED` | Session ended |
| `LINK BROKEN` | Connection lost (timeout/error) |
| `BUSY ON` | Channel became busy |
| `BUSY OFF` | Channel became clear |
| `PTT ON` | Transmitter keyed |
| `PTT OFF` | Transmitter unkeyed |
| `SNR <dB>` | Signal-to-noise ratio update |

## Data Port Usage

### Overview

The data port (default 8301) handles binary data transmission:

- Single client connection allowed
- Raw binary data (no framing required by client)
- Bidirectional communication
- Data is automatically framed, encoded, and transmitted

### Sending Data

```bash
# Send file contents
cat file.bin | nc localhost 8301

# Send string
echo "Hello, World!" | nc localhost 8301

# Send from application
printf "Binary data here" | nc localhost 8301
```

### Receiving Data

```bash
# Receive to terminal
nc localhost 8301

# Receive to file
nc localhost 8301 > received.bin

# Continuous receive with timeout
nc -w 30 localhost 8301 > received.bin
```

### Programmatic Data Transfer (Python)

```python
import socket

# Connect to data port
data_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
data_sock.connect(('127.0.0.1', 8301))

# Send data
data_sock.send(b'Hello, this is a test message!')

# Receive data
received = data_sock.recv(4096)
print(f"Received: {received}")

data_sock.close()
```

## Complete Session Example

```bash
# Terminal 1: Command session
$ nc localhost 8300
MYCALL N0TEST
OK
LISTEN ON
OK
# ... wait for incoming connection ...
CONNECTED W5ABC
# Connection established!

# Terminal 2: Send data
$ echo "Hello from N0TEST!" | nc localhost 8301

# Terminal 3: Receive data on remote station
$ nc localhost 8311
Hello from N0TEST!
```

## Error Responses

| Error | Cause |
|-------|-------|
| `ERROR Unknown command: <cmd>` | Unrecognized command |
| `ERROR Invalid callsign` | Callsign format invalid |
| `ERROR Invalid bandwidth` | Unknown bandwidth value |
| `ERROR Not connected` | Command requires active connection |
| `ERROR Already connected` | Already in a session |
| `ERROR Connection failed` | Unable to establish connection |
| `ERROR Timeout` | Operation timed out |

## Port Configuration

Default ports can be changed in the configuration file:

```toml
tcp_command_port = 8300
tcp_data_port = 8301
```

Or use `RIA_INSTANCE` to offset ports:

```bash
# Instance 0: 8300/8301
# Instance 1: 8310/8311
# Instance 2: 8320/8321
RIA_INSTANCE=1 ./target/release/ria
```
