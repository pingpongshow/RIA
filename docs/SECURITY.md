# Security Features

Ultra provides optional encryption and compression for secure and efficient data transfer.

---

## Encryption

### Overview

Ultra uses **AES-256-CBC** encryption with PKCS#7 padding for securing data transmissions. When enabled, all user data (text messages and file transfers) is encrypted before transmission and decrypted on receipt.

### Key Derivation

The encryption key is derived from a user-provided passphrase:

1. Passphrase is hashed using **SHA-256**
2. The 256-bit hash becomes the AES key
3. A random 16-byte **IV (Initialization Vector)** is generated for each encryption operation
4. The IV is prepended to the ciphertext for transmission

### What Gets Encrypted

| Data Type | Encrypted |
|-----------|-----------|
| Text messages | Yes |
| File transfers (FILE_START, FILE_DATA) | Yes |
| Control frames (ACK, NACK, CONNECT, DISCONNECT) | No |
| Protocol handshake | No |

Control frames remain unencrypted to ensure protocol operation even if keys don't match.

### Enabling Encryption

#### Via TCP Commands

```
ENCRYPTKEY MySecretPassphrase
OK
ENCRYPT ON
OK
```

Both stations must use the **same passphrase** for successful communication.

#### Via GUI

1. Open **Settings** window
2. Go to **Security** tab
3. Check **Enable Encryption**
4. Enter passphrase in **Encryption Key** field
5. Click **Apply**

### Security Considerations

- **Passphrase strength**: Use 16+ characters with mixed case, numbers, and symbols
- **Key exchange**: Share passphrases via a secure out-of-band channel (not over the air)
- **No key storage**: Passphrases are not saved to disk for security
- **Per-session keys**: Re-enter passphrase each time the modem starts
- **Forward secrecy**: Not provided (same key used for all messages in session)

### Technical Details

```
Encryption: AES-256-CBC
Key derivation: SHA-256(passphrase)
IV: 16 random bytes, prepended to ciphertext
Padding: PKCS#7
```

Implementation files:
- `src/crypto/aes_cipher.hpp` - AES cipher wrapper
- `src/crypto/aes_cipher.cpp` - Implementation using OpenSSL/mbedTLS
- `src/protocol/protocol_engine.cpp` - Encrypt/decrypt integration

---

## Compression

### Overview

Ultra uses **Deflate** compression (via miniz library) to reduce transmission time. Compression is applied before encryption when both are enabled.

### Compression Marker

Compressed data is marked with a **0xCE** byte prefix to distinguish it from uncompressed data:

```
Compressed:   [0xCE] [deflate-compressed-data...]
Uncompressed: [original-data...]
```

### What Gets Compressed

| Data Type | Compressed |
|-----------|------------|
| Text messages | Yes (if enabled) |
| File transfers | Separate flag (FileFlags::COMPRESSED) |

### Compression Behavior

- **Minimum size**: Data < 20 bytes is not compressed (overhead not worth it)
- **Ratio threshold**: Compression only applied if result is smaller than original
- **Algorithm**: Deflate (zlib-compatible)

### Enabling Compression

#### Via TCP Commands

```
COMPRESSION ON
OK
```

#### Via GUI

1. Open **Settings** window
2. Go to **Security** tab
3. Check **Enable Compression**
4. Click **Apply**

### Technical Details

```
Algorithm: Deflate (RFC 1951)
Library: miniz (embedded, no external dependency)
Marker byte: 0xCE
Compression level: Default (balanced speed/ratio)
```

Implementation files:
- `src/protocol/protocol_engine.cpp` - `compressPayload()` / `decompressPayload()`

---

## Processing Order

When both encryption and compression are enabled:

### Transmit Path

```
Original Data
    ↓
[1] Compress (if enabled)
    ↓
[2] Encrypt (if enabled)
    ↓
Transmitted Frame
```

### Receive Path

```
Received Frame
    ↓
[1] Decrypt (if enabled)
    ↓
[2] Decompress (if marker 0xCE present)
    ↓
Original Data
```

This order (compress-then-encrypt) is standard practice because:
1. Compression works better on plaintext than ciphertext
2. Encrypted data appears random and doesn't compress well

---

## File Transfer Security

### Encrypted File Transfers

When encryption is enabled, file transfers are automatically encrypted:

1. Each file chunk is encrypted individually before transmission
2. Chunks are decrypted on receipt before reassembly
3. The complete file is reconstructed from decrypted chunks

### Sending Encrypted Files

#### Via TCP Command

```
ENCRYPTKEY MySecretPassphrase
OK
ENCRYPT ON
OK
CONNECT REMOTE
PENDING
[wait for CONNECTED]
SENDFILE /path/to/file.txt
OK
```

### File Transfer Compression

File transfers have their own compression flag separate from message compression:

- `FileFlags::COMPRESSED` in FILE_START metadata
- Applied to entire file before chunking
- Independent of protocol-level compression setting

---

## TCP Commands Reference

### ENCRYPT / ENCRYPTION

Enable or disable encryption.

```
ENCRYPT ON      Enable encryption
ENCRYPT OFF     Disable encryption
```

**Response:**
- `OK` - Setting applied
- `OK (warning: no key set - use ENCRYPTKEY)` - Enabled but no key set

### ENCRYPTKEY / KEY

Set the encryption passphrase.

```
ENCRYPTKEY <passphrase>
KEY <passphrase>
```

**Example:**
```
ENCRYPTKEY MySecurePassword123!
OK
```

To clear the key:
```
ENCRYPTKEY
OK
```

### COMPRESSION

Enable or disable compression.

```
COMPRESSION ON      Enable compression
COMPRESSION OFF     Disable compression
```

### SENDFILE / SEND

Send a file to the connected station.

```
SENDFILE <filepath>
SEND <filepath>
```

**Example:**
```
SENDFILE /home/user/document.txt
OK
```

**Errors:**
- `ERROR No filepath specified` - Missing filepath argument
- `ERROR Not initialized` - Protocol not ready
- `ERROR Not connected` - No active connection
- `ERROR Failed to start file transfer` - File not found or transfer busy

---

## Configuration File

Security settings are saved in the INI configuration file:

```ini
[Security]
encryption_enabled=1
compression_enabled=1
```

**Note:** The encryption key/passphrase is NOT saved to the configuration file for security reasons. It must be re-entered each session.

---

## Interoperability

### Requirements for Secure Communication

Both stations must have:
1. **Same passphrase** - Keys must match exactly
2. **Encryption enabled** - Both stations must have encryption ON
3. **Compatible compression** - Compression state should match (recommended)

### Mismatched Settings

| Sender | Receiver | Result |
|--------|----------|--------|
| Encrypted | No key | Garbled data received |
| Encrypted | Wrong key | Garbled data received |
| Encrypted | Correct key | Success |
| Compressed | Compression OFF | Success (auto-detected via marker) |
| Both | Both matched | Success |

Compression is automatically detected via the 0xCE marker, so mismatched compression settings are handled gracefully.

---

## Limitations

1. **No authentication**: Encryption provides confidentiality but not authentication
2. **No integrity check**: Use application-level checksums if needed
3. **No perfect forward secrecy**: Compromised key exposes all past messages
4. **Control frames unencrypted**: Protocol handshake visible to observers
5. **Key distribution**: Out-of-band key exchange required

---

## Example: Secure File Transfer Session

```bash
# Terminal 1 - Station A (sender)
$ nc localhost 8300
MYCALL ALPHA
OK
ENCRYPTKEY SecretKey2024!
OK
ENCRYPT ON
OK
COMPRESSION ON
OK
CONNECT BRAVO
PENDING
CONNECTED BRAVO
SENDFILE /tmp/secret_document.pdf
OK
# File transfer completes...
DISCONNECT
OK

# Terminal 2 - Station B (receiver)
$ nc localhost 8300
MYCALL BRAVO
OK
ENCRYPTKEY SecretKey2024!
OK
ENCRYPT ON
OK
COMPRESSION ON
OK
LISTEN ON
OK
# Wait for incoming connection...
CONNECTED ALPHA
# File received automatically, saved to Downloads folder
DISCONNECTED
```

---

## Implementation Notes

### Source Files

| File | Purpose |
|------|---------|
| `src/crypto/aes_cipher.hpp` | AES-256 cipher interface |
| `src/crypto/aes_cipher.cpp` | OpenSSL/mbedTLS implementation |
| `src/protocol/protocol_engine.cpp` | Encrypt/decrypt/compress/decompress |
| `src/protocol/connection.cpp` | File transfer encryption hooks |
| `src/protocol/connection_handlers.cpp` | File transfer decryption |
| `src/interface/host_interface.cpp` | TCP command handlers |
| `src/gui/widgets/settings.cpp` | GUI settings for security |

### Constants

```cpp
// Compression marker (must not conflict with PayloadType)
constexpr uint8_t COMPRESSION_MARKER = 0xCE;

// AES parameters
constexpr size_t AES_KEY_SIZE = 32;   // 256 bits
constexpr size_t AES_IV_SIZE = 16;    // 128 bits
constexpr size_t AES_BLOCK_SIZE = 16; // 128 bits
```
