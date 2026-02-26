#pragma once

#include "connection.hpp"
#include "frame_v2.hpp"
#include "compression.hpp"
#include "crypto/aes256.hpp"
#include <functional>
#include <mutex>
#include <optional>
#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#ifdef ERROR
#undef ERROR
#endif
#endif

namespace ultra {
namespace protocol {

// Protocol callbacks can synchronously call back into ProtocolEngine APIs
// (for example, querying negotiated MC-DPSK interleave inside a
// connection-state callback). Use a recursive mutex to avoid self-deadlock
// on these re-entrant callback paths.
using ProtocolEngineMutex = std::recursive_mutex;

/**
 * Protocol Engine
 *
 * High-level interface that integrates the protocol stack:
 * - Frame serialization/deserialization (v2 format)
 * - Connection management
 * - ARQ for reliable delivery
 *
 * Connects to ModemEngine for actual TX/RX of audio.
 */
class ProtocolEngine {
public:
    using TxDataCallback = std::function<void(const Bytes& data)>;
    using MessageReceivedCallback = std::function<void(const std::string& from, const std::string& text)>;
    using ConnectionChangedCallback = std::function<void(ConnectionState state, const std::string& remote)>;
    using IncomingCallCallback = std::function<void(const std::string& from)>;

    using FileProgressCallback = Connection::FileProgressCallback;
    using FileReceivedCallback = Connection::FileReceivedCallback;
    using FileSentCallback = Connection::FileSentCallback;

    using ModeNegotiatedCallback = Connection::ModeNegotiatedCallback;
    using ConnectWaveformChangedCallback = Connection::ConnectWaveformChangedCallback;

    explicit ProtocolEngine(const ConnectionConfig& config = ConnectionConfig{});

    // --- Configuration ---

    void setLocalCallsign(const std::string& call);
    std::string getLocalCallsign() const;

    void setAutoAccept(bool auto_accept);
    bool getAutoAccept() const;

    // --- Callbacks ---

    void setTxDataCallback(TxDataCallback cb);
    void setMessageReceivedCallback(MessageReceivedCallback cb);
    void setConnectionChangedCallback(ConnectionChangedCallback cb);
    void setIncomingCallCallback(IncomingCallCallback cb);

    // --- Connection Control ---

    bool connect(const std::string& remote_call);
    void acceptCall();
    void rejectCall();
    void disconnect();
    void abortTxNow();

    // --- Data Transfer ---

    bool sendMessage(const std::string& text);
    bool sendMessages(const std::vector<std::string>& texts);  // Batch: burst-interleaved
    bool isReadyToSend() const;

    // --- File Transfer ---

    bool sendFile(const std::string& filepath);
    void setReceiveDirectory(const std::string& dir);
    void cancelFileTransfer();
    bool isFileTransferInProgress() const;
    FileTransferProgress getFileProgress() const;

    void setFileProgressCallback(FileProgressCallback cb);
    void setFileReceivedCallback(FileReceivedCallback cb);
    void setFileSentCallback(FileSentCallback cb);

    // --- Modem Interface ---

    void onRxData(const Bytes& data);
    void tick(uint32_t elapsed_ms);

    // --- State ---

    ConnectionState getState() const;
    std::string getRemoteCallsign() const;
    bool isConnected() const;

    ConnectionStats getStats() const;
    void resetStats();
    void reset();

    // --- Waveform Mode ---

    WaveformMode getNegotiatedMode() const;
    void setPreferredMode(WaveformMode mode);
    void setModeCapabilities(uint8_t caps);
    void setMCDPSKChannelInterleaveOffer(bool enable);
    bool getMCDPSKChannelInterleaveOffer() const;
    bool isMCDPSKChannelInterleaveActive() const;

    // Forced data mode - operator can override SNR-based selection
    // Set before calling connect(). Values are sent in CONNECT frame.
    // 0xFF (AUTO) = let responder decide based on SNR
    void setForcedModulation(Modulation mod);
    void setForcedCodeRate(CodeRate rate);
    Modulation getForcedModulation() const;
    CodeRate getForcedCodeRate() const;

    void setModeNegotiatedCallback(ModeNegotiatedCallback cb);
    void setConnectWaveformChangedCallback(ConnectWaveformChangedCallback cb);
    WaveformMode getConnectWaveform() const;
    void setInitialConnectWaveform(WaveformMode mode);

    // Callback when handshake is confirmed (safe to switch to negotiated waveform)
    using HandshakeConfirmedCallback = Connection::HandshakeConfirmedCallback;
    void setHandshakeConfirmedCallback(HandshakeConfirmedCallback cb);

    // --- Adaptive Data Mode ---

    // Set measured SNR from modem (used for adaptive mode selection)
    void setMeasuredSNR(float snr_db);
    float getMeasuredSNR() const;

    // Set channel quality including fading detection
    void setChannelQuality(float snr_db, float fading_index);
    float getFadingIndex() const;

    // Get current data mode
    Modulation getDataModulation() const;
    CodeRate getDataCodeRate() const;

    // Set callback for data mode changes
    using DataModeChangedCallback = Connection::DataModeChangedCallback;
    void setDataModeChangedCallback(DataModeChangedCallback cb);

    // Request mode change to remote station (for adaptive mode)
    // reason: 0=CHANNEL_IMPROVED, 1=CHANNEL_DEGRADED, 2=USER_REQUEST, 3=INITIAL_SETUP
    // Returns true when the request was queued, false if blocked (busy/not connected/unsupported).
    bool requestModeChange(Modulation new_mod, CodeRate new_rate, float measured_snr, uint8_t reason,
                           std::optional<WaveformMode> new_waveform = std::nullopt);

    // --- Beacon/CQ (broadcast without connection) ---

    // Transmit disconnected beacon broadcast (variable-length DATA frame payload).
    // Returns true if beacon was transmitted, false if blocked
    bool transmitBeacon(const Bytes& payload = {});

    // Transmit CQ frame (same as beacon, but enters listening state after TX)
    // Returns true if CQ was transmitted, false if blocked
    bool transmitCQ(const Bytes& payload = {});

    // Transmit directed disconnected DATA datagram (PING command payload path)
    // Returns true if ping was transmitted, false if blocked
    bool transmitPing(const std::string& target, const Bytes& payload = {});

    // Beacon TX callback (serialized frame ready for modulation with 4x spreading)
    using BeaconTxCallback = Connection::BeaconTxCallback;
    void setBeaconTxCallback(BeaconTxCallback cb);

    // CQ TX callback (same as beacon but enters listening after TX)
    using CQTxCallback = Connection::CQTxCallback;
    void setCQTxCallback(CQTxCallback cb);

    // Beacon received callback
    using BeaconReceivedCallback = Connection::BeaconReceivedCallback;
    void setBeaconReceivedCallback(BeaconReceivedCallback cb);

    // --- Ping/Pong (fast presence check) ---

    // Callback when protocol wants to send a ping (modem transmits raw "ULTR")
    // Burst mode TX callback - transmits multiple frames as single audio burst (OFDM only)
    using TransmitBurstCallback = Connection::TransmitBurstCallback;
    void setTransmitBurstCallback(TransmitBurstCallback cb);

    using PingTxCallback = Connection::PingTxCallback;
    void setPingTxCallback(PingTxCallback cb);

    // Callback when we receive an incoming ping while disconnected (someone's calling)
    using PingReceivedCallback = Connection::PingReceivedCallback;
    void setPingReceivedCallback(PingReceivedCallback cb);

    // Call when modem detects "ULTR" magic (either PONG to our PING, or incoming PING)
    void onPingReceived();

    // --- Encryption ---

    /**
     * Enable/disable AES-256 encryption for data payloads.
     * Both stations must use the same key for successful communication.
     */
    void setEncryptionEnabled(bool enabled);
    bool isEncryptionEnabled() const;

    /**
     * Set encryption key from passphrase (hashed with SHA-256).
     * @param passphrase User-provided passphrase
     * @return true if key was set successfully
     */
    bool setEncryptionKey(const std::string& passphrase);

    /**
     * Clear the encryption key from memory.
     */
    void clearEncryptionKey();

    /**
     * Check if a valid encryption key is set.
     */
    bool hasEncryptionKey() const;

    // --- Compression ---

    /**
     * Enable/disable deflate compression for data payloads.
     * Compression is applied before encryption (compress-then-encrypt).
     */
    void setCompressionEnabled(bool enabled);
    bool isCompressionEnabled() const;

private:
    Connection connection_;

    TxDataCallback on_tx_data_;
    MessageReceivedCallback on_message_received_;
    ConnectionChangedCallback on_connection_changed_;
    IncomingCallCallback on_incoming_call_;

    mutable ProtocolEngineMutex mutex_;

    Bytes rx_buffer_;

    std::vector<Bytes> tx_queue_;
    bool defer_tx_ = false;

    // Encryption
    crypto::Aes256 cipher_;
    bool encryption_enabled_ = false;

    // Compression
    bool compression_enabled_ = true;  // Enabled by default

    // Compress data payload (returns original if compression not beneficial)
    Bytes compressPayload(const Bytes& data);
    // Decompress data payload (returns original if not compressed)
    Bytes decompressPayload(const Bytes& data);

    // Encrypt data payload (returns empty on error)
    Bytes encryptPayload(const Bytes& plaintext);
    // Decrypt data payload (returns empty on error)
    Bytes decryptPayload(const Bytes& ciphertext);

    void handleTxFrame(const Bytes& frame_data);
    void processRxBuffer();
};

} // namespace protocol
} // namespace ultra
