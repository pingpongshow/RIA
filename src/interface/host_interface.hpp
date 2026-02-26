// HostInterface - Main integration class for TCP host interface
// Connects TcpServer to ProtocolEngine and ModemEngine

#pragma once

#include "interface.hpp"
#include "tcp_server.hpp"
#include "command_parser.hpp"
#include "response.hpp"
#include "kiss_tnc.hpp"

#include <deque>
#include <memory>
#include <mutex>
#include <vector>
#include <functional>
#include <string>
#include <chrono>

// Forward declarations
namespace ultra {
namespace protocol {
    class ProtocolEngine;
}
namespace gui {
    class ModemEngine;
    struct AppSettings;
    class AudioEngine;
    class SerialPttController;
}
namespace cat {
    class CatController;
}
}

namespace ultra {
namespace interface {

// Version string for VERSION command
constexpr const char* ULTRA_VERSION = "1.0.0";

class HostInterface {
public:
    explicit HostInterface(const TcpConfig& config = TcpConfig{});
    ~HostInterface();

    // Non-copyable
    HostInterface(const HostInterface&) = delete;
    HostInterface& operator=(const HostInterface&) = delete;

    // Lifecycle
    bool start();
    void stop();
    bool isRunning() const;

    // Called from main event loop (~20ms tick rate)
    void poll();

    // Bind to existing components (must be called before start())
    void bindProtocol(protocol::ProtocolEngine* engine);
    void bindModem(gui::ModemEngine* modem);
    void bindSettings(gui::AppSettings* settings);
    void bindAudio(gui::AudioEngine* audio);
    void bindSerialPtt(gui::SerialPttController* ptt);
    void bindCat(cat::CatController* cat);

    // Get TX data buffer size (for BUFFER command)
    size_t getTxBufferSize() const;

    // Queue data for transmission (from data port)
    void queueTxData(const std::vector<uint8_t>& data);

    // Deliver received data (to data port clients)
    void deliverRxData(const std::vector<uint8_t>& data);

    // Deliver received beacon data (formatted for data port clients)
    // Format: [0xFA][callsign_len][callsign][payload]
    void deliverBeaconData(const std::string& from_call, uint32_t from_hash,
                           const std::vector<uint8_t>& payload);

    // Async notifications to all command clients
    void notifyStateChange(const std::string& state);
    void notifyConnected(const std::string& callsign);
    void notifyDisconnected();
    void notifyLinkBroken();
    void notifySnr(float db);
    void notifyPtt(bool active);
    void notifyBusy(bool detected);

    // Callback types for events that need App integration
    using ConnectRequestCallback = std::function<void(const std::string& callsign)>;
    using DisconnectRequestCallback = std::function<void()>;
    using AbortRequestCallback = std::function<void()>;
    using SendMessageCallback = std::function<void(const std::string& text)>;
    using SendDataCallback = std::function<void(const std::vector<uint8_t>& data)>;
    using CallsignChangedCallback = std::function<void(const std::string& callsign)>;
    using TuneRequestCallback = std::function<void(bool enabled)>;
    // waveform, modulation, code_rate values (0xFF = AUTO)
    using ExpertSettingsCallback = std::function<void(uint8_t waveform, uint8_t modulation, uint8_t code_rate)>;
    // Beacon/CQ/Ping callbacks - data will be read from data port after command
    using BeaconRequestCallback = std::function<void()>;
    using CQRequestCallback = std::function<void()>;
    using PingRequestCallback = std::function<void(const std::string& callsign)>;
    using RawTxRequestCallback = std::function<void(const std::vector<uint8_t>& payload,
                                                    uint8_t waveform,
                                                    uint8_t modulation,
                                                    uint8_t code_rate)>;

    // Set callbacks for actions that need App coordination
    void setConnectRequestCallback(ConnectRequestCallback cb) { on_connect_request_ = std::move(cb); }
    void setDisconnectRequestCallback(DisconnectRequestCallback cb) { on_disconnect_request_ = std::move(cb); }
    void setAbortRequestCallback(AbortRequestCallback cb) { on_abort_request_ = std::move(cb); }
    void setSendMessageCallback(SendMessageCallback cb) { on_send_message_ = std::move(cb); }
    void setSendDataCallback(SendDataCallback cb) { on_send_data_ = std::move(cb); }
    void setCallsignChangedCallback(CallsignChangedCallback cb) { on_callsign_changed_ = std::move(cb); }
    void setTuneRequestCallback(TuneRequestCallback cb) { on_tune_request_ = std::move(cb); }
    void setExpertSettingsCallback(ExpertSettingsCallback cb) { on_expert_settings_ = std::move(cb); }
    void setBeaconRequestCallback(BeaconRequestCallback cb) { on_beacon_request_ = std::move(cb); }
    void setCQRequestCallback(CQRequestCallback cb) { on_cq_request_ = std::move(cb); }
    void setPingRequestCallback(PingRequestCallback cb) { on_ping_request_ = std::move(cb); }
    void setRawTxRequestCallback(RawTxRequestCallback cb) { on_raw_tx_request_ = std::move(cb); }

private:
    TcpConfig config_;
    std::unique_ptr<TcpServer> server_;
    std::unique_ptr<TcpServer> kiss_server_;

    // Bound components (not owned)
    protocol::ProtocolEngine* protocol_ = nullptr;
    gui::ModemEngine* modem_ = nullptr;
    gui::AppSettings* settings_ = nullptr;
    gui::AudioEngine* audio_ = nullptr;
    gui::SerialPttController* serial_ptt_ = nullptr;
    cat::CatController* cat_ = nullptr;

    // TX data buffer (from data port)
    std::vector<uint8_t> tx_buffer_;
    mutable std::mutex tx_mutex_;

    // RX payload buffering (to data port)
    // Allows transient data clients (e.g. one-shot nc readers) to receive data
    // even when they reconnect after a frame has already been decoded.
    std::deque<std::vector<uint8_t>> pending_rx_messages_;
    size_t pending_rx_bytes_ = 0;
    mutable std::mutex rx_mutex_;
    static constexpr size_t MAX_PENDING_RX_MESSAGES = 256;
    static constexpr size_t MAX_PENDING_RX_BYTES = 1024 * 1024;  // 1 MiB

    // KISS state
    KissFramer kiss_framer_;
    KissPortConfig kiss_config_;

    // Callbacks
    ConnectRequestCallback on_connect_request_;
    DisconnectRequestCallback on_disconnect_request_;
    AbortRequestCallback on_abort_request_;
    SendMessageCallback on_send_message_;
    SendDataCallback on_send_data_;
    CallsignChangedCallback on_callsign_changed_;
    TuneRequestCallback on_tune_request_;
    ExpertSettingsCallback on_expert_settings_;
    BeaconRequestCallback on_beacon_request_;
    CQRequestCallback on_cq_request_;
    PingRequestCallback on_ping_request_;
    RawTxRequestCallback on_raw_tx_request_;

    // State
    bool listen_mode_ = false;
    bool compression_enabled_ = false;
    bool chat_mode_ = false;
    bool winlink_mode_ = false;
    bool auto_mode_ = true;
    bool ptt_active_ = false;
    bool busy_detected_ = false;
    bool beacon_pending_ = false;  // Beacon TX pending (data from data port)
    bool cq_pending_ = false;      // CQ TX pending (data from data port)
    bool ping_pending_ = false;    // Ping TX pending (data from data port)
    bool rawtx_pending_ = false;   // RAWTX pending (data from data port)
    std::string ping_target_;      // Target callsign for pending ping
    uint8_t rawtx_waveform_ = 0xFF;
    uint8_t rawtx_modulation_ = 0xFF;
    uint8_t rawtx_code_rate_ = 0xFF;
    std::vector<uint8_t> pending_control_payload_;
    std::chrono::steady_clock::time_point beacon_pending_since_;
    std::chrono::steady_clock::time_point cq_pending_since_;
    std::chrono::steady_clock::time_point ping_pending_since_;
    std::chrono::steady_clock::time_point rawtx_pending_since_;
    std::chrono::steady_clock::time_point pending_control_last_rx_;
    // Unconnected command payload caps (carried via DATA-frame datagrams).
    static constexpr size_t CONTROL_PAYLOAD_MAX = 2048;
    static constexpr size_t BROADCAST_PAYLOAD_MAX = 2048;
    static constexpr size_t RAWTX_PAYLOAD_MAX = 4096;
    static constexpr int PENDING_PAYLOAD_TIMEOUT_MS = 300;
    static constexpr int PENDING_PAYLOAD_GAP_MS = 40;

    // Command handlers
    Response handleCommand(const ParsedCommand& cmd);
    Response handleConnect(const std::string& callsign);
    Response handleDisconnect();
    Response handleAbort();
    Response handleMyCall(const std::string& callsign);
    Response handleMyAux(const std::string& callsigns);
    Response handleCompression(bool enabled);
    Response handleListen(bool enabled);
    Response handleChatMode(bool enabled);
    Response handleAutoMode(bool enabled);
    Response handleWinlink(bool enabled);
    Response handleWaveform(const std::string& waveform);
    Response handleModulation(const std::string& modulation);
    Response handleCodeRate(const std::string& rate);
    Response handleMCDPSKCarriers(int num_carriers);
    Response handleVersion();
    Response handleCodec();
    Response handleState();
    Response handlePttState();
    Response handleBusy();
    Response handleBuffer();
    Response handleTune(bool enabled);
    Response handleCwId(const std::string& callsign);
    Response handleClose();
    Response handlePttLead(uint32_t ms);
    Response handlePttTail(uint32_t ms);
    Response handleTxDrive(float level);
    Response handleEncrypt(bool enabled);
    Response handleEncryptKey(const std::string& key);
    Response handleSendFile(const std::string& filepath);

    // Beacon/CQ/Ping command handlers
    Response handleBeacon();
    Response handleCQ();
    Response handlePing(const std::string& callsign);
    Response handleRawTx(const std::string& args);

    // CAT command handlers
    Response handleCatEnable(bool enabled);
    Response handleCatBackend(const std::string& backend);
    Response handleCatModel(int model);
    Response handleCatPort(const std::string& port);
    Response handleCatBaud(int baud);
    Response handleCatSlice(int slice);
    Response handleCatConnect();
    Response handleCatDisconnect();
    Response handleCatPtt(bool active);
    Response handleCatFreq(uint64_t hz);
    Response handleCatGetFreq();
    Response handleCatMode(const std::string& mode);
    Response handleCatGetMode();
    Response handleCatWatchdog(int seconds);
    Response handleCatPttLead(int ms);
    Response handleCatPttTail(int ms);
    Response handleCatStatus();

    // Process data from data port
    void processDataPort();

    // Process KISS frames
    void processKissFrames();
};

} // namespace interface
} // namespace ultra
