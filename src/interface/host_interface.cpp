// HostInterface implementation

#include "host_interface.hpp"
#include "protocol/protocol_engine.hpp"
#include "gui/modem/modem_engine.hpp"
#include "gui/widgets/settings.hpp"
#include "cat/cat_controller.hpp"
#include "cat/cat_backend.hpp"
#include "ultra/logging.hpp"

#include <algorithm>
#include <sstream>

namespace ultra {
namespace interface {

namespace {
inline size_t safeSub(size_t lhs, size_t rhs) {
    return (rhs > lhs) ? 0 : (lhs - rhs);
}

bool parseWaveformToken(const std::string& token, uint8_t& out) {
    std::string upper = token;
    std::transform(upper.begin(), upper.end(), upper.begin(), ::toupper);

    if (upper == "AUTO") {
        out = 0xFF;
        return true;
    }
    if (upper == "MC_DPSK" || upper == "DPSK") {
        out = static_cast<uint8_t>(protocol::WaveformMode::MC_DPSK);
        return true;
    }
    if (upper == "OFDM_CHIRP" || upper == "OFDM" || upper == "CHIRP") {
        out = static_cast<uint8_t>(protocol::WaveformMode::OFDM_CHIRP);
        return true;
    }
    if (upper == "OFDM_COX" || upper == "COX") {
        out = static_cast<uint8_t>(protocol::WaveformMode::OFDM_COX);
        return true;
    }
    if (upper == "OTFS") {
        out = static_cast<uint8_t>(protocol::WaveformMode::OTFS_EQ);
        return true;
    }
    if (upper == "MFSK") {
        out = static_cast<uint8_t>(protocol::WaveformMode::MFSK);
        return true;
    }
    return false;
}

bool parseModulationToken(const std::string& token, uint8_t& out) {
    std::string upper = token;
    std::transform(upper.begin(), upper.end(), upper.begin(), ::toupper);

    if (upper == "AUTO") {
        out = 0xFF;
        return true;
    }
    if (upper == "DBPSK") {
        out = static_cast<uint8_t>(Modulation::DBPSK);
        return true;
    }
    if (upper == "BPSK") {
        out = static_cast<uint8_t>(Modulation::BPSK);
        return true;
    }
    if (upper == "DQPSK") {
        out = static_cast<uint8_t>(Modulation::DQPSK);
        return true;
    }
    if (upper == "QPSK") {
        out = static_cast<uint8_t>(Modulation::QPSK);
        return true;
    }
    if (upper == "D8PSK" || upper == "8PSK") {
        out = static_cast<uint8_t>(Modulation::D8PSK);
        return true;
    }
    if (upper == "QAM8") {
        out = static_cast<uint8_t>(Modulation::QAM8);
        return true;
    }
    if (upper == "QAM16" || upper == "16QAM") {
        out = static_cast<uint8_t>(Modulation::QAM16);
        return true;
    }
    if (upper == "QAM32" || upper == "32QAM") {
        out = static_cast<uint8_t>(Modulation::QAM32);
        return true;
    }
    if (upper == "QAM64" || upper == "64QAM") {
        out = static_cast<uint8_t>(Modulation::QAM64);
        return true;
    }
    if (upper == "QAM256" || upper == "256QAM") {
        out = static_cast<uint8_t>(Modulation::QAM256);
        return true;
    }
    return false;
}

bool parseCodeRateToken(const std::string& token, uint8_t& out) {
    std::string upper = token;
    std::transform(upper.begin(), upper.end(), upper.begin(), ::toupper);

    if (upper == "AUTO") {
        out = 0xFF;
        return true;
    }
    if (upper == "R1_4" || upper == "R1/4" || upper == "1/4") {
        out = static_cast<uint8_t>(CodeRate::R1_4);
        return true;
    }
    if (upper == "R1_3" || upper == "R1/3" || upper == "1/3") {
        out = static_cast<uint8_t>(CodeRate::R1_3);
        return true;
    }
    if (upper == "R1_2" || upper == "R1/2" || upper == "1/2") {
        out = static_cast<uint8_t>(CodeRate::R1_2);
        return true;
    }
    if (upper == "R2_3" || upper == "R2/3" || upper == "2/3") {
        out = static_cast<uint8_t>(CodeRate::R2_3);
        return true;
    }
    if (upper == "R3_4" || upper == "R3/4" || upper == "3/4") {
        out = static_cast<uint8_t>(CodeRate::R3_4);
        return true;
    }
    if (upper == "R5_6" || upper == "R5/6" || upper == "5/6") {
        out = static_cast<uint8_t>(CodeRate::R5_6);
        return true;
    }
    if (upper == "R7_8" || upper == "R7/8" || upper == "7/8") {
        out = static_cast<uint8_t>(CodeRate::R7_8);
        return true;
    }
    return false;
}
}  // namespace

HostInterface::HostInterface(const TcpConfig& config)
    : config_(config) {}

HostInterface::~HostInterface() {
    stop();
}

bool HostInterface::start() {
    if (server_ && server_->isRunning()) {
        return true;
    }

    server_ = std::make_unique<TcpServer>(config_);
    if (!server_->start()) {
        LOG_MODEM(ERROR, "HostInterface: Failed to start TCP server");
        server_.reset();
        return false;
    }

    // Start KISS server if enabled
    if (config_.kiss_enabled) {
        TcpConfig kiss_config = config_;
        kiss_config.cmd_port = config_.kiss_port;
        kiss_config.data_port = config_.kiss_port + 1;  // Not used for KISS
        kiss_server_ = std::make_unique<TcpServer>(kiss_config);
        if (!kiss_server_->start()) {
            LOG_MODEM(WARN, "HostInterface: Failed to start KISS server on port %d",
                      config_.kiss_port);
            kiss_server_.reset();
            // Continue without KISS
        }
    }

    LOG_MODEM(INFO, "HostInterface: Started (cmd=%d, data=%d)",
              config_.cmd_port, config_.data_port);
    return true;
}

void HostInterface::stop() {
    if (server_) {
        server_->stop();
        server_.reset();
    }
    if (kiss_server_) {
        kiss_server_->stop();
        kiss_server_.reset();
    }

    {
        std::lock_guard<std::mutex> lock(tx_mutex_);
        tx_buffer_.clear();
    }
    {
        std::lock_guard<std::mutex> lock(rx_mutex_);
        pending_rx_messages_.clear();
        pending_rx_bytes_ = 0;
    }

    LOG_MODEM(INFO, "HostInterface: Stopped");
}

bool HostInterface::isRunning() const {
    return server_ && server_->isRunning();
}

void HostInterface::poll() {
    if (!server_) return;

    // Accept new connections
    server_->accept();
    if (kiss_server_) {
        kiss_server_->accept();
    }

    // Read and process commands
    auto commands = server_->readCommands();
    for (const auto& [client_idx, cmd] : commands) {
        Response response = handleCommand(cmd);
        server_->sendResponseTo(client_idx, response);

        // Handle CLOSE command
        if (response.shouldClose()) {
            // Client will disconnect itself
        }
    }

    // Process data port
    processDataPort();

    // Process KISS frames
    if (kiss_server_) {
        processKissFrames();
    }
}

void HostInterface::bindProtocol(protocol::ProtocolEngine* engine) {
    protocol_ = engine;
}

void HostInterface::bindModem(gui::ModemEngine* modem) {
    modem_ = modem;
}

void HostInterface::bindSettings(gui::AppSettings* settings) {
    settings_ = settings;
}

void HostInterface::bindAudio(gui::AudioEngine* audio) {
    audio_ = audio;
}

void HostInterface::bindSerialPtt(gui::SerialPttController* ptt) {
    serial_ptt_ = ptt;
}

size_t HostInterface::getTxBufferSize() const {
    std::lock_guard<std::mutex> lock(tx_mutex_);
    return tx_buffer_.size();
}

void HostInterface::queueTxData(const std::vector<uint8_t>& data) {
    std::lock_guard<std::mutex> lock(tx_mutex_);
    tx_buffer_.insert(tx_buffer_.end(), data.begin(), data.end());
}

void HostInterface::deliverRxData(const std::vector<uint8_t>& data) {
    if (data.empty()) {
        return;
    }

    // Fast path: deliver immediately when a data client is connected.
    if (server_ && server_->hasDataClient() &&
        server_->sendData(data.data(), data.size())) {
        LOG_MODEM(INFO, "HostInterface: Delivered RX payload immediately (%zu bytes)", data.size());
        return;
    }

    // Fallback for transient readers: buffer payloads until a client reconnects.
    std::lock_guard<std::mutex> lock(rx_mutex_);
    pending_rx_messages_.push_back(data);
    pending_rx_bytes_ += data.size();
    LOG_MODEM(INFO, "HostInterface: Buffered RX payload (%zu bytes, pending=%zu msgs/%zu bytes)",
              data.size(), pending_rx_messages_.size(), pending_rx_bytes_);

    // Keep a bounded backlog.
    while ((pending_rx_messages_.size() > MAX_PENDING_RX_MESSAGES) ||
           (pending_rx_bytes_ > MAX_PENDING_RX_BYTES)) {
        if (pending_rx_messages_.empty()) {
            pending_rx_bytes_ = 0;
            break;
        }
        pending_rx_bytes_ -= pending_rx_messages_.front().size();
        pending_rx_messages_.pop_front();
    }
}

void HostInterface::deliverBeaconData(const std::string& from_call, uint32_t from_hash,
                                       const std::vector<uint8_t>& payload) {
    // Format: [0xFA][callsign_len][callsign][payload]
    // 0xFA = beacon marker (matches app's beacon protocol)
    constexpr uint8_t BEACON_MARKER = 0xFA;

    std::vector<uint8_t> beacon_data;
    beacon_data.reserve(2 + from_call.size() + payload.size());

    // Marker
    beacon_data.push_back(BEACON_MARKER);

    // Callsign length and callsign
    // If we have a callsign, use it. Otherwise, encode the hash as hex.
    if (!from_call.empty()) {
        beacon_data.push_back(static_cast<uint8_t>(from_call.size()));
        beacon_data.insert(beacon_data.end(), from_call.begin(), from_call.end());
    } else {
        // No callsign known - encode hash as 6-char hex
        char hash_str[7];
        snprintf(hash_str, sizeof(hash_str), "%06X", from_hash & 0xFFFFFF);
        beacon_data.push_back(6);
        beacon_data.insert(beacon_data.end(), hash_str, hash_str + 6);
    }

    // Payload
    beacon_data.insert(beacon_data.end(), payload.begin(), payload.end());

    LOG_MODEM(INFO, "HostInterface: Delivering beacon from %s (%zu bytes total)",
              from_call.empty() ? "(unknown)" : from_call.c_str(), beacon_data.size());

    // Deliver via standard RX path
    deliverRxData(beacon_data);
}

void HostInterface::notifyStateChange(const std::string& state) {
    if (server_) {
        server_->sendResponse(Response::state(state));
    }
}

void HostInterface::notifyConnected(const std::string& callsign) {
    if (server_) {
        server_->sendResponse(Response::connected(callsign));
    }
}

void HostInterface::notifyDisconnected() {
    if (server_) {
        server_->sendResponse(Response::disconnected());
    }
}

void HostInterface::notifyLinkBroken() {
    if (server_) {
        server_->sendResponse(Response::linkBroken());
    }
}

void HostInterface::notifySnr(float db) {
    if (server_) {
        server_->sendResponse(Response::snr(db));
    }
}

void HostInterface::notifyPtt(bool active) {
    // Edge-triggered notify to avoid flooding command clients with duplicate
    // "PTT OFF"/"PTT ON" lines at UI frame rate.
    if (active == ptt_active_) {
        return;
    }
    ptt_active_ = active;
    if (server_) {
        server_->sendResponse(Response::ptt(active));
    }
}

void HostInterface::notifyBusy(bool detected) {
    if (detected == busy_detected_) {
        return;
    }
    busy_detected_ = detected;
    if (server_) {
        server_->sendResponse(Response::busy(detected));
    }
}

Response HostInterface::handleCommand(const ParsedCommand& cmd) {
    switch (cmd.cmd) {
        case Command::Connect:
            return handleConnect(cmd.args);
        case Command::Disconnect:
            return handleDisconnect();
        case Command::Abort:
            return handleAbort();
        case Command::Beacon:
            return handleBeacon();
        case Command::CQ:
            return handleCQ();
        case Command::Ping:
            return handlePing(cmd.args);
        case Command::RawTx:
            return handleRawTx(cmd.args);
        case Command::MyCall:
            return handleMyCall(cmd.args);
        case Command::MyAux:
            return handleMyAux(cmd.args);
        case Command::Compression:
            return handleCompression(cmd.argAsBool());
        case Command::Listen:
            return handleListen(cmd.argAsBool());
        case Command::ChatMode:
            return handleChatMode(cmd.argAsBool());
        case Command::AutoMode:
            return handleAutoMode(cmd.argAsBool());
        case Command::WinlinkSession:
            return handleWinlink(cmd.argAsBool());
        case Command::Waveform:
            return handleWaveform(cmd.args);
        case Command::Modulation:
            return handleModulation(cmd.args);
        case Command::CodeRate:
            return handleCodeRate(cmd.args);
        case Command::MCDPSKCarriers:
            return handleMCDPSKCarriers(cmd.argAsInt(10));
        case Command::Version:
            return handleVersion();
        case Command::Codec:
            return handleCodec();
        case Command::State:
            return handleState();
        case Command::PttState:
            return handlePttState();
        case Command::Busy:
            return handleBusy();
        case Command::Buffer:
            return handleBuffer();
        case Command::Tune:
            return handleTune(cmd.argAsBool());
        case Command::CwId:
            return handleCwId(cmd.args);
        case Command::Close:
            return handleClose();
        case Command::PttLead:
            return handlePttLead(cmd.argAsUint32(50));
        case Command::PttTail:
            return handlePttTail(cmd.argAsUint32(20));
        case Command::TxDrive: {
            // Parse float from string (0.0 - 1.0)
            float level = 0.8f;
            try {
                level = std::stof(cmd.args);
            } catch (...) {}
            return handleTxDrive(level);
        }
        case Command::Encrypt:
            return handleEncrypt(cmd.argAsBool());
        case Command::EncryptKey:
            return handleEncryptKey(cmd.args);
        case Command::SendFile:
            return handleSendFile(cmd.args);
        // CAT commands
        case Command::CatEnable:
            return handleCatEnable(cmd.argAsBool());
        case Command::CatBackend:
            return handleCatBackend(cmd.args);
        case Command::CatModel:
            return handleCatModel(cmd.argAsInt());
        case Command::CatPort:
            return handleCatPort(cmd.args);
        case Command::CatBaud:
            return handleCatBaud(cmd.argAsInt(9600));
        case Command::CatSlice:
            return handleCatSlice(cmd.argAsInt());
        case Command::CatConnect:
            return handleCatConnect();
        case Command::CatDisconnect:
            return handleCatDisconnect();
        case Command::CatPtt:
            return handleCatPtt(cmd.argAsBool());
        case Command::CatFreq:
            return handleCatFreq(cmd.argAsUint64());
        case Command::CatGetFreq:
            return handleCatGetFreq();
        case Command::CatMode:
            return handleCatMode(cmd.args);
        case Command::CatGetMode:
            return handleCatGetMode();
        case Command::CatWatchdog:
            return handleCatWatchdog(cmd.argAsInt(120));
        case Command::CatPttLead:
            return handleCatPttLead(cmd.argAsInt(50));
        case Command::CatPttTail:
            return handleCatPttTail(cmd.argAsInt(50));
        case Command::CatStatus:
            return handleCatStatus();
        case Command::Unknown:
        default:
            LOG_MODEM(WARN, "HostInterface: Unknown command: %s", cmd.args.c_str());
            return Response::error("Unknown command");
    }
}

// Connection commands

Response HostInterface::handleConnect(const std::string& callsign) {
    if (callsign.empty() || callsign.length() > 10) {
        return Response::error("Invalid callsign");
    }

    if (on_connect_request_) {
        on_connect_request_(callsign);
        return Response::pending();
    }

    return Response::error("Not initialized");
}

Response HostInterface::handleDisconnect() {
    if (on_disconnect_request_) {
        on_disconnect_request_();
        return Response::ok();
    }
    return Response::error("Not initialized");
}

Response HostInterface::handleAbort() {
    if (on_abort_request_) {
        on_abort_request_();
        return Response::ok();
    }
    return Response::error("Not initialized");
}

// Configuration commands

Response HostInterface::handleMyCall(const std::string& callsign) {
    if (callsign.empty() || callsign.length() > 10) {
        return Response::error("Invalid callsign");
    }

    if (on_callsign_changed_) {
        on_callsign_changed_(callsign);
    }

    if (settings_) {
        strncpy(settings_->callsign, callsign.c_str(), sizeof(settings_->callsign) - 1);
        settings_->callsign[sizeof(settings_->callsign) - 1] = '\0';
    }

    return Response::ok();
}

Response HostInterface::handleMyAux(const std::string& callsigns) {
    // Parse comma-separated callsigns
    // For now, just acknowledge - Ultra doesn't support aux callsigns yet
    (void)callsigns;
    return Response::ok();
}

Response HostInterface::handleCompression(bool enabled) {
    compression_enabled_ = enabled;

    // Update settings if bound
    if (settings_) {
        settings_->compression_enabled = enabled;
    }

    // Update protocol engine if bound
    if (protocol_) {
        protocol_->setCompressionEnabled(enabled);
    }

    LOG_MODEM(INFO, "HostInterface: Compression %s", enabled ? "enabled" : "disabled");
    return Response::ok();
}

Response HostInterface::handleListen(bool enabled) {
    listen_mode_ = enabled;
    return Response::ok();
}

Response HostInterface::handleChatMode(bool enabled) {
    chat_mode_ = enabled;
    return Response::ok();
}

Response HostInterface::handleAutoMode(bool enabled) {
    auto_mode_ = enabled;

    // If AUTO mode enabled, reset all forced settings
    if (enabled && settings_) {
        settings_->forced_waveform = 0xFF;
        settings_->forced_modulation = 0xFF;
        settings_->forced_code_rate = 0xFF;
        if (on_expert_settings_) {
            on_expert_settings_(0xFF, 0xFF, 0xFF);
        }
    }
    return Response::ok();
}

Response HostInterface::handleWinlink(bool enabled) {
    winlink_mode_ = enabled;
    return Response::ok();
}

// Ultra-specific waveform/mode commands

Response HostInterface::handleWaveform(const std::string& waveform) {
    fprintf(stderr, "[HOST] === handleWaveform('%s') ===\n", waveform.c_str());
    fflush(stderr);

    if (!settings_) {
        return Response::error("Not initialized");
    }

    std::string upper = waveform;
    std::transform(upper.begin(), upper.end(), upper.begin(), ::toupper);

    // Waveform values from WaveformMode enum (frame_v2.hpp):
    // OFDM_COX=0x00, OTFS_EQ=0x01, OTFS_RAW=0x02, MFSK=0x03, MC_DPSK=0x04,
    // OFDM_CHIRP=0x05, AUTO=0xFF
    uint8_t value = 0xFF;
    if (upper == "AUTO") {
        value = 0xFF;  // WaveformMode::AUTO
    } else if (upper == "MC_DPSK" || upper == "DPSK") {
        value = 0x04;  // WaveformMode::MC_DPSK
    } else if (upper == "OFDM_CHIRP" || upper == "OFDM" || upper == "CHIRP") {
        value = 0x05;  // WaveformMode::OFDM_CHIRP
    } else if (upper == "OFDM_COX" || upper == "COX") {
        value = 0x00;  // WaveformMode::OFDM_COX
    } else if (upper == "OTFS") {
        value = 0x01;  // WaveformMode::OTFS_EQ
    } else if (upper == "MFSK") {
        value = 0x03;  // WaveformMode::MFSK
    } else {
        return Response::error("Invalid waveform (use AUTO, MC_DPSK, OFDM_CHIRP, OFDM_COX, OTFS, MFSK)");
    }

    fprintf(stderr, "[HOST] Setting forced_waveform=%d (was %d)\n", value, settings_->forced_waveform);
    fflush(stderr);

    settings_->forced_waveform = value;
    auto_mode_ = (value == 0xFF);

    if (on_expert_settings_) {
        fprintf(stderr, "[HOST] Calling on_expert_settings_ callback\n");
        fflush(stderr);
        on_expert_settings_(settings_->forced_waveform,
                           settings_->forced_modulation,
                           settings_->forced_code_rate);
    } else {
        fprintf(stderr, "[HOST] WARNING: on_expert_settings_ callback is null!\n");
        fflush(stderr);
    }

    fprintf(stderr, "[HOST] === handleWaveform COMPLETE ===\n");
    fflush(stderr);
    return Response::ok();
}

Response HostInterface::handleModulation(const std::string& modulation) {
    if (!settings_) {
        return Response::error("Not initialized");
    }

    std::string upper = modulation;
    std::transform(upper.begin(), upper.end(), upper.begin(), ::toupper);

    // Modulation values match Modulation enum in types.hpp
    uint8_t value = 0xFF;
    if (upper == "AUTO") {
        value = 0xFF;
    } else if (upper == "DBPSK") {
        value = 0;  // Modulation::DBPSK
    } else if (upper == "BPSK") {
        value = 1;  // Modulation::BPSK
    } else if (upper == "DQPSK") {
        value = 2;  // Modulation::DQPSK
    } else if (upper == "QPSK") {
        value = 3;  // Modulation::QPSK
    } else if (upper == "D8PSK" || upper == "8PSK") {
        value = 4;  // Modulation::D8PSK
    } else if (upper == "QAM8") {
        value = 5;  // Modulation::QAM8
    } else if (upper == "QAM16" || upper == "16QAM") {
        value = 6;  // Modulation::QAM16
    } else if (upper == "QAM32" || upper == "32QAM") {
        value = 7;  // Modulation::QAM32
    } else if (upper == "QAM64" || upper == "64QAM") {
        value = 8;  // Modulation::QAM64
    } else if (upper == "QAM256" || upper == "256QAM") {
        value = 10; // Modulation::QAM256
    } else {
        return Response::error("Invalid modulation (use AUTO, DQPSK, QPSK, QAM16, QAM32, QAM64)");
    }

    settings_->forced_modulation = value;
    auto_mode_ = (value == 0xFF && settings_->forced_waveform == 0xFF &&
                  settings_->forced_code_rate == 0xFF);

    if (on_expert_settings_) {
        on_expert_settings_(settings_->forced_waveform,
                           settings_->forced_modulation,
                           settings_->forced_code_rate);
    }

    return Response::ok();
}

Response HostInterface::handleCodeRate(const std::string& rate) {
    if (!settings_) {
        return Response::error("Not initialized");
    }

    std::string upper = rate;
    std::transform(upper.begin(), upper.end(), upper.begin(), ::toupper);

    // CodeRate values match CodeRate enum in types.hpp
    uint8_t value = 0xFF;
    if (upper == "AUTO") {
        value = 0xFF;
    } else if (upper == "R1_4" || upper == "R1/4" || upper == "1/4") {
        value = 0;  // CodeRate::R1_4
    } else if (upper == "R1_3" || upper == "R1/3" || upper == "1/3") {
        value = 1;  // CodeRate::R1_3
    } else if (upper == "R1_2" || upper == "R1/2" || upper == "1/2") {
        value = 2;  // CodeRate::R1_2
    } else if (upper == "R2_3" || upper == "R2/3" || upper == "2/3") {
        value = 3;  // CodeRate::R2_3
    } else if (upper == "R3_4" || upper == "R3/4" || upper == "3/4") {
        value = 4;  // CodeRate::R3_4
    } else if (upper == "R5_6" || upper == "R5/6" || upper == "5/6") {
        value = 5;  // CodeRate::R5_6
    } else if (upper == "R7_8" || upper == "R7/8" || upper == "7/8") {
        value = 6;  // CodeRate::R7_8
    } else {
        return Response::error("Invalid code rate (use AUTO, R1/4, R1/2, R2/3, R3/4, R5/6, R7/8)");
    }

    settings_->forced_code_rate = value;
    auto_mode_ = (value == 0xFF && settings_->forced_waveform == 0xFF &&
                  settings_->forced_modulation == 0xFF);

    if (on_expert_settings_) {
        on_expert_settings_(settings_->forced_waveform,
                           settings_->forced_modulation,
                           settings_->forced_code_rate);
    }

    return Response::ok();
}

Response HostInterface::handleMCDPSKCarriers(int num_carriers) {
    if (!modem_) {
        return Response::error("Modem not initialized");
    }

    // Validate carrier count (3-20 supported by MC-DPSK)
    if (num_carriers < 3 || num_carriers > 20) {
        return Response::error("Invalid carrier count (use 3-20, typical: 5, 8, 10)");
    }

    modem_->setMCDPSKCarriers(num_carriers);
    return Response::ok();
}

// Status commands

Response HostInterface::handleVersion() {
    return Response::version(ULTRA_VERSION);
}

Response HostInterface::handleCodec() {
    // Build codec description based on actual waveform mode
    std::string codec = "Ultra ";

    // Get waveform mode - use negotiated if connected, otherwise use forced setting
    if (protocol_) {
        auto state = protocol_->getState();
        if (state == protocol::ConnectionState::CONNECTED ||
            state == protocol::ConnectionState::PROBING ||
            state == protocol::ConnectionState::CONNECTING) {
            auto mode = protocol_->getNegotiatedMode();
            codec += protocol::waveformModeToString(mode);
        } else if (settings_ && settings_->forced_waveform != 0xFF) {
            // Show forced waveform when idle
            auto mode = static_cast<protocol::WaveformMode>(settings_->forced_waveform);
            codec += protocol::waveformModeToString(mode);
        } else {
            codec += "AUTO";
        }
    } else if (settings_ && settings_->forced_waveform != 0xFF) {
        auto mode = static_cast<protocol::WaveformMode>(settings_->forced_waveform);
        codec += protocol::waveformModeToString(mode);
    } else {
        codec += "AUTO";
    }

    // Standard modem bandwidth
    codec += " 2300 Hz, LDPC FEC";

    return Response::custom(codec);
}

Response HostInterface::handlePttState() {
    return Response::ptt(ptt_active_);
}

Response HostInterface::handleBusy() {
    return Response::busy(busy_detected_);
}

Response HostInterface::handleBuffer() {
    return Response::buffer(getTxBufferSize());
}

// Control commands

Response HostInterface::handleTune(bool enabled) {
    if (on_tune_request_) {
        on_tune_request_(enabled);
    }
    return Response::ok();
}

Response HostInterface::handleCwId(const std::string& callsign) {
    if (callsign.empty()) {
        // Query current CWID
        if (settings_) {
            return Response::custom(std::string("CWID ") + settings_->callsign);
        }
        return Response::custom("CWID");
    }
    // Set CWID - not implemented yet
    return Response::ok();
}

Response HostInterface::handleClose() {
    return Response::close();
}

Response HostInterface::handleState() {
    // Return current protocol state
    if (protocol_) {
        auto state = protocol_->getState();
        switch (state) {
            case protocol::ConnectionState::DISCONNECTED:
                return Response::state("IDLE");
            case protocol::ConnectionState::PROBING:
                return Response::state("PROBING");
            case protocol::ConnectionState::CONNECTING:
                return Response::state("CONNECTING");
            case protocol::ConnectionState::CONNECTED:
                return Response::state("CONNECTED");
            case protocol::ConnectionState::DISCONNECTING:
                return Response::state("DISCONNECTING");
            default:
                return Response::state("IDLE");
        }
    }
    return Response::state("IDLE");
}

Response HostInterface::handlePttLead(uint32_t ms) {
    if (settings_) {
        settings_->tx_delay_ms = static_cast<int>(ms);
        return Response::ok();
    }
    return Response::error("Not initialized");
}

Response HostInterface::handlePttTail(uint32_t ms) {
    if (settings_) {
        settings_->tx_tail_ms = static_cast<int>(ms);
        return Response::ok();
    }
    return Response::error("Not initialized");
}

Response HostInterface::handleTxDrive(float level) {
    if (settings_) {
        // Clamp to valid range
        if (level < 0.0f) level = 0.0f;
        if (level > 1.0f) level = 1.0f;
        settings_->tx_drive = level;
        return Response::ok();
    }
    return Response::error("Not initialized");
}

Response HostInterface::handleEncrypt(bool enabled) {
    if (settings_) {
        settings_->encryption_enabled = enabled;
        LOG_MODEM(INFO, "HostInterface: Encryption %s", enabled ? "enabled" : "disabled");

        // Also update protocol engine if bound
        if (protocol_) {
            protocol_->setEncryptionEnabled(enabled);
        }

        // Note: Encryption key must also be set for encryption to work
        if (enabled && settings_->encryption_key[0] == '\0') {
            return Response::custom("OK (warning: no key set - use ENCRYPTKEY)");
        }
        return Response::ok();
    }
    return Response::error("Not initialized");
}

Response HostInterface::handleEncryptKey(const std::string& key) {
    if (settings_) {
        if (key.empty()) {
            // Clear key
            settings_->encryption_key[0] = '\0';
            LOG_MODEM(INFO, "HostInterface: Encryption key cleared");

            // Also clear from protocol engine
            if (protocol_) {
                protocol_->clearEncryptionKey();
            }
        } else {
            // Set key
            strncpy(settings_->encryption_key, key.c_str(),
                    sizeof(settings_->encryption_key) - 1);
            settings_->encryption_key[sizeof(settings_->encryption_key) - 1] = '\0';
            LOG_MODEM(INFO, "HostInterface: Encryption key set (%zu chars)", key.length());

            // Also set in protocol engine
            if (protocol_) {
                protocol_->setEncryptionKey(key);
            }
        }
        return Response::ok();
    }
    return Response::error("Not initialized");
}

Response HostInterface::handleSendFile(const std::string& filepath) {
    if (filepath.empty()) {
        return Response::error("No filepath specified");
    }

    if (!protocol_) {
        return Response::error("Not initialized");
    }

    if (protocol_->getState() != protocol::ConnectionState::CONNECTED) {
        return Response::error("Not connected");
    }

    // Try to send the file
    if (protocol_->sendFile(filepath)) {
        LOG_MODEM(INFO, "HostInterface: File transfer started: %s", filepath.c_str());
        return Response::ok();
    } else {
        return Response::error("Failed to start file transfer");
    }
}

// Beacon/CQ command handlers

Response HostInterface::handleBeacon() {
    if (!protocol_) {
        return Response::error("Not initialized");
    }

    // Check state - only allow when IDLE or DISCONNECTED
    auto state = protocol_->getState();
    if (state != protocol::ConnectionState::DISCONNECTED) {
        // Also check for PROBING/CONNECTING which should block
        if (state == protocol::ConnectionState::PROBING ||
            state == protocol::ConnectionState::CONNECTING ||
            state == protocol::ConnectionState::CONNECTED) {
            return Response::error("Cannot beacon while connected or connecting");
        }
    }

    // Set beacon pending flag - data will be read from data port
    beacon_pending_ = true;
    cq_pending_ = false;
    ping_pending_ = false;
    rawtx_pending_ = false;
    ping_target_.clear();
    rawtx_waveform_ = 0xFF;
    rawtx_modulation_ = 0xFF;
    rawtx_code_rate_ = 0xFF;
    pending_control_payload_.clear();
    pending_control_last_rx_ = {};
    beacon_pending_since_ = std::chrono::steady_clock::now();

    // Call the callback to notify app/protocol
    if (on_beacon_request_) {
        on_beacon_request_();
        LOG_MODEM(INFO, "HostInterface: Beacon TX requested");
        return Response::pending();
    }

    return Response::error("Beacon not configured");
}

Response HostInterface::handleCQ() {
    if (!protocol_) {
        return Response::error("Not initialized");
    }

    // Check state - only allow when IDLE or DISCONNECTED
    auto state = protocol_->getState();
    if (state != protocol::ConnectionState::DISCONNECTED) {
        // Also check for PROBING/CONNECTING which should block
        if (state == protocol::ConnectionState::PROBING ||
            state == protocol::ConnectionState::CONNECTING ||
            state == protocol::ConnectionState::CONNECTED) {
            return Response::error("Cannot CQ while connected or connecting");
        }
    }

    // Set CQ pending flag - data will be read from data port
    cq_pending_ = true;
    beacon_pending_ = false;
    ping_pending_ = false;
    rawtx_pending_ = false;
    ping_target_.clear();
    rawtx_waveform_ = 0xFF;
    rawtx_modulation_ = 0xFF;
    rawtx_code_rate_ = 0xFF;
    pending_control_payload_.clear();
    pending_control_last_rx_ = {};
    cq_pending_since_ = std::chrono::steady_clock::now();

    // Call the callback to notify app/protocol
    if (on_cq_request_) {
        on_cq_request_();
        LOG_MODEM(INFO, "HostInterface: CQ TX requested");
        return Response::pending();
    }

    return Response::error("CQ not configured");
}

Response HostInterface::handlePing(const std::string& callsign) {
    if (!protocol_) {
        return Response::error("Not initialized");
    }

    if (callsign.empty()) {
        return Response::error("No callsign specified");
    }

    // Check state - only allow when IDLE or DISCONNECTED
    auto state = protocol_->getState();
    if (state == protocol::ConnectionState::PROBING ||
        state == protocol::ConnectionState::CONNECTING ||
        state == protocol::ConnectionState::CONNECTED) {
        return Response::error("Cannot ping while connected or connecting");
    }

    // Set ping pending flag and store target callsign
    ping_pending_ = true;
    ping_target_ = callsign;
    beacon_pending_ = false;
    cq_pending_ = false;
    rawtx_pending_ = false;
    rawtx_waveform_ = 0xFF;
    rawtx_modulation_ = 0xFF;
    rawtx_code_rate_ = 0xFF;
    pending_control_payload_.clear();
    pending_control_last_rx_ = {};
    ping_pending_since_ = std::chrono::steady_clock::now();

    // Call the callback to notify app/protocol
    if (on_ping_request_) {
        on_ping_request_(callsign);
        LOG_MODEM(INFO, "HostInterface: PING TX requested to %s", callsign.c_str());
        return Response::pending();
    }

    return Response::error("Ping not configured");
}

Response HostInterface::handleRawTx(const std::string& args) {
    if (!on_raw_tx_request_) {
        return Response::error("RAWTX not configured");
    }

    if (protocol_) {
        auto state = protocol_->getState();
        if (state != protocol::ConnectionState::DISCONNECTED) {
            return Response::error("Cannot RAWTX while connected or connecting");
        }
    }

    uint8_t waveform = settings_ ? settings_->forced_waveform : 0xFF;
    uint8_t modulation = settings_ ? settings_->forced_modulation : 0xFF;
    uint8_t code_rate = settings_ ? settings_->forced_code_rate : 0xFF;

    std::vector<std::string> tokens;
    std::istringstream iss(args);
    std::string token;
    while (iss >> token) {
        tokens.push_back(token);
    }

    if (tokens.size() > 3) {
        return Response::error("Usage: RAWTX [waveform] [modulation] [coderate]");
    }

    if (!tokens.empty() && !parseWaveformToken(tokens[0], waveform)) {
        return Response::error("Invalid RAWTX waveform");
    }
    if (tokens.size() > 1 && !parseModulationToken(tokens[1], modulation)) {
        return Response::error("Invalid RAWTX modulation");
    }
    if (tokens.size() > 2 && !parseCodeRateToken(tokens[2], code_rate)) {
        return Response::error("Invalid RAWTX coderate");
    }

    if (waveform == 0xFF) {
        waveform = static_cast<uint8_t>(protocol::WaveformMode::MC_DPSK);
    }

    switch (static_cast<protocol::WaveformMode>(waveform)) {
        case protocol::WaveformMode::MC_DPSK:
        case protocol::WaveformMode::OFDM_CHIRP:
        case protocol::WaveformMode::OFDM_COX:
        case protocol::WaveformMode::OTFS_EQ:
        case protocol::WaveformMode::OTFS_RAW:
        case protocol::WaveformMode::MFSK:
            break;
        default:
            waveform = static_cast<uint8_t>(protocol::WaveformMode::MC_DPSK);
            break;
    }

    const bool use_mc_dpsk = (waveform == static_cast<uint8_t>(protocol::WaveformMode::MC_DPSK));
    if (modulation == 0xFF) {
        modulation = static_cast<uint8_t>(use_mc_dpsk ? Modulation::DBPSK : Modulation::DQPSK);
    }
    if (code_rate == 0xFF) {
        code_rate = static_cast<uint8_t>(use_mc_dpsk ? CodeRate::R1_4 : CodeRate::R1_2);
    }

    switch (static_cast<Modulation>(modulation)) {
        case Modulation::DBPSK:
        case Modulation::BPSK:
        case Modulation::DQPSK:
        case Modulation::QPSK:
        case Modulation::D8PSK:
        case Modulation::QAM8:
        case Modulation::QAM16:
        case Modulation::QAM32:
        case Modulation::QAM64:
        case Modulation::QAM256:
            break;
        default:
            modulation = static_cast<uint8_t>(use_mc_dpsk ? Modulation::DBPSK : Modulation::DQPSK);
            break;
    }

    switch (static_cast<CodeRate>(code_rate)) {
        case CodeRate::R1_4:
        case CodeRate::R1_3:
        case CodeRate::R1_2:
        case CodeRate::R2_3:
        case CodeRate::R3_4:
        case CodeRate::R5_6:
        case CodeRate::R7_8:
            break;
        default:
            code_rate = static_cast<uint8_t>(use_mc_dpsk ? CodeRate::R1_4 : CodeRate::R1_2);
            break;
    }

    rawtx_pending_ = true;
    rawtx_waveform_ = waveform;
    rawtx_modulation_ = modulation;
    rawtx_code_rate_ = code_rate;

    beacon_pending_ = false;
    cq_pending_ = false;
    ping_pending_ = false;
    ping_target_.clear();

    pending_control_payload_.clear();
    pending_control_last_rx_ = {};
    rawtx_pending_since_ = std::chrono::steady_clock::now();

    LOG_MODEM(INFO, "HostInterface: RAWTX requested (wf=%u mod=%u rate=%u)",
              static_cast<unsigned>(rawtx_waveform_),
              static_cast<unsigned>(rawtx_modulation_),
              static_cast<unsigned>(rawtx_code_rate_));

    return Response::pending();
}

// Data port processing

void HostInterface::processDataPort() {
    if (!server_) return;

    // Read data from data port and queue for transmission
    auto data = server_->readData();
    if (data && !data->empty()) {
        queueTxData(*data);
    }

    // Drain TX buffer to protocol (batches multiple reads into one send)
    std::vector<uint8_t> to_send;
    {
        std::lock_guard<std::mutex> lock(tx_mutex_);
        if (!tx_buffer_.empty()) {
            to_send = std::move(tx_buffer_);
            tx_buffer_.clear();
        }
    }

    // Assemble pending control payload robustly across TCP fragmentation.
    const bool control_pending = beacon_pending_ || cq_pending_ || ping_pending_ || rawtx_pending_;
    const auto now = std::chrono::steady_clock::now();
    if (control_pending) {
        const bool pending_ping = ping_pending_;
        const bool pending_rawtx = rawtx_pending_;
        const size_t max_payload = pending_rawtx ? RAWTX_PAYLOAD_MAX :
                                   (pending_ping ? CONTROL_PAYLOAD_MAX : BROADCAST_PAYLOAD_MAX);
        if (!to_send.empty()) {
            size_t remaining = (pending_control_payload_.size() < max_payload)
                ? (max_payload - pending_control_payload_.size())
                : 0;
            size_t take = std::min(remaining, to_send.size());
            if (take > 0) {
                pending_control_payload_.insert(
                    pending_control_payload_.end(), to_send.begin(), to_send.begin() + take);
            }
            if (to_send.size() > take) {
                const char* label = pending_rawtx ? "RAWTX" : (pending_ping ? "PING" : "broadcast");
                LOG_MODEM(INFO, "HostInterface: %s payload truncated to %zu bytes (dropped %zu bytes)",
                          label, max_payload, to_send.size() - take);
            }
            pending_control_last_rx_ = now;
        }

        std::chrono::steady_clock::time_point pending_since = now;
        if (beacon_pending_) {
            pending_since = beacon_pending_since_;
        } else if (cq_pending_) {
            pending_since = cq_pending_since_;
        } else if (ping_pending_) {
            pending_since = ping_pending_since_;
        } else {
            pending_since = rawtx_pending_since_;
        }

        auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - pending_since).count();
        bool has_payload = !pending_control_payload_.empty();
        bool payload_full = pending_control_payload_.size() >= max_payload;
        bool gap_elapsed = false;
        if (has_payload && pending_control_last_rx_.time_since_epoch().count() != 0) {
            auto gap_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                now - pending_control_last_rx_).count();
            gap_elapsed = gap_ms >= PENDING_PAYLOAD_GAP_MS;
        }
        bool timeout_elapsed = age_ms >= PENDING_PAYLOAD_TIMEOUT_MS;
        bool should_transmit = payload_full ||
                               (has_payload && (gap_elapsed || timeout_elapsed)) ||
                               (!has_payload && timeout_elapsed);

        if (should_transmit && protocol_) {
            std::vector<uint8_t> payload = pending_control_payload_;
            bool was_beacon = beacon_pending_;
            bool was_cq = cq_pending_;
            bool was_ping = ping_pending_;
            bool was_rawtx = rawtx_pending_;
            std::string ping_target = ping_target_;
            uint8_t rawtx_waveform = rawtx_waveform_;
            uint8_t rawtx_modulation = rawtx_modulation_;
            uint8_t rawtx_code_rate = rawtx_code_rate_;

            beacon_pending_ = false;
            cq_pending_ = false;
            ping_pending_ = false;
            rawtx_pending_ = false;
            ping_target_.clear();
            rawtx_waveform_ = 0xFF;
            rawtx_modulation_ = 0xFF;
            rawtx_code_rate_ = 0xFF;
            pending_control_payload_.clear();
            pending_control_last_rx_ = {};

            if (was_beacon) {
                protocol_->transmitBeacon(payload);
                LOG_MODEM(INFO, "HostInterface: Transmitted BEACON with %zu bytes payload", payload.size());
            } else if (was_cq) {
                protocol_->transmitCQ(payload);
                LOG_MODEM(INFO, "HostInterface: Transmitted CQ with %zu bytes payload", payload.size());
            } else if (was_ping) {
                protocol_->transmitPing(ping_target, payload);
                LOG_MODEM(INFO, "HostInterface: Transmitted PING to %s with %zu bytes payload",
                          ping_target.c_str(), payload.size());
            } else if (was_rawtx) {
                on_raw_tx_request_(payload, rawtx_waveform, rawtx_modulation, rawtx_code_rate);
                LOG_MODEM(INFO, "HostInterface: Transmitted RAWTX with %zu bytes payload (wf=%u mod=%u rate=%u)",
                          payload.size(),
                          static_cast<unsigned>(rawtx_waveform),
                          static_cast<unsigned>(rawtx_modulation),
                          static_cast<unsigned>(rawtx_code_rate));
            }
        }
    } else if (!to_send.empty() && on_send_data_) {
        // Normal data transmission (when connected)
        on_send_data_(to_send);
    }

    // Drain buffered RX payloads to connected data client.
    if (server_->hasDataClient()) {
        while (true) {
            std::vector<uint8_t> payload;
            {
                std::lock_guard<std::mutex> lock(rx_mutex_);
                if (pending_rx_messages_.empty()) {
                    break;
                }
                payload = std::move(pending_rx_messages_.front());
                pending_rx_messages_.pop_front();
                pending_rx_bytes_ = safeSub(pending_rx_bytes_, payload.size());
            }

            if (!server_->sendData(payload.data(), payload.size())) {
                // Data client disappeared while flushing; put payload back.
                std::lock_guard<std::mutex> lock(rx_mutex_);
                pending_rx_messages_.push_front(std::move(payload));
                pending_rx_bytes_ += pending_rx_messages_.front().size();
                break;
            }

            LOG_MODEM(INFO, "HostInterface: Flushed buffered RX payload (%zu bytes, remaining=%zu msgs/%zu bytes)",
                      payload.size(), pending_rx_messages_.size(), pending_rx_bytes_);
        }
    }
}

// KISS processing

void HostInterface::processKissFrames() {
    if (!kiss_server_) return;

    // Read data from KISS clients
    auto data = kiss_server_->readData();
    if (data && !data->empty()) {
        auto frames = kiss_framer_.process(data->data(), data->size());

        for (const auto& frame : frames) {
            if (frame.command == KissCommand::DataFrame) {
                // Queue data for transmission
                queueTxData(frame.data);
                if (on_send_data_) {
                    on_send_data_(frame.data);
                }
            } else {
                // Handle KISS configuration command
                if (!frame.data.empty()) {
                    kiss_config_.handleCommand(frame.command, frame.data[0]);
                }
            }
        }
    }
}

// CAT Control

void HostInterface::bindCat(cat::CatController* cat) {
    cat_ = cat;
}

Response HostInterface::handleCatEnable(bool enabled) {
    if (!settings_) {
        return Response::error("Not initialized");
    }

    settings_->cat_enabled = enabled;
    LOG_MODEM(INFO, "HostInterface: CAT control %s", enabled ? "enabled" : "disabled");

    // Reconfigure controller if bound
    if (cat_) {
        cat::CatConfig config;
        config.enabled = settings_->cat_enabled;
        config.backend_type = static_cast<cat::CatBackendType>(settings_->cat_backend);
        config.serial_port = settings_->cat_port;
        config.serial_baud = settings_->cat_baud;
        config.ptt_line = settings_->cat_ptt_line;
        config.ptt_invert = settings_->cat_ptt_invert;
        config.hamlib_model = settings_->cat_hamlib_model;
        config.tcp_host = settings_->cat_tcp_host;
        config.tcp_port = settings_->cat_tcp_port;
        config.flex_slice = settings_->cat_flex_slice;
        config.ptt_lead_ms = settings_->cat_ptt_lead_ms;
        config.ptt_tail_ms = settings_->cat_ptt_tail_ms;
        config.watchdog_seconds = settings_->cat_watchdog_seconds;
        cat_->configure(config);
    }

    return Response::ok();
}

Response HostInterface::handleCatBackend(const std::string& backend) {
    if (!settings_) {
        return Response::error("Not initialized");
    }

    auto type = cat::stringToBackendType(backend);
    settings_->cat_backend = static_cast<int>(type);
    LOG_MODEM(INFO, "HostInterface: CAT backend set to %s", cat::backendTypeToString(type));

    return Response::ok();
}

Response HostInterface::handleCatModel(int model) {
    if (!settings_) {
        return Response::error("Not initialized");
    }

    settings_->cat_hamlib_model = model;
    LOG_MODEM(INFO, "HostInterface: CAT Hamlib model set to %d", model);

    return Response::ok();
}

Response HostInterface::handleCatPort(const std::string& port) {
    if (!settings_) {
        return Response::error("Not initialized");
    }

    strncpy(settings_->cat_port, port.c_str(), sizeof(settings_->cat_port) - 1);
    settings_->cat_port[sizeof(settings_->cat_port) - 1] = '\0';

    // Check if it looks like host:port for TCP
    size_t colon = port.find(':');
    if (colon != std::string::npos) {
        std::string host = port.substr(0, colon);
        std::string port_str = port.substr(colon + 1);
        strncpy(settings_->cat_tcp_host, host.c_str(), sizeof(settings_->cat_tcp_host) - 1);
        settings_->cat_tcp_host[sizeof(settings_->cat_tcp_host) - 1] = '\0';
        try {
            settings_->cat_tcp_port = static_cast<uint16_t>(std::stoi(port_str));
        } catch (...) {}
    }

    LOG_MODEM(INFO, "HostInterface: CAT port set to %s", port.c_str());

    return Response::ok();
}

Response HostInterface::handleCatBaud(int baud) {
    if (!settings_) {
        return Response::error("Not initialized");
    }

    settings_->cat_baud = baud;
    LOG_MODEM(INFO, "HostInterface: CAT baud rate set to %d", baud);

    return Response::ok();
}

Response HostInterface::handleCatSlice(int slice) {
    if (!settings_) {
        return Response::error("Not initialized");
    }

    if (slice < 0 || slice > 7) {
        return Response::error("Invalid slice (use 0-7)");
    }

    settings_->cat_flex_slice = slice;
    LOG_MODEM(INFO, "HostInterface: CAT Flex slice set to %d", slice);

    return Response::ok();
}

Response HostInterface::handleCatConnect() {
    if (!cat_) {
        return Response::error("CAT not initialized");
    }

    if (cat_->connect()) {
        return Response::ok();
    } else {
        return Response::error("CAT connect failed: " + cat_->getLastError());
    }
}

Response HostInterface::handleCatDisconnect() {
    if (!cat_) {
        return Response::error("CAT not initialized");
    }

    cat_->disconnect();
    return Response::ok();
}

Response HostInterface::handleCatPtt(bool active) {
    if (!cat_) {
        return Response::error("CAT not initialized");
    }

    if (active) {
        if (cat_->assertPtt("TCP_CATPTT")) {
            return Response::ok();
        } else {
            return Response::error("CAT PTT failed: " + cat_->getLastError());
        }
    } else {
        cat_->releasePttImmediate("TCP_CATPTT");
        return Response::ok();
    }
}

Response HostInterface::handleCatFreq(uint64_t hz) {
    if (!cat_) {
        return Response::error("CAT not initialized");
    }

    if (!cat_->supportsFrequency()) {
        return Response::error("Backend does not support frequency control");
    }

    if (cat_->setFrequency(hz)) {
        return Response::ok();
    } else {
        return Response::error("Set frequency failed: " + cat_->getLastError());
    }
}

Response HostInterface::handleCatGetFreq() {
    if (!cat_) {
        return Response::error("CAT not initialized");
    }

    if (!cat_->supportsFrequency()) {
        return Response::error("Backend does not support frequency control");
    }

    uint64_t freq = cat_->getFrequency();
    return Response::custom("FREQ " + std::to_string(freq));
}

Response HostInterface::handleCatMode(const std::string& mode) {
    if (!cat_) {
        return Response::error("CAT not initialized");
    }

    if (!cat_->supportsMode()) {
        return Response::error("Backend does not support mode control");
    }

    auto radio_mode = cat::stringToRadioMode(mode);
    if (radio_mode == cat::RadioMode::UNKNOWN) {
        return Response::error("Invalid mode (use USB, LSB, AM, FM, CW, DATA)");
    }

    if (cat_->setMode(radio_mode)) {
        return Response::ok();
    } else {
        return Response::error("Set mode failed: " + cat_->getLastError());
    }
}

Response HostInterface::handleCatGetMode() {
    if (!cat_) {
        return Response::error("CAT not initialized");
    }

    if (!cat_->supportsMode()) {
        return Response::error("Backend does not support mode control");
    }

    auto mode = cat_->getMode();
    return Response::custom("MODE " + std::string(cat::radioModeToString(mode)));
}

Response HostInterface::handleCatWatchdog(int seconds) {
    if (!settings_) {
        return Response::error("Not initialized");
    }

    if (seconds < 0 || seconds > 600) {
        return Response::error("Invalid watchdog timeout (use 0-600 seconds)");
    }

    settings_->cat_watchdog_seconds = seconds;
    LOG_MODEM(INFO, "HostInterface: CAT watchdog set to %d seconds", seconds);

    return Response::ok();
}

Response HostInterface::handleCatPttLead(int ms) {
    if (!settings_) {
        return Response::error("Not initialized");
    }

    if (ms < 0 || ms > 1000) {
        return Response::error("Invalid PTT lead delay (use 0-1000 ms)");
    }

    settings_->cat_ptt_lead_ms = ms;
    LOG_MODEM(INFO, "HostInterface: CAT PTT lead delay set to %d ms", ms);

    return Response::ok();
}

Response HostInterface::handleCatPttTail(int ms) {
    if (!settings_) {
        return Response::error("Not initialized");
    }

    if (ms < 0 || ms > 1000) {
        return Response::error("Invalid PTT tail delay (use 0-1000 ms)");
    }

    settings_->cat_ptt_tail_ms = ms;
    LOG_MODEM(INFO, "HostInterface: CAT PTT tail delay set to %d ms", ms);

    return Response::ok();
}

Response HostInterface::handleCatStatus() {
    if (!cat_) {
        return Response::error("CAT not initialized");
    }

    auto status = cat_->getStatus();
    std::string result = "CAT ";
    result += status.connected ? "CONNECTED" : "DISCONNECTED";
    result += " PTT=";
    result += status.ptt_active ? "ON" : "OFF";

    if (cat_->supportsFrequency() && status.frequency_hz > 0) {
        result += " FREQ=" + std::to_string(status.frequency_hz);
    }

    if (cat_->supportsMode() && status.mode != cat::RadioMode::UNKNOWN) {
        result += " MODE=";
        result += cat::radioModeToString(status.mode);
    }

    int watchdog = cat_->getWatchdogRemaining();
    if (watchdog >= 0) {
        result += " WATCHDOG=" + std::to_string(watchdog) + "s";
    }

    if (!status.error_message.empty()) {
        result += " ERROR=\"" + status.error_message + "\"";
    }

    return Response::custom(result);
}

} // namespace interface
} // namespace ultra
