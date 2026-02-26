#include "app.hpp"
#include "startup_trace.hpp"
#include "imgui.h"
#include "ultra/logging.hpp"
#include "sim/hf_channel.hpp"
#include "protocol/frame_v2.hpp"
#include "cat/cat_backend.hpp"
#include <SDL.h>
#include <cstring>
#include <cmath>
#include <fstream>
#include <cstdarg>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <limits>
#include <deque>
#include <vector>
#include <sys/stat.h>

#ifdef _WIN32
#include <direct.h>
#define MKDIR(dir) _mkdir(dir)
#else
#define MKDIR(dir) mkdir(dir, 0755)
#endif

namespace ultra {
namespace gui {

// File logger for GUI debugging - writes to logs/gui.log next to binary
// ALL logging (including modem, protocol, etc.) goes to this file
static FILE* g_gui_log_file = nullptr;
static bool g_log_initialized = false;
static std::string g_gui_log_path;

static void initLog() {
    if (g_log_initialized) return;
    g_log_initialized = true;

#ifdef _WIN32
    auto tryOpenLog = [](const char* path) -> FILE* {
        if (!path || path[0] == '\0') {
            return nullptr;
        }

        const char* slash_back = std::strrchr(path, '\\');
        const char* slash_fwd = std::strrchr(path, '/');
        const char* sep = slash_back;
        if (!sep || (slash_fwd && slash_fwd > sep)) {
            sep = slash_fwd;
        }
        if (sep) {
            std::string dir(path, static_cast<size_t>(sep - path));
            if (!dir.empty()) {
                MKDIR(dir.c_str());
            }
        }

        return std::fopen(path, "w");
    };

    if (!g_gui_log_file) {
        g_gui_log_file = tryOpenLog("logs\\gui.log");
        if (g_gui_log_file) {
            g_gui_log_path = "logs\\gui.log";
        }
    }

    if (!g_gui_log_file) {
        g_gui_log_file = tryOpenLog("gui.log");
        if (g_gui_log_file) {
            g_gui_log_path = "gui.log";
        }
    }

    if (!g_gui_log_file) {
        const char* temp = std::getenv("TEMP");
        if (temp && temp[0] != '\0') {
            std::string temp_path = std::string(temp) + "\\RIAModem\\gui.log";
            g_gui_log_file = tryOpenLog(temp_path.c_str());
            if (g_gui_log_file) {
                g_gui_log_path = temp_path;
            }
        }
    }
#else
    auto tryOpenLog = [](const std::filesystem::path& path) -> FILE* {
        std::error_code ec;
        if (!path.parent_path().empty()) {
            std::filesystem::create_directories(path.parent_path(), ec);
        }
        return fopen(path.string().c_str(), "w");
    };

    std::vector<std::filesystem::path> candidates;
    candidates.emplace_back(std::filesystem::path("logs") / "gui.log");
    candidates.emplace_back("gui.log");
    if (const char* temp = std::getenv("TMPDIR")) {
        candidates.emplace_back(std::filesystem::path(temp) / "ria_modem_gui.log");
    }
    candidates.emplace_back("/tmp/ria_modem_gui.log");

    for (const auto& path : candidates) {
        g_gui_log_file = tryOpenLog(path);
        if (g_gui_log_file) {
            g_gui_log_path = path.string();
            break;
        }
    }
#endif

    if (g_gui_log_file) {
        // Redirect ALL logging (modem, protocol, etc.) to this file
        ultra::setLogFile(g_gui_log_file);
        ultra::log(ultra::LogLevel::INFO, "GUI", "File logger initialized: %s",
                   g_gui_log_path.c_str());
    }
}

static void guiLog(const char* fmt, ...) {
    initLog();
    if (!g_gui_log_file) return;

    // Use the same timestamp format as the global logger
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - ultra::g_log_start_time).count();
    int secs = static_cast<int>(elapsed / 1000);
    int ms = static_cast<int>(elapsed % 1000);

    // Format message
    char buf[1024];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    fprintf(g_gui_log_file, "[%3d.%03d][INFO ][GUI  ] %s\n", secs, ms, buf);
    fflush(g_gui_log_file);
}

static std::string trimF32Extension(const std::string& path) {
    if (path.size() >= 4 && path.substr(path.size() - 4) == ".f32") {
        return path.substr(0, path.size() - 4);
    }
    return path;
}

static bool writeF32File(const std::string& path, const std::vector<float>& samples) {
    if (samples.empty()) {
        return false;
    }
    std::ofstream file(path, std::ios::binary);
    if (!file.is_open()) {
        return false;
    }
    file.write(reinterpret_cast<const char*>(samples.data()), samples.size() * sizeof(float));
    return file.good();
}

// Convert fading index to channel quality string
// Thresholds aligned with waveform_selection.hpp (2026-02-03)
// Combined index = freq_cv + temporal_cv (includes Doppler spread measurement)
// Calibrated: AWGN ~0.04, Good ~0.62, Moderate ~0.90, Poor ~0.82
static const char* fadingToQuality(float fading) {
    if (fading < 0.15f) return "AWGN";
    if (fading < 0.65f) return "Good";
    if (fading < 1.10f) return "Moderate";
    return "Poor";
}

// Same as above but also sets color for GUI display
static const char* fadingToQualityWithColor(float fading, ImVec4& color) {
    if (fading < 0.15f) {
        color = ImVec4(0.0f, 1.0f, 0.5f, 1.0f);  // Cyan
        return "AWGN";
    } else if (fading < 0.65f) {
        color = ImVec4(0.2f, 1.0f, 0.2f, 1.0f);  // Green
        return "Good";
    } else if (fading < 1.10f) {
        color = ImVec4(0.8f, 0.8f, 0.0f, 1.0f);  // Yellow
        return "Moderate";
    } else {
        color = ImVec4(1.0f, 0.5f, 0.0f, 1.0f);  // Orange
        return "Poor";
    }
}

// User-friendly waveform name (hides internal variants like OFDM_COX/OFDM_CHIRP)
static const char* waveformDisplayName(protocol::WaveformMode mode) {
    switch (mode) {
        case protocol::WaveformMode::MC_DPSK: return "MC-DPSK";
        case protocol::WaveformMode::MFSK: return "MFSK";
        case protocol::WaveformMode::OTFS_EQ:
        case protocol::WaveformMode::OTFS_RAW: return "OTFS";
        case protocol::WaveformMode::OFDM_CHIRP:
        case protocol::WaveformMode::OFDM_COX:
        default: return "OFDM";
    }
}

static uint32_t safeFileSizeBytes(const std::string& path) {
    std::error_code ec;
    uintmax_t size = std::filesystem::file_size(path, ec);
    if (ec || size > static_cast<uintmax_t>(std::numeric_limits<uint32_t>::max())) {
        return 0;
    }
    return static_cast<uint32_t>(size);
}

template <size_t N>
static size_t boundedCStringLen(const char (&buf)[N]) {
    const void* term = std::memchr(buf, '\0', N);
    return term ? static_cast<size_t>(static_cast<const char*>(term) - buf) : N;
}

static float codeRateValue(CodeRate rate) {
    switch (rate) {
        case CodeRate::R1_4: return 0.25f;
        case CodeRate::R1_2: return 0.50f;
        case CodeRate::R2_3: return 2.0f / 3.0f;
        case CodeRate::R3_4: return 0.75f;
        default: return 0.25f;
    }
}

static float modulationBitsPerSymbol(Modulation mod) {
    switch (mod) {
        case Modulation::BPSK: return 1.0f;
        case Modulation::QPSK:
        case Modulation::DQPSK: return 2.0f;
        case Modulation::D8PSK:
        case Modulation::QAM8: return 3.0f;
        case Modulation::QAM16: return 4.0f;
        case Modulation::QAM32: return 5.0f;
        case Modulation::QAM64: return 6.0f;
        default: return 1.0f;
    }
}

static float modeEfficiency(Modulation mod, CodeRate rate) {
    return modulationBitsPerSymbol(mod) * codeRateValue(rate);
}

static const char* adaptationDirection(Modulation from_mod, CodeRate from_rate,
                                       Modulation to_mod, CodeRate to_rate) {
    float from_eff = modeEfficiency(from_mod, from_rate);
    float to_eff = modeEfficiency(to_mod, to_rate);
    if (to_eff > from_eff + 0.05f) return "improving";
    if (to_eff < from_eff - 0.05f) return "degrading";
    return "changing";
}

App::App() : App(Options{}) {}

App::App(const Options& opts) : options_(opts), sim_ui_visible_(opts.enable_sim) {
    ultra::gui::startupTrace("App", "ctor-body-enter");
    ultra::gui::startupTrace("App", "gui-log-enter");
    guiLog("=== GUI Started ===");
    ultra::gui::startupTrace("App", "gui-log-exit");
    if (options_.record_audio) {
        recording_enabled_ = true;
        guiLog("Recording enabled (-rec): base path '%s'", options_.record_path.c_str());
    }
    // Load persistent settings
    ultra::gui::startupTrace("App", "settings-load-enter");
    settings_.load(options_.config_path);
    last_forced_waveform_ = settings_.forced_waveform;
    if (!options_.config_path.empty()) {
        guiLog("Loaded config from: %s", options_.config_path.c_str());
    }
    ultra::gui::startupTrace("App", "settings-load-exit");

    ultra::gui::startupTrace("App", "presets-balanced-enter");
    config_ = presets::balanced();
    ultra::gui::startupTrace("App", "presets-balanced-exit");

    // Use dedicated RX decode thread by default.
    modem_.setSynchronousMode(false);

    if (!options_.disable_waterfall) {
        ultra::gui::startupTrace("App", "waterfall-create-begin");
        waterfall_ = std::make_unique<WaterfallWidget>();
        ultra::gui::startupTrace("App", "waterfall-create-end");
    } else {
        guiLog("Waterfall disabled by startup option");
    }

    // Initialize protocol with saved callsign
    ultra::gui::startupTrace("App", "callsign-init-enter");
    size_t local_call_len = boundedCStringLen(settings_.callsign);
    if (local_call_len > 0) {
        std::string local_call(settings_.callsign, local_call_len);
        ultra::gui::startupTrace("App", "callsign-set-protocol-enter");
        protocol_.setLocalCallsign(local_call);
        ultra::gui::startupTrace("App", "callsign-set-protocol-exit");
        ultra::gui::startupTrace("App", "callsign-set-modem-enter");
        modem_.setLogPrefix(local_call);
        ultra::gui::startupTrace("App", "callsign-set-modem-exit");
    }
    ultra::gui::startupTrace("App", "callsign-init-exit");

    // Set up raw data callback for protocol layer
    ultra::gui::startupTrace("App", "set-raw-callback-enter");
    modem_.setRawDataCallback([this](const Bytes& data) {
        guiLog("Our modem decoded %zu bytes", data.size());
        // Update protocol layer with current SNR and fading before processing frame
        // In simulation mode, use the known simulation SNR (DPSK doesn't measure SNR)
        // In real mode, use measured SNR from OFDM demodulator
        float snr_db = simulation_enabled_ ? simulation_snr_db_ : modem_.getStats().snr_db;
        float fading = modem_.getFadingIndex();
        protocol_.setMeasuredSNR(snr_db);
        protocol_.setChannelQuality(snr_db, fading);
        protocol_.onRxData(data);
        updateAdaptiveAdvisory(snr_db, fading);
    });
    ultra::gui::startupTrace("App", "set-raw-callback-exit");

    // Set up status callback to show codeword progress in RX log
    ultra::gui::startupTrace("App", "set-status-callback-enter");
    modem_.setStatusCallback([this](const std::string& status) {
        appendRxLogLine(status);
    });
    ultra::gui::startupTrace("App", "set-status-callback-exit");

    // Set up protocol engine callbacks
    ultra::gui::startupTrace("App", "protocol-callbacks-enter");
    protocol_.setTxDataCallback([this](const Bytes& data) {
        // When protocol layer wants to transmit, convert to audio
        auto samples = modem_.transmit(data);
        if (!samples.empty()) {
            if (simulation_enabled_) {
                // Add PTT noise once at start of transmission (100-300ms)
                std::uniform_int_distribution<size_t> ptt_dist(4800, 14400);
                size_t ptt_samples = ptt_dist(sim_rng_);

                float typical_rms = 0.1f;
                float snr_linear = std::pow(10.0f, simulation_snr_db_ / 10.0f);
                float noise_power = (typical_rms * typical_rms) / snr_linear;
                float noise_stddev = std::sqrt(noise_power);
                std::normal_distribution<float> noise_dist(0.0f, noise_stddev);

                std::vector<float> ptt_noise(ptt_samples);
                for (float& s : ptt_noise) s = noise_dist(sim_rng_);

                // Mark TX active (include PTT noise in duration)
                size_t total_samples = ptt_samples + samples.size();
                size_t tx_duration_ms = (total_samples * 1000) / 48000;
                tx_in_progress_ = true;
                tx_end_time_ = std::chrono::steady_clock::now() + std::chrono::milliseconds(tx_duration_ms + 100);

                // Queue PTT noise + signal for real-time streaming
                std::lock_guard<std::mutex> lock(our_tx_pending_mutex_);
                our_tx_pending_.insert(our_tx_pending_.end(), ptt_noise.begin(), ptt_noise.end());
                our_tx_pending_.insert(our_tx_pending_.end(), samples.begin(), samples.end());
                guiLog("SIM: Queued %zu TX samples (+ %zu PTT noise) for streaming", samples.size(), ptt_samples);
            } else {
                queueRealTxSamples(samples, "TX audio");
            }
        }
    });

    // Burst TX callback - encode multiple frames as single OFDM burst
    protocol_.setTransmitBurstCallback([this](const std::vector<Bytes>& frames) {
        auto samples = modem_.transmitBurst(frames);
        if (!samples.empty()) {
            if (simulation_enabled_) {
                size_t tx_duration_ms = (samples.size() * 1000) / 48000;
                tx_in_progress_ = true;
                tx_end_time_ = std::chrono::steady_clock::now() + std::chrono::milliseconds(tx_duration_ms + 100);
                std::lock_guard<std::mutex> lock(our_tx_pending_mutex_);
                our_tx_pending_.insert(our_tx_pending_.end(), samples.begin(), samples.end());
                guiLog("SIM: Queued burst of %zu frames (%zu samples)", frames.size(), samples.size());
            } else {
                queueRealTxSamples(samples, "TX burst audio");
            }
        }
    });
    ultra::gui::startupTrace("App", "protocol-callbacks-mid1");

    protocol_.setMessageReceivedCallback([this](const std::string& from, const std::string& text) {
        // Received a message via ARQ
        std::string msg = "[RX " + from + "] " + text;
        appendRxLogLine(msg);

        // Deliver to TCP data port clients
        if (host_interface_) {
            std::vector<uint8_t> data(text.begin(), text.end());
            host_interface_->deliverRxData(data);
        }
    });

    protocol_.setBeaconReceivedCallback([this](const std::string& from, uint32_t from_hash,
                                                const Bytes& payload) {
        // Received a beacon/CQ broadcast
        std::string from_str = from.empty() ?
            ("HASH:" + std::to_string(from_hash)) : from;
        std::string msg = "[BEACON " + from_str + "] payload=" + std::to_string(payload.size()) + " bytes";
        appendRxLogLine(msg);

        // Deliver to TCP data port clients with beacon marker
        if (host_interface_) {
            host_interface_->deliverBeaconData(from, from_hash, payload);
        }
    });
    ultra::gui::startupTrace("App", "protocol-callbacks-mid2");

    protocol_.setConnectionChangedCallback([this](protocol::ConnectionState state, const std::string& info) {
        guiLog("Connection state changed: %d (%s)", static_cast<int>(state), info.c_str());

        // Update modem engine connection state (affects waveform selection)
        // Stay "connected" during DISCONNECTING so we can receive the ACK via OFDM
        bool modem_connected = (state == protocol::ConnectionState::CONNECTED ||
                                state == protocol::ConnectionState::DISCONNECTING);
        modem_.setConnected(modem_connected);
        modem_.setMCDPSKChannelInterleave(
            modem_connected ? protocol_.isMCDPSKChannelInterleaveActive() : false);

        std::string msg;
        switch (state) {
            case protocol::ConnectionState::PROBING:
                resetAdaptiveAdvisory();
                msg = "[SYS] Probing " + info + "...";
                break;
            case protocol::ConnectionState::CONNECTING:
                resetAdaptiveAdvisory();
                // Cancel CQ listening state - got a response
                if (cq_listening_) {
                    cq_listening_ = false;
                    guiLog("CQ: Response received, entering connection handshake");
                }
                msg = "[SYS] Connecting to " + info + "...";
                break;
            case protocol::ConnectionState::CONNECTED:
                resetAdaptiveAdvisory();
                if (cq_listening_) {
                    cq_listening_ = false;
                    guiLog("CQ: Connected, clearing CQ listen timeout");
                }
                msg = "[SYS] Connected to " + info;  // info contains remote callsign
                flushTcpTxQueue();
                break;
            case protocol::ConnectionState::DISCONNECTING:
                msg = "[SYS] Disconnecting...";
                break;
            case protocol::ConnectionState::DISCONNECTED:
                fprintf(stderr, "[APP] === DISCONNECTED STATE RECEIVED ===\n");
                fprintf(stderr, "[APP] info='%s'\n", info.c_str());
                fprintf(stderr, "[APP] Current modem waveform_mode=%d\n",
                        static_cast<int>(modem_.getWaveformMode()));
                fflush(stderr);

                resetAdaptiveAdvisory();
                if (info.find("timeout") != std::string::npos) {
                    msg = "[FAILED] " + info;  // Make failures more visible
                } else {
                    msg = "[SYS] Disconnected" + (info.empty() ? "" : ": " + info);
                }

                // Reset waveform mode to OFDM when disconnected
                fprintf(stderr, "[APP] Calling modem_.setWaveformMode(OFDM_COX)\n");
                fflush(stderr);
                modem_.setWaveformMode(protocol::WaveformMode::OFDM_COX);
                fprintf(stderr, "[APP] Calling modem_.setConnectWaveform(MC_DPSK)\n");
                fflush(stderr);
                modem_.setConnectWaveform(protocol::WaveformMode::MC_DPSK);
                cq_listening_ = false;

                fprintf(stderr, "[APP] After reset: modem waveform_mode=%d, connect_waveform=%d\n",
                        static_cast<int>(modem_.getWaveformMode()),
                        static_cast<int>(modem_.getConnectWaveform()));
                fprintf(stderr, "[APP] === DISCONNECTED STATE COMPLETE ===\n");
                fflush(stderr);
                clearTcpTxQueue("link disconnected");
                break;
        }
        appendRxLogLine(msg);

        // Notify TCP host interface of connection state changes
        if (host_interface_) {
            switch (state) {
                case protocol::ConnectionState::CONNECTED:
                    host_interface_->notifyConnected(info);
                    break;
                case protocol::ConnectionState::DISCONNECTED:
                    if (info.find("timeout") != std::string::npos ||
                        info.find("failed") != std::string::npos) {
                        host_interface_->notifyLinkBroken();
                    } else {
                        host_interface_->notifyDisconnected();
                    }
                    break;
                default:
                    break;
            }
        }
    });
    ultra::gui::startupTrace("App", "protocol-callbacks-mid3");

    protocol_.setIncomingCallCallback([this](const std::string& from) {
        {
            std::lock_guard<std::mutex> lock(rx_log_mutex_);
            pending_incoming_call_ = from;
        }
        std::string msg = "[SYS] Incoming call from " + from;
        appendRxLogLine(msg);
    });
    ultra::gui::startupTrace("App", "protocol-callbacks-mid4");

    // PING TX callback - protocol wants to send a fast presence probe
    protocol_.setPingTxCallback([this]() {
        guiLog("TX PING: Probing for remote station...");
        auto samples = modem_.transmitPing();
        if (!samples.empty()) {
            if (simulation_enabled_) {
                // Queue for sim
                std::lock_guard<std::mutex> lock(our_tx_pending_mutex_);
                our_tx_pending_.insert(our_tx_pending_.end(), samples.begin(), samples.end());
                guiLog("SIM: Queued %zu PING samples", samples.size());
            } else {
                queueRealTxSamples(samples, "PING audio");
            }
        }
    });

    // PING received callback - someone is probing us
    protocol_.setPingReceivedCallback([this]() {
        guiLog("RX PING: Incoming probe, sending PONG...");
        auto samples = modem_.transmitPong();
        if (!samples.empty()) {
            if (simulation_enabled_) {
                // Queue for sim
                std::lock_guard<std::mutex> lock(our_tx_pending_mutex_);
                our_tx_pending_.insert(our_tx_pending_.end(), samples.begin(), samples.end());
                guiLog("SIM: Queued %zu PONG samples", samples.size());
            } else {
                queueRealTxSamples(samples, "PONG audio");
            }
        }
    });

    // Beacon TX callback - protocol wants to send a beacon/CQ broadcast
    protocol_.setBeaconTxCallback([this](const Bytes& frame_data) {
        guiLog("TX BEACON: Broadcasting...");
        auto samples = modem_.transmitBeacon(frame_data);
        if (!samples.empty()) {
            if (simulation_enabled_) {
                // Queue for sim
                std::lock_guard<std::mutex> lock(our_tx_pending_mutex_);
                our_tx_pending_.insert(our_tx_pending_.end(), samples.begin(), samples.end());
                guiLog("SIM: Queued %zu BEACON samples", samples.size());
            } else {
                queueRealTxSamples(samples, "BEACON audio");
            }
        }
    });

    // CQ TX callback - same as beacon but enters listening state after TX
    protocol_.setCQTxCallback([this](const Bytes& frame_data) {
        guiLog("TX CQ: Broadcasting CQ...");
        auto samples = modem_.transmitBeacon(frame_data);  // Same modulation as beacon
        if (!samples.empty()) {
            if (simulation_enabled_) {
                // Queue for sim
                std::lock_guard<std::mutex> lock(our_tx_pending_mutex_);
                our_tx_pending_.insert(our_tx_pending_.end(), samples.begin(), samples.end());
                guiLog("SIM: Queued %zu CQ samples", samples.size());
            } else {
                queueRealTxSamples(samples, "CQ audio");
            }
            // Enter listening state - wait for PING response with timeout
            cq_listening_ = true;
            cq_listen_start_ = std::chrono::steady_clock::now();
            guiLog("CQ: Entering listening state (%d second timeout)", CQ_LISTEN_TIMEOUT_SECONDS);
        }
    });
    ultra::gui::startupTrace("App", "protocol-callbacks-mid5");

    // Wire up modem ping detection to protocol
    modem_.setPingReceivedCallback([this](float snr) {
        // In simulation mode, use the configured SNR instead of detected SNR
        // (chirp detection sees clean loopback signal, not the simulated channel)
        float display_snr = simulation_enabled_ ? simulation_snr_db_ : snr;

        // Note: Fading index not shown here - it's only reliable after decoding data frames
        // Check state to show appropriate message
        if (protocol_.getState() == protocol::ConnectionState::PROBING) {
            guiLog("RX PONG: Remote station responded! (SNR=%.1f dB)", display_snr);
            // Add to message log so user sees it in the app
            char buf[80];
            snprintf(buf, sizeof(buf), "[PONG] Station responded (SNR=%.0f dB)", display_snr);
            appendRxLogLine(buf);
        } else {
            guiLog("MODEM: Detected PING/PONG (SNR=%.1f dB)", display_snr);
        }
        protocol_.onPingReceived();
    });
    ultra::gui::startupTrace("App", "protocol-callbacks-mid6");

    protocol_.setDataModeChangedCallback([this](Modulation mod, CodeRate rate, float snr_db, float peer_fading) {
        // Update modem engine with new data mode
        modem_.setDataMode(mod, rate, snr_db, peer_fading);
        resetAdaptiveAdvisory();

        // Local estimate for operator visibility/debugging.
        auto waveform = modem_.getWaveformMode();
        float local_fading = modem_.getFadingIndex();

        // Update MC-DPSK carrier count
        // NOTE: 5-carrier mode disabled - ARQ timeouts too slow for practical use
        // Always use 10 carriers for both DBPSK and DQPSK
        int mc_dpsk_carriers = 10;
        if (waveform == protocol::WaveformMode::MC_DPSK) {
            modem_.setMCDPSKCarriers(mc_dpsk_carriers);
        }

        const char* local_quality = fadingToQuality(local_fading);
        bool peer_fading_valid = (peer_fading >= 0.0f);
        const char* peer_quality = peer_fading_valid ? fadingToQuality(peer_fading) : "n/a";
        char peer_fading_text[32];
        if (peer_fading_valid) {
            snprintf(peer_fading_text, sizeof(peer_fading_text), "%.2f %s", peer_fading, peer_quality);
        } else {
            snprintf(peer_fading_text, sizeof(peer_fading_text), "n/a");
        }
        const char* wf_name = waveformDisplayName(waveform);
        guiLog("MODE_CHANGE: %s %s %s (peer_snr=%.1f dB, peer_fading=%s, local_fading=%.2f %s)",
               wf_name, modulationToString(mod), codeRateToString(rate),
               snr_db, peer_fading_text,
               local_fading, local_quality);

        // Format display with waveform info and channel quality
        char buf[200];
        if (waveform == protocol::WaveformMode::MC_DPSK) {
            snprintf(buf, sizeof(buf),
                     "[MODE] MC-DPSK %d carriers %s %s (peer SNR=%d dB, peer fading=%s, local fading=%.2f %s)",
                     mc_dpsk_carriers, modulationToString(mod), codeRateToString(rate),
                     static_cast<int>(snr_db), peer_fading_text,
                     local_fading, local_quality);
        } else {
            snprintf(buf, sizeof(buf),
                     "[MODE] %s %s %s (peer SNR=%d dB, peer fading=%s, local fading=%.2f %s)",
                     wf_name, modulationToString(mod), codeRateToString(rate),
                     static_cast<int>(snr_db), peer_fading_text,
                     local_fading, local_quality);
        }
        appendRxLogLine(buf);

        // Advisory-only peer view (does not change mode yet).
        if (peer_fading_valid) {
            Modulation peer_mod = mod;
            CodeRate peer_rate = rate;
            protocol::recommendDataMode(snr_db, waveform, peer_mod, peer_rate, peer_fading);
            bool peer_change = (peer_mod != mod || peer_rate != rate);

            char adpt_buf[220];
            if (peer_change) {
                const char* direction = adaptationDirection(mod, rate, peer_mod, peer_rate);
                snprintf(adpt_buf, sizeof(adpt_buf),
                         "[ADPT] Peer reports %s conditions (SNR=%.1f dB, F.I.=%.2f): %s -> %s %s",
                         direction, snr_db, peer_fading,
                         direction, modulationToString(peer_mod), codeRateToString(peer_rate));
            } else {
                snprintf(adpt_buf, sizeof(adpt_buf),
                         "[ADPT] Peer reports stable conditions (SNR=%.1f dB, F.I.=%.2f): keep %s %s",
                         snr_db, peer_fading, modulationToString(mod), codeRateToString(rate));
            }

            guiLog("%s", adpt_buf);
            appendRxLogLine(adpt_buf);
        }
    });
    ultra::gui::startupTrace("App", "protocol-callbacks-mid7");

    // Waveform mode negotiation callback (OFDM, DPSK, MFSK switching)
    protocol_.setModeNegotiatedCallback([this](protocol::WaveformMode mode) {
        std::string mode_name;
        switch (mode) {
            case protocol::WaveformMode::MC_DPSK: mode_name = "MC-DPSK"; break;
            case protocol::WaveformMode::MFSK: mode_name = "MFSK"; break;
            case protocol::WaveformMode::OTFS_EQ: mode_name = "OTFS"; break;
            case protocol::WaveformMode::OTFS_RAW: mode_name = "OTFS"; break;
            case protocol::WaveformMode::OFDM_CHIRP: mode_name = "OFDM"; break;
            case protocol::WaveformMode::OFDM_COX: mode_name = "OFDM"; break;
            default: mode_name = "OFDM"; break;
        }
        guiLog("WAVEFORM_CHANGE: %s", mode_name.c_str());

        // Update modem engine with new waveform mode
        modem_.setWaveformMode(mode);

        std::string msg = "[WAVEFORM] " + mode_name;
        appendRxLogLine(msg);
    });
    ultra::gui::startupTrace("App", "protocol-callbacks-mid8");

    // Connect waveform fallback callback (DPSK -> MFSK when connection attempts fail)
    protocol_.setConnectWaveformChangedCallback([this](protocol::WaveformMode mode) {
        const char* mode_name = (mode == protocol::WaveformMode::MFSK) ? "MFSK" : "DPSK";
        guiLog("CONNECT_WAVEFORM: Switching to %s for connection attempts", mode_name);
        modem_.setConnectWaveform(mode);
    });
    ultra::gui::startupTrace("App", "protocol-callbacks-mid9");

    // Handshake confirmed callback - now safe to use negotiated waveform
    protocol_.setHandshakeConfirmedCallback([this]() {
        guiLog("HANDSHAKE: Confirmed, switching to negotiated waveform");
        modem_.setHandshakeComplete(true);
    });
    ultra::gui::startupTrace("App", "protocol-callbacks-mid10");

    // File transfer callbacks
    protocol_.setFileProgressCallback([this](const protocol::FileTransferProgress& p) {
        // Start timing on first progress (for receiving files)
        if (last_progress_milestone_ == 0 && !p.is_sending) {
            file_transfer_start_time_ = std::chrono::steady_clock::now();
        }

        // Log progress milestones (25%, 50%, 75%)
        int pct = static_cast<int>(p.percentage());
        int milestone = (pct / 25) * 25;  // Round down to 25, 50, 75
        if (milestone > 0 && milestone < 100 && milestone > last_progress_milestone_) {
            last_progress_milestone_ = milestone;
            std::string msg = "[FILE] " + std::string(p.is_sending ? "TX" : "RX") +
                              " " + std::to_string(p.transferred_bytes) + "/" +
                              std::to_string(p.total_bytes) + " bytes (" +
                              std::to_string(milestone) + "%)";
            appendRxLogLine(msg);
        }
    });
    ultra::gui::startupTrace("App", "protocol-callbacks-mid11");

    protocol_.setFileReceivedCallback([this](const std::string& path, bool success) {
        last_progress_milestone_ = 0;  // Reset for next transfer
        auto duration = std::chrono::steady_clock::now() - file_transfer_start_time_;
        float seconds = std::chrono::duration<float>(duration).count();
        uint32_t file_bytes = success ? safeFileSizeBytes(path) : 0;

        std::string msg;
        if (success) {
            if (seconds > 0.0f && file_bytes > 0) {
                last_effective_goodput_bps_ = (8.0f * static_cast<float>(file_bytes)) / seconds;
                last_goodput_label_ = "RX file";
            }
            char buf[320];
            if (last_goodput_label_ == "RX file" && last_effective_goodput_bps_ > 0.0f) {
                snprintf(buf, sizeof(buf), "[FILE] Received: %s (%.1fs, %.2f kbps)",
                         path.c_str(), seconds, last_effective_goodput_bps_ / 1000.0f);
            } else {
                snprintf(buf, sizeof(buf), "[FILE] Received: %s (%.1fs)", path.c_str(), seconds);
            }
            msg = buf;
            last_received_file_ = path;
        } else {
            msg = "[FILE] Receive failed";
        }
        appendRxLogLine(msg);
    });
    ultra::gui::startupTrace("App", "protocol-callbacks-mid12");

    protocol_.setFileSentCallback([this](bool success, const std::string& error) {
        last_progress_milestone_ = 0;  // Reset for next transfer
        auto duration = std::chrono::steady_clock::now() - file_transfer_start_time_;
        float seconds = std::chrono::duration<float>(duration).count();

        std::string msg;
        if (success) {
            if (seconds > 0.0f && pending_file_tx_payload_bytes_ > 0) {
                last_effective_goodput_bps_ =
                    (8.0f * static_cast<float>(pending_file_tx_payload_bytes_)) / seconds;
                last_goodput_label_ = "TX file";
            }
            char buf[196];
            if (last_goodput_label_ == "TX file" && last_effective_goodput_bps_ > 0.0f) {
                snprintf(buf, sizeof(buf), "[FILE] Transfer complete (%.1fs, %.2f kbps)",
                         seconds, last_effective_goodput_bps_ / 1000.0f);
            } else {
                snprintf(buf, sizeof(buf), "[FILE] Transfer complete (%.1fs)", seconds);
            }
            msg = buf;
        } else {
            msg = "[FILE] Transfer failed: " + error;
        }
        pending_file_tx_payload_bytes_ = 0;
        appendRxLogLine(msg);
    });
    ultra::gui::startupTrace("App", "protocol-callbacks-exit");

    // Set receive directory from settings (defaults to Downloads folder)
    ultra::gui::startupTrace("App", "set-rx-dir-enter");
    protocol_.setReceiveDirectory(settings_.getReceiveDirectory());
    ultra::gui::startupTrace("App", "set-rx-dir-exit");

    // Auto-accept incoming connections (required for TCP-controlled operation)
    protocol_.setAutoAccept(true);

    // Configure waterfall display
    ultra::gui::startupTrace("App", "waterfall-config-enter");
    if (waterfall_) {
        waterfall_->setSampleRate(48000.0f);
        waterfall_->setFrequencyRange(0.0f, 3000.0f);
        waterfall_->setDynamicRange(-60.0f, 0.0f);
    }
    ultra::gui::startupTrace("App", "waterfall-config-exit");

    // Settings window callbacks
    ultra::gui::startupTrace("App", "settings-callbacks-enter");
    settings_window_.setCallsignChangedCallback([this](const std::string& call) {
        protocol_.setLocalCallsign(call);
        modem_.setLogPrefix(call);
        settings_.save(options_.config_path);
    });

    settings_window_.setAudioResetCallback([this]() {
        releaseTxPtt("audio_reset", true);
        closeSerialPtt();
        if (radio_rx_enabled_) {
            stopRadioRx();
        }
        audio_.stopPlayback();
        audio_.stopCapture();
        audio_.closeInput();
        audio_.closeOutput();
        audio_.shutdown();
        audio_initialized_ = false;

        initAudio();
        if (audio_initialized_) {
            audio_.setOutputGain(settings_.tx_drive);
            startRadioRx();
        }
    });

    settings_window_.setClosedCallback([this]() {
        settings_.save(options_.config_path);
        releaseTxPtt("settings_closed", true);
        closeSerialPtt();

        if (radio_rx_enabled_) {
            stopRadioRx();
        }
        audio_.stopPlayback();
        audio_.stopCapture();
        audio_.closeInput();
        audio_.closeOutput();

        if (audio_initialized_) {
            audio_.setOutputGain(settings_.tx_drive);
            startRadioRx();
        }
    });

    settings_window_.setFilterChangedCallback([this](bool enabled, float center, float bw, int taps) {
        FilterConfig filter_config;
        filter_config.enabled = enabled;
        filter_config.center_freq = center;
        filter_config.bandwidth = bw;
        filter_config.taps = taps;
        modem_.setFilterConfig(filter_config);
        settings_.save(options_.config_path);
    });

    settings_window_.setReceiveDirChangedCallback([this](const std::string& dir) {
        protocol_.setReceiveDirectory(dir);
        settings_.save(options_.config_path);
    });

    settings_window_.setExpertSettingsChangedCallback([this](uint8_t waveform, uint8_t modulation, uint8_t code_rate) {
        // Apply forced settings to protocol (used on next connect)
        protocol_.setPreferredMode(static_cast<protocol::WaveformMode>(waveform));
        protocol_.setForcedModulation(static_cast<Modulation>(modulation));
        protocol_.setForcedCodeRate(static_cast<CodeRate>(code_rate));
        handleForcedWaveformUpdate(waveform, modulation, code_rate, "settings");
        settings_.save(options_.config_path);
    });

    settings_window_.setTcpSettingsChangedCallback([this]() {
        // TCP settings changed - restart host interface with new settings
        guiLog("TCP settings changed, restarting host interface");
        if (host_interface_) {
            host_interface_->stop();
            host_interface_.reset();
        }
        settings_.save(options_.config_path);
        initHostInterface();
    });

    settings_window_.setEncryptionChangedCallback([this](bool enabled, const std::string& key) {
        guiLog("Encryption %s", enabled ? "enabled" : "disabled");
        protocol_.setEncryptionEnabled(enabled);
        if (!key.empty()) {
            protocol_.setEncryptionKey(key);
        } else if (!enabled) {
            protocol_.clearEncryptionKey();
        }
    });

    settings_window_.setCompressionChangedCallback([this](bool enabled) {
        guiLog("Compression %s", enabled ? "enabled" : "disabled");
        protocol_.setCompressionEnabled(enabled);
    });

    settings_window_.setCatSettingsChangedCallback([this]() {
        guiLog("CAT settings changed");
        // Configure CatController from settings
        cat::CatConfig cat_config;
        cat_config.enabled = settings_.cat_enabled;
        cat_config.backend_type = static_cast<cat::CatBackendType>(settings_.cat_backend);
        cat_config.serial_port = settings_.cat_port;
        cat_config.serial_baud = settings_.cat_baud;
        cat_config.ptt_line = settings_.cat_ptt_line;
        cat_config.ptt_invert = settings_.cat_ptt_invert;
        cat_config.hamlib_model = settings_.cat_hamlib_model;
        cat_config.tcp_host = settings_.cat_tcp_host;
        cat_config.tcp_port = settings_.cat_tcp_port;
        cat_config.flex_slice = settings_.cat_flex_slice;
        cat_config.ptt_lead_ms = settings_.cat_ptt_lead_ms;
        cat_config.ptt_tail_ms = settings_.cat_ptt_tail_ms;
        cat_config.watchdog_seconds = settings_.cat_watchdog_seconds;
        cat_controller_.configure(cat_config);
        settings_.save(options_.config_path);
    });
    ultra::gui::startupTrace("App", "settings-callbacks-exit");

    // Apply initial expert settings from loaded config
    ultra::gui::startupTrace("App", "apply-expert-enter");
    protocol_.setPreferredMode(static_cast<protocol::WaveformMode>(settings_.forced_waveform));
    protocol_.setForcedModulation(static_cast<Modulation>(settings_.forced_modulation));
    protocol_.setForcedCodeRate(static_cast<CodeRate>(settings_.forced_code_rate));
    ultra::gui::startupTrace("App", "apply-expert-exit");

    // Apply initial encryption settings from loaded config
    ultra::gui::startupTrace("App", "apply-encryption-enter");
    protocol_.setEncryptionEnabled(settings_.encryption_enabled);
    if (settings_.encryption_key[0] != '\0') {
        protocol_.setEncryptionKey(settings_.encryption_key);
    }
    ultra::gui::startupTrace("App", "apply-encryption-exit");

    // Apply initial compression settings from loaded config
    ultra::gui::startupTrace("App", "apply-compression-enter");
    protocol_.setCompressionEnabled(settings_.compression_enabled);
    ultra::gui::startupTrace("App", "apply-compression-exit");

    // Apply initial CAT settings from loaded config
    ultra::gui::startupTrace("App", "apply-cat-enter");
    {
        cat::CatConfig cat_config;
        cat_config.enabled = settings_.cat_enabled;
        cat_config.backend_type = static_cast<cat::CatBackendType>(settings_.cat_backend);
        cat_config.serial_port = settings_.cat_port;
        cat_config.serial_baud = settings_.cat_baud;
        cat_config.ptt_line = settings_.cat_ptt_line;
        cat_config.ptt_invert = settings_.cat_ptt_invert;
        cat_config.hamlib_model = settings_.cat_hamlib_model;
        cat_config.tcp_host = settings_.cat_tcp_host;
        cat_config.tcp_port = settings_.cat_tcp_port;
        cat_config.flex_slice = settings_.cat_flex_slice;
        cat_config.ptt_lead_ms = settings_.cat_ptt_lead_ms;
        cat_config.ptt_tail_ms = settings_.cat_ptt_tail_ms;
        cat_config.watchdog_seconds = settings_.cat_watchdog_seconds;
        cat_controller_.configure(cat_config);

        // Set watchdog callback
        cat_controller_.setWatchdogCallback([this]() {
            guiLog("[CAT] TX watchdog triggered - forcing PTT release");
            appendRxLogLine("[CAT] TX watchdog timeout - PTT released");
            stopTxNow("watchdog_timeout");
        });
    }
    ultra::gui::startupTrace("App", "apply-cat-exit");

    // Apply initial filter settings from loaded config
    ultra::gui::startupTrace("App", "apply-filter-enter");
    FilterConfig initial_filter;
    initial_filter.enabled = settings_.filter_enabled;
    initial_filter.center_freq = settings_.filter_center;
    initial_filter.bandwidth = settings_.filter_bandwidth;
    initial_filter.taps = settings_.filter_taps;
    modem_.setFilterConfig(initial_filter);
    audio_.setOutputGain(settings_.tx_drive);
    ultra::gui::startupTrace("App", "apply-filter-exit");

    // Initialize virtual station only when simulator UI is shown and startup is not constrained.
    // In safe-startup mode we defer this until the user enables simulation.
    if (sim_ui_visible_ && !options_.safe_startup) {
        ultra::gui::startupTrace("App", "init-virtual-enter");
        initVirtualStation();
        ultra::gui::startupTrace("App", "init-virtual-exit");
    }

    // Auto-initialize audio on startup unless safe-startup mode is requested.
    // This avoids crashing on fragile audio stacks during process bring-up.
    if (!options_.safe_startup) {
        ultra::gui::startupTrace("App", "init-audio-enter");
        initAudio();
        if (audio_initialized_) {
            deferred_radio_rx_start_pending_ = true;
            uint32_t now_ms = SDL_GetTicks();
            deferred_radio_rx_start_deadline_ms_ = now_ms;
            deferred_radio_rx_start_timeout_ms_ = now_ms + 3000;
            deferred_radio_rx_start_attempts_ = 0;
            guiLog("Startup audio stage 1/2 complete: core ready, starting RX capture ASAP (timeout=3000ms)");
        }
        ultra::gui::startupTrace("App", "init-audio-exit");
    } else {
        guiLog("Safe startup enabled: deferred audio/simulator initialization");
        deferred_audio_auto_init_pending_ = true;
        deferred_audio_auto_init_deadline_ms_ = SDL_GetTicks() + 300;
        deferred_audio_auto_init_attempts_ = 0;
        deferred_audio_wait_logged_ = false;
        ultra::gui::startupTrace("App", "deferred-audio-scheduled");
    }

    // Initialize TCP host interface if enabled
    ultra::gui::startupTrace("App", "init-host-interface-enter");
    initHostInterface();
    ultra::gui::startupTrace("App", "init-host-interface-exit");

    ultra::gui::startupTrace("App", "ctor-body-exit");
}

App::~App() {
    releaseTxPtt("app_shutdown", true);
    closeSerialPtt();

    // Stop TCP host interface
    if (host_interface_) {
        host_interface_->stop();
        host_interface_.reset();
    }

    // Stop simulator threads first
    stopSimulator();

    settings_.save(options_.config_path);
    audio_.shutdown();

    // Write recording to file if -rec was enabled
    if (options_.record_audio) {
        writeRecordingToFile();
    }
}

void App::initHostInterface() {
    fprintf(stderr, "[APP] initHostInterface: tcp_enabled=%d, cmd_port=%d, data_port=%d\n",
            settings_.tcp_enabled ? 1 : 0, settings_.tcp_cmd_port, settings_.tcp_data_port);
    fflush(stderr);

    if (!settings_.tcp_enabled) {
        fprintf(stderr, "[APP] TCP host interface disabled in settings\n");
        fflush(stderr);
        guiLog("TCP host interface disabled in settings");
        return;
    }

    interface::TcpConfig tcp_config;
    tcp_config.cmd_port = settings_.tcp_cmd_port;
    tcp_config.data_port = settings_.tcp_data_port;
    tcp_config.kiss_port = settings_.tcp_kiss_port;
    tcp_config.kiss_enabled = settings_.tcp_kiss_enabled;
    tcp_config.bind_addr = settings_.tcp_bind_addr;
    tcp_config.enabled = true;

    host_interface_ = std::make_unique<interface::HostInterface>(tcp_config);

    // Bind to protocol engine, modem, and settings
    host_interface_->bindProtocol(&protocol_);
    host_interface_->bindModem(&modem_);
    host_interface_->bindSettings(&settings_);
    host_interface_->bindCat(&cat_controller_);

    // Set up callbacks for actions that need App coordination
    host_interface_->setConnectRequestCallback([this](const std::string& callsign) {
        guiLog("TCP: Connect request to %s", callsign.c_str());
        strncpy(remote_callsign_, callsign.c_str(), sizeof(remote_callsign_) - 1);
        remote_callsign_[sizeof(remote_callsign_) - 1] = '\0';
        protocol_.connect(callsign);
    });

    host_interface_->setDisconnectRequestCallback([this]() {
        guiLog("TCP: Disconnect request");
        protocol_.disconnect();
    });

    host_interface_->setAbortRequestCallback([this]() {
        guiLog("TCP: Abort request");
        protocol_.disconnect();  // Abort = force disconnect
    });

    host_interface_->setBeaconRequestCallback([this]() {
        guiLog("TCP: Beacon TX request");
        // HostInterface will transmit after payload arrives on data port.
        // If no payload arrives, it will send an empty beacon after timeout.
    });

    host_interface_->setCQRequestCallback([this]() {
        guiLog("TCP: CQ TX request");
        // HostInterface will transmit after payload arrives on data port.
        // If no payload arrives, it will send an empty CQ after timeout.
    });

    host_interface_->setPingRequestCallback([this](const std::string& target) {
        guiLog("TCP: PING TX request to %s", target.c_str());
        // Note: Data will be read from data port and sent via protocol_.transmitPing()
    });

    host_interface_->setRawTxRequestCallback(
        [this](const std::vector<uint8_t>& payload, uint8_t waveform, uint8_t modulation, uint8_t code_rate) {
            auto wf = static_cast<protocol::WaveformMode>(waveform);
            auto mod = static_cast<Modulation>(modulation);
            auto rate = static_cast<CodeRate>(code_rate);

            guiLog("TCP: RAWTX request (%zu bytes, %s, %s %s)",
                   payload.size(),
                   protocol::waveformModeToString(wf),
                   modulationToString(mod),
                   codeRateToString(rate));

            auto samples = modem_.transmitRaw(payload, wf, mod, rate, false);
            if (samples.empty()) {
                guiLog("TCP: RAWTX failed (no TX samples)");
                return;
            }

            if (simulation_enabled_) {
                size_t tx_duration_ms = (samples.size() * 1000) / 48000;
                tx_in_progress_ = true;
                tx_end_time_ = std::chrono::steady_clock::now() +
                               std::chrono::milliseconds(tx_duration_ms + 100);
                std::lock_guard<std::mutex> lock(our_tx_pending_mutex_);
                our_tx_pending_.insert(our_tx_pending_.end(), samples.begin(), samples.end());
                guiLog("SIM: Queued %zu RAWTX samples", samples.size());
            } else {
                queueRealTxSamples(samples, "RAWTX audio");
            }
        });

    host_interface_->setSendDataCallback([this](const std::vector<uint8_t>& data) {
        // Queue outbound host data and retry when ARQ has space.
        std::string text(data.begin(), data.end());
        if (tcp_tx_queue_.size() >= MAX_TCP_TX_QUEUE_MESSAGES) {
            tcp_tx_queue_.pop_front();
            guiLog("TCP: TX queue full, dropped oldest queued message");
        }
        tcp_tx_queue_.push_back(std::move(text));
        guiLog("TCP: Queued data (%zu bytes), pending=%zu",
               data.size(), tcp_tx_queue_.size());
        flushTcpTxQueue();
    });

    host_interface_->setCallsignChangedCallback([this](const std::string& callsign) {
        guiLog("TCP: Callsign changed to %s", callsign.c_str());
        protocol_.setLocalCallsign(callsign);
        modem_.setLogPrefix(callsign);
    });

    host_interface_->setExpertSettingsCallback([this](uint8_t waveform, uint8_t modulation, uint8_t code_rate) {
        guiLog("TCP: Expert settings changed: waveform=%d, modulation=%d, code_rate=%d",
               waveform, modulation, code_rate);
        // Apply the new settings to protocol engine
        auto waveform_mode = static_cast<protocol::WaveformMode>(waveform);
        protocol_.setPreferredMode(waveform_mode);
        protocol_.setForcedModulation(static_cast<Modulation>(modulation));
        protocol_.setForcedCodeRate(static_cast<CodeRate>(code_rate));
        handleForcedWaveformUpdate(waveform, modulation, code_rate, "tcp");

        // Save settings to persist changes
        settings_.save(options_.config_path);
    });

    host_interface_->setTuneRequestCallback([this](bool enabled) {
        guiLog("TCP: Tune request %s", enabled ? "ON" : "OFF");
        // Tune functionality would key PTT with carrier tone
    });

    // Start the TCP server
    fprintf(stderr, "[APP] Starting TCP server on cmd=%d, data=%d\n",
            tcp_config.cmd_port, tcp_config.data_port);
    fflush(stderr);

    if (!host_interface_->start()) {
        fprintf(stderr, "[APP] TCP: Failed to start host interface!\n");
        fflush(stderr);
        guiLog("TCP: Failed to start host interface");
        host_interface_.reset();
        return;
    }

    fprintf(stderr, "[APP] TCP host interface started successfully (cmd=%d, data=%d)\n",
            settings_.tcp_cmd_port, settings_.tcp_data_port);
    fflush(stderr);
    guiLog("TCP host interface started (cmd=%d, data=%d)",
           settings_.tcp_cmd_port, settings_.tcp_data_port);
}

void App::pollHostInterface() {
    if (!host_interface_) return;

    // Poll for commands
    host_interface_->poll();

    // Drain any queued host data when the protocol/ARQ layer is ready.
    flushTcpTxQueue();

    // Update PTT state
    host_interface_->notifyPtt(ptt_active_);
}

void App::checkCQListenTimeout() {
    if (!cq_listening_) return;

    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - cq_listen_start_).count();

    if (elapsed >= CQ_LISTEN_TIMEOUT_SECONDS) {
        cq_listening_ = false;
        auto state = protocol_.getState();
        if (state == protocol::ConnectionState::DISCONNECTED) {
            guiLog("CQ: No response received, returning to IDLE");
            if (host_interface_) {
                host_interface_->notifyStateChange("IDLE");
            }
        } else {
            guiLog("CQ: Listen timeout ignored (state=%d)", static_cast<int>(state));
        }
    }
}

void App::flushTcpTxQueue() {
    while (!tcp_tx_queue_.empty()) {
        if (protocol_.getState() != protocol::ConnectionState::CONNECTED) {
            break;
        }
        if (!protocol_.isReadyToSend()) {
            break;
        }
        if (!protocol_.sendMessage(tcp_tx_queue_.front())) {
            break;
        }
        tcp_tx_queue_.pop_front();
    }
}

void App::clearTcpTxQueue(const char* reason) {
    if (tcp_tx_queue_.empty()) {
        return;
    }
    guiLog("TCP: Dropped %zu queued messages (%s)",
           tcp_tx_queue_.size(),
           reason ? reason : "unspecified");
    tcp_tx_queue_.clear();
}

void App::handleForcedWaveformUpdate(uint8_t waveform, uint8_t modulation, uint8_t code_rate,
                                     const char* source) {
    bool waveform_changed = (waveform != last_forced_waveform_);
    last_forced_waveform_ = waveform;
    if (!waveform_changed) {
        return;
    }

    auto mode = static_cast<protocol::WaveformMode>(waveform);
    const char* mode_name = protocol::waveformModeToString(mode);
    const char* src = source ? source : "unknown";

    if (mode == protocol::WaveformMode::AUTO) {
        guiLog("WAVEFORM: AUTO set via %s (applies on next connect)", src);
        appendRxLogLine("[SYS] AUTO waveform policy will apply on next connect");
        return;
    }

    if (protocol_.getState() != protocol::ConnectionState::CONNECTED) {
        guiLog("WAVEFORM: set to %s via %s (applies on next connect)", mode_name, src);
        return;
    }

    if (!protocol::v2::ModeChangePayload::isRuntimeWaveform(mode)) {
        guiLog("WAVEFORM: invalid runtime mode %d via %s", static_cast<int>(mode), src);
        appendRxLogLine("[SYS] Runtime waveform switch rejected: invalid mode");
        return;
    }

    Modulation requested_mod = static_cast<Modulation>(modulation);
    CodeRate requested_rate = static_cast<CodeRate>(code_rate);

    float snr_db = simulation_enabled_ ? simulation_snr_db_ : modem_.getStats().snr_db;
    if (!std::isfinite(snr_db)) {
        snr_db = protocol_.getMeasuredSNR();
    }
    if (!std::isfinite(snr_db)) {
        snr_db = 0.0f;
    }

    float fading = modem_.getFadingIndex();
    if (!std::isfinite(fading) || fading < 0.0f) {
        fading = protocol_.getFadingIndex();
    }
    if (!std::isfinite(fading) || fading < 0.0f) {
        fading = 0.0f;
    }

    if (requested_mod == Modulation::AUTO || requested_rate == CodeRate::AUTO) {
        Modulation rec_mod = protocol_.getDataModulation();
        CodeRate rec_rate = protocol_.getDataCodeRate();
        protocol::recommendDataMode(snr_db, mode, rec_mod, rec_rate, fading);
        if (requested_mod == Modulation::AUTO) {
            requested_mod = rec_mod;
        }
        if (requested_rate == CodeRate::AUTO) {
            requested_rate = rec_rate;
        }
    }

    bool queued = protocol_.requestModeChange(
        requested_mod, requested_rate, snr_db,
        protocol::v2::ModeChangeReason::USER_REQUEST,
        mode);

    if (queued) {
        guiLog("WAVEFORM: requested connected switch to %s (%s %s via %s)",
               mode_name, modulationToString(requested_mod), codeRateToString(requested_rate), src);
        char buf[192];
        snprintf(buf, sizeof(buf),
                 "[SYS] Requested waveform switch: %s (%s %s)",
                 mode_name, modulationToString(requested_mod), codeRateToString(requested_rate));
        appendRxLogLine(buf);
    } else {
        guiLog("WAVEFORM: connected switch to %s rejected (busy/unsupported)", mode_name);
        appendRxLogLine("[SYS] Runtime waveform switch rejected by protocol");
    }
}

void App::writeRecordingToFile() {
    const std::string base = trimF32Extension(options_.record_path);
    bool wrote_any = false;

    if (!recorded_tx_samples_.empty()) {
        const std::string path = base + "_tx.f32";
        if (writeF32File(path, recorded_tx_samples_)) {
            guiLog("Recording saved: %s (%zu samples, %.1f seconds)",
                   path.c_str(),
                   recorded_tx_samples_.size(),
                   recorded_tx_samples_.size() / 48000.0f);
            wrote_any = true;
        } else {
            guiLog("ERROR: Failed to save TX recording to %s", path.c_str());
        }
    }

    if (!recorded_rx_samples_.empty()) {
        const std::string path = base + "_rx.f32";
        if (writeF32File(path, recorded_rx_samples_)) {
            guiLog("Recording saved: %s (%zu samples, %.1f seconds)",
                   path.c_str(),
                   recorded_rx_samples_.size(),
                   recorded_rx_samples_.size() / 48000.0f);
            wrote_any = true;
        } else {
            guiLog("ERROR: Failed to save RX recording to %s", path.c_str());
        }
    }

    // Backward-compatible simulation capture file.
    if (!recorded_samples_.empty()) {
        const std::string path = base + "_sim.f32";
        if (writeF32File(path, recorded_samples_)) {
            guiLog("Recording saved: %s (%zu samples, %.1f seconds)",
                   path.c_str(),
                   recorded_samples_.size(),
                   recorded_samples_.size() / 48000.0f);
            wrote_any = true;
        } else {
            guiLog("ERROR: Failed to save simulation recording to %s", path.c_str());
        }
    }

    if (!wrote_any) {
        guiLog("Recording skipped: no captured samples");
    }
}

void App::initVirtualStation() {
    if (virtual_modem_) {
        return;
    }

    // Create virtual station's modem
    virtual_modem_ = std::make_unique<ModemEngine>();
    virtual_modem_->setSynchronousMode(false);

    // Set up virtual station's protocol
    virtual_protocol_.setLocalCallsign(virtual_callsign_);
    virtual_modem_->setLogPrefix(virtual_callsign_);
    virtual_protocol_.setAutoAccept(true);  // Auto-accept incoming calls
    virtual_protocol_.setReceiveDirectory(settings_.getReceiveDirectory());  // Save files to same dir

    // Virtual station TX → queue for real-time streaming → our RX
    virtual_protocol_.setTxDataCallback([this](const Bytes& data) {
        guiLog("SIM: Virtual station TX %zu bytes", data.size());
        auto samples = virtual_modem_->transmit(data);

        // Add PTT noise once at start of transmission (100-300ms)
        std::uniform_int_distribution<size_t> ptt_dist(4800, 14400);
        size_t ptt_samples = ptt_dist(sim_rng_);

        float typical_rms = 0.1f;
        float snr_linear = std::pow(10.0f, simulation_snr_db_ / 10.0f);
        float noise_power = (typical_rms * typical_rms) / snr_linear;
        float noise_stddev = std::sqrt(noise_power);
        std::normal_distribution<float> noise_dist(0.0f, noise_stddev);

        std::vector<float> ptt_noise(ptt_samples);
        for (float& s : ptt_noise) s = noise_dist(sim_rng_);

        guiLog("SIM: Virtual modem produced %zu samples (+ %zu PTT noise), queuing for stream", samples.size(), ptt_samples);

        // Queue PTT noise + signal for real-time streaming
        std::lock_guard<std::mutex> lock(virtual_tx_pending_mutex_);
        virtual_tx_pending_.insert(virtual_tx_pending_.end(), ptt_noise.begin(), ptt_noise.end());
        virtual_tx_pending_.insert(virtual_tx_pending_.end(), samples.begin(), samples.end());
    });

    // Virtual station burst TX callback
    virtual_protocol_.setTransmitBurstCallback([this](const std::vector<Bytes>& frames) {
        guiLog("SIM: Virtual station burst TX %zu frames", frames.size());
        auto samples = virtual_modem_->transmitBurst(frames);
        guiLog("SIM: Virtual burst produced %zu samples", samples.size());
        std::lock_guard<std::mutex> lock(virtual_tx_pending_mutex_);
        virtual_tx_pending_.insert(virtual_tx_pending_.end(), samples.begin(), samples.end());
    });

    // Virtual modem RX → virtual protocol
    virtual_modem_->setRawDataCallback([this](const Bytes& data) {
        guiLog("SIM: Virtual modem decoded %zu bytes", data.size());
        // Use simulation SNR and fading from virtual modem
        // The virtual station sees the same channel as our station
        float fading = virtual_modem_->getFadingIndex();
        virtual_protocol_.setMeasuredSNR(simulation_snr_db_);
        virtual_protocol_.setChannelQuality(simulation_snr_db_, fading);
        virtual_protocol_.onRxData(data);
    });

    // Log virtual station events
    virtual_protocol_.setConnectionChangedCallback([this](protocol::ConnectionState state, const std::string& info) {
        guiLog("SIM: Virtual station connection state: %d (%s)", static_cast<int>(state), info.c_str());

        // Update virtual modem engine connection state
        bool connected = (state == protocol::ConnectionState::CONNECTED);
        virtual_modem_->setConnected(connected);
        virtual_modem_->setMCDPSKChannelInterleave(
            connected ? virtual_protocol_.isMCDPSKChannelInterleaveActive() : false);

        std::string msg = "[SIM] ";
        switch (state) {
            case protocol::ConnectionState::CONNECTED:
                msg += "Virtual station connected";
                break;
            case protocol::ConnectionState::DISCONNECTED:
                msg += "Virtual station disconnected";
                // Reset waveform mode to OFDM when disconnected
                virtual_modem_->setWaveformMode(protocol::WaveformMode::OFDM_COX);
                break;
            default:
                return;  // Don't log intermediate states
        }
        appendRxLogLine(msg);
    });

    virtual_protocol_.setDataModeChangedCallback([this](Modulation mod, CodeRate rate, float snr_db, float peer_fading) {
        // Update virtual modem engine with new data mode
        virtual_modem_->setDataMode(mod, rate, snr_db, peer_fading);

        // Show [SIM-MODE] line so user can see what the responder actually measured
        auto waveform = virtual_modem_->getWaveformMode();
        float local_fading = virtual_modem_->getFadingIndex();

        // Update MC-DPSK carrier count for DBPSK modes based on SNR
        if (waveform == protocol::WaveformMode::MC_DPSK) {
            int carriers = 10;
            if (mod == Modulation::DBPSK) {
                carriers = (snr_db < 0.0f) ? 5 : 10;
            }
            virtual_modem_->setMCDPSKCarriers(carriers);
        }
        const char* local_quality = fadingToQuality(local_fading);
        bool peer_fading_valid = (peer_fading >= 0.0f);
        const char* peer_quality = peer_fading_valid ? fadingToQuality(peer_fading) : "n/a";
        char peer_fading_text[32];
        if (peer_fading_valid) {
            snprintf(peer_fading_text, sizeof(peer_fading_text), "%.2f %s", peer_fading, peer_quality);
        } else {
            snprintf(peer_fading_text, sizeof(peer_fading_text), "n/a");
        }
        const char* wf_name = waveformDisplayName(waveform);

        guiLog("SIM: Virtual MODE_CHANGE: %s %s %s (peer_snr=%.1f dB, peer_fading=%s, local_fading=%.2f %s)",
               wf_name, modulationToString(mod), codeRateToString(rate),
               snr_db, peer_fading_text,
               local_fading, local_quality);

        char buf[200];
        snprintf(buf, sizeof(buf),
                 "[SIM-MODE] %s %s %s (peer SNR=%d dB, peer fading=%s, local fading=%.2f %s)",
                 wf_name, modulationToString(mod), codeRateToString(rate),
                 static_cast<int>(snr_db), peer_fading_text,
                 local_fading, local_quality);
        appendRxLogLine(buf);
    });

    virtual_protocol_.setModeNegotiatedCallback([this](protocol::WaveformMode mode) {
        const char* mode_name = "OFDM";
        switch (mode) {
            case protocol::WaveformMode::MC_DPSK: mode_name = "MC-DPSK"; break;
            case protocol::WaveformMode::MFSK: mode_name = "MFSK"; break;
            case protocol::WaveformMode::OTFS_EQ: mode_name = "OTFS-EQ"; break;
            case protocol::WaveformMode::OTFS_RAW: mode_name = "OTFS-RAW"; break;
            default: mode_name = "OFDM"; break;
        }
        guiLog("SIM: Virtual WAVEFORM_CHANGE: %s", mode_name);
        // Update virtual modem engine with new waveform mode
        virtual_modem_->setWaveformMode(mode);
    });

    // Connect waveform fallback for virtual station
    virtual_protocol_.setConnectWaveformChangedCallback([this](protocol::WaveformMode mode) {
        const char* mode_name = (mode == protocol::WaveformMode::MFSK) ? "MFSK" : "DPSK";
        guiLog("SIM: Virtual CONNECT_WAVEFORM: Switching to %s", mode_name);
        virtual_modem_->setConnectWaveform(mode);
    });

    // Virtual station handshake confirmed callback
    virtual_protocol_.setHandshakeConfirmedCallback([this]() {
        guiLog("SIM: Virtual HANDSHAKE confirmed");
        virtual_modem_->setHandshakeComplete(true);
    });

    // Virtual station file transfer callbacks
    virtual_protocol_.setFileReceivedCallback([this](const std::string& path, bool success) {
        std::string msg = "[SIM] ";
        if (success) {
            msg += "Received file: " + path;
        } else {
            msg += "File receive failed";
        }
        appendRxLogLine(msg);
    });

    virtual_protocol_.setFileSentCallback([this](bool success, const std::string& error) {
        std::string msg = "[SIM] ";
        if (success) {
            msg += "File sent successfully";
        } else {
            msg += "File send failed: " + error;
        }
        appendRxLogLine(msg);
    });

    virtual_protocol_.setMessageReceivedCallback([this](const std::string& from, const std::string& text) {
        // Virtual station received our message - it could auto-reply here
        // For now, just log that it received the message
        guiLog("SIM: Virtual station received msg from %s: %s", from.c_str(), text.c_str());
    });

    // Virtual station PING TX callback
    virtual_protocol_.setPingTxCallback([this]() {
        guiLog("SIM: Virtual station TX PING");
        auto samples = virtual_modem_->transmitPing();
        if (!samples.empty()) {
            std::lock_guard<std::mutex> lock(virtual_tx_pending_mutex_);
            virtual_tx_pending_.insert(virtual_tx_pending_.end(), samples.begin(), samples.end());
        }
    });

    // Virtual station PING received callback - respond with PONG
    virtual_protocol_.setPingReceivedCallback([this]() {
        guiLog("SIM: Virtual station received PING, sending PONG");
        auto samples = virtual_modem_->transmitPong();
        if (!samples.empty()) {
            std::lock_guard<std::mutex> lock(virtual_tx_pending_mutex_);
            virtual_tx_pending_.insert(virtual_tx_pending_.end(), samples.begin(), samples.end());
        }
    });

    // Wire up virtual modem ping detection to virtual protocol
    virtual_modem_->setPingReceivedCallback([this](float snr) {
        guiLog("SIM: Virtual modem detected PING/PONG (SNR=%.1f dB)", snr);
        virtual_protocol_.onPingReceived();
    });

    guiLog("Virtual station initialized: callsign=%s", virtual_callsign_.c_str());
}

// ========================================
// Simplified Simulator (single thread model like cli_simulator)
// ========================================

std::vector<float> App::applyChannelEffects(const std::vector<float>& samples, int direction) {
    if (samples.empty()) return samples;

    std::vector<float> result = samples;

    // Select channel for this direction (independent fading per direction, like cli_simulator)
    auto& channel = (direction == 0) ? sim_channel_a_to_b_ : sim_channel_b_to_a_;

    // Apply fading/multipath if not AWGN
    // Use persistent channel so fading state evolves continuously across frames
    if (simulation_channel_type_ > 0) {
        // Recreate channels only when type changes (not per-frame)
        if (!channel || sim_channel_active_type_ != simulation_channel_type_) {
            sim::WattersonChannel::Config cfg;
            switch (simulation_channel_type_) {
                case 1:  // Good
                    cfg = sim::itu_r_f1487::good(simulation_snr_db_);
                    break;
                case 2:  // Moderate
                    cfg = sim::itu_r_f1487::moderate(simulation_snr_db_);
                    break;
                case 3:  // Poor
                    cfg = sim::itu_r_f1487::poor(simulation_snr_db_);
                    break;
                default:
                    cfg = sim::itu_r_f1487::good(simulation_snr_db_);
                    break;
            }
            // Disable noise in WattersonChannel - we'll add it with fixed reference below
            cfg.noise_enabled = false;
            // Use different seeds per direction (like cli_simulator: 42, 43)
            sim_channel_a_to_b_ = std::make_unique<sim::WattersonChannel>(cfg, 42);
            sim_channel_b_to_a_ = std::make_unique<sim::WattersonChannel>(cfg, 43);
            sim_channel_active_type_ = simulation_channel_type_;
        }

        SampleSpan input(result.data(), result.size());
        result = channel->process(input);
    } else if (sim_channel_a_to_b_ || sim_channel_b_to_a_) {
        // Switched to AWGN — release fading channels
        sim_channel_a_to_b_.reset();
        sim_channel_b_to_a_.reset();
        sim_channel_active_type_ = -1;
    }

    // Apply AWGN with fixed reference signal power (matches cli_simulator)
    // Using fixed reference ensures consistent SNR regardless of signal amplitude
    if (simulation_snr_db_ < 50.0f) {
        float snr_linear = std::pow(10.0f, simulation_snr_db_ / 10.0f);
        constexpr float REFERENCE_SIGNAL_POWER = 0.01f;  // Same as cli_simulator
        float noise_stddev = std::sqrt(REFERENCE_SIGNAL_POWER / snr_linear);
        std::normal_distribution<float> noise_dist(0.0f, noise_stddev);

        for (float& s : result) {
            s += noise_dist(sim_rng_);
        }
    }

    return result;
}

void App::startSimulator() {
    if (sim_thread_running_) return;
    if (!virtual_modem_) {
        guiLog("SIM: Cannot start simulator - virtual modem is not initialized");
        return;
    }

    guiLog("SIM: Starting simulator");

    // Keep both modems in asynchronous mode for consistent threaded behavior.
    modem_.setSynchronousMode(false);
    virtual_modem_->setSynchronousMode(false);

    sim_thread_running_ = true;
    sim_thread_ = std::thread(&App::simulationLoop, this);
}

void App::stopSimulator() {
    if (!sim_thread_running_) return;

    guiLog("SIM: Stopping simulator");
    sim_thread_running_ = false;

    if (sim_thread_.joinable()) sim_thread_.join();

    // Restore async decode mode for real audio operation.
    modem_.setSynchronousMode(false);
    if (virtual_modem_) {
        virtual_modem_->setSynchronousMode(false);
    }

    // Clear buffers
    {
        std::lock_guard<std::mutex> lock(our_tx_pending_mutex_);
        our_tx_pending_.clear();
    }
    {
        std::lock_guard<std::mutex> lock(virtual_tx_pending_mutex_);
        virtual_tx_pending_.clear();
    }

    guiLog("SIM: Simulator stopped");
}

void App::simulationLoop() {
    guiLog("SIM: Simulation loop started");

    constexpr size_t CHUNK_SIZE = 480;  // 10ms at 48kHz for waterfall display
    // Audio streaming: feed samples at REAL-TIME rate to match StreamingDecoder expectations
    // Loop runs every 10ms, feed 480 samples = 10ms of audio at 48kHz
    // This matches how real audio arrives and how cli_simulator works
    constexpr size_t SAMPLES_PER_TICK = 480;  // 10ms at 48kHz
    auto last_protocol_tick = std::chrono::steady_clock::now();

    // Intermediate buffers for gradual streaming
    std::vector<float> our_channel_buffer;     // Our TX -> channel -> virtual RX
    std::vector<float> virtual_channel_buffer; // Virtual TX -> channel -> our RX

    while (sim_thread_running_) {
        if (sim_drop_local_tx_requested_.exchange(false)) {
            size_t dropped_pending = 0;
            {
                std::lock_guard<std::mutex> lock(our_tx_pending_mutex_);
                dropped_pending = our_tx_pending_.size();
                our_tx_pending_.clear();
            }
            size_t dropped_in_flight = our_channel_buffer.size();
            our_channel_buffer.clear();
            tx_in_progress_ = false;
            guiLog("SIM: STOP TX NOW dropped %zu queued + %zu in-flight TX samples",
                   dropped_pending, dropped_in_flight);
        }

        bool a_to_b_active = false;  // Track activity per direction
        bool b_to_a_active = false;

        // === Our TX -> Channel buffer (queue all new samples) ===
        // Direction 0: our station -> virtual station (independent fading channel)
        {
            std::lock_guard<std::mutex> lock(our_tx_pending_mutex_);
            if (!our_tx_pending_.empty()) {
                guiLog("SIM: Queued %zu TX samples for streaming", our_tx_pending_.size());
                // Apply channel effects and queue for gradual streaming
                auto noisy = applyChannelEffects(our_tx_pending_, 0);
                our_channel_buffer.insert(our_channel_buffer.end(), noisy.begin(), noisy.end());
                // Show on waterfall
                for (size_t i = 0; i < our_tx_pending_.size(); i += CHUNK_SIZE) {
                    size_t chunk_size = std::min(CHUNK_SIZE, our_tx_pending_.size() - i);
                    if (waterfall_) {
                        waterfall_->addSamples(our_tx_pending_.data() + i, chunk_size);
                    }
                }
                // Record if enabled
                if (recording_enabled_) {
                    recorded_samples_.insert(recorded_samples_.end(), noisy.begin(), noisy.end());
                }
                our_tx_pending_.clear();
            }
        }

        // === Channel buffer -> Virtual RX (stream gradually) ===
        if (!our_channel_buffer.empty()) {
            a_to_b_active = true;
            size_t to_feed = std::min(SAMPLES_PER_TICK, our_channel_buffer.size());
            virtual_modem_->feedAudio(our_channel_buffer.data(), to_feed);
            if (virtual_modem_->isSynchronousMode()) {
                virtual_modem_->processRxBuffer();
            }
            our_channel_buffer.erase(our_channel_buffer.begin(), our_channel_buffer.begin() + to_feed);
        }

        // === Virtual TX -> Channel buffer (queue all new samples) ===
        // Direction 1: virtual station -> our station (independent fading channel)
        {
            std::lock_guard<std::mutex> lock(virtual_tx_pending_mutex_);
            if (!virtual_tx_pending_.empty()) {
                guiLog("SIM: Queued %zu RX samples for streaming", virtual_tx_pending_.size());
                // Apply channel effects and queue for gradual streaming
                auto noisy = applyChannelEffects(virtual_tx_pending_, 1);
                virtual_channel_buffer.insert(virtual_channel_buffer.end(), noisy.begin(), noisy.end());
                // Record if enabled
                if (recording_enabled_) {
                    recorded_samples_.insert(recorded_samples_.end(), noisy.begin(), noisy.end());
                }
                virtual_tx_pending_.clear();
            }
        }

        // === Channel buffer -> Our RX (stream gradually) ===
        if (!virtual_channel_buffer.empty()) {
            b_to_a_active = true;
            size_t to_feed = std::min(SAMPLES_PER_TICK, virtual_channel_buffer.size());
            modem_.feedAudio(virtual_channel_buffer.data(), to_feed);
            if (modem_.isSynchronousMode()) {
                modem_.processRxBuffer();
            }
            virtual_channel_buffer.erase(virtual_channel_buffer.begin(), virtual_channel_buffer.begin() + to_feed);
        }

        // Check if TX finished
        if (tx_in_progress_ && std::chrono::steady_clock::now() >= tx_end_time_) {
            tx_in_progress_ = false;
        }

        // Tick virtual protocol (~60Hz)
        auto now = std::chrono::steady_clock::now();
        auto protocol_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - last_protocol_tick).count();
        if (protocol_elapsed >= 16) {
            virtual_protocol_.tick(protocol_elapsed);
            last_protocol_tick = now;
        }

        // Evolve idle channels and feed noise to modems without active signal.
        // CRITICAL: Each direction's fading channel must evolve continuously by
        // processing silence, even when only the other direction has traffic.
        // Without this, the channel freezes during idle periods and
        // retransmissions hit the same deep fade repeatedly (frozen channel bug).
        {
            constexpr size_t IDLE_SAMPLES_PER_TICK = 480;  // 10ms at 48kHz

            if (!a_to_b_active) {
                // A→B channel idle: evolve fading and feed noise to virtual modem
                std::vector<float> silence(IDLE_SAMPLES_PER_TICK, 0.0f);
                auto noise = applyChannelEffects(silence, 0);
                virtual_modem_->feedAudio(noise);
                if (virtual_modem_->isSynchronousMode()) {
                    virtual_modem_->processRxBuffer();
                }
            }
            if (!b_to_a_active) {
                // B→A channel idle: evolve fading and feed noise to our modem
                std::vector<float> silence(IDLE_SAMPLES_PER_TICK, 0.0f);
                auto noise = applyChannelEffects(silence, 1);
                modem_.feedAudio(noise);
                if (modem_.isSynchronousMode()) {
                    modem_.processRxBuffer();
                }
            }
        }

        // Sleep 10ms to match real-time audio rate (480 samples / 48kHz = 10ms)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    guiLog("SIM: Simulation loop stopped");
}

void App::initAudio() {
    if (audio_initialized_) return;

    ultra::gui::startupTrace("App", "initAudio-enter");
    if (!audio_.initialize()) {
        ultra::gui::startupTrace("App", "initAudio-audio-initialize-fail");
        return;
    }
    ultra::gui::startupTrace("App", "initAudio-audio-initialize-ok");

    // Enumerate devices
    ultra::gui::startupTrace("App", "initAudio-enum-input-enter");
    input_devices_ = audio_.getInputDevices();
    ultra::gui::startupTrace("App", "initAudio-enum-input-exit");
    ultra::gui::startupTrace("App", "initAudio-enum-output-enter");
    output_devices_ = audio_.getOutputDevices();
    ultra::gui::startupTrace("App", "initAudio-enum-output-exit");

    // Populate settings window device lists
    settings_window_.input_devices = input_devices_;
    settings_window_.output_devices = output_devices_;
    guiLog("initAudio: enumerated %zu input device(s), %zu output device(s)",
           input_devices_.size(), output_devices_.size());

    audio_initialized_ = true;
    ultra::gui::startupTrace("App", "initAudio-exit");
}

void App::appendRxLogLine(const std::string& msg) {
    std::lock_guard<std::mutex> lock(rx_log_mutex_);
    rx_log_.push_back(msg);
    while (rx_log_.size() > MAX_RX_LOG) {
        rx_log_.pop_front();
    }
}

std::deque<std::string> App::snapshotRxLog() const {
    std::lock_guard<std::mutex> lock(rx_log_mutex_);
    return rx_log_;
}

void App::clearRxLog() {
    std::lock_guard<std::mutex> lock(rx_log_mutex_);
    rx_log_.clear();
}

void App::sendMessage() {
    // Not used in current implementation - messages sent via protocol
}

void App::onDataReceived(const std::string& text) {
    if (!text.empty()) {
        appendRxLogLine("[RX] " + text);
    }
}

void App::resetAdaptiveAdvisory() {
    std::lock_guard<std::mutex> lock(adapt_mutex_);
    adapt_snr_window_.clear();
    adapt_fading_window_.clear();
    adapt_candidate_valid_ = false;
    adapt_candidate_hits_ = 0;
    adapt_virtual_mode_valid_ = false;
    adapt_upgrade_hold_logged_ = false;
}

void App::updateAdaptiveAdvisory(float snr_db, float fading_index) {
    if (protocol_.getState() != protocol::ConnectionState::CONNECTED) {
        return;
    }
    if (!std::isfinite(snr_db) || !std::isfinite(fading_index)) {
        return;
    }
    std::lock_guard<std::mutex> lock(adapt_mutex_);

    adapt_snr_window_.push_back(snr_db);
    adapt_fading_window_.push_back(fading_index);
    if (adapt_snr_window_.size() > ADAPT_WINDOW_FRAMES) {
        adapt_snr_window_.pop_front();
    }
    if (adapt_fading_window_.size() > ADAPT_WINDOW_FRAMES) {
        adapt_fading_window_.pop_front();
    }
    if (adapt_snr_window_.size() < ADAPT_WINDOW_FRAMES ||
        adapt_fading_window_.size() < ADAPT_WINDOW_FRAMES) {
        return;
    }

    float avg_snr = 0.0f;
    for (float v : adapt_snr_window_) avg_snr += v;
    avg_snr /= static_cast<float>(adapt_snr_window_.size());

    float avg_fading = 0.0f;
    for (float v : adapt_fading_window_) avg_fading += v;
    avg_fading /= static_cast<float>(adapt_fading_window_.size());

    auto waveform = modem_.getWaveformMode();
    Modulation current_mod = protocol_.getDataModulation();
    CodeRate current_rate = protocol_.getDataCodeRate();

    if (!adapt_virtual_mode_valid_) {
        adapt_virtual_mode_valid_ = true;
        adapt_virtual_mod_ = current_mod;
        adapt_virtual_rate_ = current_rate;
        adapt_last_virtual_switch_ = std::chrono::steady_clock::now();
    }

    Modulation rec_mod = current_mod;
    CodeRate rec_rate = current_rate;
    int rec_carriers = 10;
    protocol::recommendDataMode(avg_snr, waveform, rec_mod, rec_rate, avg_fading, &rec_carriers);

    Modulation eval_mod = adapt_virtual_mod_;
    CodeRate eval_rate = adapt_virtual_rate_;

    if (rec_mod == eval_mod && rec_rate == eval_rate) {
        adapt_candidate_valid_ = false;
        adapt_candidate_hits_ = 0;
        adapt_upgrade_hold_logged_ = false;
        return;
    }

    if (adapt_candidate_valid_ &&
        adapt_candidate_mod_ == rec_mod &&
        adapt_candidate_rate_ == rec_rate) {
        ++adapt_candidate_hits_;
    } else {
        adapt_candidate_valid_ = true;
        adapt_candidate_mod_ = rec_mod;
        adapt_candidate_rate_ = rec_rate;
        adapt_candidate_hits_ = 1;
    }

    bool is_upgrade = modeEfficiency(rec_mod, rec_rate) > modeEfficiency(eval_mod, eval_rate) + 0.05f;
    int required_windows = is_upgrade ? ADAPT_UPGRADE_WINDOWS : ADAPT_DOWNGRADE_WINDOWS;
    if (adapt_candidate_hits_ < required_windows) {
        return;
    }

    auto now = std::chrono::steady_clock::now();
    if (is_upgrade) {
        auto elapsed_ms = static_cast<int>(
            std::chrono::duration_cast<std::chrono::milliseconds>(now - adapt_last_virtual_switch_).count());
        int hold_remaining_ms = ADAPT_UPGRADE_HOLD_MS - elapsed_ms;
        if (hold_remaining_ms > 0) {
            if (!adapt_upgrade_hold_logged_ ||
                adapt_upgrade_hold_mod_ != rec_mod ||
                adapt_upgrade_hold_rate_ != rec_rate) {
                char hold_msg[260];
                snprintf(hold_msg, sizeof(hold_msg),
                         "[ADPT] Local improving conditions (SNR=%.1f dB, F.I.=%.2f): "
                         "hysteresis hold %.1fs before upgrade to %s %s",
                         avg_snr, avg_fading,
                         hold_remaining_ms / 1000.0f,
                         modulationToString(rec_mod), codeRateToString(rec_rate));
                guiLog("%s", hold_msg);
                appendRxLogLine(hold_msg);
                adapt_upgrade_hold_logged_ = true;
                adapt_upgrade_hold_mod_ = rec_mod;
                adapt_upgrade_hold_rate_ = rec_rate;
            }
            return;
        }
    }

    adapt_upgrade_hold_logged_ = false;
    const char* direction = adaptationDirection(eval_mod, eval_rate, rec_mod, rec_rate);
    char msg[240];
    snprintf(msg, sizeof(msg),
             "[ADPT] Local %s conditions (SNR=%.1f dB, F.I.=%.2f): "
             "hysteresis allows switch %s %s -> %s %s",
             direction, avg_snr, avg_fading,
             modulationToString(eval_mod), codeRateToString(eval_rate),
             modulationToString(rec_mod), codeRateToString(rec_rate));

    guiLog("%s", msg);
    appendRxLogLine(msg);

    adapt_virtual_mod_ = rec_mod;
    adapt_virtual_rate_ = rec_rate;
    adapt_last_virtual_switch_ = now;
    adapt_candidate_valid_ = false;
    adapt_candidate_hits_ = 0;

    // If AUTO mode is enabled (not user-forced), actually request the mode change
    if (protocol_.getForcedModulation() == Modulation::AUTO) {
        uint8_t reason = is_upgrade
            ? protocol::v2::ModeChangeReason::CHANNEL_IMPROVED
            : protocol::v2::ModeChangeReason::CHANNEL_DEGRADED;
        protocol_.requestModeChange(rec_mod, rec_rate, avg_snr, reason);

        // Update carrier count for MC-DPSK DBPSK modes
        if (waveform == protocol::WaveformMode::MC_DPSK) {
            modem_.setMCDPSKCarriers(rec_carriers);
        }

        guiLog("[ADPT] Requesting mode change to %s %s (reason=%s)",
               modulationToString(rec_mod), codeRateToString(rec_rate),
               is_upgrade ? "improved" : "degraded");
    }
}

void App::render() {
    static bool first_render = true;
    render_frames_seen_++;
    if (first_render) {
        ultra::gui::startupTrace("App", "render-enter");
    }
    // Keep output attenuation synchronized with the TX Drive slider.
    if (first_render) {
        ultra::gui::startupTrace("App", "render-set-output-gain-enter");
    }
    audio_.setOutputGain(settings_.tx_drive);
    if (first_render) {
        ultra::gui::startupTrace("App", "render-set-output-gain-exit");
    }

    // Process captured RX audio in the main thread.
    // Avoids feeding modem state directly from SDL callback threads.
    pollRadioRx();

    // Poll TCP host interface for commands
    pollHostInterface();

    // Check CQ listening timeout (after CQ TX, wait for PING response)
    checkCQListenTimeout();

    // Poll CAT controller for deferred PTT release and watchdog
    cat_controller_.poll();

    // Safe-startup mode: auto-start audio shortly after first frames.
    // Keeps process bring-up lightweight while preserving "auto-listen" behavior.
    if (deferred_audio_auto_init_pending_ &&
        !simulation_enabled_ &&
        !audio_initialized_) {
        if (render_frames_seen_ < 6) {
            if (!deferred_audio_wait_logged_) {
                guiLog("Deferred audio guard: waiting for UI warm-up frames (%u/6)",
                       render_frames_seen_);
                deferred_audio_wait_logged_ = true;
            }
        } else {
            deferred_audio_wait_logged_ = false;
            uint32_t now_ms = SDL_GetTicks();
            if (now_ms >= deferred_audio_auto_init_deadline_ms_) {
                ultra::gui::startupTrace("App", "deferred-audio-init-enter");
                initAudio();
                if (audio_initialized_) {
                    ultra::gui::startupTrace("App", "deferred-audio-open-output-enter");
                    deferred_radio_rx_start_pending_ = true;
                    deferred_radio_rx_start_deadline_ms_ = now_ms;
                    deferred_radio_rx_start_timeout_ms_ = now_ms + 3000;
                    deferred_radio_rx_start_attempts_ = 0;
                    guiLog("Deferred audio stage 1/2 complete: core ready, starting RX capture ASAP (timeout=3000ms)");
                    ultra::gui::startupTrace("App", "deferred-audio-open-output-exit");
                    deferred_audio_auto_init_pending_ = false;
                } else {
                    deferred_audio_auto_init_attempts_++;
                    ultra::gui::startupTrace("App", "deferred-audio-init-fail");
                    if (deferred_audio_auto_init_attempts_ >= 3) {
                        guiLog("Deferred audio auto-init failed after %d attempts, manual init required",
                               deferred_audio_auto_init_attempts_);
                        deferred_audio_auto_init_pending_ = false;
                    } else {
                        deferred_audio_auto_init_deadline_ms_ = now_ms + 700;
                        guiLog("Deferred audio auto-init retry %d/3 scheduled",
                               deferred_audio_auto_init_attempts_ + 1);
                    }
                }
            }
        }
    }

    // Stage 2: start RX capture only after playback is confirmed and warm.
    if (deferred_radio_rx_start_pending_ &&
        !simulation_enabled_ &&
        audio_initialized_) {
        uint32_t now_ms = SDL_GetTicks();
        if (now_ms >= deferred_radio_rx_start_deadline_ms_) {
            if (startRadioRx()) {
                guiLog("Deferred audio stage 2/2 complete: RX capture started");
                deferred_radio_rx_start_pending_ = false;
                deferred_radio_rx_start_attempts_ = 0;
            } else {
                deferred_radio_rx_start_attempts_++;
                if (now_ms >= deferred_radio_rx_start_timeout_ms_) {
                    guiLog("Deferred audio stage 2/2 timeout after %d attempts (%ums); manual audio init required",
                           deferred_radio_rx_start_attempts_, 3000u);
                    deferred_radio_rx_start_pending_ = false;
                } else {
                    deferred_radio_rx_start_deadline_ms_ = now_ms + 100;
                    if (deferred_radio_rx_start_attempts_ == 1 ||
                        (deferred_radio_rx_start_attempts_ % 10) == 0) {
                        uint32_t remaining_ms = deferred_radio_rx_start_timeout_ms_ - now_ms;
                        guiLog("Deferred audio stage 2/2 waiting for RX readiness (attempt=%d, remaining=%ums)",
                               deferred_radio_rx_start_attempts_, remaining_ms);
                    }
                }
            }
        }
    }

    // === DEBUG: Test signal keys (F1-F7) ===
    if (ImGui::IsKeyPressed(ImGuiKey_F1)) {
        auto tone = modem_.generateTestTone(1.0f);
        if (!queueRealTxSamples(tone, "TEST tone")) {
            appendRxLogLine("[TEST] Failed to queue tone TX");
        } else {
            appendRxLogLine("[TEST] Sent 1500 Hz tone");
        }
    }
    if (ImGui::IsKeyPressed(ImGuiKey_F2)) {
        auto samples = modem_.transmitTestPattern(0);
        if (!queueRealTxSamples(samples, "TEST pattern0")) {
            appendRxLogLine("[TEST] Failed to queue pattern TX");
        } else {
            appendRxLogLine("[TEST] Sent pattern: ALL ZEROS (LDPC encoded)");
        }
    }
    if (ImGui::IsKeyPressed(ImGuiKey_F3)) {
        auto samples = modem_.transmitTestPattern(1);
        if (!queueRealTxSamples(samples, "TEST pattern1")) {
            appendRxLogLine("[TEST] Failed to queue pattern TX");
        } else {
            appendRxLogLine("[TEST] Sent pattern: DEADBEEF (LDPC encoded)");
        }
    }
    if (ImGui::IsKeyPressed(ImGuiKey_F7)) {
        const char* test_file = "tests/data/test_connect_data_sequence.f32";
        size_t injected = modem_.injectSignalFromFile(test_file);
        if (injected > 0) {
            appendRxLogLine("[TEST] Injected " + std::to_string(injected) + " samples");
        } else {
            appendRxLogLine("[TEST] Failed to inject signal");
        }
    }

    // Protocol engine tick (our protocol always ticks in main thread for UI responsiveness)
    // Virtual station's protocol ticks in its own thread (virtualProtocolLoop)
    uint32_t now = SDL_GetTicks();
    uint32_t elapsed = (last_tick_time_ == 0) ? 0 : (now - last_tick_time_);
    last_tick_time_ = now;

    if (elapsed > 0 && elapsed < 1000) {
        protocol_.tick(elapsed);
    }

    // Create main window
    if (first_render) {
        ultra::gui::startupTrace("App", "render-main-window-enter");
    }
    ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(viewport->WorkPos);
    ImGui::SetNextWindowSize(viewport->WorkSize);

    ImGuiWindowFlags window_flags =
        ImGuiWindowFlags_NoTitleBar |
        ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoCollapse |
        ImGuiWindowFlags_NoBringToFrontOnFocus;

    ImGui::Begin("MainWindow", nullptr, window_flags);

    // Title bar
    ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "RIA");
    ImGui::SameLine();
    ImGui::TextDisabled("High-Speed HF Modem");

    // Log and Settings buttons
    ImGui::SameLine(ImGui::GetWindowWidth() - 150);
    if (ImGui::SmallButton("Log")) {
        log_window_open_ = !log_window_open_;
    }
    ImGui::SameLine();
    if (ImGui::SmallButton("Settings")) {
        settings_window_.open();
    }

    ImGui::Separator();

    // Main content area - Two column layout
    float content_height = ImGui::GetContentRegionAvail().y - 30;
    bool defer_monitoring = options_.safe_startup && !audio_initialized_;

    ImGui::BeginChild("ContentArea", ImVec2(0, content_height), false);

    float total_width = ImGui::GetContentRegionAvail().x;
    float left_width = total_width * 0.60f;  // Monitoring column

    // ========================================
    // LEFT COLUMN: Monitoring (Constellation + Channel Status + Waterfall)
    // ========================================
    ImGui::BeginChild("LeftPanel", ImVec2(left_width, 0), true);

    if (defer_monitoring) {
        ImGui::TextDisabled("Startup safety mode active");
        ImGui::TextDisabled("Monitoring widgets enabled after audio init");
    } else {
        // Constellation diagram
        ImGui::BeginChild("ConstellationArea", ImVec2(0, 220), false);
        auto symbols = modem_.getConstellationSymbols();
        constellation_.render(symbols, config_.modulation);
        ImGui::EndChild();

        ImGui::Separator();

        // Compact Channel Status (horizontal layout)
        auto modem_stats = modem_.getStats();
        // In simulation mode, use the slider SNR (that's the actual channel quality)
        if (simulation_enabled_) {
            modem_stats.snr_db = simulation_snr_db_;
        }
        auto data_mod = protocol_.getDataModulation();
        auto data_rate = protocol_.getDataCodeRate();
        auto conn_stats = protocol_.getStats();
        renderCompactChannelStatus(modem_stats, data_mod, data_rate, conn_stats);

        ImGui::Separator();

        // Waterfall (uses remaining space)
        if (waterfall_) {
            waterfall_->render();
        } else {
            ImGui::TextDisabled("Waterfall disabled");
        }
    }

    ImGui::EndChild();
    ImGui::SameLine();

    // ========================================
    // RIGHT COLUMN: Operating (Controls + Message Log)
    // ========================================
    ImGui::BeginChild("RightPanel", ImVec2(0, 0), true);
    renderOperateTab();
    ImGui::EndChild();

    ImGui::EndChild();

    // Status bar removed - stats moved to operate tab

    ImGui::End();
    if (first_render) {
        ultra::gui::startupTrace("App", "render-main-window-exit");
    }

    // Render settings window
    if (settings_window_.isVisible() && settings_window_.input_devices.empty()) {
        if (!audio_.isInitialized()) {
            audio_.initialize();
        }
        settings_window_.input_devices = audio_.getInputDevices();
        settings_window_.output_devices = audio_.getOutputDevices();
    }
    settings_window_.render(settings_);

    // Message Log popup window
    if (log_window_open_) {
        ImGui::SetNextWindowSize(ImVec2(500, 400), ImGuiCond_FirstUseEver);
        if (ImGui::Begin("Message Log", &log_window_open_)) {
            if (ImGui::SmallButton("Clear")) clearRxLog();
            ImGui::SameLine();
            auto rx_log_snapshot = snapshotRxLog();
            if (ImGui::SmallButton("Copy")) {
                std::string all_log;
                for (const auto& msg : rx_log_snapshot) all_log += msg + "\n";
                ImGui::SetClipboardText(all_log.c_str());
            }
            ImGui::SameLine();
            auto mstats = modem_.getStats();
            ImGui::TextDisabled("TX:%d RX:%d", mstats.frames_sent, mstats.frames_received);

            ImGui::Separator();
            ImGui::BeginChild("LogContent", ImVec2(0, 0), true);
            for (const auto& msg : rx_log_snapshot) {
                ImVec4 color(0.7f, 0.7f, 0.7f, 1.0f);
                if (msg.size() >= 4 && msg.substr(0, 4) == "[TX]") {
                    color = ImVec4(0.5f, 0.8f, 1.0f, 1.0f);
                } else if (msg.size() >= 3 && msg.substr(0, 3) == "[RX") {
                    color = ImVec4(0.5f, 1.0f, 0.5f, 1.0f);
                } else if (msg.size() >= 4 && msg.substr(0, 4) == "[SYS") {
                    color = ImVec4(0.8f, 0.8f, 0.8f, 1.0f);
                } else if (msg.find("[FAILED]") != std::string::npos) {
                    color = ImVec4(1.0f, 0.4f, 0.4f, 1.0f);
                }
                ImGui::PushStyleColor(ImGuiCol_Text, color);
                ImGui::TextWrapped("%s", msg.c_str());
                ImGui::PopStyleColor();
            }
            if (!rx_log_snapshot.empty()) ImGui::SetScrollHereY(1.0f);
            ImGui::EndChild();
        }
        ImGui::End();
    }

    if (first_render) {
        ultra::gui::startupTrace("App", "render-exit");
        first_render = false;
    }
}

std::string App::getInputDeviceName() const {
    if (strcmp(settings_.input_device, "Default") == 0 || settings_.input_device[0] == '\0') {
        return "";
    }
    return settings_.input_device;
}

std::string App::getOutputDeviceName() const {
    if (strcmp(settings_.output_device, "Default") == 0 || settings_.output_device[0] == '\0') {
        return "";
    }
    return settings_.output_device;
}

static SerialPttLine serialPttLineFromSettings(const AppSettings& settings) {
    return (settings.ptt_serial_line == 1) ? SerialPttLine::RTS : SerialPttLine::DTR;
}

bool App::ensureSerialPttReady() {
    if (!settings_.use_cat_ptt) {
        return true;
    }

    size_t port_len = boundedCStringLen(settings_.rig_port);
    if (port_len == 0) {
        guiLog("PTT: enabled but serial port is empty");
        appendRxLogLine("[PTT] Serial PTT enabled but no serial port configured");
        return false;
    }

    std::string port(settings_.rig_port, port_len);
    int baud = (settings_.rig_baud > 0) ? settings_.rig_baud : 9600;

    if (serial_ptt_.matches(port, baud)) {
        return true;
    }

    closeSerialPtt();
    if (!serial_ptt_.open(port, baud)) {
        char buf[200];
        snprintf(buf, sizeof(buf), "[PTT] Failed to open %s @ %d", port.c_str(), baud);
        appendRxLogLine(buf);
        return false;
    }

    // Force selected line into known inactive state immediately after opening.
    // This avoids relying on platform/driver defaults.
    bool active_high = !settings_.ptt_invert;
    bool inactive_state = active_high ? false : true;
    SerialPttLine line = serialPttLineFromSettings(settings_);
    if (!serial_ptt_.setLine(line, inactive_state)) {
        char buf[220];
        snprintf(buf, sizeof(buf), "[PTT] Failed to initialize %s line state",
                 line == SerialPttLine::DTR ? "DTR" : "RTS");
        appendRxLogLine(buf);
        guiLog("PTT: failed to set initial inactive state on %s",
               line == SerialPttLine::DTR ? "DTR" : "RTS");
        closeSerialPtt();
        return false;
    }

    ptt_active_ = false;
    guiLog("PTT: serial line ready on %s @ %d", port.c_str(), baud);
    return true;
}

bool App::setSerialPtt(bool asserted, const char* reason) {
    if (!settings_.use_cat_ptt) {
        ptt_active_ = false;
        return true;
    }

    if (!ensureSerialPttReady()) {
        ptt_active_ = false;
        return false;
    }

    bool active_high = !settings_.ptt_invert;
    bool line_state = active_high ? asserted : !asserted;
    SerialPttLine line = serialPttLineFromSettings(settings_);
    if (!serial_ptt_.setLine(line, line_state)) {
        guiLog("PTT: failed to set %s=%d (%s)",
               line == SerialPttLine::DTR ? "DTR" : "RTS",
               static_cast<int>(line_state),
               reason ? reason : "n/a");
        if (asserted) {
            appendRxLogLine("[PTT] Failed to key serial PTT line");
        }
        ptt_active_ = false;
        return false;
    }

    ptt_active_ = asserted;
    guiLog("PTT: %s via %s (%s, invert=%d)",
           asserted ? "ASSERT" : "RELEASE",
           line == SerialPttLine::DTR ? "DTR" : "RTS",
           reason ? reason : "n/a",
           settings_.ptt_invert ? 1 : 0);
    return true;
}

bool App::assertTxPtt(const char* reason) {
    ptt_release_pending_ = false;
    ptt_release_deadline_ms_ = 0;

    // Prefer CAT PTT when CAT is already connected.
    if (settings_.cat_enabled && cat_controller_.isConnected()) {
        if (!cat_controller_.assertPtt(reason ? reason : "tx_start")) {
            guiLog("CAT PTT: assert failed (%s)", cat_controller_.getLastError().c_str());
            appendRxLogLine("[CAT] Failed to key CAT PTT");
            ptt_active_ = false;
            tx_ptt_path_ = TxPttPath::None;
            return false;
        }

        tx_ptt_path_ = TxPttPath::Cat;
        ptt_active_ = true;
        if (settings_.cat_ptt_lead_ms > 0) {
            SDL_Delay(static_cast<Uint32>(settings_.cat_ptt_lead_ms));
        }
        return true;
    }

    if (settings_.use_cat_ptt) {
        if (!setSerialPtt(true, reason ? reason : "tx_start")) {
            tx_ptt_path_ = TxPttPath::None;
            return false;
        }
        tx_ptt_path_ = TxPttPath::Serial;
        if (settings_.tx_delay_ms > 0) {
            SDL_Delay(static_cast<Uint32>(settings_.tx_delay_ms));
        }
        return true;
    }

    tx_ptt_path_ = TxPttPath::None;
    ptt_active_ = false;
    return true;
}

void App::releaseTxPtt(const char* reason, bool immediate) {
    const char* why = reason ? reason : (immediate ? "tx_stop_immediate" : "tx_stop");
    ptt_release_pending_ = false;
    ptt_release_deadline_ms_ = 0;

    if (tx_ptt_path_ == TxPttPath::Cat || cat_controller_.isPttActive()) {
        if (immediate) {
            cat_controller_.releasePttImmediate(why);
        } else {
            cat_controller_.releasePtt(why);
            if (cat_controller_.isPttActive()) {
                ptt_release_pending_ = true;
                ptt_active_ = true;
                tx_ptt_path_ = TxPttPath::Cat;
                return;
            }
        }

        ptt_active_ = false;
        tx_ptt_path_ = TxPttPath::None;
        return;
    }

    if (tx_ptt_path_ == TxPttPath::Serial) {
        if (immediate) {
            releaseSerialPtt(why);
            tx_ptt_path_ = TxPttPath::None;
        } else {
            ptt_release_pending_ = true;
            ptt_release_deadline_ms_ =
                SDL_GetTicks() + static_cast<uint32_t>(std::max(0, settings_.tx_tail_ms));
            ptt_active_ = true;
        }
        return;
    }

    ptt_active_ = false;
    tx_ptt_path_ = TxPttPath::None;
}

void App::releaseSerialPtt(const char* reason) {
    ptt_release_pending_ = false;
    ptt_release_deadline_ms_ = 0;
    if (!serial_ptt_.isOpen()) {
        ptt_active_ = false;
        tx_ptt_path_ = TxPttPath::None;
        return;
    }

    bool active_high = !settings_.ptt_invert;
    bool line_state = active_high ? false : true;
    SerialPttLine line = serialPttLineFromSettings(settings_);
    if (!serial_ptt_.setLine(line, line_state)) {
        guiLog("PTT: failed to release line (%s)", reason ? reason : "n/a");
    } else {
        guiLog("PTT: RELEASE via %s (%s)",
               line == SerialPttLine::DTR ? "DTR" : "RTS",
               reason ? reason : "n/a");
    }
    ptt_active_ = false;
    tx_ptt_path_ = TxPttPath::None;
}

void App::closeSerialPtt() {
    ptt_active_ = false;
    ptt_release_pending_ = false;
    ptt_release_deadline_ms_ = 0;
    tx_ptt_path_ = TxPttPath::None;
    serial_ptt_.close();
}

bool App::queueRealTxSamples(const std::vector<float>& samples, const char* context) {
    if (samples.empty()) {
        return false;
    }

    // Abort pending release if a new TX starts quickly after previous frame.
    ptt_release_pending_ = false;
    ptt_release_deadline_ms_ = 0;

    // Mute RX and clear buffers to avoid local feedback decode.
    audio_.setRxMuted(true);
    audio_.stopCapture();
    audio_.clearRxBuffer();
    modem_.clearRxBuffer();

    if (!audio_.hasOutputDevice()) {
        std::string output_dev = getOutputDeviceName();
        if (!audio_.openOutput(output_dev)) {
            guiLog("%s: openOutput failed for '%s'",
                   context ? context : "TX audio",
                   output_dev.empty() ? "Default" : output_dev.c_str());
            audio_.setRxMuted(false);
            if (radio_rx_enabled_ && !audio_.isCapturing()) {
                audio_.startCapture();
            }
            return false;
        }
    }

    if (!assertTxPtt(context ? context : "tx_start")) {
        audio_.setRxMuted(false);
        if (radio_rx_enabled_ && !audio_.isCapturing()) {
            audio_.startCapture();
        }
        return false;
    }

    size_t tx_duration_ms = (samples.size() * 1000) / 48000;
    tx_in_progress_ = true;
    tx_end_time_ = std::chrono::steady_clock::now() + std::chrono::milliseconds(tx_duration_ms + 100);

    if (waterfall_) {
        waterfall_->addSamples(samples.data(), samples.size());
    }

    if (recording_enabled_) {
        recorded_tx_samples_.insert(recorded_tx_samples_.end(), samples.begin(), samples.end());
    }

    audio_.startPlayback();
    audio_.queueTxSamples(samples);
    return true;
}

bool App::startRadioRx() {
    if (!audio_initialized_) {
        guiLog("startRadioRx guard: audio not initialized");
        return false;
    }
    if (simulation_enabled_) {
        guiLog("startRadioRx guard: simulation enabled");
        return false;
    }
    if (radio_rx_enabled_) {
        return true;
    }

    audio_.setInputCaptureMode(
        radio_rx_force_queue_mode_
            ? AudioEngine::InputCaptureMode::Queue
            : AudioEngine::InputCaptureMode::Auto);

    std::string input_dev = getInputDeviceName();
    if (!input_dev.empty()) {
        bool found = false;
        for (const auto& dev : input_devices_) {
            if (dev == input_dev) {
                found = true;
                break;
            }
        }
        if (!found) {
            guiLog("startRadioRx: configured input device missing: '%s', falling back to Default",
                   input_dev.c_str());
            input_dev.clear();
        }
    }
    if (!audio_.openInput(input_dev)) {
        guiLog("startRadioRx: openInput failed for '%s'",
               input_dev.empty() ? "Default" : input_dev.c_str());
        return false;
    }

    // Main thread polls captured samples via pollRadioRx().
    audio_.setRxCallback(AudioEngine::RxCallback{});

    audio_.setLoopbackEnabled(false);
    audio_.startCapture();
    if (!audio_.isCapturing()) {
        guiLog("startRadioRx: capture did not start");
        audio_.closeInput();
        return false;
    }
    radio_rx_enabled_ = true;
    radio_rx_started_ms_ = SDL_GetTicks();
    radio_rx_warmup_logged_ = false;
    radio_rx_first_chunk_logged_ = false;
    radio_rx_no_data_deadline_ms_ = radio_rx_started_ms_ + 1000;
    radio_rx_rearm_attempts_ = 0;
    radio_rx_rearm_exhausted_logged_ = false;
    radio_rx_active_device_ = input_dev.empty() ? "Default" : input_dev;
    radio_rx_output_prime_attempted_ = false;
    guiLog("startRadioRx: capture started on '%s'",
           input_dev.empty() ? "Default" : input_dev.c_str());
    guiLog("startRadioRx: input capture backend=%s",
           audio_.isInputQueueMode() ? "queue" : "callback");
    return true;
}

void App::stopRadioRx() {
    audio_.stopCapture();
    audio_.closeInput();
    audio_.setRxCallback(AudioEngine::RxCallback{});
    radio_rx_enabled_ = false;
    radio_rx_started_ms_ = 0;
    radio_rx_warmup_logged_ = false;
    radio_rx_first_chunk_logged_ = false;
    radio_rx_no_data_deadline_ms_ = 0;
    radio_rx_rearm_attempts_ = 0;
    radio_rx_rearm_exhausted_logged_ = false;
    radio_rx_active_device_.clear();
    radio_rx_output_prime_attempted_ = false;
}

void App::pollRadioRx() {
    if (!audio_initialized_ || simulation_enabled_ || !radio_rx_enabled_) {
        return;
    }

    uint32_t now_ms = SDL_GetTicks();
    if (radio_rx_started_ms_ > 0 && (now_ms - radio_rx_started_ms_) < 200) {
        if (!radio_rx_warmup_logged_) {
            guiLog("pollRadioRx warm-up: delaying modem feed for 200ms after capture start");
            radio_rx_warmup_logged_ = true;
        }
        // Drain a small amount during warm-up to avoid unbounded growth.
        (void)audio_.getRxSamples(2048);
        return;
    }

    // Some Linux/USB stacks can start capture without delivering samples immediately.
    // Rearm capture on the SAME configured device a few times before giving up.
    if (!radio_rx_first_chunk_logged_ &&
        radio_rx_no_data_deadline_ms_ > 0 &&
        now_ms >= radio_rx_no_data_deadline_ms_) {
        SDL_AudioStatus in_status = audio_.getInputStatus();
        const char* in_status_str = (in_status == SDL_AUDIO_PLAYING) ? "playing" :
                                    (in_status == SDL_AUDIO_PAUSED)  ? "paused"  :
                                                                       "stopped";
        guiLog("pollRadioRx: no RX data diagnostics: backend=%s status=%s queued=%u bytes",
               audio_.isInputQueueMode() ? "queue" : "callback",
               in_status_str,
               static_cast<unsigned>(audio_.getQueuedInputBytes()));

#ifndef _WIN32
        // Some USB radio codecs require an active output stream to keep duplex
        // clocking alive. Prime output once before escalating capture recovery.
        if (!radio_rx_output_prime_attempted_) {
            radio_rx_output_prime_attempted_ = true;
            std::string output_dev = getOutputDeviceName();
            if (!audio_.hasOutputDevice()) {
                if (audio_.openOutput(output_dev)) {
                    audio_.startPlayback();  // plays silence until TX queue has data
                    guiLog("pollRadioRx: primed output '%s' to wake duplex capture",
                           output_dev.empty() ? "Default" : output_dev.c_str());
                } else {
                    guiLog("pollRadioRx: output prime failed for '%s'",
                           output_dev.empty() ? "Default" : output_dev.c_str());
                }
            } else {
                audio_.startPlayback();
                guiLog("pollRadioRx: output already open; playback started for duplex prime");
            }
            radio_rx_no_data_deadline_ms_ = now_ms + 1000;
            return;
        }
#endif

        // Linux USB edge case: callback capture can stall until mixer state changes.
        // Fall back to queued capture on the SAME configured device.
#ifndef _WIN32
        if (!radio_rx_force_queue_mode_ && radio_rx_rearm_attempts_ >= 2) {
            guiLog("pollRadioRx: no data on callback capture; switching '%s' to queued capture mode",
                   radio_rx_active_device_.empty() ? "Default" : radio_rx_active_device_.c_str());
            audio_.stopCapture();
            audio_.closeInput();
            radio_rx_enabled_ = false;
            radio_rx_force_queue_mode_ = true;
            if (startRadioRx()) {
                guiLog("pollRadioRx: queued capture fallback armed on '%s'",
                       radio_rx_active_device_.empty() ? "Default" : radio_rx_active_device_.c_str());
                return;
            }
            guiLog("pollRadioRx: queued capture fallback failed on '%s'",
                   radio_rx_active_device_.empty() ? "Default" : radio_rx_active_device_.c_str());
        }
#endif

        if (radio_rx_rearm_attempts_ < 4) {
            radio_rx_rearm_attempts_++;
            guiLog("pollRadioRx: no RX chunks after %ums on '%s'; rearming capture (%d/4)",
                   now_ms - radio_rx_started_ms_,
                   radio_rx_active_device_.empty() ? "Default" : radio_rx_active_device_.c_str(),
                   radio_rx_rearm_attempts_);
            // Escalate from soft pause/unpause to hard close/open on repeated stalls.
            bool hard_reopen = audio_.isInputQueueMode() || radio_rx_rearm_attempts_ >= 3;
            if (hard_reopen) {
                std::string input_dev = getInputDeviceName();
                audio_.stopCapture();
                audio_.closeInput();
                if (!audio_.openInput(input_dev)) {
                    guiLog("pollRadioRx: hard reopen failed for '%s'",
                           input_dev.empty() ? "Default" : input_dev.c_str());
                } else {
                    audio_.setRxCallback(AudioEngine::RxCallback{});
                    audio_.setLoopbackEnabled(false);
                    audio_.startCapture();
                    guiLog("pollRadioRx: hard reopened input '%s' (backend=%s)",
                           input_dev.empty() ? "Default" : input_dev.c_str(),
                           audio_.isInputQueueMode() ? "queue" : "callback");
                }
            } else {
                audio_.stopCapture();
                audio_.startCapture();
            }
            radio_rx_no_data_deadline_ms_ = now_ms + 1000;
        } else if (!radio_rx_rearm_exhausted_logged_) {
            guiLog("pollRadioRx: still no RX chunks after rearm attempts on '%s'; check OS input level/source",
                   radio_rx_active_device_.empty() ? "Default" : radio_rx_active_device_.c_str());
            radio_rx_rearm_exhausted_logged_ = true;
        }
    }

    // Bounded drain per frame to keep UI responsive while preventing RX backlog.
    constexpr size_t kChunkSamples = 2048;
    constexpr int kMaxChunksPerFrame = 8;
    for (int i = 0; i < kMaxChunksPerFrame; ++i) {
        auto samples = audio_.getRxSamples(kChunkSamples);
        if (samples.empty()) {
            break;
        }

        if (!radio_rx_first_chunk_logged_) {
            guiLog("pollRadioRx: first RX chunk=%zu samples", samples.size());
        }
        if (recording_enabled_) {
            recorded_rx_samples_.insert(recorded_rx_samples_.end(), samples.begin(), samples.end());
        }
        modem_.feedAudio(samples);
        if (!radio_rx_first_chunk_logged_) {
            guiLog("pollRadioRx: first RX chunk fed to modem");
        }
        if (modem_.isSynchronousMode()) {
            modem_.processRxBuffer();
        }
        if (!radio_rx_first_chunk_logged_) {
            guiLog("pollRadioRx: first RX chunk modem processing complete");
            radio_rx_first_chunk_logged_ = true;
            radio_rx_no_data_deadline_ms_ = 0;
            radio_rx_rearm_attempts_ = 0;
            radio_rx_rearm_exhausted_logged_ = false;
        }
        if (waterfall_) {
            waterfall_->addSamples(samples.data(), samples.size());
        }
    }
}

void App::stopTxNow(const char* reason) {
    // Abort protocol-level retransmit/resend timers before clearing audio queues
    // so no new outbound frames are scheduled after operator stop.
    protocol_.abortTxNow();

    size_t dropped_audio = audio_.getTxQueueSize();
    if (dropped_audio > 0) {
        audio_.clearTxQueue();
    }

    size_t dropped_sim_pending = 0;
    {
        std::lock_guard<std::mutex> lock(our_tx_pending_mutex_);
        dropped_sim_pending = our_tx_pending_.size();
        our_tx_pending_.clear();
    }

    if (simulation_enabled_ && sim_thread_running_) {
        sim_drop_local_tx_requested_ = true;
    }

    tx_in_progress_ = false;
    tx_end_time_ = std::chrono::steady_clock::time_point{};
    releaseTxPtt(reason ? reason : "stop_tx_now", true);

    // Return to RX immediately after aborting TX.
    if (!simulation_enabled_ && radio_rx_enabled_ && !ptt_active_) {
        audio_.setRxMuted(false);
        if (!audio_.isCapturing()) {
            audio_.startCapture();
        }
    }

    guiLog("STOP TX NOW: reason='%s', dropped_audio=%zu, dropped_sim_pending=%zu",
           reason, dropped_audio, dropped_sim_pending);

    appendRxLogLine("[SYS] TX stopped immediately");
}

void App::renderCompactChannelStatus(const LoopbackStats& stats, Modulation data_mod, CodeRate data_rate,
                                     const protocol::ConnectionStats& conn_stats) {
    // Compact horizontal Channel Status display
    ImGui::BeginChild("ChannelStatus", ImVec2(0, 85), false);

    // Mode indicator (moved from constellation widget)
    const char* mod_name = "Unknown";
    switch (data_mod) {
        case Modulation::DBPSK:  mod_name = "DBPSK"; break;
        case Modulation::BPSK:   mod_name = "BPSK"; break;
        case Modulation::DQPSK:  mod_name = "DQPSK"; break;
        case Modulation::QPSK:   mod_name = "QPSK"; break;
        case Modulation::D8PSK:  mod_name = "D8PSK"; break;
        case Modulation::QAM16:  mod_name = "16-QAM"; break;
        case Modulation::QAM32:  mod_name = "32-QAM"; break;
        case Modulation::QAM64:  mod_name = "64-QAM"; break;
        case Modulation::QAM256: mod_name = "256-QAM"; break;
        default: break;
    }
    ImGui::Text("Mode: %s", mod_name);

    auto conn_state = protocol_.getState();

    // Row 1: Connection state + Channel Quality (only when connected) + SNR bar
    if (conn_state == protocol::ConnectionState::DISCONNECTED) {
        // Not connected - show idle state
        ImGui::TextColored(ImVec4(0.5f, 0.5f, 0.5f, 1.0f), "IDLE");
        ImGui::SameLine();
        ImGui::TextDisabled("[Standby]");
        ImGui::SameLine();
        ImGui::Text("SNR:");
        ImGui::SameLine();
        // Empty SNR bar
        ImGui::PushStyleColor(ImGuiCol_PlotHistogram, ImVec4(0.3f, 0.3f, 0.3f, 1.0f));
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x);
        ImGui::ProgressBar(0.0f, ImVec2(-1, 16), "-- dB");
        ImGui::PopStyleColor();
    } else if (conn_state == protocol::ConnectionState::CONNECTING) {
        // Connecting - show our outgoing mode, no channel quality yet
        ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.2f, 1.0f), "CALL");
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.2f, 1.0f), "[Connecting...]");
        ImGui::SameLine();
        ImGui::Text("SNR:");
        ImGui::SameLine();
        // Animated/pulsing bar to show activity
        ImGui::PushStyleColor(ImGuiCol_PlotHistogram, ImVec4(1.0f, 0.8f, 0.2f, 1.0f));
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x);
        ImGui::ProgressBar(0.3f, ImVec2(-1, 16), "awaiting...");
        ImGui::PopStyleColor();
    } else if (conn_state == protocol::ConnectionState::DISCONNECTING) {
        // Disconnecting
        ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.2f, 1.0f), "DISC");
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.2f, 1.0f), "[Disconnecting...]");
        ImGui::SameLine();
        ImGui::Text("SNR:");
        ImGui::SameLine();
        ImGui::PushStyleColor(ImGuiCol_PlotHistogram, ImVec4(1.0f, 0.5f, 0.2f, 1.0f));
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x);
        ImGui::ProgressBar(0.5f, ImVec2(-1, 16), "closing...");
        ImGui::PopStyleColor();
    } else {
        // CONNECTED - show remote mode (their view) AND our SNR (our view)
        // Row 1: Remote's negotiated mode + implied channel condition
        auto waveform = modem_.getWaveformMode();
        const char* wf_str = waveformDisplayName(waveform);
        ImVec4 wf_color = (waveform == protocol::WaveformMode::OFDM_COX) ? ImVec4(0.4f, 0.8f, 1.0f, 1.0f) :
                          (waveform == protocol::WaveformMode::MC_DPSK) ? ImVec4(0.8f, 0.8f, 0.4f, 1.0f) :
                          (waveform == protocol::WaveformMode::MFSK) ? ImVec4(0.8f, 0.4f, 0.8f, 1.0f) :
                                                                       ImVec4(0.6f, 0.6f, 0.6f, 1.0f);
        ImGui::Text("RX:");
        ImGui::SameLine();
        ImGui::TextColored(wf_color, "%s", wf_str);
        ImGui::SameLine();

        // Show mode-appropriate settings and throughput
        float throughput_bps = 0.0f;
        ImVec4 mode_quality_color;
        const char* mode_quality = "Good";

        // Get actual channel quality from fading measurement
        float fading = modem_.getFadingIndex();
        mode_quality = fadingToQualityWithColor(fading, mode_quality_color);

        if (waveform == protocol::WaveformMode::MC_DPSK) {
            // For MC-DPSK, just show carrier count (DQPSK R1/4 is implicit)
            int carriers = modem_.getMCDPSKCarriers();
            throughput_bps = modem_.getMCDPSKThroughput();
            ImGui::Text("%d carriers", carriers);
        } else {
            // For OFDM modes, show negotiated modulation/rate
            ImGui::Text("%s %s", modulationToString(data_mod), codeRateToString(data_rate));
            throughput_bps = config_.getTheoreticalThroughput(data_mod, data_rate);
        }
        ImGui::SameLine();
        ImGui::TextColored(mode_quality_color, "[%s]", mode_quality);
        ImGui::TextDisabled("PHY ~%.1f kbps", throughput_bps / 1000.0f);

        // Row 2: Our SNR measurement
        ImVec4 sync_color = stats.synced ? ImVec4(0.2f, 1.0f, 0.2f, 1.0f) : ImVec4(0.5f, 0.5f, 0.5f, 1.0f);
        ImGui::TextColored(sync_color, "%s", stats.synced ? "SYNC" : "----");
        ImGui::SameLine();
        ImGui::Text("SNR:");
        ImGui::SameLine();

        // SNR bar - color indicates signal strength
        float snr_normalized = stats.snr_db / 40.0f;
        snr_normalized = std::max(0.0f, std::min(1.0f, snr_normalized));
        // Color based on SNR value (green=good signal, yellow=moderate, red=weak)
        ImVec4 snr_color;
        if (stats.snr_db >= 15.0f) {
            snr_color = ImVec4(0.2f, 1.0f, 0.2f, 1.0f);  // Green
        } else if (stats.snr_db >= 5.0f) {
            snr_color = ImVec4(0.8f, 0.8f, 0.0f, 1.0f);  // Yellow
        } else {
            snr_color = ImVec4(1.0f, 0.4f, 0.2f, 1.0f);  // Orange-red
        }
        ImGui::PushStyleColor(ImGuiCol_PlotHistogram, snr_color);
        char snr_text[16];
        snprintf(snr_text, sizeof(snr_text), "%.1f dB", stats.snr_db);
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x);
        ImGui::ProgressBar(snr_normalized, ImVec2(-1, 16), snr_text);
        ImGui::PopStyleColor();
    }

    // Row 2: Waveform + Mode (for connecting state only)
    if (conn_state == protocol::ConnectionState::CONNECTING) {
        // Show actual waveform being used for connection attempt (always R1/4)
        auto connect_wf = protocol_.getConnectWaveform();
        const char* wf_str = waveformDisplayName(connect_wf);
        ImVec4 wf_color = (connect_wf == protocol::WaveformMode::OFDM_COX) ? ImVec4(0.4f, 0.8f, 1.0f, 1.0f) :
                          (connect_wf == protocol::WaveformMode::MC_DPSK) ? ImVec4(0.8f, 0.8f, 0.4f, 1.0f) :
                          (connect_wf == protocol::WaveformMode::MFSK) ? ImVec4(0.8f, 0.4f, 0.8f, 1.0f) :
                                                                         ImVec4(0.6f, 0.6f, 0.6f, 1.0f);
        ImGui::TextColored(wf_color, "%s R1/4 (calling)", wf_str);
    }
    // Connected state already shows mode in Row 1

    // Row 3: Modem frame stats
    if (stats.frames_sent > 0 || stats.frames_received > 0) {
        ImGui::Text("TX:%d RX:%d", stats.frames_sent, stats.frames_received);
        if (stats.frames_failed > 0) {
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "(%d fail)", stats.frames_failed);
        }
    } else if (conn_state == protocol::ConnectionState::DISCONNECTED) {
        ImGui::TextDisabled("Ready to connect");
    }

    // Row 4: ARQ health (control-path reliability visibility)
    const auto& arq = conn_stats.arq;
    bool has_arq_activity =
        arq.frames_sent > 0 || arq.frames_received > 0 || arq.acks_sent > 0 || arq.acks_received > 0 ||
        arq.retransmissions > 0 || arq.timeouts > 0;
    if (has_arq_activity) {
        ImVec4 arq_color = (arq.failed > 0 || arq.timeouts > 0) ? ImVec4(1.0f, 0.4f, 0.4f, 1.0f) :
                           (arq.retransmissions > 0) ? ImVec4(0.9f, 0.8f, 0.3f, 1.0f) :
                           ImVec4(0.7f, 0.7f, 0.7f, 1.0f);
        char arq_line[256];
        snprintf(arq_line, sizeof(arq_line),
                 "ARQ retx:%d to:%d fast:%d probe:%d nack:%d dupACK:%d",
                 arq.retransmissions, arq.timeouts, arq.retransmissions_fast_hole,
                 arq.retransmissions_hole_probe, arq.retransmissions_nack,
                 arq.duplicate_acks_ignored);
        ImGui::TextColored(arq_color, "%s", arq_line);
        if (arq.failed > 0) {
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f), "fail:%d", arq.failed);
        }
    }

    ImGui::EndChild();
}

void App::renderOperateTab() {
    // Calculate available height for layout
    float total_height = ImGui::GetContentRegionAvail().y;

    // ========================================
    // TOP SECTION: Connection Controls (compact)
    // ========================================

    // Audio initialization (auto-start when not in simulation)
    if (!simulation_enabled_ && !audio_initialized_) {
        if (deferred_audio_auto_init_pending_) {
            ImGui::TextDisabled("Starting audio...");
            if (ImGui::Button("Start Audio Now", ImVec2(-1, 28))) {
                deferred_audio_auto_init_deadline_ms_ = 0;
            }
        } else {
            if (ImGui::Button("Initialize Audio", ImVec2(-1, 28))) {
                initAudio();
            }
        }
        return;
    }

    // ========================================
    // Status Row: Connection state + controls
    // (Connect/Disconnect handled via TCP commands)
    // ========================================
    auto conn_state = protocol_.getState();

    // Status line - auto-start RX if not running
    if (!simulation_enabled_ && !radio_rx_enabled_) {
        // Auto-start RX when audio is ready
        if (audio_initialized_) {
            startRadioRx();
        }
        ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.2f, 1.0f), "STARTING...");
    } else {
        ImVec4 state_color;
        const char* state_icon;
        switch (conn_state) {
            case protocol::ConnectionState::CONNECTED:
                state_color = ImVec4(0.2f, 1.0f, 0.2f, 1.0f);
                state_icon = "CONNECTED";
                break;
            case protocol::ConnectionState::CONNECTING:
                state_color = ImVec4(1.0f, 1.0f, 0.2f, 1.0f);
                state_icon = "CONNECTING...";
                break;
            case protocol::ConnectionState::DISCONNECTING:
                state_color = ImVec4(1.0f, 1.0f, 0.2f, 1.0f);
                state_icon = "DISCONNECTING...";
                break;
            default:
                state_color = ImVec4(0.3f, 0.8f, 1.0f, 1.0f);
                state_icon = "LISTENING";
                break;
        }
        ImGui::TextColored(state_color, "%s", state_icon);
        if (conn_state == protocol::ConnectionState::CONNECTED) {
            ImGui::SameLine();
            ImGui::Text("to %s", protocol_.getRemoteCallsign().c_str());
        }
    }

    // STOP TX button (emergency abort)
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.78f, 0.18f, 0.18f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.88f, 0.24f, 0.24f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.68f, 0.14f, 0.14f, 1.0f));
    if (ImGui::Button("STOP TX", ImVec2(80, 0))) {
        stopTxNow("operator_button");
    }
    ImGui::PopStyleColor(3);

    // Show connected remote callsign
    if (conn_state == protocol::ConnectionState::CONNECTED) {
        ImGui::SameLine();
        ImGui::Text("Connected: %s", protocol_.getRemoteCallsign().c_str());
    }

    // Incoming call notification (display only - accept/reject via TCP)
    std::string pending_call_snapshot;
    {
        std::lock_guard<std::mutex> lock(rx_log_mutex_);
        pending_call_snapshot = pending_incoming_call_;
    }
    if (!pending_call_snapshot.empty()) {
        ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.0f, 1.0f),
                           "Incoming call from %s (use TCP to accept/reject)", pending_call_snapshot.c_str());
    }

    // Audio level meter (compact, only when RX active)
    if (!simulation_enabled_ && radio_rx_enabled_) {
        float input_level = audio_.getInputLevel();
        float input_db = (input_level > 0.0001f) ? 20.0f * log10f(input_level) : -80.0f;
        float level_normalized = (input_db + 60.0f) / 60.0f;
        level_normalized = std::max(0.0f, std::min(1.0f, level_normalized));
        ImVec4 level_color = (level_normalized > 0.8f) ? ImVec4(1.0f, 0.3f, 0.3f, 1.0f) :
                             (level_normalized > 0.5f) ? ImVec4(1.0f, 1.0f, 0.3f, 1.0f) :
                                                         ImVec4(0.3f, 1.0f, 0.3f, 1.0f);
        ImGui::PushStyleColor(ImGuiCol_PlotHistogram, level_color);
        ImGui::ProgressBar(level_normalized, ImVec2(100, 14), "");
        ImGui::PopStyleColor();
        ImGui::SameLine();
        ImGui::TextDisabled("%.0fdB", input_db);
        if (modem_.isSynced()) {
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "SIGNAL");
        }
    }

    ImGui::Separator();

    // Stats column (moved from bottom status bar)
    {
        auto mstats = modem_.getStats();
        auto dstats = modem_.getDecoderStats();
        const char* mode_str = simulation_enabled_ ? "SIM" : (ptt_active_ ? "TX" : (radio_rx_enabled_ ? "RX" : "IDLE"));

        ImGui::TextDisabled("Mode:");
        ImGui::SameLine();
        ImGui::Text("%s", mode_str);

        ImGui::TextDisabled("SNR:");
        ImGui::SameLine();
        ImGui::Text("%.1f dB", mstats.snr_db);

        ImGui::TextDisabled("TX/RX:");
        ImGui::SameLine();
        ImGui::Text("%d / %d", mstats.frames_sent, mstats.frames_received);

        ImGui::TextDisabled("PHY:");
        ImGui::SameLine();
        ImGui::Text("%d bps", mstats.throughput_bps);

        ImGui::TextDisabled("Goodput:");
        ImGui::SameLine();
        if (last_effective_goodput_bps_ > 0.0f) {
            ImGui::Text("%.2f kbps", last_effective_goodput_bps_ / 1000.0f);
        } else {
            ImGui::Text("n/a");
        }

        ImGui::TextDisabled("RXQ:");
        ImGui::SameLine();
        ImGui::Text("%.0f ms", dstats.backlog_ms);

        if (dstats.buffer_overflows > 0) {
            ImGui::TextDisabled("OF:");
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "%llu",
                static_cast<unsigned long long>(dstats.buffer_overflows));
        }
    }

    // TX state management (message log moved to popup)
    if (tx_in_progress_ && audio_.isTxQueueEmpty()) {
        tx_in_progress_ = false;
        if (ptt_active_) {
            releaseTxPtt("tx_complete", false);
        } else if (!simulation_enabled_) {
            audio_.setRxMuted(false);
            if (!audio_.isCapturing()) {
                audio_.startCapture();
            }
        }
    }

    if (ptt_release_pending_ && !simulation_enabled_) {
        bool ready_to_resume_rx = false;
        if (tx_ptt_path_ == TxPttPath::Cat) {
            if (!cat_controller_.isPttActive()) {
                ptt_release_pending_ = false;
                ptt_active_ = false;
                tx_ptt_path_ = TxPttPath::None;
                ready_to_resume_rx = true;
            }
        } else if (SDL_TICKS_PASSED(SDL_GetTicks(), ptt_release_deadline_ms_)) {
            releaseSerialPtt("tx_tail_elapsed");
            ready_to_resume_rx = true;
        }

        if (ready_to_resume_rx && radio_rx_enabled_) {
            audio_.setRxMuted(false);
            if (!audio_.isCapturing()) {
                audio_.startCapture();
            }
        }
    }

    // File Transfer progress display (read-only monitoring)
    // File send is initiated via TCP SENDFILE command
    if (protocol_.isFileTransferInProgress()) {
        auto progress = protocol_.getFileProgress();
        ImGui::TextColored(progress.is_sending ? ImVec4(0.5f, 0.8f, 1.0f, 1.0f) : ImVec4(0.5f, 1.0f, 0.5f, 1.0f),
            "%s: %s", progress.is_sending ? "TX" : "RX", progress.filename.c_str());
        ImGui::SameLine();
        ImGui::ProgressBar(progress.percentage() / 100.0f, ImVec2(100, 16));
        ImGui::SameLine();
        // Show bytes transferred / total with appropriate units
        if (progress.total_bytes >= 1024) {
            ImGui::Text("%.1f/%.1f KB (%.0f%%)",
                progress.transferred_bytes / 1024.0f,
                progress.total_bytes / 1024.0f,
                progress.percentage());
        } else {
            ImGui::Text("%u/%u B (%.0f%%)",
                progress.transferred_bytes, progress.total_bytes,
                progress.percentage());
        }
        ImGui::SameLine();
        if (ImGui::SmallButton("Cancel")) protocol_.cancelFileTransfer();
    }
}

} // namespace gui
} // namespace ultra
