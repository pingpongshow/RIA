#include "settings.hpp"
#include "imgui.h"
#include "gui/startup_trace.hpp"
#include <algorithm>
#include <cstring>
#include <fstream>
#include <cstdlib>

#ifdef _WIN32
#include <direct.h>
#define MKDIR(path) _mkdir(path)
#else
#include <sys/stat.h>
#define MKDIR(path) mkdir(path, 0755)
#endif

namespace ultra {
namespace gui {

AppSettings::AppSettings() {
    startupTrace("AppSettings", "ctor");
}

// Get default settings file path
// Supports ULTRA_CONFIG environment variable for multi-instance operation
std::string AppSettings::getDefaultPath() {
    // Check for explicit config path override (for running multiple instances)
    const char* config_override = std::getenv("ULTRA_CONFIG");
    if (config_override && config_override[0] != '\0') {
        return std::string(config_override);
    }

    // Check for instance number (ULTRA_INSTANCE=2 -> settings_2.ini)
    const char* instance = std::getenv("ULTRA_INSTANCE");
    std::string suffix = "";
    if (instance && instance[0] != '\0' && std::string(instance) != "1") {
        suffix = std::string("_") + instance;
    }

#ifdef _WIN32
    const char* appdata = std::getenv("APPDATA");
    if (appdata) {
        return std::string(appdata) + "\\RIAModem\\settings" + suffix + ".ini";
    }
    return "settings" + suffix + ".ini";
#else
    const char* home = std::getenv("HOME");
    if (home) {
        return std::string(home) + "/.config/ultra/settings" + suffix + ".ini";
    }
    return "settings" + suffix + ".ini";
#endif
}

// Get platform-specific Downloads folder
std::string AppSettings::getDefaultDownloadsPath() {
#ifdef _WIN32
    const char* userprofile = std::getenv("USERPROFILE");
    if (userprofile) {
        return std::string(userprofile) + "\\Downloads";
    }
    return ".";
#else
    const char* home = std::getenv("HOME");
    if (home) {
        return std::string(home) + "/Downloads";
    }
    return ".";
#endif
}

// Get effective receive directory (default to Downloads if not set)
std::string AppSettings::getReceiveDirectory() const {
    if (receive_directory[0] != '\0') {
        return std::string(receive_directory);
    }
    return getDefaultDownloadsPath();
}

// Helper to create directory if it doesn't exist
static void ensureDirectory(const std::string& path) {
    size_t pos = path.find_last_of("/\\");
    if (pos != std::string::npos) {
        std::string dir = path.substr(0, pos);
        // Create parent directories recursively
        for (size_t i = 0; i < dir.size(); i++) {
            if (dir[i] == '/' || dir[i] == '\\') {
                std::string subdir = dir.substr(0, i);
                if (!subdir.empty()) {
                    MKDIR(subdir.c_str());
                }
            }
        }
        MKDIR(dir.c_str());
    }
}

static void copyBounded(char* dst, size_t dst_size, const std::string& value) {
    if (!dst || dst_size == 0) {
        return;
    }
    std::strncpy(dst, value.c_str(), dst_size - 1);
    dst[dst_size - 1] = '\0';
}

template <size_t N>
static size_t boundedCStringLen(const char (&buf)[N]) {
    const void* term = std::memchr(buf, '\0', N);
    return term ? static_cast<size_t>(static_cast<const char*>(term) - buf) : N;
}

// Save settings to INI file
bool AppSettings::save(const std::string& path) const {
    std::string filepath = path.empty() ? getDefaultPath() : path;
    ensureDirectory(filepath);

    std::ofstream file(filepath);
    if (!file.is_open()) {
        return false;
    }

    file << "[Station]\n";
    file << "callsign=" << callsign << "\n";
    file << "grid_square=" << grid_square << "\n";
    file << "name=" << name << "\n";

    file << "\n[Radio]\n";
    file << "rig_model=" << rig_model << "\n";
    file << "rig_port=" << rig_port << "\n";
    file << "rig_baud=" << rig_baud << "\n";
    file << "use_cat_ptt=" << (use_cat_ptt ? "1" : "0") << "\n";
    file << "ptt_serial_line=" << ptt_serial_line << "\n";
    file << "ptt_invert=" << (ptt_invert ? "1" : "0") << "\n";

    file << "\n[Audio]\n";
    file << "input_device=" << input_device << "\n";
    file << "output_device=" << output_device << "\n";
    file << "tx_delay_ms=" << tx_delay_ms << "\n";
    file << "tx_tail_ms=" << tx_tail_ms << "\n";
    file << "tx_drive=" << tx_drive << "\n";

    file << "\n[Filter]\n";
    file << "enabled=" << (filter_enabled ? "1" : "0") << "\n";
    file << "center=" << filter_center << "\n";
    file << "bandwidth=" << filter_bandwidth << "\n";
    file << "taps=" << filter_taps << "\n";

    file << "\n[FileTransfer]\n";
    file << "receive_directory=" << receive_directory << "\n";

    file << "\n[Expert]\n";
    file << "forced_waveform=" << static_cast<int>(forced_waveform) << "\n";
    file << "forced_modulation=" << static_cast<int>(forced_modulation) << "\n";
    file << "forced_code_rate=" << static_cast<int>(forced_code_rate) << "\n";

    file << "\n[Network]\n";
    file << "tcp_enabled=" << (tcp_enabled ? "1" : "0") << "\n";
    file << "tcp_cmd_port=" << tcp_cmd_port << "\n";
    file << "tcp_data_port=" << tcp_data_port << "\n";
    file << "tcp_kiss_port=" << tcp_kiss_port << "\n";
    file << "tcp_kiss_enabled=" << (tcp_kiss_enabled ? "1" : "0") << "\n";
    file << "tcp_bind_addr=" << tcp_bind_addr << "\n";

    file << "\n[Security]\n";
    file << "encryption_enabled=" << (encryption_enabled ? "1" : "0") << "\n";
    // Don't save the actual key to file for security reasons
    // Key must be re-entered each session or set via TCP command
    file << "# encryption_key is not saved for security\n";
    file << "compression_enabled=" << (compression_enabled ? "1" : "0") << "\n";

    file << "\n[CAT]\n";
    file << "enabled=" << (cat_enabled ? "1" : "0") << "\n";
    file << "backend=" << cat_backend << "\n";
    file << "hamlib_model=" << cat_hamlib_model << "\n";
    file << "port=" << cat_port << "\n";
    file << "baud=" << cat_baud << "\n";
    file << "ptt_line=" << cat_ptt_line << "\n";
    file << "ptt_invert=" << (cat_ptt_invert ? "1" : "0") << "\n";
    file << "tcp_host=" << cat_tcp_host << "\n";
    file << "tcp_port=" << cat_tcp_port << "\n";
    file << "flex_slice=" << cat_flex_slice << "\n";
    file << "ptt_lead_ms=" << cat_ptt_lead_ms << "\n";
    file << "ptt_tail_ms=" << cat_ptt_tail_ms << "\n";
    file << "watchdog_seconds=" << cat_watchdog_seconds << "\n";

    return true;
}

// Load settings from INI file
bool AppSettings::load(const std::string& path) {
    std::string filepath = path.empty() ? getDefaultPath() : path;

    std::ifstream file(filepath);
    if (!file.is_open()) {
        return false;
    }

    std::string line;
    while (std::getline(file, line)) {
        // Skip empty lines and comments
        if (line.empty() || line[0] == '#' || line[0] == '[') {
            continue;
        }

        size_t eq = line.find('=');
        if (eq == std::string::npos) continue;

        std::string key = line.substr(0, eq);
        std::string value = line.substr(eq + 1);

        // Station settings
        if (key == "callsign") {
            copyBounded(callsign, sizeof(callsign), value);
        } else if (key == "grid_square") {
            copyBounded(grid_square, sizeof(grid_square), value);
        } else if (key == "name") {
            copyBounded(name, sizeof(name), value);
        }
        // Radio settings
        else if (key == "rig_model") {
            copyBounded(rig_model, sizeof(rig_model), value);
        } else if (key == "rig_port") {
            copyBounded(rig_port, sizeof(rig_port), value);
        } else if (key == "rig_baud") {
            rig_baud = std::atoi(value.c_str());
        } else if (key == "use_cat_ptt") {
            use_cat_ptt = (value == "1" || value == "true");
        } else if (key == "ptt_serial_line") {
            ptt_serial_line = std::atoi(value.c_str());
            if (ptt_serial_line < 0 || ptt_serial_line > 1) {
                ptt_serial_line = 0;
            }
        } else if (key == "ptt_invert") {
            ptt_invert = (value == "1" || value == "true");
        }
        // Audio settings
        else if (key == "input_device") {
            copyBounded(input_device, sizeof(input_device), value);
        } else if (key == "output_device") {
            copyBounded(output_device, sizeof(output_device), value);
        } else if (key == "tx_delay_ms") {
            tx_delay_ms = std::atoi(value.c_str());
        } else if (key == "tx_tail_ms") {
            tx_tail_ms = std::atoi(value.c_str());
        } else if (key == "tx_drive") {
            tx_drive = std::strtof(value.c_str(), nullptr);
        }
        // Filter settings
        else if (key == "enabled") {
            filter_enabled = (value == "1" || value == "true");
        } else if (key == "center") {
            filter_center = std::strtof(value.c_str(), nullptr);
        } else if (key == "bandwidth") {
            filter_bandwidth = std::strtof(value.c_str(), nullptr);
        } else if (key == "taps") {
            filter_taps = std::atoi(value.c_str());
        }
        // File transfer settings
        else if (key == "receive_directory") {
            copyBounded(receive_directory, sizeof(receive_directory), value);
        }
        // Expert settings
        else if (key == "forced_waveform") {
            forced_waveform = static_cast<uint8_t>(std::atoi(value.c_str()));
        } else if (key == "forced_modulation") {
            forced_modulation = static_cast<uint8_t>(std::atoi(value.c_str()));
        } else if (key == "forced_code_rate") {
            forced_code_rate = static_cast<uint8_t>(std::atoi(value.c_str()));
        }
        // Network settings
        else if (key == "tcp_enabled") {
            tcp_enabled = (value == "1" || value == "true");
        } else if (key == "tcp_cmd_port") {
            tcp_cmd_port = static_cast<uint16_t>(std::atoi(value.c_str()));
        } else if (key == "tcp_data_port") {
            tcp_data_port = static_cast<uint16_t>(std::atoi(value.c_str()));
        } else if (key == "tcp_kiss_port") {
            tcp_kiss_port = static_cast<uint16_t>(std::atoi(value.c_str()));
        } else if (key == "tcp_kiss_enabled") {
            tcp_kiss_enabled = (value == "1" || value == "true");
        } else if (key == "tcp_bind_addr") {
            copyBounded(tcp_bind_addr, sizeof(tcp_bind_addr), value);
        }
        // Security settings
        else if (key == "encryption_enabled") {
            encryption_enabled = (value == "1" || value == "true");
        }
        // Note: encryption_key is intentionally not loaded from file for security
        else if (key == "compression_enabled") {
            compression_enabled = (value == "1" || value == "true");
        }
        // CAT settings
        else if (key == "cat_enabled" || (key == "enabled" && cat_backend >= 0)) {
            // Handle both "cat_enabled" and "enabled" in [CAT] section
            cat_enabled = (value == "1" || value == "true");
        } else if (key == "cat_backend" || key == "backend") {
            cat_backend = std::atoi(value.c_str());
            if (cat_backend < 0 || cat_backend > 3) cat_backend = 0;
        } else if (key == "cat_hamlib_model" || key == "hamlib_model") {
            cat_hamlib_model = std::atoi(value.c_str());
        } else if (key == "cat_port" || key == "port") {
            copyBounded(cat_port, sizeof(cat_port), value);
        } else if (key == "cat_baud" || key == "baud") {
            cat_baud = std::atoi(value.c_str());
        } else if (key == "cat_ptt_line" || key == "ptt_line") {
            cat_ptt_line = std::atoi(value.c_str());
            if (cat_ptt_line < 0 || cat_ptt_line > 1) cat_ptt_line = 0;
        } else if (key == "cat_ptt_invert" || key == "ptt_invert") {
            cat_ptt_invert = (value == "1" || value == "true");
        } else if (key == "cat_tcp_host" || key == "tcp_host") {
            copyBounded(cat_tcp_host, sizeof(cat_tcp_host), value);
        } else if (key == "cat_tcp_port" || key == "tcp_port") {
            cat_tcp_port = static_cast<uint16_t>(std::atoi(value.c_str()));
        } else if (key == "cat_flex_slice" || key == "flex_slice") {
            cat_flex_slice = std::atoi(value.c_str());
        } else if (key == "cat_ptt_lead_ms" || key == "ptt_lead_ms") {
            cat_ptt_lead_ms = std::atoi(value.c_str());
        } else if (key == "cat_ptt_tail_ms" || key == "ptt_tail_ms") {
            cat_ptt_tail_ms = std::atoi(value.c_str());
        } else if (key == "cat_watchdog_seconds" || key == "watchdog_seconds") {
            cat_watchdog_seconds = std::atoi(value.c_str());
        }
    }

    return true;
}

SettingsWindow::SettingsWindow() {
    startupTrace("SettingsWindow", "ctor");
}

bool SettingsWindow::render(AppSettings& settings) {
    just_closed_ = false;

    if (!visible_) {
        // Check if we just closed
        if (was_visible_) {
            just_closed_ = true;
            if (on_closed_) {
                on_closed_();
            }
        }
        was_visible_ = false;
        return false;
    }

    bool changed = false;

    ImGui::SetNextWindowSize(ImVec2(450, 400), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Settings", &visible_, ImGuiWindowFlags_NoCollapse)) {

        if (ImGui::BeginTabBar("SettingsTabs")) {

            if (ImGui::BeginTabItem("Station")) {
                renderStationTab(settings);
                ImGui::EndTabItem();
            }

            if (ImGui::BeginTabItem("CAT")) {
                renderCatTab(settings);
                ImGui::EndTabItem();
            }

            if (ImGui::BeginTabItem("Radio")) {
                renderRadioTab(settings);
                ImGui::EndTabItem();
            }

            if (ImGui::BeginTabItem("Audio")) {
                renderAudioTab(settings);
                ImGui::EndTabItem();
            }

            if (ImGui::BeginTabItem("Expert")) {
                renderExpertTab(settings);
                ImGui::EndTabItem();
            }

            if (ImGui::BeginTabItem("Network")) {
                renderNetworkTab(settings);
                ImGui::EndTabItem();
            }

            if (ImGui::BeginTabItem("Security")) {
                renderSecurityTab(settings);
                ImGui::EndTabItem();
            }

            ImGui::EndTabBar();
        }
    }
    ImGui::End();

    // Track visibility for next frame (to detect close via X button)
    was_visible_ = visible_;

    return changed;
}

void SettingsWindow::renderStationTab(AppSettings& settings) {
    ImGui::Spacing();
    ImGui::Text("Station Information");
    ImGui::Separator();
    ImGui::Spacing();

    // Callsign
    ImGui::Text("Callsign");
    ImGui::SetNextItemWidth(150);
    char old_call[16];
    std::strncpy(old_call, settings.callsign, sizeof(old_call) - 1);
    old_call[sizeof(old_call) - 1] = '\0';

    if (ImGui::InputText("##callsign", settings.callsign, sizeof(settings.callsign),
                         ImGuiInputTextFlags_CharsUppercase)) {
        // Notify if changed
        if (strcmp(old_call, settings.callsign) != 0 && on_callsign_changed_) {
            on_callsign_changed_(settings.callsign);
        }
    }
    ImGui::SameLine();
    ImGui::TextDisabled("(Required)");

    ImGui::Spacing();
    ImGui::Spacing();

    // Validation indicator
    bool valid_call = boundedCStringLen(settings.callsign) >= 3;
    if (valid_call) {
        ImGui::TextColored(ImVec4(0.2f, 1.0f, 0.2f, 1.0f), "Station configured");
    } else {
        ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.2f, 1.0f), "Enter your callsign to use ARQ mode");
    }

    ImGui::Spacing();
    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    // File Transfer Settings
    ImGui::Text("File Transfer");
    ImGui::Spacing();

    ImGui::Text("Receive Directory");
    ImGui::SetNextItemWidth(-80);

    // Show placeholder with default path if empty
    std::string default_path = AppSettings::getDefaultDownloadsPath();
    std::string placeholder = "Default: " + default_path;

    char old_dir[512];
    std::strncpy(old_dir, settings.receive_directory, sizeof(old_dir) - 1);
    old_dir[sizeof(old_dir) - 1] = '\0';

    if (ImGui::InputTextWithHint("##receive_dir", placeholder.c_str(),
                                  settings.receive_directory, sizeof(settings.receive_directory))) {
        // Notify if changed
        if (strcmp(old_dir, settings.receive_directory) != 0 && on_receive_dir_changed_) {
            on_receive_dir_changed_(settings.getReceiveDirectory());
        }
    }

    ImGui::SameLine();
    if (ImGui::Button("Reset")) {
        settings.receive_directory[0] = '\0';  // Clear to use default
        if (on_receive_dir_changed_) {
            on_receive_dir_changed_(settings.getReceiveDirectory());
        }
    }

    ImGui::TextDisabled("Leave empty to use Downloads folder");

    // Show effective path
    ImGui::Spacing();
    std::string effective = settings.getReceiveDirectory();
    ImGui::Text("Files will be saved to:");
    ImGui::TextColored(ImVec4(0.5f, 0.8f, 1.0f, 1.0f), "%s", effective.c_str());
}

void SettingsWindow::renderRadioTab(AppSettings& settings) {
    ImGui::Spacing();
    ImGui::Text("Radio PTT");
    ImGui::Separator();
    ImGui::Spacing();

    ImGui::Checkbox("Enable Serial PTT (DTR/RTS)", &settings.use_cat_ptt);
    ImGui::TextDisabled("Keys TX from serial control line; no CAT command protocol needed.");

    ImGui::Spacing();
    ImGui::Text("Serial Port");
    ImGui::SetNextItemWidth(240);
    ImGui::InputText("##rig_port", settings.rig_port, sizeof(settings.rig_port));
    ImGui::TextDisabled("Examples: /dev/ttyUSB0, /dev/ttyACM0, COM3");

    ImGui::Spacing();
    ImGui::Text("Baud Rate");
    ImGui::SetNextItemWidth(120);
    const char* bauds[] = { "1200", "2400", "4800", "9600", "19200", "38400", "57600", "115200" };
    int baud_values[] = { 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200 };
    int baud_idx = 3;  // 9600 default
    for (int i = 0; i < 8; ++i) {
        if (settings.rig_baud == baud_values[i]) {
            baud_idx = i;
            break;
        }
    }
    if (ImGui::Combo("##baud", &baud_idx, bauds, 8)) {
        settings.rig_baud = baud_values[baud_idx];
    }

    ImGui::Spacing();
    ImGui::Text("PTT Line");
    ImGui::SetNextItemWidth(120);
    const char* ptt_lines[] = {"DTR", "RTS"};
    int line_idx = (settings.ptt_serial_line == 1) ? 1 : 0;
    if (ImGui::Combo("##ptt_line", &line_idx, ptt_lines, 2)) {
        settings.ptt_serial_line = line_idx;
    }

    ImGui::Checkbox("Invert PTT polarity", &settings.ptt_invert);

    ImGui::Spacing();
    ImGui::TextDisabled("Rig side: set USB SEND to matching line (DTR or RTS).");
}

void SettingsWindow::renderCatTab(AppSettings& settings) {
    ImGui::Spacing();
    ImGui::Text("CAT Control");
    ImGui::Separator();
    ImGui::Spacing();

    // Enable CAT control
    bool cat_changed = false;
    if (ImGui::Checkbox("Enable CAT Control", &settings.cat_enabled)) {
        cat_changed = true;
    }

    ImGui::Spacing();

    // Backend selection
    ImGui::Text("Backend");
    ImGui::SetNextItemWidth(200);
    const char* backends[] = { "None", "Serial PTT", "Hamlib", "Kenwood TCP (FlexRadio)" };
    if (ImGui::Combo("##cat_backend", &settings.cat_backend, backends, 4)) {
        cat_changed = true;
    }

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    // Backend-specific settings
    if (settings.cat_backend == 1) {
        // Serial PTT settings
        ImGui::Text("Serial PTT Settings");
        ImGui::Spacing();

        ImGui::Text("Serial Port");
        ImGui::SetNextItemWidth(240);
        if (ImGui::InputText("##cat_port", settings.cat_port, sizeof(settings.cat_port))) {
            cat_changed = true;
        }
        ImGui::TextDisabled("Examples: /dev/ttyUSB0, COM3");

        ImGui::Spacing();
        ImGui::Text("Baud Rate");
        ImGui::SetNextItemWidth(120);
        const char* bauds[] = { "1200", "2400", "4800", "9600", "19200", "38400", "57600", "115200" };
        int baud_values[] = { 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200 };
        int baud_idx = 3;
        for (int i = 0; i < 8; ++i) {
            if (settings.cat_baud == baud_values[i]) {
                baud_idx = i;
                break;
            }
        }
        if (ImGui::Combo("##cat_baud", &baud_idx, bauds, 8)) {
            settings.cat_baud = baud_values[baud_idx];
            cat_changed = true;
        }

        ImGui::Spacing();
        ImGui::Text("PTT Line");
        ImGui::SetNextItemWidth(120);
        const char* ptt_lines[] = {"DTR", "RTS"};
        if (ImGui::Combo("##cat_ptt_line", &settings.cat_ptt_line, ptt_lines, 2)) {
            cat_changed = true;
        }

        if (ImGui::Checkbox("Invert PTT Polarity", &settings.cat_ptt_invert)) {
            cat_changed = true;
        }

    } else if (settings.cat_backend == 2) {
        // Hamlib settings
        ImGui::Text("Hamlib Settings");
        ImGui::Spacing();

        ImGui::Text("Rig Model");
        ImGui::SetNextItemWidth(240);
        // Common models
        const char* models[] = {
            "1 - Dummy (Test)",
            "2014 - Kenwood TS-2000",
            "3073 - Icom IC-7300",
            "3070 - Icom IC-7100",
            "3078 - Icom IC-7610",
            "1035 - Yaesu FT-991",
            "1042 - Yaesu FTDX-10",
            "2029 - Elecraft K3",
            "2043 - Elecraft KX3",
            "Other (enter ID below)"
        };
        int model_idx = 9;  // Default to "Other"
        int model_ids[] = { 1, 2014, 3073, 3070, 3078, 1035, 1042, 2029, 2043 };
        for (int i = 0; i < 9; ++i) {
            if (settings.cat_hamlib_model == model_ids[i]) {
                model_idx = i;
                break;
            }
        }
        if (ImGui::Combo("##hamlib_preset", &model_idx, models, 10)) {
            if (model_idx < 9) {
                settings.cat_hamlib_model = model_ids[model_idx];
            }
            cat_changed = true;
        }

        // Allow manual entry
        ImGui::Text("Model ID");
        ImGui::SetNextItemWidth(100);
        if (ImGui::InputInt("##hamlib_model", &settings.cat_hamlib_model)) {
            cat_changed = true;
        }
        ImGui::TextDisabled("See rigctl -l for full list");

        ImGui::Spacing();
        ImGui::Text("Serial Port");
        ImGui::SetNextItemWidth(240);
        if (ImGui::InputText("##hamlib_port", settings.cat_port, sizeof(settings.cat_port))) {
            cat_changed = true;
        }

        ImGui::Spacing();
        ImGui::Text("Baud Rate");
        ImGui::SetNextItemWidth(120);
        const char* bauds[] = { "4800", "9600", "19200", "38400", "57600", "115200" };
        int baud_values[] = { 4800, 9600, 19200, 38400, 57600, 115200 };
        int baud_idx = 1;
        for (int i = 0; i < 6; ++i) {
            if (settings.cat_baud == baud_values[i]) {
                baud_idx = i;
                break;
            }
        }
        if (ImGui::Combo("##hamlib_baud", &baud_idx, bauds, 6)) {
            settings.cat_baud = baud_values[baud_idx];
            cat_changed = true;
        }

    } else if (settings.cat_backend == 3) {
        // Kenwood TCP (FlexRadio) settings
        ImGui::Text("Kenwood TCP Settings (FlexRadio)");
        ImGui::Spacing();

        ImGui::Text("Host");
        ImGui::SetNextItemWidth(200);
        if (ImGui::InputText("##tcp_host", settings.cat_tcp_host, sizeof(settings.cat_tcp_host))) {
            cat_changed = true;
        }

        ImGui::Spacing();
        ImGui::Text("Port");
        ImGui::SetNextItemWidth(100);
        int port = settings.cat_tcp_port;
        if (ImGui::InputInt("##tcp_port", &port)) {
            if (port > 0 && port < 65536) {
                settings.cat_tcp_port = static_cast<uint16_t>(port);
                cat_changed = true;
            }
        }

        ImGui::Spacing();
        ImGui::Text("Slice");
        ImGui::SetNextItemWidth(80);
        if (ImGui::InputInt("##flex_slice", &settings.cat_flex_slice)) {
            if (settings.cat_flex_slice < 0) settings.cat_flex_slice = 0;
            if (settings.cat_flex_slice > 7) settings.cat_flex_slice = 7;
            cat_changed = true;
        }
        ImGui::TextDisabled("FlexRadio slice number (0-7)");
    }

    // PTT Timing (common to all backends)
    if (settings.cat_backend > 0) {
        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();
        ImGui::Text("PTT Timing");

        ImGui::Text("Lead Delay (ms)");
        ImGui::SetNextItemWidth(100);
        if (ImGui::InputInt("##ptt_lead", &settings.cat_ptt_lead_ms)) {
            if (settings.cat_ptt_lead_ms < 0) settings.cat_ptt_lead_ms = 0;
            if (settings.cat_ptt_lead_ms > 1000) settings.cat_ptt_lead_ms = 1000;
            cat_changed = true;
        }
        ImGui::SameLine();
        ImGui::TextDisabled("Delay before TX after PTT");

        ImGui::Text("Tail Delay (ms)");
        ImGui::SetNextItemWidth(100);
        if (ImGui::InputInt("##ptt_tail", &settings.cat_ptt_tail_ms)) {
            if (settings.cat_ptt_tail_ms < 0) settings.cat_ptt_tail_ms = 0;
            if (settings.cat_ptt_tail_ms > 1000) settings.cat_ptt_tail_ms = 1000;
            cat_changed = true;
        }
        ImGui::SameLine();
        ImGui::TextDisabled("Hold PTT after TX ends");

        // Watchdog
        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();
        ImGui::Text("TX Watchdog");

        ImGui::Text("Timeout (seconds)");
        ImGui::SetNextItemWidth(100);
        if (ImGui::InputInt("##watchdog", &settings.cat_watchdog_seconds)) {
            if (settings.cat_watchdog_seconds < 0) settings.cat_watchdog_seconds = 0;
            if (settings.cat_watchdog_seconds > 600) settings.cat_watchdog_seconds = 600;
            cat_changed = true;
        }
        ImGui::SameLine();
        ImGui::TextDisabled("0 = disabled");
        ImGui::TextDisabled("Auto-releases PTT if TX exceeds this duration");
    }

    // Notify if changed
    if (cat_changed && on_cat_settings_changed_) {
        on_cat_settings_changed_();
    }
}

void SettingsWindow::renderAudioTab(AppSettings& settings) {
    ImGui::Spacing();
    ImGui::Text("Audio Devices");
    ImGui::Separator();
    ImGui::Spacing();

    // Output device selection
    ImGui::Text("Output Device (Speaker)");
    ImGui::SetNextItemWidth(-1);
    if (ImGui::BeginCombo("##output_device", settings.output_device)) {
        for (const auto& dev : output_devices) {
            bool selected = (strcmp(settings.output_device, dev.c_str()) == 0);
            if (ImGui::Selectable(dev.c_str(), selected)) {
                copyBounded(settings.output_device, sizeof(settings.output_device), dev);
            }
            if (selected) {
                ImGui::SetItemDefaultFocus();
            }
        }
        ImGui::EndCombo();
    }

    ImGui::Spacing();

    // Input device selection
    ImGui::Text("Input Device (Microphone)");
    ImGui::SetNextItemWidth(-1);
    if (ImGui::BeginCombo("##input_device", settings.input_device)) {
        for (const auto& dev : input_devices) {
            bool selected = (strcmp(settings.input_device, dev.c_str()) == 0);
            if (ImGui::Selectable(dev.c_str(), selected)) {
                copyBounded(settings.input_device, sizeof(settings.input_device), dev);
            }
            if (selected) {
                ImGui::SetItemDefaultFocus();
            }
        }
        ImGui::EndCombo();
    }

    ImGui::Spacing();

    // Rescan button
    if (ImGui::Button("Rescan Audio Devices", ImVec2(200, 0))) {
        if (on_audio_reset_) {
            on_audio_reset_();
        }
    }

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    ImGui::Text("TX Settings");
    ImGui::Spacing();

    // TX Delay
    ImGui::Text("TX Delay (ms)");
    ImGui::SetNextItemWidth(150);
    ImGui::SliderInt("##tx_delay", &settings.tx_delay_ms, 0, 500);
    ImGui::SameLine();
    ImGui::TextDisabled("Delay after PTT before audio");

    // TX Tail
    ImGui::Text("TX Tail (ms)");
    ImGui::SetNextItemWidth(150);
    ImGui::SliderInt("##tx_tail", &settings.tx_tail_ms, 0, 500);
    ImGui::SameLine();
    ImGui::TextDisabled("Hold PTT after audio ends");

    ImGui::Spacing();

    // TX Drive
    ImGui::Text("TX Drive Level");
    ImGui::SetNextItemWidth(200);
    float tx_drive_pct = settings.tx_drive * 100.0f;
    if (ImGui::SliderFloat("##tx_drive", &tx_drive_pct, 0.0f, 100.0f, "%.0f%%")) {
        settings.tx_drive = std::clamp(tx_drive_pct / 100.0f, 0.0f, 1.0f);
    }
    ImGui::SameLine();
    ImGui::TextDisabled("Audio output level");

    ImGui::Spacing();
    ImGui::Spacing();

    // Visual indicator
    ImGui::Text("TX Level Preview:");
    ImGui::ProgressBar(settings.tx_drive, ImVec2(200, 20), "");

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    // Audio Filter Settings
    ImGui::Text("Audio Bandpass Filter");
    ImGui::Spacing();

    bool filter_changed = false;

    // Enable/disable checkbox
    if (ImGui::Checkbox("Enable Filter", &settings.filter_enabled)) {
        filter_changed = true;
    }
    ImGui::SameLine();
    ImGui::TextDisabled("Bandpass filter on TX/RX audio");

    ImGui::Spacing();

    // Only show controls if enabled
    ImGui::BeginDisabled(!settings.filter_enabled);

    // Center frequency
    ImGui::Text("Center Frequency");
    ImGui::SetNextItemWidth(200);
    if (ImGui::SliderFloat("##filter_center", &settings.filter_center, 500.0f, 3000.0f, "%.0f Hz")) {
        filter_changed = true;
    }
    ImGui::SameLine();
    ImGui::TextDisabled("Audio passband center");

    // Bandwidth
    ImGui::Text("Bandwidth");
    ImGui::SetNextItemWidth(200);
    if (ImGui::SliderFloat("##filter_bw", &settings.filter_bandwidth, 200.0f, 3000.0f, "%.0f Hz")) {
        filter_changed = true;
    }
    ImGui::SameLine();
    ImGui::TextDisabled("Total passband width");

    // Filter taps (advanced)
    ImGui::Text("Filter Taps");
    ImGui::SetNextItemWidth(150);
    if (ImGui::SliderInt("##filter_taps", &settings.filter_taps, 31, 255)) {
        // Ensure odd number of taps
        if (settings.filter_taps % 2 == 0) {
            settings.filter_taps++;
        }
        filter_changed = true;
    }
    ImGui::SameLine();
    ImGui::TextDisabled("More = sharper cutoff");

    ImGui::EndDisabled();

    // Display passband
    ImGui::Spacing();
    float low = settings.filter_center - settings.filter_bandwidth / 2.0f;
    float high = settings.filter_center + settings.filter_bandwidth / 2.0f;
    ImGui::Text("Passband: %.0f - %.0f Hz", low, high);

    // Call callback if filter settings changed
    if (filter_changed && on_filter_changed_) {
        on_filter_changed_(settings.filter_enabled, settings.filter_center,
                          settings.filter_bandwidth, settings.filter_taps);
    }
}

void SettingsWindow::renderExpertTab(AppSettings& settings) {
    ImGui::Spacing();
    ImGui::Text("Expert Mode Settings");
    ImGui::Separator();
    ImGui::Spacing();

    // Warning message
    ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.2f, 1.0f),
        "* Leave as AUTO if you are unsure.");
    ImGui::TextDisabled("These settings force specific waveform and modulation modes.");
    ImGui::TextDisabled("AUTO lets the protocol negotiate optimal settings based on SNR.");
    ImGui::Spacing();
    ImGui::Spacing();

    bool changed = false;

    // --- Forced Waveform ---
    ImGui::Text("Forced Waveform");
    ImGui::SetNextItemWidth(200);

    // Current waveform display string
    const char* waveform_items[] = { "AUTO", "OFDM", "DPSK" };
    int waveform_idx = 0;  // AUTO
    if (settings.forced_waveform == 0x00) waveform_idx = 1;       // OFDM
    else if (settings.forced_waveform == 0x04) waveform_idx = 2;  // DPSK

    if (ImGui::Combo("##waveform", &waveform_idx, waveform_items, 3)) {
        switch (waveform_idx) {
            case 0: settings.forced_waveform = 0xFF; break;  // AUTO
            case 1: settings.forced_waveform = 0x00; break;  // OFDM
            case 2: settings.forced_waveform = 0x04; break;  // DPSK
        }
        changed = true;
    }
    ImGui::SameLine();
    ImGui::TextDisabled("OFDM=fast, DPSK=robust");

    ImGui::Spacing();

    // --- Forced Modulation ---
    ImGui::Text("Forced Modulation");
    ImGui::SetNextItemWidth(200);

    const char* modulation_items[] = { "AUTO", "DBPSK", "DQPSK", "D8PSK", "QPSK", "16QAM", "32QAM", "64QAM" };
    int mod_idx = 0;  // AUTO
    switch (settings.forced_modulation) {
        case 0:    mod_idx = 1; break;  // DBPSK
        case 2:    mod_idx = 2; break;  // DQPSK
        case 4:    mod_idx = 3; break;  // D8PSK
        case 3:    mod_idx = 4; break;  // QPSK
        case 6:    mod_idx = 5; break;  // QAM16
        case 7:    mod_idx = 6; break;  // QAM32
        case 8:    mod_idx = 7; break;  // QAM64
        default:   mod_idx = 0; break;  // AUTO
    }

    if (ImGui::Combo("##modulation", &mod_idx, modulation_items, 8)) {
        switch (mod_idx) {
            case 0: settings.forced_modulation = 0xFF; break;  // AUTO
            case 1: settings.forced_modulation = 0; break;     // DBPSK
            case 2: settings.forced_modulation = 2; break;     // DQPSK
            case 3: settings.forced_modulation = 4; break;     // D8PSK
            case 4: settings.forced_modulation = 3; break;     // QPSK
            case 5: settings.forced_modulation = 6; break;     // QAM16
            case 6: settings.forced_modulation = 7; break;     // QAM32
            case 7: settings.forced_modulation = 8; break;     // QAM64
        }
        changed = true;
    }
    ImGui::SameLine();
    ImGui::TextDisabled("D*=differential, coherent needs 20+ dB SNR");

    ImGui::Spacing();

    // --- Forced Code Rate ---
    ImGui::Text("Forced Code Rate");
    ImGui::SetNextItemWidth(200);

    const char* rate_items[] = { "AUTO", "R1/4", "R1/2", "R2/3", "R3/4", "R5/6" };
    int rate_idx = 0;  // AUTO
    switch (settings.forced_code_rate) {
        case 0:    rate_idx = 1; break;  // R1/4
        case 2:    rate_idx = 2; break;  // R1/2
        case 3:    rate_idx = 3; break;  // R2/3
        case 4:    rate_idx = 4; break;  // R3/4
        case 5:    rate_idx = 5; break;  // R5/6
        default:   rate_idx = 0; break;  // AUTO
    }

    if (ImGui::Combo("##coderate", &rate_idx, rate_items, 6)) {
        switch (rate_idx) {
            case 0: settings.forced_code_rate = 0xFF; break;  // AUTO
            case 1: settings.forced_code_rate = 0; break;     // R1/4
            case 2: settings.forced_code_rate = 2; break;     // R1/2
            case 3: settings.forced_code_rate = 3; break;     // R2/3
            case 4: settings.forced_code_rate = 4; break;     // R3/4
            case 5: settings.forced_code_rate = 5; break;     // R5/6
        }
        changed = true;
    }
    ImGui::SameLine();
    ImGui::TextDisabled("R1/4=robust, R5/6=fast");

    ImGui::Spacing();
    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    // Summary of current settings
    ImGui::Text("Current Settings:");
    ImGui::Spacing();

    auto getWaveformStr = [](uint8_t w) -> const char* {
        switch (w) {
            case 0x00: return "OFDM";
            case 0x04: return "DPSK";
            case 0xFF: return "AUTO";
            default: return "Unknown";
        }
    };

    auto getModulationStr = [](uint8_t m) -> const char* {
        switch (m) {
            case 0: return "DBPSK";
            case 2: return "DQPSK";
            case 4: return "D8PSK";
            case 3: return "QPSK";
            case 6: return "16QAM";
            case 0xFF: return "AUTO";
            default: return "Unknown";
        }
    };

    auto getCodeRateStr = [](uint8_t r) -> const char* {
        switch (r) {
            case 0: return "R1/4";
            case 2: return "R1/2";
            case 3: return "R2/3";
            case 4: return "R3/4";
            case 5: return "R5/6";
            case 0xFF: return "AUTO";
            default: return "Unknown";
        }
    };

    ImGui::BulletText("Waveform: %s", getWaveformStr(settings.forced_waveform));
    ImGui::BulletText("Modulation: %s", getModulationStr(settings.forced_modulation));
    ImGui::BulletText("Code Rate: %s", getCodeRateStr(settings.forced_code_rate));

    // Call callback if settings changed
    if (changed && on_expert_settings_changed_) {
        on_expert_settings_changed_(settings.forced_waveform,
                                     settings.forced_modulation,
                                     settings.forced_code_rate);
    }
}

void SettingsWindow::renderNetworkTab(AppSettings& settings) {
    ImGui::Spacing();
    ImGui::Text("TCP Host Interface");
    ImGui::Separator();
    ImGui::Spacing();

    bool changed = false;

    // Enable/disable TCP interface
    if (ImGui::Checkbox("Enable TCP Interface", &settings.tcp_enabled)) {
        changed = true;
    }

    ImGui::Spacing();

    // Only show port settings if TCP is enabled
    ImGui::BeginDisabled(!settings.tcp_enabled);

    // Bind address
    ImGui::Text("Bind Address");
    ImGui::SetNextItemWidth(200);
    if (ImGui::InputText("##bind_addr", settings.tcp_bind_addr,
                         sizeof(settings.tcp_bind_addr))) {
        changed = true;
    }
    ImGui::SameLine();
    ImGui::TextDisabled("0.0.0.0 = all interfaces");

    ImGui::Spacing();

    // Command port
    ImGui::Text("Command Port");
    ImGui::SetNextItemWidth(150);
    int cmd_port = settings.tcp_cmd_port;
    if (ImGui::InputInt("##cmd_port", &cmd_port)) {
        if (cmd_port >= 1 && cmd_port <= 65535) {
            settings.tcp_cmd_port = static_cast<uint16_t>(cmd_port);
            changed = true;
        }
    }
    ImGui::SameLine();
    ImGui::TextDisabled("Default: 8300");

    // Data port
    ImGui::Text("Data Port");
    ImGui::SetNextItemWidth(150);
    int data_port = settings.tcp_data_port;
    if (ImGui::InputInt("##data_port", &data_port)) {
        if (data_port >= 1 && data_port <= 65535) {
            settings.tcp_data_port = static_cast<uint16_t>(data_port);
            changed = true;
        }
    }
    ImGui::SameLine();
    ImGui::TextDisabled("Default: 8301");

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    // KISS TNC settings
    ImGui::Text("KISS TNC Interface");
    ImGui::Spacing();

    if (ImGui::Checkbox("Enable KISS TNC", &settings.tcp_kiss_enabled)) {
        changed = true;
    }

    ImGui::BeginDisabled(!settings.tcp_kiss_enabled);

    ImGui::Text("KISS Port");
    ImGui::SetNextItemWidth(150);
    int kiss_port = settings.tcp_kiss_port;
    if (ImGui::InputInt("##kiss_port", &kiss_port)) {
        if (kiss_port >= 1 && kiss_port <= 65535) {
            settings.tcp_kiss_port = static_cast<uint16_t>(kiss_port);
            changed = true;
        }
    }
    ImGui::SameLine();
    ImGui::TextDisabled("Default: 8302");

    ImGui::EndDisabled();  // KISS disabled

    ImGui::EndDisabled();  // TCP disabled

    // Call callback if TCP settings changed
    if (changed && on_tcp_settings_changed_) {
        on_tcp_settings_changed_();
    }
}

void SettingsWindow::renderSecurityTab(AppSettings& settings) {
    bool changed = false;

    ImGui::Spacing();
    ImGui::Text("AES-256 Encryption");
    ImGui::Separator();
    ImGui::Spacing();

    // Enable checkbox
    if (ImGui::Checkbox("Enable Encryption", &settings.encryption_enabled)) {
        changed = true;
    }

    ImGui::BeginDisabled(!settings.encryption_enabled);

    ImGui::Spacing();

    // Passphrase input (password-style)
    ImGui::Text("Passphrase");
    ImGui::SetNextItemWidth(300);
    static char passphrase_input[128] = "";
    static bool show_passphrase = false;

    ImGuiInputTextFlags flags = ImGuiInputTextFlags_None;
    if (!show_passphrase) {
        flags |= ImGuiInputTextFlags_Password;
    }

    if (ImGui::InputText("##passphrase", passphrase_input, sizeof(passphrase_input), flags)) {
        copyBounded(settings.encryption_key, sizeof(settings.encryption_key),
                    std::string(passphrase_input));
        changed = true;
    }

    ImGui::SameLine();
    if (ImGui::Checkbox("Show", &show_passphrase)) {
        // Just toggle visibility
    }

    // Key strength indicator
    size_t key_len = strlen(passphrase_input);
    if (key_len == 0) {
        ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "Enter a passphrase");
    } else if (key_len < 8) {
        ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f), "Weak (use 8+ characters)");
    } else if (key_len < 16) {
        ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.0f, 1.0f), "Moderate");
    } else {
        ImGui::TextColored(ImVec4(0.3f, 1.0f, 0.3f, 1.0f), "Strong");
    }

    ImGui::EndDisabled();

    // Call callback if encryption settings changed
    if (changed && on_encryption_changed_) {
        on_encryption_changed_(settings.encryption_enabled, settings.encryption_key);
    }

    // ========================================
    // Compression Section
    // ========================================
    ImGui::Spacing();
    ImGui::Spacing();
    ImGui::Text("Data Compression");
    ImGui::Separator();
    ImGui::Spacing();

    bool compression_changed = false;

    if (ImGui::Checkbox("Enable Compression", &settings.compression_enabled)) {
        compression_changed = true;
    }

    // Call callback if compression settings changed
    if (compression_changed && on_compression_changed_) {
        on_compression_changed_(settings.compression_enabled);
    }
}

} // namespace gui
} // namespace ultra
