// Command parsing implementation
// Ported from RIA modem commands.rs

#include "command_parser.hpp"
#include <algorithm>
#include <cctype>
#include <sstream>

namespace ultra {
namespace interface {

// Helper: convert string to uppercase
static std::string toUpper(const std::string& s) {
    std::string result = s;
    std::transform(result.begin(), result.end(), result.begin(), ::toupper);
    return result;
}

// Helper: trim whitespace
static std::string trim(const std::string& s) {
    size_t start = s.find_first_not_of(" \t\r\n");
    if (start == std::string::npos) return "";
    size_t end = s.find_last_not_of(" \t\r\n");
    return s.substr(start, end - start + 1);
}

bool ParsedCommand::argAsBool() const {
    std::string upper = toUpper(args);
    return upper == "ON" || upper == "1" || upper == "TRUE" || upper == "YES";
}

int ParsedCommand::argAsInt(int default_val) const {
    try {
        return std::stoi(args);
    } catch (...) {
        return default_val;
    }
}

uint32_t ParsedCommand::argAsUint32(uint32_t default_val) const {
    try {
        return static_cast<uint32_t>(std::stoul(args));
    } catch (...) {
        return default_val;
    }
}

uint64_t ParsedCommand::argAsUint64(uint64_t default_val) const {
    try {
        return std::stoull(args);
    } catch (...) {
        return default_val;
    }
}

std::vector<std::string> ParsedCommand::argAsList(char delimiter) const {
    std::vector<std::string> result;
    std::stringstream ss(args);
    std::string item;
    while (std::getline(ss, item, delimiter)) {
        std::string trimmed = trim(item);
        if (!trimmed.empty()) {
            result.push_back(trimmed);
        }
    }
    return result;
}

std::vector<ParsedCommand> CommandParser::parse(const uint8_t* data, size_t len) {
    return parse(std::string(reinterpret_cast<const char*>(data), len));
}

std::vector<ParsedCommand> CommandParser::parse(const std::string& data) {
    buffer_ += data;

    std::vector<ParsedCommand> commands;

    // Extract complete lines (terminated by \r or \n)
    size_t pos;
    while ((pos = buffer_.find_first_of("\r\n")) != std::string::npos) {
        std::string line = buffer_.substr(0, pos);
        buffer_ = buffer_.substr(pos + 1);

        // Skip empty lines
        line = trim(line);
        if (line.empty()) continue;

        commands.push_back(parseLine(line));
    }

    return commands;
}

void CommandParser::clear() {
    buffer_.clear();
}

ParsedCommand CommandParser::parseLine(const std::string& line) {
    ParsedCommand result;

    // Split into command and arguments
    size_t space_pos = line.find(' ');
    std::string cmd_str;

    if (space_pos != std::string::npos) {
        cmd_str = toUpper(line.substr(0, space_pos));
        result.args = trim(line.substr(space_pos + 1));
    } else {
        cmd_str = toUpper(line);
        result.args = "";
    }

    // Match command (case-insensitive)
    if (cmd_str == "CONNECT") {
        result.cmd = Command::Connect;
    } else if (cmd_str == "DISCONNECT") {
        result.cmd = Command::Disconnect;
    } else if (cmd_str == "ABORT") {
        result.cmd = Command::Abort;
    } else if (cmd_str == "BEACON") {
        result.cmd = Command::Beacon;
    } else if (cmd_str == "CQ") {
        result.cmd = Command::CQ;
    } else if (cmd_str == "PING") {
        result.cmd = Command::Ping;
    } else if (cmd_str == "RAWTX") {
        result.cmd = Command::RawTx;
    } else if (cmd_str == "MYCALL") {
        result.cmd = Command::MyCall;
    } else if (cmd_str == "MYAUX") {
        result.cmd = Command::MyAux;
    } else if (cmd_str == "COMPRESSION") {
        result.cmd = Command::Compression;
    } else if (cmd_str == "LISTEN") {
        result.cmd = Command::Listen;
    } else if (cmd_str == "CHATMODE") {
        result.cmd = Command::ChatMode;
    } else if (cmd_str == "AUTOMODE" || cmd_str == "AUTO") {
        result.cmd = Command::AutoMode;
    } else if (cmd_str == "WINLINK" || cmd_str == "WINLINKSESSION") {
        result.cmd = Command::WinlinkSession;
    } else if (cmd_str == "WAVEFORM") {
        result.cmd = Command::Waveform;
    } else if (cmd_str == "MODULATION" || cmd_str == "MOD") {
        result.cmd = Command::Modulation;
    } else if (cmd_str == "CODERATE" || cmd_str == "RATE" || cmd_str == "FEC") {
        result.cmd = Command::CodeRate;
    } else if (cmd_str == "MCDPSKCARRIERS" || cmd_str == "DPSKCARRIERS" || cmd_str == "CARRIERS") {
        result.cmd = Command::MCDPSKCarriers;
    } else if (cmd_str == "VERSION") {
        result.cmd = Command::Version;
    } else if (cmd_str == "CODEC") {
        result.cmd = Command::Codec;
    } else if (cmd_str == "STATE") {
        result.cmd = Command::State;
    } else if (cmd_str == "PTT" || cmd_str == "PTTSTATE") {
        result.cmd = Command::PttState;
    } else if (cmd_str == "BUSY" || cmd_str == "BUSYSTATE") {
        result.cmd = Command::Busy;
    } else if (cmd_str == "BUFFER") {
        result.cmd = Command::Buffer;
    } else if (cmd_str == "TUNE") {
        result.cmd = Command::Tune;
    } else if (cmd_str == "CWID") {
        result.cmd = Command::CwId;
    } else if (cmd_str == "CLOSE") {
        result.cmd = Command::Close;
    } else if (cmd_str == "PTTLEAD" || cmd_str == "TXDELAY") {
        result.cmd = Command::PttLead;
    } else if (cmd_str == "PTTTAIL") {
        result.cmd = Command::PttTail;
    } else if (cmd_str == "TXDRIVE") {
        result.cmd = Command::TxDrive;
    } else if (cmd_str == "ENCRYPT" || cmd_str == "ENCRYPTION") {
        result.cmd = Command::Encrypt;
    } else if (cmd_str == "ENCRYPTKEY" || cmd_str == "KEY") {
        result.cmd = Command::EncryptKey;
    } else if (cmd_str == "SENDFILE" || cmd_str == "SEND") {
        result.cmd = Command::SendFile;
    }
    // CAT control commands
    else if (cmd_str == "CATENABLE") {
        result.cmd = Command::CatEnable;
    } else if (cmd_str == "CATBACKEND") {
        result.cmd = Command::CatBackend;
    } else if (cmd_str == "CATMODEL") {
        result.cmd = Command::CatModel;
    } else if (cmd_str == "CATPORT") {
        result.cmd = Command::CatPort;
    } else if (cmd_str == "CATBAUD") {
        result.cmd = Command::CatBaud;
    } else if (cmd_str == "CATSLICE") {
        result.cmd = Command::CatSlice;
    } else if (cmd_str == "CATCONNECT") {
        result.cmd = Command::CatConnect;
    } else if (cmd_str == "CATDISCONNECT") {
        result.cmd = Command::CatDisconnect;
    } else if (cmd_str == "CATPTT") {
        result.cmd = Command::CatPtt;
    } else if (cmd_str == "CATFREQ") {
        result.cmd = Command::CatFreq;
    } else if (cmd_str == "CATGETFREQ") {
        result.cmd = Command::CatGetFreq;
    } else if (cmd_str == "CATMODE") {
        result.cmd = Command::CatMode;
    } else if (cmd_str == "CATGETMODE") {
        result.cmd = Command::CatGetMode;
    } else if (cmd_str == "CATWATCHDOG") {
        result.cmd = Command::CatWatchdog;
    } else if (cmd_str == "CATPTTLEAD") {
        result.cmd = Command::CatPttLead;
    } else if (cmd_str == "CATPTTTAIL") {
        result.cmd = Command::CatPttTail;
    } else if (cmd_str == "CATSTATUS") {
        result.cmd = Command::CatStatus;
    } else {
        result.cmd = Command::Unknown;
        result.args = line;  // Store original line for unknown commands
    }

    return result;
}

const char* commandToString(Command cmd) {
    switch (cmd) {
        case Command::Connect: return "CONNECT";
        case Command::Disconnect: return "DISCONNECT";
        case Command::Abort: return "ABORT";
        case Command::Beacon: return "BEACON";
        case Command::CQ: return "CQ";
        case Command::Ping: return "PING";
        case Command::RawTx: return "RAWTX";
        case Command::MyCall: return "MYCALL";
        case Command::MyAux: return "MYAUX";
        case Command::Compression: return "COMPRESSION";
        case Command::Listen: return "LISTEN";
        case Command::ChatMode: return "CHATMODE";
        case Command::AutoMode: return "AUTOMODE";
        case Command::WinlinkSession: return "WINLINK";
        case Command::Waveform: return "WAVEFORM";
        case Command::Modulation: return "MODULATION";
        case Command::CodeRate: return "CODERATE";
        case Command::MCDPSKCarriers: return "MCDPSKCARRIERS";
        case Command::Version: return "VERSION";
        case Command::Codec: return "CODEC";
        case Command::State: return "STATE";
        case Command::PttState: return "PTT";
        case Command::Busy: return "BUSY";
        case Command::Buffer: return "BUFFER";
        case Command::Tune: return "TUNE";
        case Command::CwId: return "CWID";
        case Command::Close: return "CLOSE";
        case Command::PttLead: return "PTTLEAD";
        case Command::PttTail: return "PTTTAIL";
        case Command::TxDrive: return "TXDRIVE";
        case Command::Encrypt: return "ENCRYPT";
        case Command::EncryptKey: return "ENCRYPTKEY";
        case Command::SendFile: return "SENDFILE";
        case Command::CatEnable: return "CATENABLE";
        case Command::CatBackend: return "CATBACKEND";
        case Command::CatModel: return "CATMODEL";
        case Command::CatPort: return "CATPORT";
        case Command::CatBaud: return "CATBAUD";
        case Command::CatSlice: return "CATSLICE";
        case Command::CatConnect: return "CATCONNECT";
        case Command::CatDisconnect: return "CATDISCONNECT";
        case Command::CatPtt: return "CATPTT";
        case Command::CatFreq: return "CATFREQ";
        case Command::CatGetFreq: return "CATGETFREQ";
        case Command::CatMode: return "CATMODE";
        case Command::CatGetMode: return "CATGETMODE";
        case Command::CatWatchdog: return "CATWATCHDOG";
        case Command::CatPttLead: return "CATPTTLEAD";
        case Command::CatPttTail: return "CATPTTTAIL";
        case Command::CatStatus: return "CATSTATUS";
        case Command::Unknown: return "UNKNOWN";
        default: return "UNKNOWN";
    }
}

} // namespace interface
} // namespace ultra
