// Command parsing for TCP interface
// Ported from RIA modem commands.rs

#pragma once

#include <string>
#include <vector>
#include <utility>
#include <optional>

namespace ultra {
namespace interface {

// Commands supported by the Ultra modem
enum class Command {
    // Connection commands
    Connect,        // CONNECT <callsign>
    Disconnect,     // DISCONNECT
    Abort,          // ABORT

    // Beacon/CQ/Ping commands (broadcast without connection)
    Beacon,         // BEACON - transmit beacon, data follows on data port
    CQ,             // CQ - transmit CQ, data follows on data port, then listen
    Ping,           // PING <callsign> - send ping probe to target station
    RawTx,          // RAWTX [waveform] [modulation] [rate] - disconnected raw PHY TX

    // Configuration commands
    MyCall,         // MYCALL <callsign>
    MyAux,          // MYAUX <callsign1>,<callsign2>,...
    Compression,    // COMPRESSION ON|OFF
    Listen,         // LISTEN ON|OFF
    ChatMode,       // CHATMODE ON|OFF
    AutoMode,       // AUTOMODE/AUTO ON|OFF
    WinlinkSession, // WINLINK/WINLINKSESSION ON|OFF

    // Ultra-specific waveform/mode commands
    Waveform,       // WAVEFORM <AUTO|MC_DPSK|OFDM_CHIRP|OFDM_COX>
    Modulation,     // MODULATION <AUTO|DBPSK|DQPSK|QPSK|D8PSK|QAM16|QAM32|QAM64>
    CodeRate,       // CODERATE <AUTO|R1_4|R1_2|R2_3|R3_4|R5_6|R7_8>
    MCDPSKCarriers, // MCDPSKCARRIERS <5|8|10|15|20> - MC-DPSK carrier count

    // Status queries
    Version,        // VERSION
    Codec,          // CODEC
    State,          // STATE - get connection state
    PttState,       // PTT/PTTSTATE
    Busy,           // BUSY/BUSYSTATE
    Buffer,         // BUFFER

    // Control commands
    Tune,           // TUNE ON|OFF
    CwId,           // CWID [callsign]
    Close,          // CLOSE

    // PTT timing (used by serial PTT)
    PttLead,        // PTTLEAD <ms> - delay before TX
    PttTail,        // PTTTAIL <ms> - delay after TX
    TxDrive,        // TXDRIVE <0.0-1.0> - TX audio level

    // Encryption commands
    Encrypt,        // ENCRYPT ON|OFF - enable/disable encryption
    EncryptKey,     // ENCRYPTKEY <passphrase> - set encryption key

    // File transfer commands
    SendFile,       // SENDFILE <filepath> - send a file

    // CAT control commands
    CatEnable,      // CATENABLE ON|OFF - enable/disable CAT
    CatBackend,     // CATBACKEND none|serial|hamlib|flex - select backend
    CatModel,       // CATMODEL <hamlib_model_id> - set Hamlib model
    CatPort,        // CATPORT <path or host:port> - set port
    CatBaud,        // CATBAUD <rate> - set serial baud rate
    CatSlice,       // CATSLICE <n> - set FlexRadio slice number
    CatConnect,     // CATCONNECT - connect to radio
    CatDisconnect,  // CATDISCONNECT - disconnect from radio
    CatPtt,         // CATPTT ON|OFF - manual PTT control
    CatFreq,        // CATFREQ <hz> - set frequency
    CatGetFreq,     // CATGETFREQ - get frequency
    CatMode,        // CATMODE USB|LSB|AM|FM|CW|DATA - set mode
    CatGetMode,     // CATGETMODE - get mode
    CatWatchdog,    // CATWATCHDOG <seconds> - set TX watchdog timeout
    CatPttLead,     // CATPTTLEAD <ms> - set PTT lead delay
    CatPttTail,     // CATPTTTAIL <ms> - set PTT tail delay
    CatStatus,      // CATSTATUS - get CAT status

    Unknown
};

// Parsed command with arguments
struct ParsedCommand {
    Command cmd = Command::Unknown;
    std::string args;

    // Helper parsers for common argument types
    bool argAsBool() const;
    int argAsInt(int default_val = 0) const;
    uint32_t argAsUint32(uint32_t default_val = 0) const;
    uint64_t argAsUint64(uint64_t default_val = 0) const;
    std::vector<std::string> argAsList(char delimiter = ',') const;
};

// Command parser with buffering for partial lines
class CommandParser {
public:
    CommandParser() = default;

    // Parse incoming data, return complete commands
    std::vector<ParsedCommand> parse(const uint8_t* data, size_t len);
    std::vector<ParsedCommand> parse(const std::string& data);

    // Clear internal buffer
    void clear();

private:
    std::string buffer_;

    // Parse a single line into command
    static ParsedCommand parseLine(const std::string& line);
};

// Command to string (for debugging)
const char* commandToString(Command cmd);

} // namespace interface
} // namespace ultra
