// KISS TNC protocol support
// Ported from RIA modem kiss.rs

#pragma once

#include <cstdint>
#include <vector>
#include <optional>

namespace ultra {
namespace interface {

// KISS frame types
enum class KissCommand : uint8_t {
    DataFrame = 0x00,
    TxDelay = 0x01,
    Persistence = 0x02,
    SlotTime = 0x03,
    TxTail = 0x04,
    FullDuplex = 0x05,
    SetHardware = 0x06,
    Return = 0xFF
};

// KISS special bytes
constexpr uint8_t KISS_FEND = 0xC0;  // Frame end
constexpr uint8_t KISS_FESC = 0xDB;  // Frame escape
constexpr uint8_t KISS_TFEND = 0xDC; // Transposed frame end
constexpr uint8_t KISS_TFESC = 0xDD; // Transposed frame escape

// KISS frame
struct KissFrame {
    uint8_t port = 0;          // Port number (0-15)
    KissCommand command = KissCommand::DataFrame;
    std::vector<uint8_t> data;

    // Create new data frame
    static KissFrame newData(uint8_t port, const std::vector<uint8_t>& data);

    // Create command frame
    static KissFrame newCommand(uint8_t port, KissCommand cmd, uint8_t value);

    // Encode frame to bytes with KISS framing
    std::vector<uint8_t> encode() const;

    // Decode frame from bytes (without FEND delimiters)
    static std::optional<KissFrame> decode(const std::vector<uint8_t>& data);
};

// KISS frame parser (handles stream of bytes)
class KissFramer {
public:
    KissFramer() = default;

    // Process incoming bytes and extract complete frames
    std::vector<KissFrame> process(const uint8_t* data, size_t len);

    // Reset framer state
    void reset();

private:
    std::vector<uint8_t> buffer_;
    bool in_frame_ = false;
};

// KISS port configuration
struct KissPortConfig {
    uint8_t port = 0;
    uint8_t tx_delay = 50;       // 500ms (in 10ms units)
    uint8_t persistence = 63;    // 25%
    uint8_t slot_time = 10;      // 100ms (in 10ms units)
    uint8_t tx_tail = 1;         // 10ms (in 10ms units)
    bool full_duplex = false;
    bool exit_requested = false; // Return command received

    // Handle configuration command
    void handleCommand(KissCommand cmd, uint8_t value);

    // Create configuration frames for this port
    std::vector<KissFrame> configFrames() const;
};

} // namespace interface
} // namespace ultra
