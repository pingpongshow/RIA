// KISS TNC protocol implementation

#include "kiss_tnc.hpp"

namespace ultra {
namespace interface {

KissFrame KissFrame::newData(uint8_t port, const std::vector<uint8_t>& data) {
    KissFrame frame;
    frame.port = port & 0x0F;
    frame.command = KissCommand::DataFrame;
    frame.data = data;
    return frame;
}

KissFrame KissFrame::newCommand(uint8_t port, KissCommand cmd, uint8_t value) {
    KissFrame frame;
    frame.port = port & 0x0F;
    frame.command = cmd;
    frame.data = {value};
    return frame;
}

std::vector<uint8_t> KissFrame::encode() const {
    std::vector<uint8_t> output;

    // Start with FEND
    output.push_back(KISS_FEND);

    // Command byte (port << 4 | command)
    uint8_t cmd_byte = (port << 4) | (static_cast<uint8_t>(command) & 0x0F);

    // Encode command byte
    if (cmd_byte == KISS_FEND) {
        output.push_back(KISS_FESC);
        output.push_back(KISS_TFEND);
    } else if (cmd_byte == KISS_FESC) {
        output.push_back(KISS_FESC);
        output.push_back(KISS_TFESC);
    } else {
        output.push_back(cmd_byte);
    }

    // Encode data with escape sequences
    for (uint8_t byte : data) {
        if (byte == KISS_FEND) {
            output.push_back(KISS_FESC);
            output.push_back(KISS_TFEND);
        } else if (byte == KISS_FESC) {
            output.push_back(KISS_FESC);
            output.push_back(KISS_TFESC);
        } else {
            output.push_back(byte);
        }
    }

    // End with FEND
    output.push_back(KISS_FEND);

    return output;
}

std::optional<KissFrame> KissFrame::decode(const std::vector<uint8_t>& data) {
    if (data.size() < 1) {
        return std::nullopt;
    }

    // First byte is command
    uint8_t cmd_byte = data[0];
    uint8_t port = (cmd_byte >> 4) & 0x0F;
    uint8_t cmd_nibble = cmd_byte & 0x0F;

    KissCommand command;
    if (cmd_byte == 0xFF) {
        command = KissCommand::Return;
    } else {
        switch (cmd_nibble) {
            case 0x00: command = KissCommand::DataFrame; break;
            case 0x01: command = KissCommand::TxDelay; break;
            case 0x02: command = KissCommand::Persistence; break;
            case 0x03: command = KissCommand::SlotTime; break;
            case 0x04: command = KissCommand::TxTail; break;
            case 0x05: command = KissCommand::FullDuplex; break;
            case 0x06: command = KissCommand::SetHardware; break;
            default: return std::nullopt;
        }
    }

    // Unescape remaining data
    std::vector<uint8_t> frame_data;
    bool escape = false;

    for (size_t i = 1; i < data.size(); ++i) {
        uint8_t byte = data[i];
        if (escape) {
            if (byte == KISS_TFEND) {
                frame_data.push_back(KISS_FEND);
            } else if (byte == KISS_TFESC) {
                frame_data.push_back(KISS_FESC);
            } else {
                frame_data.push_back(byte);  // Invalid escape, pass through
            }
            escape = false;
        } else if (byte == KISS_FESC) {
            escape = true;
        } else {
            frame_data.push_back(byte);
        }
    }

    KissFrame frame;
    frame.port = port;
    frame.command = command;
    frame.data = std::move(frame_data);
    return frame;
}

std::vector<KissFrame> KissFramer::process(const uint8_t* data, size_t len) {
    std::vector<KissFrame> frames;

    for (size_t i = 0; i < len; ++i) {
        uint8_t byte = data[i];

        if (byte == KISS_FEND) {
            if (in_frame_ && !buffer_.empty()) {
                // End of frame
                if (auto frame = KissFrame::decode(buffer_)) {
                    frames.push_back(std::move(*frame));
                }
            }
            // Start new frame
            buffer_.clear();
            in_frame_ = true;
        } else if (in_frame_) {
            buffer_.push_back(byte);
        }
        // Bytes outside frame are ignored
    }

    return frames;
}

void KissFramer::reset() {
    buffer_.clear();
    in_frame_ = false;
}

void KissPortConfig::handleCommand(KissCommand cmd, uint8_t value) {
    switch (cmd) {
        case KissCommand::TxDelay:
            tx_delay = value;
            break;
        case KissCommand::Persistence:
            persistence = value;
            break;
        case KissCommand::SlotTime:
            slot_time = value;
            break;
        case KissCommand::TxTail:
            tx_tail = value;
            break;
        case KissCommand::FullDuplex:
            full_duplex = (value != 0);
            break;
        case KissCommand::Return:
            exit_requested = true;
            break;
        case KissCommand::SetHardware:
        case KissCommand::DataFrame:
            // Handled separately
            break;
    }
}

std::vector<KissFrame> KissPortConfig::configFrames() const {
    return {
        KissFrame::newCommand(port, KissCommand::TxDelay, tx_delay),
        KissFrame::newCommand(port, KissCommand::Persistence, persistence),
        KissFrame::newCommand(port, KissCommand::SlotTime, slot_time),
        KissFrame::newCommand(port, KissCommand::TxTail, tx_tail),
        KissFrame::newCommand(port, KissCommand::FullDuplex, full_duplex ? 1 : 0)
    };
}

} // namespace interface
} // namespace ultra
