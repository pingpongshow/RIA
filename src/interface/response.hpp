// Response formatting for TCP interface
// Ported from RIA modem commands.rs

#pragma once

#include <string>
#include <vector>
#include <cstdint>

namespace ultra {
namespace interface {

// Response types
enum class ResponseType {
    Ok,
    Error,
    Connected,
    Disconnected,
    Pending,
    LinkBroken,
    Busy,
    Version,
    Buffer,
    Ptt,
    Snr,
    State,
    DataReceived,
    Custom,
    Close,
    CatStatus,
    CatFreq
};

// Response class for formatting modem responses
class Response {
public:
    // Factory methods
    static Response ok();
    static Response error(const std::string& msg);
    static Response connected(const std::string& callsign);
    static Response disconnected();
    static Response pending();
    static Response linkBroken();
    static Response busy(bool detected);
    static Response version(const std::string& ver);
    static Response buffer(size_t bytes);
    static Response ptt(bool active);
    static Response snr(float db);
    static Response state(const std::string& state);
    static Response dataReceived(size_t bytes);
    static Response custom(const std::string& msg);
    static Response close();
    static Response catStatus(bool enabled, bool connected,
                              const std::string& backend, uint32_t model,
                              const std::string& port);
    static Response catFreq(uint64_t freq_hz);

    // Format as string (CR terminated)
    std::string toString() const;

    // Format as bytes
    std::vector<uint8_t> toBytes() const;

    // Get response type
    ResponseType type() const { return type_; }

    // Check if this response should close the connection
    bool shouldClose() const { return type_ == ResponseType::Close; }

private:
    ResponseType type_;
    std::string data_;

    Response(ResponseType type, const std::string& data = "");
};

} // namespace interface
} // namespace ultra
