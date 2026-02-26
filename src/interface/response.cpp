// Response formatting implementation

#include "response.hpp"
#include <sstream>
#include <iomanip>

namespace ultra {
namespace interface {

Response::Response(ResponseType type, const std::string& data)
    : type_(type), data_(data) {}

Response Response::ok() {
    return Response(ResponseType::Ok);
}

Response Response::error(const std::string& msg) {
    return Response(ResponseType::Error, msg);
}

Response Response::connected(const std::string& callsign) {
    return Response(ResponseType::Connected, callsign);
}

Response Response::disconnected() {
    return Response(ResponseType::Disconnected);
}

Response Response::pending() {
    return Response(ResponseType::Pending);
}

Response Response::linkBroken() {
    return Response(ResponseType::LinkBroken);
}

Response Response::busy(bool detected) {
    return Response(ResponseType::Busy, detected ? "ON" : "OFF");
}

Response Response::version(const std::string& ver) {
    return Response(ResponseType::Version, ver);
}

Response Response::buffer(size_t bytes) {
    return Response(ResponseType::Buffer, std::to_string(bytes));
}

Response Response::ptt(bool active) {
    return Response(ResponseType::Ptt, active ? "ON" : "OFF");
}

Response Response::snr(float db) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(1) << db;
    return Response(ResponseType::Snr, oss.str());
}

Response Response::state(const std::string& state) {
    return Response(ResponseType::State, state);
}

Response Response::dataReceived(size_t bytes) {
    return Response(ResponseType::DataReceived, std::to_string(bytes));
}

Response Response::custom(const std::string& msg) {
    return Response(ResponseType::Custom, msg);
}

Response Response::close() {
    return Response(ResponseType::Close);
}

Response Response::catStatus(bool enabled, bool connected,
                              const std::string& backend, uint32_t model,
                              const std::string& port) {
    std::ostringstream oss;
    oss << (enabled ? "ENABLED" : "DISABLED") << " "
        << (connected ? "CONNECTED" : "DISCONNECTED") << " "
        << backend << " " << model << " " << port;
    return Response(ResponseType::CatStatus, oss.str());
}

Response Response::catFreq(uint64_t freq_hz) {
    return Response(ResponseType::CatFreq, std::to_string(freq_hz));
}

std::string Response::toString() const {
    std::string result;

    switch (type_) {
        case ResponseType::Ok:
            result = "OK";
            break;
        case ResponseType::Error:
            result = "ERROR " + data_;
            break;
        case ResponseType::Connected:
            result = "CONNECTED " + data_;
            break;
        case ResponseType::Disconnected:
            result = "DISCONNECTED";
            break;
        case ResponseType::Pending:
            result = "PENDING";
            break;
        case ResponseType::LinkBroken:
            result = "LINK BROKEN";
            break;
        case ResponseType::Busy:
            result = "BUSY " + data_;
            break;
        case ResponseType::Version:
            result = "VERSION " + data_;
            break;
        case ResponseType::Buffer:
            result = "BUFFER " + data_;
            break;
        case ResponseType::Ptt:
            result = "PTT " + data_;
            break;
        case ResponseType::Snr:
            result = "SNR " + data_;
            break;
        case ResponseType::State:
            result = "STATE " + data_;
            break;
        case ResponseType::DataReceived:
            result = "DATA " + data_;
            break;
        case ResponseType::Custom:
            result = data_;
            break;
        case ResponseType::Close:
            result = "OK";
            break;
        case ResponseType::CatStatus:
            result = "CAT " + data_;
            break;
        case ResponseType::CatFreq:
            result = "CATFREQ " + data_;
            break;
    }

    return result + "\r";
}

std::vector<uint8_t> Response::toBytes() const {
    std::string str = toString();
    return std::vector<uint8_t>(str.begin(), str.end());
}

} // namespace interface
} // namespace ultra
