#include "cat_backend.hpp"
#include <algorithm>
#include <cctype>

namespace ultra {
namespace cat {

const char* radioModeToString(RadioMode mode) {
    switch (mode) {
        case RadioMode::USB:     return "USB";
        case RadioMode::LSB:     return "LSB";
        case RadioMode::AM:      return "AM";
        case RadioMode::FM:      return "FM";
        case RadioMode::CW:      return "CW";
        case RadioMode::CW_R:    return "CW-R";
        case RadioMode::RTTY:    return "RTTY";
        case RadioMode::RTTY_R:  return "RTTY-R";
        case RadioMode::DATA:    return "DATA";
        case RadioMode::DATA_R:  return "DATA-R";
        case RadioMode::UNKNOWN:
        default:                 return "UNKNOWN";
    }
}

RadioMode stringToRadioMode(const std::string& str) {
    std::string upper = str;
    std::transform(upper.begin(), upper.end(), upper.begin(),
                   [](unsigned char c) { return std::toupper(c); });

    if (upper == "USB")                     return RadioMode::USB;
    if (upper == "LSB")                     return RadioMode::LSB;
    if (upper == "AM")                      return RadioMode::AM;
    if (upper == "FM")                      return RadioMode::FM;
    if (upper == "CW")                      return RadioMode::CW;
    if (upper == "CW-R" || upper == "CWR")  return RadioMode::CW_R;
    if (upper == "RTTY")                    return RadioMode::RTTY;
    if (upper == "RTTY-R" || upper == "RTTYR") return RadioMode::RTTY_R;
    if (upper == "DATA" || upper == "DIG" || upper == "DIGITAL")
        return RadioMode::DATA;
    if (upper == "DATA-R" || upper == "DATAR" || upper == "DIG-R")
        return RadioMode::DATA_R;

    return RadioMode::UNKNOWN;
}

const char* backendTypeToString(CatBackendType type) {
    switch (type) {
        case CatBackendType::None:       return "None";
        case CatBackendType::SerialPtt:  return "Serial PTT";
        case CatBackendType::Hamlib:     return "Hamlib";
        case CatBackendType::KenwoodTcp: return "Kenwood TCP";
        default:                         return "Unknown";
    }
}

CatBackendType stringToBackendType(const std::string& str) {
    std::string lower = str;
    std::transform(lower.begin(), lower.end(), lower.begin(),
                   [](unsigned char c) { return std::tolower(c); });

    if (lower == "none" || lower == "disabled" || lower == "off")
        return CatBackendType::None;
    if (lower == "serial" || lower == "serialptt" || lower == "serial_ptt" || lower == "dtr" || lower == "rts")
        return CatBackendType::SerialPtt;
    if (lower == "hamlib" || lower == "rig")
        return CatBackendType::Hamlib;
    if (lower == "kenwood" || lower == "kenwoodtcp" || lower == "kenwood_tcp" ||
        lower == "flex" || lower == "flexradio" || lower == "smartsdr")
        return CatBackendType::KenwoodTcp;

    return CatBackendType::None;
}

std::vector<const char*> getBackendTypeNames() {
    return {
        "None",
        "Serial PTT",
        "Hamlib",
        "Kenwood TCP (FlexRadio)"
    };
}

} // namespace cat
} // namespace ultra
