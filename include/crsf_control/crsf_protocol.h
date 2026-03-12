#ifndef CRSF_CONTROL_CRSF_PROTOCOL_H
#define CRSF_CONTROL_CRSF_PROTOCOL_H

#include <cstdint>
#include <cstddef>
#include <algorithm>

namespace crsf_control {

// CRSF protocol constants
constexpr uint8_t CRSF_SYNC_BYTE          = 0xC8;
constexpr uint8_t CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16;
constexpr uint8_t CRSF_CRC_POLY           = 0xD5;

constexpr size_t  CRSF_NUM_CHANNELS       = 16;
constexpr size_t  CRSF_CHANNEL_BITS       = 11;
constexpr size_t  CRSF_CHANNELS_PAYLOAD_SIZE = 22;  // 16 * 11 / 8 = 22 bytes
constexpr size_t  CRSF_RC_FRAME_SIZE      = 26;     // sync + len + type + 22 payload + crc

// CRSF channel value range
constexpr uint16_t CRSF_CHANNEL_MIN       = 172;    // 988us
constexpr uint16_t CRSF_CHANNEL_CENTER    = 992;    // 1500us
constexpr uint16_t CRSF_CHANNEL_MAX       = 1811;   // 2012us
constexpr uint16_t CRSF_CHANNEL_VALUE_MAX = 1984;   // absolute max 11-bit

// Packed RC channels bitfield (16 channels x 11 bits = 176 bits = 22 bytes)
struct __attribute__((packed)) CrsfChannelsPacked {
    unsigned ch0  : 11;
    unsigned ch1  : 11;
    unsigned ch2  : 11;
    unsigned ch3  : 11;
    unsigned ch4  : 11;
    unsigned ch5  : 11;
    unsigned ch6  : 11;
    unsigned ch7  : 11;
    unsigned ch8  : 11;
    unsigned ch9  : 11;
    unsigned ch10 : 11;
    unsigned ch11 : 11;
    unsigned ch12 : 11;
    unsigned ch13 : 11;
    unsigned ch14 : 11;
    unsigned ch15 : 11;
};

class CrsfProtocol {
public:
    // Compute CRC8 with polynomial 0xD5
    // Covers bytes from type to end of payload
    static uint8_t crc8(const uint8_t* data, size_t len);

    // Convert float value [-1.0, 1.0] to CRSF channel value [172, 1811]
    static uint16_t floatToCrsf(float value);

    // Pack 16 channel values into a complete CRSF RC_CHANNELS_PACKED frame
    // channels: array of 16 CRSF channel values (0-1984)
    // outBuf: output buffer, must be at least CRSF_RC_FRAME_SIZE (26) bytes
    // Returns number of bytes written (26)
    static size_t packChannelsFrame(const uint16_t channels[CRSF_NUM_CHANNELS],
                                    uint8_t* outBuf);
};

}  // namespace crsf_control

#endif  // CRSF_CONTROL_CRSF_PROTOCOL_H
