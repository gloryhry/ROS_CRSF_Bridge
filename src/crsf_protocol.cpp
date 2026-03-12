#include "crsf_control/crsf_protocol.h"
#include <cstring>

namespace crsf_control {

uint8_t CrsfProtocol::crc8(const uint8_t* data, size_t len)
{
    uint8_t crc = 0;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int j = 0; j < 8; ++j) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ CRSF_CRC_POLY;
            } else {
                crc = crc << 1;
            }
        }
    }
    return crc;
}

uint16_t CrsfProtocol::floatToCrsf(float value)
{
    // Clamp input to [-1.0, 1.0]
    if (value < -1.0f) value = -1.0f;
    if (value > 1.0f) value = 1.0f;

    // Linear map [-1.0, 1.0] to [CRSF_CHANNEL_MIN, CRSF_CHANNEL_MAX]
    // -1.0 → 172, 0.0 → 991.5 ≈ 992, 1.0 → 1811
    float crsf_f = static_cast<float>(CRSF_CHANNEL_MIN)
                 + (value + 1.0f) * 0.5f
                   * static_cast<float>(CRSF_CHANNEL_MAX - CRSF_CHANNEL_MIN);
    uint16_t crsf_val = static_cast<uint16_t>(crsf_f + 0.5f);

    // Clamp to valid range
    if (crsf_val < CRSF_CHANNEL_MIN) crsf_val = CRSF_CHANNEL_MIN;
    if (crsf_val > CRSF_CHANNEL_MAX) crsf_val = CRSF_CHANNEL_MAX;

    return crsf_val;
}

size_t CrsfProtocol::packChannelsFrame(const uint16_t channels[CRSF_NUM_CHANNELS],
                                        uint8_t* outBuf)
{
    // Frame structure: [sync] [len] [type] [22 bytes payload] [crc8]
    outBuf[0] = CRSF_SYNC_BYTE;
    outBuf[1] = CRSF_CHANNELS_PAYLOAD_SIZE + 2;  // len = payload + type + crc = 24
    outBuf[2] = CRSF_FRAMETYPE_RC_CHANNELS_PACKED;

    // Pack 16 channels x 11 bits into payload using bitfield struct
    CrsfChannelsPacked packed;
    std::memset(&packed, 0, sizeof(packed));

    packed.ch0  = channels[0]  & 0x7FF;
    packed.ch1  = channels[1]  & 0x7FF;
    packed.ch2  = channels[2]  & 0x7FF;
    packed.ch3  = channels[3]  & 0x7FF;
    packed.ch4  = channels[4]  & 0x7FF;
    packed.ch5  = channels[5]  & 0x7FF;
    packed.ch6  = channels[6]  & 0x7FF;
    packed.ch7  = channels[7]  & 0x7FF;
    packed.ch8  = channels[8]  & 0x7FF;
    packed.ch9  = channels[9]  & 0x7FF;
    packed.ch10 = channels[10] & 0x7FF;
    packed.ch11 = channels[11] & 0x7FF;
    packed.ch12 = channels[12] & 0x7FF;
    packed.ch13 = channels[13] & 0x7FF;
    packed.ch14 = channels[14] & 0x7FF;
    packed.ch15 = channels[15] & 0x7FF;

    std::memcpy(&outBuf[3], &packed, CRSF_CHANNELS_PAYLOAD_SIZE);

    // CRC8 over type + payload (bytes [2] to [2 + payload_size])
    outBuf[3 + CRSF_CHANNELS_PAYLOAD_SIZE] = crc8(&outBuf[2], CRSF_CHANNELS_PAYLOAD_SIZE + 1);

    return CRSF_RC_FRAME_SIZE;
}

}  // namespace crsf_control
