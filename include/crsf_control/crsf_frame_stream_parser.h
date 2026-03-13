#ifndef CRSF_CONTROL_CRSF_FRAME_STREAM_PARSER_H
#define CRSF_CONTROL_CRSF_FRAME_STREAM_PARSER_H

#include <cstddef>
#include <cstdint>
#include <vector>

#include "crsf_control/crsf_protocol.h"

namespace crsf_control {

struct CrsfRxFrame {
    uint8_t address = 0;
    uint8_t length = 0;
    uint8_t type = 0;
    std::vector<uint8_t> payload;
    uint8_t crc = 0;
};

class FrameStreamParser {
public:
    FrameStreamParser();

    void reset();

    size_t pushBytes(const uint8_t* data, size_t len, std::vector<CrsfRxFrame>* outFrames);

    size_t bufferedSize() const;

private:
    static constexpr uint8_t kSyncByte = CRSF_SYNC_BYTE;
    static constexpr uint8_t kMinLength = 2;   // type + crc
    static constexpr uint8_t kMaxLength = 62;  // max frame size is 64 including address and length

    size_t extractFrames(std::vector<CrsfRxFrame>* outFrames);
    void compactIfNeeded();

    std::vector<uint8_t> buffer_;
    size_t start_;
};

}  // namespace crsf_control

#endif  // CRSF_CONTROL_CRSF_FRAME_STREAM_PARSER_H
