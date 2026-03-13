#include "crsf_control/crsf_frame_stream_parser.h"

#include <algorithm>

namespace crsf_control {

FrameStreamParser::FrameStreamParser()
    : buffer_(), start_(0)
{
}

void FrameStreamParser::reset()
{
    buffer_.clear();
    start_ = 0;
}

size_t FrameStreamParser::pushBytes(const uint8_t* data, size_t len, std::vector<CrsfRxFrame>* outFrames)
{
    if (outFrames == nullptr) {
        return 0;
    }

    if (data != nullptr && len > 0) {
        compactIfNeeded();
        buffer_.insert(buffer_.end(), data, data + len);
    }

    return extractFrames(outFrames);
}

size_t FrameStreamParser::bufferedSize() const
{
    if (start_ >= buffer_.size()) {
        return 0;
    }
    return buffer_.size() - start_;
}

size_t FrameStreamParser::extractFrames(std::vector<CrsfRxFrame>* outFrames)
{
    size_t frameCount = 0;

    while (true) {
        if (bufferedSize() < 2) {
            break;
        }

        const auto syncIt = std::find(buffer_.begin() + static_cast<std::ptrdiff_t>(start_), buffer_.end(), kSyncByte);
        if (syncIt == buffer_.end()) {
            buffer_.clear();
            start_ = 0;
            break;
        }

        const size_t syncIndex = static_cast<size_t>(std::distance(buffer_.begin(), syncIt));
        start_ = syncIndex;

        if (bufferedSize() < 2) {
            break;
        }

        const uint8_t length = buffer_[start_ + 1];
        if (length < kMinLength || length > kMaxLength) {
            start_ += 1;
            compactIfNeeded();
            continue;
        }

        const size_t frameSize = 2 + static_cast<size_t>(length);
        if (bufferedSize() < frameSize) {
            break;
        }

        const size_t typeIndex = start_ + 2;
        const uint8_t type = buffer_[typeIndex];

        const size_t crcIndex = start_ + frameSize - 1;
        const uint8_t receivedCrc = buffer_[crcIndex];
        const uint8_t computedCrc = CrsfProtocol::crc8(&buffer_[typeIndex], static_cast<size_t>(length - 1));

        if (receivedCrc != computedCrc) {
            start_ += 1;
            compactIfNeeded();
            continue;
        }

        CrsfRxFrame frame;
        frame.address = buffer_[start_];
        frame.length = length;
        frame.type = type;
        const size_t payloadLen = static_cast<size_t>(length - 2);
        frame.payload.assign(buffer_.begin() + static_cast<std::ptrdiff_t>(typeIndex + 1),
                             buffer_.begin() + static_cast<std::ptrdiff_t>(typeIndex + 1 + payloadLen));
        frame.crc = receivedCrc;

        outFrames->push_back(std::move(frame));
        frameCount += 1;

        start_ += frameSize;
        compactIfNeeded();
    }

    compactIfNeeded();
    return frameCount;
}

void FrameStreamParser::compactIfNeeded()
{
    if (start_ == 0) {
        return;
    }

    if (start_ >= buffer_.size()) {
        buffer_.clear();
        start_ = 0;
        return;
    }

    if (start_ > 1024 || start_ > (buffer_.size() / 2)) {
        buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(start_));
        start_ = 0;
    }
}

}  // namespace crsf_control
