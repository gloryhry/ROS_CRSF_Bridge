#include "crsf_control/serial_port.h"

#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/ioctl.h>
#include <poll.h>

// Use asm/termbits.h for termios2 + BOTHER (custom baud rate)
// Do NOT include <termios.h> — it conflicts with asm/termbits.h
#include <asm/termbits.h>

namespace crsf_control {

SerialPort::SerialPort(const std::string& device, int baud_rate)
    : device_(device)
    , baud_rate_(baud_rate)
    , fd_(-1)
{
}

SerialPort::~SerialPort()
{
    close();
}

bool SerialPort::open()
{
    if (fd_ >= 0) {
        close();
    }

    fd_ = ::open(device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        return false;
    }

    // Clear non-blocking after open
    int flags = fcntl(fd_, F_GETFL, 0);
    fcntl(fd_, F_SETFL, flags & ~O_NONBLOCK);

    if (!configurePort()) {
        close();
        return false;
    }

    return true;
}

void SerialPort::close()
{
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool SerialPort::isOpen() const
{
    return fd_ >= 0;
}

bool SerialPort::write(const uint8_t* data, size_t len)
{
    if (fd_ < 0 || data == nullptr || len == 0) {
        return false;
    }

    size_t total_written = 0;
    while (total_written < len) {
        ssize_t written = ::write(fd_, data + total_written, len - total_written);
        if (written < 0) {
            if (errno == EINTR) {
                continue;
            }
            return false;
        }
        total_written += static_cast<size_t>(written);
    }
    return true;
}

ssize_t SerialPort::read(uint8_t* buf, size_t max_len)
{
    if (fd_ < 0 || buf == nullptr || max_len == 0) {
        return -1;
    }

    ssize_t n = ::read(fd_, buf, max_len);
    if (n < 0) {
        if (errno == EINTR) {
            return 0;
        }
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return 0;
        }
        return -1;
    }

    return n;
}

bool SerialPort::waitReadable(int timeout_ms)
{
    if (fd_ < 0) {
        return false;
    }

    struct pollfd pfd;
    pfd.fd = fd_;
    pfd.events = POLLIN;
    pfd.revents = 0;

    int ret;
    do {
        ret = ::poll(&pfd, 1, timeout_ms);
    } while (ret < 0 && errno == EINTR);

    if (ret <= 0) {
        return false;
    }

    if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
        return false;
    }

    return (pfd.revents & POLLIN) != 0;
}

bool SerialPort::configurePort()
{
    // Use termios2 for custom baud rate (e.g., 420000)
    struct termios2 tio;
    memset(&tio, 0, sizeof(tio));

    if (ioctl(fd_, TCGETS2, &tio) < 0) {
        return false;
    }

    // Configure 8N1, no flow control
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;          // 8 data bits
    tio.c_cflag &= ~PARENB;      // No parity
    tio.c_cflag &= ~CSTOPB;      // 1 stop bit
    tio.c_cflag &= ~CRTSCTS;     // No hardware flow control
    tio.c_cflag |= CLOCAL;       // Ignore modem control lines
    tio.c_cflag |= CREAD;        // Enable receiver

    // Raw input mode
    tio.c_iflag &= ~(IXON | IXOFF | IXANY);  // No software flow control
    tio.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

    // Raw output mode
    tio.c_oflag &= ~OPOST;

    // Raw mode (no echo, no canonical)
    tio.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

    // Set custom baud rate using BOTHER
    tio.c_cflag &= ~CBAUD;
    tio.c_cflag |= BOTHER;
    tio.c_ispeed = baud_rate_;
    tio.c_ospeed = baud_rate_;

    // Minimum characters and timeout
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;

    if (ioctl(fd_, TCSETS2, &tio) < 0) {
        return false;
    }

    // Flush buffers
    ioctl(fd_, TCFLSH, TCIOFLUSH);

    return true;
}

}  // namespace crsf_control
