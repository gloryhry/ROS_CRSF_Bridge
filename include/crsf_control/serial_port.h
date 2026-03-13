#ifndef CRSF_CONTROL_SERIAL_PORT_H
#define CRSF_CONTROL_SERIAL_PORT_H

#include <string>
#include <cstdint>
#include <cstddef>
#include <sys/types.h>

namespace crsf_control {

class SerialPort {
public:
    SerialPort(const std::string& device, int baud_rate);
    ~SerialPort();

    // Non-copyable
    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    bool open();
    void close();
    bool isOpen() const;

    // Write data to serial port. Returns true if all bytes written.
    bool write(const uint8_t* data, size_t len);

    // Read data from serial port.
    // Returns number of bytes read (>0), 0 on timeout/no data, or -1 on error.
    ssize_t read(uint8_t* buf, size_t max_len);

    // Wait until the serial port becomes readable (poll/select).
    // timeout_ms < 0 means wait forever.
    // Returns true if readable, false on timeout or error.
    bool waitReadable(int timeout_ms);

    const std::string& device() const { return device_; }
    int baudRate() const { return baud_rate_; }

private:
    bool configurePort();

    std::string device_;
    int baud_rate_;
    int fd_;
};

}  // namespace crsf_control

#endif  // CRSF_CONTROL_SERIAL_PORT_H
