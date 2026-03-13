#include "crsf_control/crsf_node.h"

#include <chrono>
#include <cstring>
#include <cmath>

namespace crsf_control {

CrsfNode::CrsfNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh)
    , pnh_(pnh)
    , serial_baud_(420000)
    , crsf_rate_(50)
    , min_frequency_(10.0)
    , timeout_(0.5)
    , joy_received_(false)
    , output_enabled_(false)
    , frequency_warned_(false)
    , running_(false)
    , joy_frequency_(0.0)
    , joy_msg_count_(0)
{
    // Initialize all channels to center value
    for (size_t i = 0; i < CRSF_NUM_CHANNELS; ++i) {
        channels_[i] = CRSF_CHANNEL_CENTER;
    }
}

CrsfNode::~CrsfNode()
{
    shutdown();
}

bool CrsfNode::init()
{
    // Load parameters
    pnh_.param<std::string>("joy_topic", joy_topic_, "/joy");
    pnh_.param<std::string>("serial_port", serial_device_, "/dev/ttyUSB0");
    pnh_.param<int>("serial_baud", serial_baud_, 420000);
    pnh_.param<int>("crsf_rate", crsf_rate_, 50);
    pnh_.param<double>("min_frequency", min_frequency_, 10.0);
    pnh_.param<double>("timeout", timeout_, 0.5);

    ROS_INFO("=== crsf_control configuration ===");
    ROS_INFO("  joy_topic:    %s", joy_topic_.c_str());
    ROS_INFO("  serial_port:  %s", serial_device_.c_str());
    ROS_INFO("  serial_baud:  %d", serial_baud_);
    ROS_INFO("  crsf_rate:    %d Hz", crsf_rate_);
    ROS_INFO("  min_frequency: %.1f Hz", min_frequency_);
    ROS_INFO("  timeout:      %.2f s", timeout_);

    // Validate parameters
    if (crsf_rate_ <= 0 || crsf_rate_ > 500) {
        ROS_ERROR("Invalid crsf_rate: %d (must be 1-500)", crsf_rate_);
        return false;
    }
    if (min_frequency_ <= 0.0) {
        ROS_ERROR("Invalid min_frequency: %.1f (must be > 0)", min_frequency_);
        return false;
    }
    if (timeout_ <= 0.0) {
        ROS_ERROR("Invalid timeout: %.2f (must be > 0)", timeout_);
        return false;
    }

    // Open serial port
    serial_ = std::make_unique<SerialPort>(serial_device_, serial_baud_);
    if (!serial_->open()) {
        ROS_ERROR("Failed to open serial port: %s at %d baud",
                  serial_device_.c_str(), serial_baud_);
        return false;
    }
    ROS_INFO("Serial port opened: %s at %d baud", serial_device_.c_str(), serial_baud_);

    // Subscribe to Joy topic
    joy_sub_ = nh_.subscribe(joy_topic_, 1, &CrsfNode::joyCallback, this);
    ROS_INFO("Subscribed to Joy topic: %s", joy_topic_.c_str());

    // Telemetry publishers (private namespace: ~telemetry/...)
    telemetry_raw_pub_ = pnh_.advertise<crsf_control::CrsfFrame>("telemetry/raw", 10);
    battery_pub_ = pnh_.advertise<sensor_msgs::BatteryState>("telemetry/battery", 10);
    gps_pub_ = pnh_.advertise<sensor_msgs::NavSatFix>("telemetry/gps", 10);
    imu_pub_ = pnh_.advertise<sensor_msgs::Imu>("telemetry/imu", 10);
    flight_mode_pub_ = pnh_.advertise<std_msgs::String>("telemetry/flight_mode", 10);
    link_stats_pub_ = pnh_.advertise<crsf_control::CrsfLinkStatistics>("telemetry/link_statistics", 10);

    // Start sender/receiver threads
    running_ = true;
    sender_thread_ = std::thread(&CrsfNode::senderLoop, this);
    receiver_thread_ = std::thread(&CrsfNode::receiverLoop, this);
    ROS_INFO("CRSF sender thread started at %d Hz", crsf_rate_);
    ROS_INFO("CRSF receiver thread started");

    ROS_INFO("crsf_control node initialized successfully");
    return true;
}

void CrsfNode::shutdown()
{
    // Guard against multiple shutdown calls
    bool expected = true;
    if (!running_.compare_exchange_strong(expected, false)) {
        return;  // Already shut down or never started
    }

    ROS_INFO("Shutting down crsf_control node...");

    if (receiver_thread_.joinable()) {
        receiver_thread_.join();
    }

    if (sender_thread_.joinable()) {
        sender_thread_.join();
    }

    if (serial_ && serial_->isOpen()) {
        serial_->close();
        ROS_INFO("Serial port closed");
    }

    ROS_INFO("crsf_control node shutdown complete");
}

void CrsfNode::joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
    ros::Time now = ros::Time::now();

    // Convert axes to CRSF channel values and update timing
    {
        std::lock_guard<std::mutex> lock(channels_mutex_);
        size_t num_axes = std::min(msg->axes.size(), static_cast<size_t>(CRSF_NUM_CHANNELS));

        for (size_t i = 0; i < num_axes; ++i) {
            channels_[i] = CrsfProtocol::floatToCrsf(msg->axes[i]);
        }
        // Remaining channels stay at their previous values (center on first init)

        last_joy_time_ = now;
        joy_received_ = true;
    }

    // Update frequency statistics
    {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        if (prev_joy_time_ > ros::Time(0)) {
            double dt = (now - prev_joy_time_).toSec();
            if (dt > 0.0) {
                // Exponential moving average for frequency estimation
                double instant_freq = 1.0 / dt;
                if (joy_msg_count_ <= 1) {
                    joy_frequency_ = instant_freq;
                } else {
                    joy_frequency_ = 0.8 * joy_frequency_ + 0.2 * instant_freq;
                }
            }
        }
        prev_joy_time_ = now;
        joy_msg_count_++;
    }

    // Re-enable output if it was disabled due to timeout
    if (!output_enabled_.load()) {
        output_enabled_ = true;
        ROS_INFO("Joy messages resumed. CRSF output re-enabled.");
    }

    // Clear frequency warning if frequency is back to normal
    if (frequency_warned_.load()) {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        if (joy_frequency_ >= min_frequency_) {
            frequency_warned_ = false;
            ROS_INFO("Joy message frequency recovered to %.1f Hz", joy_frequency_);
        }
    }
}

static geometry_msgs::Quaternion quatFromRollPitchYaw(double roll_rad, double pitch_rad, double yaw_rad)
{
    const double cy = std::cos(yaw_rad * 0.5);
    const double sy = std::sin(yaw_rad * 0.5);
    const double cp = std::cos(pitch_rad * 0.5);
    const double sp = std::sin(pitch_rad * 0.5);
    const double cr = std::cos(roll_rad * 0.5);
    const double sr = std::sin(roll_rad * 0.5);

    geometry_msgs::Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
    return q;
}

void CrsfNode::senderLoop()
{
    const auto period = std::chrono::microseconds(1000000 / crsf_rate_);
    uint8_t frame_buf[CRSF_RC_FRAME_SIZE];
    uint16_t local_channels[CRSF_NUM_CHANNELS];
    uint64_t frame_count = 0;
    uint64_t error_count = 0;

    ROS_INFO("Sender thread running. Period: %ld us", period.count());

    while (running_.load() && ros::ok()) {
        auto loop_start = std::chrono::steady_clock::now();

        // Check timeout and copy channels under lock
        bool local_joy_received;
        double time_since_last_joy = 0.0;
        {
            std::lock_guard<std::mutex> lock(channels_mutex_);
            local_joy_received = joy_received_;
            if (local_joy_received) {
                time_since_last_joy = (ros::Time::now() - last_joy_time_).toSec();
            }
        }

        if (local_joy_received) {
            if (time_since_last_joy > timeout_) {
                if (output_enabled_.load()) {
                    output_enabled_ = false;
                    ROS_ERROR("Joy message timeout! No message for %.2f s (threshold: %.2f s). "
                              "CRSF output DISABLED.", time_since_last_joy, timeout_);
                }
            }

            // Check frequency
            {
                std::lock_guard<std::mutex> lock(stats_mutex_);
                if (joy_frequency_ > 0.0 && joy_frequency_ < min_frequency_
                    && !frequency_warned_.load()) {
                    frequency_warned_ = true;
                    ROS_WARN("Joy message frequency low: %.1f Hz (threshold: %.1f Hz)",
                             joy_frequency_, min_frequency_);
                }
            }
        }

        // Only send if output is enabled
        if (output_enabled_.load()) {
            // Copy channels under lock
            {
                std::lock_guard<std::mutex> lock(channels_mutex_);
                std::memcpy(local_channels, channels_, sizeof(channels_));
            }

            // Pack CRSF frame
            CrsfProtocol::packChannelsFrame(local_channels, frame_buf);

            // Send via serial
            if (serial_ && serial_->isOpen()) {
                if (!serial_->write(frame_buf, CRSF_RC_FRAME_SIZE)) {
                    error_count++;
                    if (error_count % 100 == 1) {
                        ROS_ERROR("Serial write failed (error count: %lu)", error_count);
                    }
                } else {
                    frame_count++;
                    if (frame_count % (crsf_rate_ * 10) == 0) {
                        ROS_DEBUG("CRSF frames sent: %lu, errors: %lu", frame_count, error_count);
                    }
                }
            } else {
                if (frame_count == 0 || error_count % 100 == 0) {
                    ROS_ERROR("Serial port not open!");
                }
                error_count++;
            }
        }

        // Sleep for remaining time in this period
        auto loop_end = std::chrono::steady_clock::now();
        auto elapsed = loop_end - loop_start;
        if (elapsed < period) {
            std::this_thread::sleep_for(period - elapsed);
        }
    }

    ROS_INFO("Sender thread exiting. Total frames: %lu, errors: %lu", frame_count, error_count);
}

void CrsfNode::receiverLoop()
{
    FrameStreamParser parser;
    uint8_t rx_buf[256];
    std::vector<CrsfRxFrame> frames;

    uint64_t bytes_total = 0;
    uint64_t frames_total = 0;

    ROS_INFO("Receiver thread running.");

    while (running_.load() && ros::ok()) {
        if (!(serial_ && serial_->isOpen())) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }

        // Wait readable with timeout to allow clean shutdown.
        if (!serial_->waitReadable(100)) {
            continue;
        }

        ssize_t n = serial_->read(rx_buf, sizeof(rx_buf));
        if (n <= 0) {
            continue;
        }

        bytes_total += static_cast<uint64_t>(n);

        frames.clear();
        parser.pushBytes(rx_buf, static_cast<size_t>(n), &frames);

        for (const auto& f : frames) {
            frames_total += 1;

            // Raw publish
            if (telemetry_raw_pub_) {
                crsf_control::CrsfFrame msg;
                msg.header.stamp = ros::Time::now();
                msg.device_address = f.address;
                msg.frame_length = f.length;
                msg.type = f.type;
                msg.payload = f.payload;
                msg.crc = f.crc;
                telemetry_raw_pub_.publish(msg);
            }

            // Decoded publish
            if (f.type == TelemetryDecoder::CRSF_FRAMETYPE_GPS) {
                TelemetryDecoder::GpsData gps;
                if (TelemetryDecoder::decodeGps(f.payload.data(), f.payload.size(), &gps)) {
                    sensor_msgs::NavSatFix nav;
                    nav.header.stamp = ros::Time::now();
                    nav.latitude = gps.latitude_deg;
                    nav.longitude = gps.longitude_deg;
                    nav.altitude = gps.altitude_m;
                    gps_pub_.publish(nav);
                }
            } else if (f.type == TelemetryDecoder::CRSF_FRAMETYPE_BATTERY_SENSOR) {
                TelemetryDecoder::BatteryData bat;
                if (TelemetryDecoder::decodeBatterySensor(f.payload.data(), f.payload.size(), &bat)) {
                    sensor_msgs::BatteryState bs;
                    bs.header.stamp = ros::Time::now();
                    bs.voltage = static_cast<float>(bat.voltage_v);
                    bs.current = static_cast<float>(bat.current_a);
                    bs.percentage = static_cast<float>(bat.remaining_percent) / 100.0f;
                    battery_pub_.publish(bs);
                }
            } else if (f.type == TelemetryDecoder::CRSF_FRAMETYPE_LINK_STATISTICS) {
                TelemetryDecoder::LinkStatisticsData ls;
                if (TelemetryDecoder::decodeLinkStatistics(f.payload.data(), f.payload.size(), &ls)) {
                    crsf_control::CrsfLinkStatistics out;
                    out.header.stamp = ros::Time::now();
                    out.uplink_rssi_1 = ls.uplink_rssi_1;
                    out.uplink_rssi_2 = ls.uplink_rssi_2;
                    out.uplink_link_quality = ls.uplink_link_quality;
                    out.uplink_snr = ls.uplink_snr;
                    out.active_antenna = ls.active_antenna;
                    out.rf_mode = ls.rf_mode;
                    out.uplink_tx_power = ls.uplink_tx_power;
                    out.downlink_rssi = ls.downlink_rssi;
                    out.downlink_link_quality = ls.downlink_link_quality;
                    out.downlink_snr = ls.downlink_snr;
                    link_stats_pub_.publish(out);
                }
            } else if (f.type == TelemetryDecoder::CRSF_FRAMETYPE_ATTITUDE) {
                TelemetryDecoder::AttitudeData att;
                if (TelemetryDecoder::decodeAttitude(f.payload.data(), f.payload.size(), &att)) {
                    sensor_msgs::Imu imu;
                    imu.header.stamp = ros::Time::now();
                    imu.orientation = quatFromRollPitchYaw(att.roll_rad, att.pitch_rad, att.yaw_rad);

                    // Unknown covariances
                    imu.orientation_covariance[0] = -1.0;
                    imu.angular_velocity_covariance[0] = -1.0;
                    imu.linear_acceleration_covariance[0] = -1.0;

                    imu_pub_.publish(imu);
                }
            } else if (f.type == TelemetryDecoder::CRSF_FRAMETYPE_FLIGHT_MODE) {
                TelemetryDecoder::FlightModeData fm;
                if (TelemetryDecoder::decodeFlightMode(f.payload.data(), f.payload.size(), &fm)) {
                    std_msgs::String s;
                    s.data = fm.mode;
                    flight_mode_pub_.publish(s);
                }
            }
        }

        if (frames_total % 500 == 0 && frames_total > 0) {
            ROS_DEBUG("Telemetry RX: bytes=%lu frames=%lu buffered=%lu", bytes_total, frames_total,
                      static_cast<unsigned long>(parser.bufferedSize()));
        }
    }

    ROS_INFO("Receiver thread exiting. bytes=%lu frames=%lu", bytes_total, frames_total);
}

}  // namespace crsf_control
