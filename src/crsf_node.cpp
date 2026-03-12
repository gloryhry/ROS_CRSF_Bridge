#include "crsf_control/crsf_node.h"

#include <chrono>
#include <cstring>

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

    // Start sender thread
    running_ = true;
    sender_thread_ = std::thread(&CrsfNode::senderLoop, this);
    ROS_INFO("CRSF sender thread started at %d Hz", crsf_rate_);

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

}  // namespace crsf_control
