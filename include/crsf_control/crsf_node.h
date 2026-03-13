#ifndef CRSF_CONTROL_CRSF_NODE_H
#define CRSF_CONTROL_CRSF_NODE_H

#include <ros/ros.h>

#include <sensor_msgs/Joy.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>

#include <std_msgs/String.h>

#include <geometry_msgs/Quaternion.h>

#include <atomic>
#include <mutex>
#include <thread>
#include <cstdint>

#include "crsf_control/CrsfFrame.h"
#include "crsf_control/CrsfLinkStatistics.h"
#include "crsf_control/crsf_frame_stream_parser.h"
#include "crsf_control/crsf_protocol.h"
#include "crsf_control/crsf_telemetry_decoder.h"
#include "crsf_control/serial_port.h"

namespace crsf_control {

class CrsfNode {
public:
    CrsfNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    ~CrsfNode();

    // Initialize node: load params, open serial, start sender thread
    bool init();

    // Shutdown: stop sender thread, close serial
    void shutdown();

private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);
    void senderLoop();
    void receiverLoop();

    // ROS
    ros::NodeHandle& nh_;
    ros::NodeHandle& pnh_;
    ros::Subscriber joy_sub_;

    ros::Publisher telemetry_raw_pub_;
    ros::Publisher battery_pub_;
    ros::Publisher gps_pub_;
    ros::Publisher imu_pub_;
    ros::Publisher flight_mode_pub_;
    ros::Publisher link_stats_pub_;

    // Parameters
    std::string joy_topic_;
    std::string serial_device_;
    int serial_baud_;
    int crsf_rate_;          // Hz
    double min_frequency_;   // Hz
    double timeout_;         // seconds

    // Serial port
    std::unique_ptr<SerialPort> serial_;

    // Channel data (mutex-protected)
    std::mutex channels_mutex_;
    uint16_t channels_[CRSF_NUM_CHANNELS];

    // Timing (protected by channels_mutex_)
    ros::Time last_joy_time_;
    bool joy_received_;

    // Safety state
    std::atomic<bool> output_enabled_;
    std::atomic<bool> frequency_warned_;

    // Sender/receiver threads
    std::thread sender_thread_;
    std::thread receiver_thread_;
    std::atomic<bool> running_;

    // Statistics for frequency monitoring
    std::mutex stats_mutex_;
    ros::Time prev_joy_time_;
    double joy_frequency_;
    uint64_t joy_msg_count_;
};

}  // namespace crsf_control

#endif  // CRSF_CONTROL_CRSF_NODE_H
