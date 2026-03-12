#include <ros/ros.h>
#include <signal.h>

#include "crsf_control/crsf_node.h"

static crsf_control::CrsfNode* g_node = nullptr;

void signalHandler(int sig)
{
    (void)sig;
    // Only do async-signal-safe operations here.
    // ros::shutdown() is safe and will cause ros::spin() to exit,
    // then normal cleanup runs in main().
    ros::shutdown();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "crsf_control_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    crsf_control::CrsfNode node(nh, pnh);
    g_node = &node;

    if (!node.init()) {
        ROS_FATAL("Failed to initialize crsf_control node");
        return 1;
    }

    ros::spin();

    node.shutdown();
    g_node = nullptr;

    return 0;
}
