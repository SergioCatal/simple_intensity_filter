#include <ros/ros.h>

#include "lidar_filter_node.h"

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "lidar_filter");
    LidarFilterNode lidar_filter_node;

    ROS_INFO("Setup complete, waiting for messages!");
    ros::spin();

    return 0;
}
