#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "intensity_filter.h"

/**
 * Class for setting up a ROS Node for a simple point cloud filter
 */
class LidarFilterNode {
public:
    /**
     * Create a LidarFilterNode object.
     *
     * This constructor loads parameters, and sets up publishers, subscribers and the LidarFilter
     */
    LidarFilterNode();

private:
    /**
     * Callback for new point cloud messages.
     *
     * Runs the configured _lidar_filter on the received point cloud and publishes the results.
     * Nothing is published if:
     * - the received point cloud does not have an "intensity" field, nothing is published.
     * - the _lidar_filter is throttling
     *
     * @param msg Point cloud message to filter
     */
    void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);

    std::shared_ptr<IntensityFilter> _intensity_filter;
    ros::Subscriber _sub;
    ros::Publisher _pub;
};
