#include "lidar_filter_node.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

using PointT = pcl::PointXYZI;
using PointCloudT = pcl::PointCloud<PointT>;

LidarFilterNode::LidarFilterNode() {
    ros::NodeHandle n("~");

    // Load parameters
    std::string input_pcl_topic, filtered_pcl_topic;
    float intensity_threshold;
    int throttle_period;
    n.getParam("input_pcl_topic", input_pcl_topic);
    n.getParam("filtered_pcl_topic", filtered_pcl_topic);
    n.getParam("intensity_threshold", intensity_threshold);
    n.getParam("throttle_period", throttle_period);
    ROS_INFO(
            "Initializing node with parameters:\n\t- input_pcl_topic: %s\n\t- filtered_pcl_topic: %s"
            "\n\t- intensity_threshold: %f\n\t- throttle_period: %d",
            input_pcl_topic.c_str(), filtered_pcl_topic.c_str(), intensity_threshold, throttle_period);

    // Setup LidarFilter, publisher and subscriber
    _intensity_filter = std::make_shared<IntensityFilter>(intensity_threshold, throttle_period);
    _pub = n.advertise<sensor_msgs::PointCloud2>(filtered_pcl_topic, 1000);
    _sub = n.subscribe(input_pcl_topic, 1000, &LidarFilterNode::lidarCallback, this);
}

void LidarFilterNode::lidarCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    // Make sure the intensity field is available in the pcl
    bool contains_intensity = false;
    for (const sensor_msgs::PointField &field: msg->fields) {
        if (field.name == "intensity") {
            contains_intensity = true;
        }
    }
    if (not contains_intensity) {
        ROS_WARN("Received a pcl without intensity!");
        return;
    }

    // Convert sensor_msgs::PointCloud2 to PointCloudT
    PointCloudT::Ptr point_cloud(new PointCloudT);
    pcl::fromROSMsg(*msg, *point_cloud);

    // Filter point cloud and publish or skip (throttle)
    PointCloudT::Ptr filtered_pcl = _intensity_filter->filterPcl(point_cloud);
    if (filtered_pcl) {
        sensor_msgs::PointCloud2::Ptr filtered_msg(new sensor_msgs::PointCloud2);
        pcl::toROSMsg(*filtered_pcl, *filtered_msg);
        filtered_msg->header = msg->header;
        _pub.publish(filtered_msg);
    } else {
        ROS_DEBUG("Throttle: dropping a message!");
    }
}