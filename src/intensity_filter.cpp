#include "intensity_filter.h"

using PointT = pcl::PointXYZI;
using PointCloudT = pcl::PointCloud<PointT>;

IntensityFilter::IntensityFilter(float intensity_threshold, int throttle_period) :
        _throttle_period(throttle_period),
        _n_filtered_msgs(0) {
    // Create the pcl::Condition that will be used to filter the point clouds
    pcl::ConditionAnd<PointT>::Ptr intensity_condition(new pcl::ConditionAnd <PointT>);
    intensity_condition->addComparison(pcl::FieldComparison<PointT>::ConstPtr(
            new pcl::FieldComparison<PointT>("intensity", pcl::ComparisonOps::GE, intensity_threshold)));
    _pcl_filter.setCondition(intensity_condition);
}

pcl::PointCloud<pcl::PointXYZI>::Ptr IntensityFilter::filterPcl(pcl::PointCloud<pcl::PointXYZI>::Ptr pcl) {
    _n_filtered_msgs++;
    // Throttle if needed
    if (_n_filtered_msgs >= _throttle_period) {
        _n_filtered_msgs = 0;
        return PointCloudT::Ptr();
    }

    PointCloudT::Ptr filtered_pcl(new PointCloudT);
    _pcl_filter.setInputCloud(pcl);
    _pcl_filter.filter(*filtered_pcl);

    return filtered_pcl;
}
