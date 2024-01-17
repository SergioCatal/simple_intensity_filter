#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/conditional_removal.h>

/**
 * Class to create filter objects that can filter point clouds based on point intensity.
 */
class IntensityFilter {
public:
    /**
     * Create an IntensityFilter object.
     *
     * The filter object stores the given parameters and will be ready to filter out the points
     * with intensity lower than intensity_threshold. Also, every throttle_period point clouds,
     * a point cloud is dropped.
     *
     * @param intensity_threshold Threshold intensity value for filtered point clouds.
     * @param throttle_period Every throttle_period point clouds, one is dropped.
     */
    IntensityFilter(float intensity_threshold, int throttle_period);

    /**
     * Runs intensity filter.
     *
     * The filter returns a new filtered point cloud obtained from the input point cloud based
     * on the parameters given in the constructor
     *
     * @param pcl Input pcl to be filtered
     * @return New filtered pcl, or empty shared_ptr if throttling
     */
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterPcl(pcl::PointCloud<pcl::PointXYZI>::Ptr pcl);

private:
    pcl::ConditionalRemoval<pcl::PointXYZI> _pcl_filter;
    int _throttle_period, _n_filtered_msgs;
};
