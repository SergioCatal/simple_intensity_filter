#include <gtest/gtest.h>
#include <intensity_filter.h>

using PointT = pcl::PointXYZI;
using PointCloudT = pcl::PointCloud<PointT>;

TEST(IntensityFilterTest, EmptyInput) {
    // Setup
    PointCloudT::Ptr cloud(new PointCloudT);
    cloud->width = 0;
    cloud->height = 1;
    IntensityFilter lf(100, 100);

    // Run test
    PointCloudT::Ptr filtered = lf.filterPcl(cloud);

    // Check results
    ASSERT_TRUE(filtered);
    ASSERT_TRUE(filtered->empty());
}

TEST(IntensityFilterTest, AllValuesPass) {
    // Setup
    float threshold = 100.0f;
    PointCloudT::Ptr cloud(new PointCloudT);
    cloud->width = 10;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);

    for (std::size_t i = 0; i < cloud->size(); i++) {
        (*cloud)[i].x = i;
        (*cloud)[i].y = i;
        (*cloud)[i].z = i;
        (*cloud)[i].intensity = (static_cast<float>(rand()) / RAND_MAX) * 1000 + threshold;
    }

    IntensityFilter lf(threshold, 100);

    // Run test
    PointCloudT::Ptr filtered = lf.filterPcl(cloud);

    // Check results
    ASSERT_TRUE(filtered);
    ASSERT_EQ(filtered->size(), cloud->size());
}

TEST(IntensityFilterTest, NoValuesPass) {
    // Setup
    float threshold = 100.0f;
    PointCloudT::Ptr cloud(new PointCloudT);
    cloud->width = 10;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);

    for (std::size_t i = 0; i < cloud->size(); i++) {
        (*cloud)[i].x = i;
        (*cloud)[i].y = i;
        (*cloud)[i].z = i;
        (*cloud)[i].intensity = (rand() / (static_cast<float>(RAND_MAX) + 1)) * threshold;
    }

    IntensityFilter lf(threshold, 100);

    // Run test
    PointCloudT::Ptr filtered = lf.filterPcl(cloud);

    // Check results
    ASSERT_TRUE(filtered);
    ASSERT_TRUE(filtered->empty());
}

TEST(IntensityFilterTest, SomeValuesPass) {
    // Setup
    float threshold = 100.0f;
    PointCloudT::Ptr cloud(new PointCloudT);
    cloud->width = 10;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);

    for (std::size_t i = 0; i < cloud->size(); i++) {
        (*cloud)[i].x = i;
        (*cloud)[i].y = i;
        (*cloud)[i].z = i;
        if (i % 3 == 0) {
            (*cloud)[i].intensity = (rand() / (static_cast<float>(RAND_MAX) + 1)) * threshold;
        } else {
            (*cloud)[i].intensity = (static_cast<float>(rand()) / RAND_MAX) * 1000 + threshold;
        }
    }

    IntensityFilter lf(threshold, 100);

    // Run test
    PointCloudT::Ptr filtered = lf.filterPcl(cloud);

    // Check results
    ASSERT_TRUE(filtered);
    ASSERT_EQ(filtered->size(), 6);
}


TEST(IntensityFilterTest, ThrottleTest) {
    // Setup
    float threshold = 100.0f;
    int throttle_period = 10;
    PointCloudT::Ptr cloud(new PointCloudT);
    cloud->width = 10;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);

    for (std::size_t i = 0; i < cloud->size(); i++) {
        (*cloud)[i].x = i;
        (*cloud)[i].y = i;
        (*cloud)[i].z = i;
        if (i % 3 == 0) {
            (*cloud)[i].intensity = (rand() / (static_cast<float>(RAND_MAX) + 1)) * threshold;
        } else {
            (*cloud)[i].intensity = (static_cast<float>(rand()) / RAND_MAX) * 1000 + threshold;
        }
    }

    IntensityFilter lf(threshold, throttle_period);
    int filtered_clouds = 0;

    // Run test
    for (int i = 0; i < 45; i++) {
        if (lf.filterPcl(cloud)) {
            filtered_clouds++;
        }
    }

    // Check results
    ASSERT_EQ(filtered_clouds, 41);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}