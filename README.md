# simple_intensity_filter

Simple intensity filter ROS node.

### Assumptions:
- The throttle is implemented deterministically (every X input point clouds, one is dropped)
- If input point cloud does not contain intensity, nothing is published but warning is shown
