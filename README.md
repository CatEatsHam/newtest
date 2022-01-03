# pcl_pipeline

The AGC PCL pipeline contains the packages located between the lidar drivers and the output pointclouds used for costmap creation.

Lidar Calibration creates an optimal transformation for Lidar synchronization through point locations relative to the environment.

PCL Fusion takes in two ROS point cloud 2 messages and outputs a single point cloud 2 messages based on the transformation given in the header of the first message.

PCL Voxel Filter takes in one ROS point cloud 2 message and downsamples the points into a grid of rectangular voxels. All the points inside of a voxel are reduced to a single point inside. In addition a minimum number of points threshhold can be used to filter points out based on density. The voxel size and minimum points parameters are dynamically reconfigurable using rqt_reconfigure.
