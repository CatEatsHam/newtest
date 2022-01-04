# pcl_fusion

PCL Fusion takes in two ROS point cloud 2 messages and outputs a single point cloud 2 messages based on the transformation given in the header of the first message.

To accomplish this, the pcl_fusion node uses an approxomate time synchronizer to create one synchronized callback for both lidar messages based on time.

Using a TF transform listener and the pcl_ros transformPointCloud function, the second input lidar message is transformed into the frame of the first. The transform is pulled from ROS which is presently setup to use a transform from the LidarCalibration package.

The two point cloud messages, now in the same frame are then combined together using the PCL concatenatePointCloud function.

Finally the combined message is output into ros under the configured topic.