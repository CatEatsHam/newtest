# pcl_pipeline

PCL Voxel Filter takes in one ROS point cloud 2 message and downsamples the points into a grid of rectangular voxels. All the points inside of each voxel are reduced to a single point within, creating a downsampled uniform point cloud. In addition a minimum number of points threshhold can be used to filter points out based on density. A range filter has also been implemented to filter out point returns from vehicle framing, sensers, etc. The voxel size, minimum points and range parameters are dynamically reconfigurable using rqt_reconfigure.


Workarounds:

In the creation of the voxel filter two main issues arised with the minimum points and range filters requiring workarounds.


Minimum Points:
In order to maintain the data within the ros point cloud 2 fields such as reflectivity, intensity, range, etc, the entire PCL pipeline needs to use either ROS or PCL point cloud 2 messages. Unfortunately, until the PCL 1.12.1 release, any PCL voxel grid of point cloud 2 objects cannot be filtered by minimum points. This is described in the following github issue.
https://github.com/PointCloudLibrary/pcl/issues/4388

In order to get around this PCL 1.12.1 needs to be used which is also incompatible with the pcl_ros package's PCL version 1.8. This means that in order to use this package, PCL 1.12.1 is required and several pcl_ros functions for conversions had to be copied to maintain their functionality without ROS-PCL conflicts.

To build 1.12.1 from source:
1. Download the PCL 1.12.1 Source code (zip) from the releases tab
https://github.com/PointCloudLibrary/pcl/releases

2. Follow their provided linux build guide: 
https://pcl.readthedocs.io/projects/tutorials/en/latest/compiling_pcl_posix.html

The CMakeLists.txt file for the package is setup to use the newest version and should also warn the user should they not have PCL 1.12.1 installed correctly.


Range Filtering:
Upon attempting to implement a filter based on the point cloud 2 range field, it was discovered that the Ouster OS1 driver writes the range field data using unsigned 32 bit integers of mm. The PCL voxel grid field filter presently only supports filtering for float32 and float64 datatypes and displayed an error on runtime.

The currently implimented workaround manually sets the field data type in the input message to float32 without any sort of casting or value management. This value, as interpreted by ROS is then correct but skewed by a factor of 10 to the -29th power. This is then divided away in the input to the function setting the filter limits.

In the future other potential solutions could be to modify the ouster driver, impliment PCL unsigned integer filtering ourselves or convert all of the range data to floats inside of the filter. 