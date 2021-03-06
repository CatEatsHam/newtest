#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <chrono>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/pcl_config.h>
#include <pcl/types.h>

#include <dynamic_reconfigure/server.h>
#include <pcl_voxel_filter/VoxelConfig.h>

using namespace sensor_msgs;

class voxel_filter_node {

    private:
    // ROS node, pub and sub variables
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher pub;

    // Dynamic reconfigure setup
    dynamic_reconfigure::Server<pcl_voxel_filter::VoxelConfig> server;
    dynamic_reconfigure::Server<pcl_voxel_filter::VoxelConfig>::CallbackType f;

    // ROS parameters / RQT reconfigure variables
    std::string inputTopic;
    std::string outputTopic;
    std::vector<double> leafSize;
    int minPoints;
    double minPointsRange;
    double minRange;

    // PCL variable setup
    pcl::PCLPointCloud2 cloud_filtered;
    pcl::PCLPointCloud2 short_filtered;
    pcl::PCLPointCloud2 long_filtered;
    pcl::VoxelGrid<pcl::PCLPointCloud2> pcl_voxel;

    // Final ROS output message
    sensor_msgs::PointCloud2 output;

    // Loop Timing Variables
    std::chrono::_V2::system_clock::time_point start;
    std::chrono::_V2::system_clock::time_point stop;
    std::chrono::microseconds duration;

    std::double_t averageTime = 0;
    std::double_t sumTime = 0;
    std::int16_t loop_count = 0;


    public:
    voxel_filter_node() {
      std::cout << std::endl;
      ROS_INFO("Voxel Filter Node Initialize");

      // Print PCL Version to ensure 1.12.1 is used (required for voxel min point filtering)
      // Problem is described in PCL github issue: https://github.com/PointCloudLibrary/pcl/issues/4388
      std::cout <<"PCL VERSION: "<< PCL_VERSION << std::endl;

      // Get parameters from ROS parameter server
      ros::param::get("~inputTopic", inputTopic);
      ros::param::get("~outputTopic", outputTopic);
      ros::param::get("~leafSize", leafSize);
      ros::param::get("~minPoints", minPoints);
      ros::param::get("~minPointsRange", minPointsRange);
      ros::param::get("~minRange", minRange);

      // Print used parameters
      ROS_INFO("The input topic is %s" , inputTopic.c_str());
      ROS_INFO("The output topic is %s" , outputTopic.c_str());
      ROS_INFO("Leaf size set to: %0.3f, %0.3f, %0.3f" , leafSize[0], leafSize[1], leafSize[2]);
      ROS_INFO("The minimum points in a voxel is %i" , minPoints);
      ROS_INFO("The minimum points range is: %0.3f", minPointsRange);
      ROS_INFO("The minimum range is: %0.3f", minRange);

      // Subscribe to lidar input topic
      sub = nh.subscribe(inputTopic, 1, &voxel_filter_node::lidar_callback, this);

      // Setup callback for dynamic reconfigure
      f = boost::bind(&voxel_filter_node::rqt_callback, this, _1, _2);
      server.setCallback(f);

      // Create a ROS publisher for the output point cloud
      pub = nh.advertise<PointCloud2> (outputTopic, 1);

    }

    void lidar_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
      {
        // Start time for callback timing purposes
        start = std::chrono::high_resolution_clock::now();

        // Debug size point cloud size print message
        // std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
        // << " data points (" << pcl::getFieldsList (*cloud) << ")." << std::endl;

        // Create pointer for storage and convert to PCL data type
        pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
        toPCL(*cloud_msg, *cloud);

        // Workaround for ouster lidar:
        // PCL requires either float32 or float64 values for range filtering, ouster uses uint32 values in mm
        // Current workaround changes pointcloud message range datatype from uint32 (6) to float32 (7)
        // This then requires compensation (division by 10^-29) which is done in the call to setFilterLimits
        cloud->fields[8].datatype = 7;

        // Load the modified PCL pointcloud2 into a constPtr for usage in the PCL voxel class
        pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);

        // Load point cloud into voxel class
        pcl_voxel.setInputCloud(cloudPtr);

        // Previous function using one stage filtering
        // filterAll();

        // Seperate and filter the further away section of the point cloud with no min_points filter
        filterLong();

        //Seperate and filter the closer section of the point cloud 
        filterShort();

        // Combine the two seperate point clouds back together, currently adding only a few milliseconds of delay to the process
        combine();

        publish();
      }


    void filterLong()
    {
      // Sets voxel grid to downsample all fields rather than just XYZ
      pcl_voxel.setDownsampleAllData(true);

      // Sets up for range based filtering
      pcl_voxel.setFilterFieldName("range");

      // Filters range between minRange and 120 (max distance of lidar)
      // For ouster workaround minRange is multiplied by 10^-41 for uint32 (mm) to float32 (m) compensation
      pcl_voxel.setFilterLimits(minPointsRange * pow(10, -41), 120);

      // Leaf size (length, width and height of voxels) is set
      pcl_voxel.setLeafSize (leafSize[0], leafSize[1], leafSize[2]);

      // Minimum points filter is loaded
      pcl_voxel.setMinimumPointsNumberPerVoxel(1);

      // Voxel is filtered
      pcl_voxel.filter(long_filtered);
    }

    void filterShort()
    {
      // Sets voxel grid to downsample all fields rather than just XYZ
      pcl_voxel.setDownsampleAllData(true);

      // Sets up for range based filtering
      pcl_voxel.setFilterFieldName("range");

      // Filters range between minRange and 120 (max distance of lidar)
      // For ouster workaround minRange is multiplied by 10^-41 for uint32 (mm) to float32 (m) compensation
      pcl_voxel.setFilterLimits(minRange * pow(10, -41), minPointsRange * pow(10, -41));

      // Leaf size (length, width and height of voxels) is set
      pcl_voxel.setLeafSize (leafSize[0], leafSize[1], leafSize[2]);

      // Minimum points filter is loaded
      pcl_voxel.setMinimumPointsNumberPerVoxel(minPoints);

      // Voxel is filtered
      pcl_voxel.filter(short_filtered);
    }

    void combine()
    {
      // Combining clouds
        pcl::concatenate(short_filtered, long_filtered, cloud_filtered);
    }

    void filterAll()
    {
        // Sets voxel grid to downsample all fields rather than just XYZ
        pcl_voxel.setDownsampleAllData(true);

        // Sets up for range based filtering
        pcl_voxel.setFilterFieldName("range");

        // Filters range between minRange and 120 (max distance of lidar)
        // For ouster workaround minRange is multiplied by 10^-41 for uint32 (mm) to float32 (m) compensation
        pcl_voxel.setFilterLimits(minRange * pow(10, -41), 120);

        // Leaf size (length, width and height of voxels) is set
        pcl_voxel.setLeafSize (leafSize[0], leafSize[1], leafSize[2]);

        // Minimum points filter is loaded
        pcl_voxel.setMinimumPointsNumberPerVoxel(minPoints);

        // Voxel is filtered
        pcl_voxel.filter(cloud_filtered);


        
    }

    void publish()
    {
      // Debug size point cloud size print message
      // std::cerr << "PointCloud after filtering: " << cloud_filtered.width * cloud_filtered.height 
      // << " data points (" << pcl::getFieldsList (cloud_filtered) << ")." << std::endl;

      // Convert data into ROS type
      fromPCL(cloud_filtered, output);

      // Publish the data
      pub.publish (output);

      // Timing calculation and print out
      stop = std::chrono::high_resolution_clock::now();
      duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
      loop_count +=1;
      sumTime += duration.count();
      averageTime = sumTime / loop_count;
      std::cout << "Average delay over " << loop_count << " callbacks: " << averageTime / 1000 << " milliseconds" << std::endl;

    }

    // Callback used to dynamically reconfigure parameters
    void rqt_callback(pcl_voxel_filter::VoxelConfig &config, uint32_t level)
    {
      // Assign callback variables to globals
      leafSize[0] = config.leaf_length;
      leafSize[1] = config.leaf_width;
      leafSize[2] = config.leaf_height;
      minPoints = config.min_points;
      minPointsRange = config.min_points_range;
      minRange = config.min_range;
      
      // Display recieved variables
      ROS_INFO("Reconfigure Request:");
      ROS_INFO("Leaf Size, lwh:%f %f %f", leafSize[1], leafSize[1], leafSize[2]);
      ROS_INFO("Minimum Points: %d", minPoints);
      ROS_INFO("Minimum Points Filter Max Range: %d", minPointsRange);
      ROS_INFO("Minimum Range: %f", minRange);
      
    }


    // PCL->ROS Conversion methods borrowed from pcl_ros
    void toPCL(const sensor_msgs::PointCloud2 &pc2, pcl::PCLPointCloud2 &pcl_pc2)
    {
      copyPointCloud2MetaData(pc2, pcl_pc2);
      pcl_pc2.data = pc2.data;
    }
    void copyPointCloud2MetaData(const sensor_msgs::PointCloud2 &pc2, pcl::PCLPointCloud2 &pcl_pc2)
    {
      toPCL(pc2.header, pcl_pc2.header);
      pcl_pc2.height = pc2.height;
      pcl_pc2.width = pc2.width;
      toPCL(pc2.fields, pcl_pc2.fields);
      pcl_pc2.is_bigendian = pc2.is_bigendian;
      pcl_pc2.point_step = pc2.point_step;
      pcl_pc2.row_step = pc2.row_step;
      pcl_pc2.is_dense = pc2.is_dense;
    }
    void toPCL(const std_msgs::Header &header, pcl::PCLHeader &pcl_header)
    {
      toPCL(header.stamp, pcl_header.stamp);
      pcl_header.seq = header.seq;
      pcl_header.frame_id = header.frame_id;
    }
    void toPCL(const ros::Time &stamp, uint64_t &pcl_stamp)
    {
      pcl_stamp = stamp.toNSec() / 1000ull;  // Convert from ns to us
    }
    void toPCL(const sensor_msgs::PointField &pf, pcl::PCLPointField &pcl_pf)
    {
      pcl_pf.name = pf.name;
      pcl_pf.offset = pf.offset;
      pcl_pf.datatype = pf.datatype;
      pcl_pf.count = pf.count;
    }
    void toPCL(const std::vector<sensor_msgs::PointField> &pfs, std::vector<pcl::PCLPointField> &pcl_pfs)
    {
      pcl_pfs.resize(pfs.size());
      std::vector<sensor_msgs::PointField>::const_iterator it = pfs.begin();
      int i = 0;
      for(; it != pfs.end(); ++it, ++i) {
        toPCL(*(it), pcl_pfs[i]);
      }
    }
    void fromPCL(const pcl::PCLPointCloud2 &pcl_pc2, sensor_msgs::PointCloud2 &pc2)
    {
      copyPCLPointCloud2MetaData(pcl_pc2, pc2);
      pc2.data = pcl_pc2.data;
    }
    void copyPCLPointCloud2MetaData(const pcl::PCLPointCloud2 &pcl_pc2, sensor_msgs::PointCloud2 &pc2)
    {
      fromPCL(pcl_pc2.header, pc2.header);
      pc2.height = pcl_pc2.height;
      pc2.width = pcl_pc2.width;
      fromPCL(pcl_pc2.fields, pc2.fields);
      pc2.is_bigendian = pcl_pc2.is_bigendian;
      pc2.point_step = pcl_pc2.point_step;
      pc2.row_step = pcl_pc2.row_step;
      pc2.is_dense = pcl_pc2.is_dense;
    }
    void fromPCL(const pcl::PCLHeader &pcl_header, std_msgs::Header &header)
    {
      fromPCL(pcl_header.stamp, header.stamp);
      header.seq = pcl_header.seq;
      header.frame_id = pcl_header.frame_id;
    }
    void fromPCL(const pcl::PCLPointField &pcl_pf, sensor_msgs::PointField &pf)
    {
      pf.name = pcl_pf.name;
      pf.offset = pcl_pf.offset;
      pf.datatype = pcl_pf.datatype;
      pf.count = pcl_pf.count;
    }
    void fromPCL(const std::vector<pcl::PCLPointField> &pcl_pfs, std::vector<sensor_msgs::PointField> &pfs)
    {
      pfs.resize(pcl_pfs.size());
      std::vector<pcl::PCLPointField>::const_iterator it = pcl_pfs.begin();
      int i = 0;
      for(; it != pcl_pfs.end(); ++it, ++i) {
        fromPCL(*(it), pfs[i]);
      }
    }
    void fromPCL(const uint64_t &pcl_stamp, ros::Time &stamp)
    {
      stamp.fromNSec(pcl_stamp * 1000ull);  // Convert from us to ns
    }
};

int main (int argc, char **argv)
{
    // Initialize node
    ros::init(argc, argv, "voxel_filter");

    // Create voxel filter object
    voxel_filter_node voxel_filter;

    // Spin some threads!
    ros::spin();
}