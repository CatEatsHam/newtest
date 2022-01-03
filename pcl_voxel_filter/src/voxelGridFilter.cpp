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

#include <dynamic_reconfigure/server.h>
#include <pcl_voxel_filter/VoxelConfig.h>

using namespace sensor_msgs;

class voxel_filter_node {

    private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher pub;

    std::string inputTopic;
    std::string outputTopic;

    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2 cloud_filtered;

    pcl::VoxelGrid<pcl::PCLPointCloud2> pcl_voxel;
    std::vector<double> leafSize;
    int minPoints;
    double minRange;

    sensor_msgs::PointCloud2 output;

    // Timing Variables
    std::chrono::_V2::system_clock::time_point start;
    std::chrono::_V2::system_clock::time_point stop;
    std::chrono::microseconds duration;

    std::double_t averageTime = 0;
    std::double_t sumTime = 0;
    std::int16_t loop_count = 0;

    // Dynamic reconfigure setup
    dynamic_reconfigure::Server<pcl_voxel_filter::VoxelConfig> server;
    dynamic_reconfigure::Server<pcl_voxel_filter::VoxelConfig>::CallbackType f;

    public:
    voxel_filter_node() {
      std::cout << std::endl;
      ROS_INFO("Voxel Filter Node Initialize");
      std::cout << PCL_VERSION << std::endl;

      // Get parameters from ROS parameter server
      ros::param::get("~inputTopic", inputTopic);
      ros::param::get("~outputTopic", outputTopic);
      ros::param::get("~leafSize", leafSize);
      ros::param::get("~minPoints", minPoints);
      ros::param::get("~minRange", minRange);

      ROS_INFO("The input topic is %s" , inputTopic.c_str());
      ROS_INFO("The output topic is %s" , outputTopic.c_str());
      ROS_INFO("The minimum points in a voxel is %i" , minPoints);
      ROS_INFO("Leaf size set to: %0.3f, %0.3f, %0.3f" , leafSize[0], leafSize[1], leafSize[2]);
      // ROS_INFO("The minimum range is: #")

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

        // Convert to PCL data type
        pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
        toPCL(*cloud_msg, *cloud);
        pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
        
        // std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
        // << " data points (" << pcl::getFieldsList (*cloud) << ")." << std::endl;

        // Load point cloud into voxel class
        pcl_voxel.setInputCloud(cloudPtr);
        // pcl_voxel.setFilterFieldName("range");
        pcl_voxel.setDownsampleAllData(true);

        filter();

        publish();
      }

    void filter()
    {
        // Set size of voxel leaves and output voxel grid to cloud_filtered
        pcl_voxel.setDownsampleAllData(true);
        // pcl_voxel.setFilterLimits((float)(minRange/1000), (float)120);
        pcl_voxel.setLeafSize (leafSize[0], leafSize[1], leafSize[2]);
        pcl_voxel.setMinimumPointsNumberPerVoxel(minPoints);

        // std::cout<<"Min points: " << pcl_voxel.getMinimumPointsNumberPerVoxel()  <<std::endl;

        pcl_voxel.filter(cloud_filtered);

        // std::cerr << "PointCloud after filtering: " << cloud_filtered.width * cloud_filtered.height 
        // << " data points (" << pcl::getFieldsList (cloud_filtered) << ")." << std::endl;
        
    }

    void publish()
    {
      // Convert data into ROS type
      fromPCL(cloud_filtered, output);

      // Publish the data
      pub.publish (output);

      // ******** Timing calculation and print out

      stop = std::chrono::high_resolution_clock::now();
      duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

      loop_count +=1;
      sumTime += duration.count();
      averageTime = sumTime / loop_count;
      std::cout << "Average delay over " << loop_count << " callbacks: " << averageTime / 1000 << " milliseconds" << std::endl;

    }

    void rqt_callback(pcl_voxel_filter::VoxelConfig &config, uint32_t level)
    {
      // Display recieved variables
      ROS_INFO("Reconfigure Request:");
      ROS_INFO("Leaf Size, lwh:%f %f %f", config.leaf_length, config.leaf_width, config.leaf_height);
      ROS_INFO("Minimum Points: %d", config.min_points);
      ROS_INFO("Minimum Range: %d", config.min_range);
      
      // Assign callback variables to globals
      leafSize[0] = config.leaf_length;
      leafSize[1] = config.leaf_width;
      leafSize[2] = config.leaf_height;
      minPoints = config.min_points;

    }


    // PCL->ROS Conversion methods borrowed from pcl_ros to allow for newer PCL version to be used without header mismatches
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
    void toPCL(const ros::Time &stamp, pcl::uint64_t &pcl_stamp)
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
    void fromPCL(const pcl::uint64_t &pcl_stamp, ros::Time &stamp)
    {
      stamp.fromNSec(pcl_stamp * 1000ull);  // Convert from us to ns
    }
};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "voxel_filter");
    voxel_filter_node voxel_filter;

    ros::spin();
}