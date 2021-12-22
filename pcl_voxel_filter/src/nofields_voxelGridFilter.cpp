#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <chrono>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl_ros/filters/voxel_grid.h>

using namespace sensor_msgs;

class voxel_filter_node {

    private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher pub;

    std::string inputTopic;
    std::string outputTopic;

    // pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2 cloud_filtered;

    pcl::VoxelGrid<pcl::PCLPointCloud2> pcl_voxel;
    std::vector<double> leafSize;
    int minPoints;

    sensor_msgs::PointCloud2 output;

    // Timing Variables
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

      // Get parameters from ROS parameter server
      ros::param::get("~inputTopic", inputTopic);
      ros::param::get("~outputTopic", outputTopic);
      ros::param::get("~leafSize", leafSize);
      ros::param::get("~minPoints", minPoints);

      ROS_INFO("The input topic is %s" , inputTopic.c_str());
      ROS_INFO("The output topic is %s" , outputTopic.c_str());
      ROS_INFO("The minimum points in a voxel is %i" , minPoints);
      ROS_INFO("Leaf size set to: %.3f, %0.3f, %0.3f" , leafSize[0], leafSize[1], leafSize[2]);

      // Subscribe to lidar input topic
      sub = nh.subscribe(inputTopic, 1, &voxel_filter_node::callback, this);

      // Create a ROS publisher for the output point cloud
      pub = nh.advertise<PointCloud2> (outputTopic, 1);

    }

    void callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
      {
        // Start time for callback timing purposes
        start = std::chrono::high_resolution_clock::now();

        // // Convert to PCL data type
        // pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
        // pcl_conversions::toPCL(*cloud_msg, *cloud);
        // pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
        



        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::PointCloud<pcl::PointXYZRGB> cloud_filtered;
        pcl::fromROSMsg(*cloud_msg, *cloud);

        pcl::VoxelGrid<pcl::PointXYZRGB> vox_filt;
        vox_filt.setInputCloud(cloud);
        vox_filt.setDownsampleAllData(true);
        vox_filt.setLeafSize(leafSize[0], leafSize[1], leafSize[2]);
        vox_filt.setMinimumPointsNumberPerVoxel(minPoints);
        vox_filt.filter(cloud_filtered);
        pcl::toROSMsg(cloud_filtered, output);






        // std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
        // << " data points (" << pcl::getFieldsList (*cloud) << ")." << std::endl;

        // Load point cloud into voxel class
        // pcl_voxel.setInputCloud(cloudPtr);

        // filter();

        publish();
      }

    void filter()
    {
        // Set size of voxel leaves and output voxel grid to cloud_filtered
        pcl_voxel.setLeafSize (leafSize[0], leafSize[1], leafSize[2]);
        pcl_voxel.setMinimumPointsNumberPerVoxel((int)minPoints);
        // std::cout<<"Min points: " << pcl_voxel.getMinimumPointsNumberPerVoxel()  <<std::endl;
        pcl_voxel.filter(cloud_filtered);
        // std::cerr << "PointCloud after filtering: " << cloud_filtered.width * cloud_filtered.height 
        // << " data points (" << pcl::getFieldsList (cloud_filtered) << ")." << std::endl;
        
    }

    void publish()
    {
      // Convert data into ROS type
      // pcl_conversions::fromPCL(cloud_filtered, output);

      // Publish the data
      pub.publish (output);


      //******** Timing calculation and print out

      stop = std::chrono::high_resolution_clock::now();
      duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

      loop_count +=1;
      sumTime += duration.count();
      averageTime = sumTime / loop_count;
      std::cout << "Average delay over " << loop_count << " callbacks: " << averageTime / 1000 << " milliseconds" << std::endl;

    }
};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "voxel_filter");
    voxel_filter_node voxel_filter;

    ros::spin();
}