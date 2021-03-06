#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <chrono>

using namespace sensor_msgs;
using namespace message_filters;

class pcl_fusion_node {

    private:
    ros::NodeHandle nh;
    ros::Publisher pub;

    int queue_size;

    std::string inputTopic1;
    std::string inputTopic2;
    std::string outputTopic;

    // Initialize Lidar subscriber objects
    message_filters::Subscriber<PointCloud2> lidar_sub1;
    message_filters::Subscriber<PointCloud2> lidar_sub2;

    // Create time synchronizer using lidar subscribers
    typedef sync_policies::ApproximateTime<PointCloud2, PointCloud2> MySyncPolicy;
    typedef Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync;

    // Initialize target and child tf frames
    std::string targetFrame;
    std::string childFrame;

    // Create tf transformer and listener
    tf::TransformListener listener;
    tf::StampedTransform transform;

    // Create ros sensor messages
    sensor_msgs::PointCloud2 ros_cloud2;
    sensor_msgs::PointCloud2 transformed_cloud2;

    // Creates PCL point cloud 2 variable to store data
    pcl::PCLPointCloud2* cloud1 = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2* cloud2 = new pcl::PCLPointCloud2;

    // Create output PCL and ROS variables
    pcl::PCLPointCloud2 combined_cloud;
    sensor_msgs::PointCloud2 output;

    // Timing Variables
    std::chrono::_V2::system_clock::time_point start;
    std::chrono::_V2::system_clock::time_point stop;
    std::chrono::microseconds duration;
    std::double_t averageTime = 0;
    std::double_t sumTime = 0;
    std::int16_t loop_count = 0;

    public:
    pcl_fusion_node() {
      std::cout << std::endl;
      ROS_INFO("pcl_fusion Node Initialize");

      // Get parameters from ROS parameter server
      ros::param::get("/inputTopic1", inputTopic1);
      ros::param::get("/inputTopic2", inputTopic2);
      ros::param::get("/outputTopic", outputTopic);
      ros::param::get("/queue_size", queue_size);

      ROS_INFO("The first input topic is %s" , inputTopic1.c_str());
      ROS_INFO("The second input topic is %s" , inputTopic2.c_str());
      ROS_INFO("The output topic is %s" , outputTopic.c_str());
      ROS_INFO("The queue_size is %i" , queue_size);

      // Subscribe to lidar input topics using message filter
      lidar_sub1.subscribe(nh, inputTopic1, 1);
      lidar_sub2.subscribe(nh, inputTopic2, 1);

      // Register pointcloud2 messages to callback method
      sync.reset(new Sync(MySyncPolicy(queue_size), lidar_sub1, lidar_sub2));
      sync->registerCallback(boost::bind(&pcl_fusion_node::callback, this, _1, _2));

      // Create a ROS publisher for the output point cloud
      pub = nh.advertise<PointCloud2> (outputTopic, 1);

    }

    void callback(const PointCloud2ConstPtr& cloud_msg1, const PointCloud2ConstPtr& cloud_msg2)
      {
        // Start time for callback timing purposes
        auto start = std::chrono::high_resolution_clock::now();


        //****** TF transform creation

        // Save target and child tf frames to variables
        targetFrame = cloud_msg1->header.frame_id;
        childFrame = cloud_msg2->header.frame_id;

        try{
        // Try to lookup transform from child frame to target frame
            listener.waitForTransform(targetFrame, childFrame, ros::Time(0), ros::Duration(4));
            listener.lookupTransform(targetFrame, childFrame,  ros::Time(0), transform);
        }

        catch (tf::TransformException ex){
        // Write ros error if transform fails

            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }



        // ****** Transforming second lidar message

        // Dereference the second lidar cloud by storing to a variable
        ros_cloud2 = *cloud_msg2;

        // Transform the lidar cloud into the target frame -- uses the <pcl_ros/transforms.h> import
        pcl_ros::transformPointCloud(targetFrame, transform, ros_cloud2, transformed_cloud2);



        //******** PCL data management

        // Writes the callback and transformed data to a PCL point cloud 2 message
        pcl_conversions::toPCL(*cloud_msg1, *cloud1);
        pcl_conversions::toPCL(transformed_cloud2, *cloud2);

        // Combining clouds
        pcl::concatenatePointCloud(*cloud1, *cloud2, combined_cloud);

        // ROS conversion and output
        pcl_conversions::fromPCL(combined_cloud, output);

        // Publish the combined output
        pub.publish(output);



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
    ros::init(argc, argv, "pcl_fusion");
    pcl_fusion_node pcl_fusion;

    ros::spin();
}