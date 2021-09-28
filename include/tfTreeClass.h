#ifndef _TFTREECLASS_H
#define _TFTREECLASS_H

#include "ros/ros.h"

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>

#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <string>

using namespace std;

class tfTreeClass
{
public:

    // Assuming one sensor for each lidar and ins but will become flexible in the future
    /* TODO: Make methods to take in possible multiple sensors such as LiDAR and that transfrom
        is relative to the ins. */

    // Identifying how many sensors
    int ins_number = 1;
    int lidar_number = 1;
    int camera_number = 0;
    int radar_number = 0;

    // Frame names. These can be changed from the launch file
    string world_frame = "world";
    string map_frame = "map";
    string ins_frame = "vectornav";
    string base_link_frame = "base_link";
    string lidar_frame = "os1_sensor";
    string roof_lidar_frame = "roof/os1_sensor";
    string camera_frame = "camera";
    string radar_frame = "radar";
    string friction_frame = "friction_image";

    // Map translation and rotation from world to ins
    float map_x = 0.0f, map_y = 0.0f, map_z = 0.0f, map_roll = 0.0f, map_pitch = 0.0f, map_yaw = 0.0f;

    // Ins translation and rotation from map to ins
    float ins_x = 0.0f, ins_y = 0.0f, ins_z = 0.0f, ins_roll = 0.0f, ins_pitch = 0.0f, ins_yaw = 0.0f;

    // base_link translation and rotation from ins to base_link
    float base_link_x = 0.0f, base_link_y = 0.0f, base_link_z = 0.0f, base_link_roll = 0.0f, base_link_pitch = 0.0f, base_link_yaw = 0.0f;

    // LiDAR translation and rotation from base_link to lidar
    float lidar_x = 0.0f, lidar_y = 0.0f, lidar_z = 0.0f, lidar_roll = 0.0f, lidar_pitch = 0.0f, lidar_yaw = 0.0f;

    // LiDAR translation and rotation from base_link to lidar
    float roof_lidar_x = 0.0f, roof_lidar_y = 0.0f, roof_lidar_z = 0.0f, roof_lidar_roll = 0.0f, roof_lidar_pitch = 0.0f, roof_lidar_yaw = 0.0f;

    // Camera translation and rotation from base_link to camera
    float camera_x = 0.0f, camera_y = 0.0f, camera_z = 0.0f, camera_roll = 0.0f, camera_pitch = 0.0f, camera_yaw = 0.0f;

    // friction translation and rotation from camera to friction image
    float friction_x = 0.0f, friction_y = 0.0f, friction_z = 0.0f, friction_roll = 0.0f, friction_pitch = 0.0f, friction_yaw = 0.0f;

    // Methods to perform the transforms
    void WorldToMapTransform();
    void MapToInsTransform(const nav_msgs::Odometry::ConstPtr& msg);
    void WorldToInsTransform(const nav_msgs::Odometry::ConstPtr& msg);
    void InsToBaseLinkTransform();
    void BaseLinkToLidarTransform();
    void BaseLinkToRoofLidarTransform();
    void BaseLinkToCameraTransform();
    void BaseLinkToRadarTransform();
    void CameraToFrictionImageTransform();

    void InsToLidarTransform();
    void InsToCameraTransform();
    void InsToRadarTransform();

    // ROS stuff
    ros::NodeHandle tf_node;
    ros::Subscriber sub_ins;

    void Initialize(int argc, char **argv);
    void UpdateLoop();


};

#endif
