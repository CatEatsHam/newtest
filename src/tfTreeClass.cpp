#include "tfTreeClass.h" 

void tfTreeClass::WorldToMapTransform()
{
    static tf2_ros::StaticTransformBroadcaster static_WorldToMap;
    geometry_msgs::TransformStamped static_transformStamped;

    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = world_frame;
    static_transformStamped.child_frame_id = map_frame;

    static_transformStamped.transform.translation.x = map_x;
    static_transformStamped.transform.translation.y = map_y;
    static_transformStamped.transform.translation.z = map_z;

    tf2::Quaternion map_quat;
    map_quat.setRPY(map_roll, map_pitch, map_yaw);
    static_transformStamped.transform.rotation.x = map_quat.x();
    static_transformStamped.transform.rotation.y = map_quat.y();
    static_transformStamped.transform.rotation.z = map_quat.z();
    static_transformStamped.transform.rotation.w = map_quat.w();

    static_WorldToMap.sendTransform(static_transformStamped);
}

void tfTreeClass::WorldToInsTransform(const nav_msgs::Odometry::ConstPtr& msg)
{
    static tf2_ros::StaticTransformBroadcaster WorldToIns;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = world_frame;
    transformStamped.child_frame_id = ins_frame;

    transformStamped.transform.translation.x = msg->pose.pose.position.x;
    transformStamped.transform.translation.y = msg->pose.pose.position.y;
    transformStamped.transform.translation.z = msg->pose.pose.position.z;

    transformStamped.transform.rotation.x = msg->pose.pose.orientation.x;
    transformStamped.transform.rotation.y = msg->pose.pose.orientation.y;
    transformStamped.transform.rotation.z = msg->pose.pose.orientation.z;
    transformStamped.transform.rotation.w = msg->pose.pose.orientation.w;
    WorldToIns.sendTransform(transformStamped);

    InsToLidarTransform();
    WorldToMapTransform();


    // tf2::Quaternion quat;
    // quat.setRPY(map_roll, map_pitch, map_yaw);
    // double roll, pitch, yaw;
    // tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
}

void tfTreeClass::InsToLidarTransform()
{
    static tf2_ros::StaticTransformBroadcaster static_InsTolidar;
    geometry_msgs::TransformStamped static_transformStamped;

    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = ins_frame;
    static_transformStamped.child_frame_id = lidar_frame;

    static_transformStamped.transform.translation.x = lidar_x;
    static_transformStamped.transform.translation.y = lidar_y;
    static_transformStamped.transform.translation.z = lidar_z;

    tf2::Quaternion lidar_quat;
    lidar_quat.setRPY(lidar_roll, lidar_pitch, lidar_yaw);
    static_transformStamped.transform.rotation.x = lidar_quat.x();
    static_transformStamped.transform.rotation.y = lidar_quat.y();
    static_transformStamped.transform.rotation.z = lidar_quat.z();
    static_transformStamped.transform.rotation.w = lidar_quat.w();

    static_InsTolidar.sendTransform(static_transformStamped);
}


void tfTreeClass::Initialize(int argc, char **argv)
{
    WorldToMapTransform();
    sub_ins = tf_node.subscribe("odometry_data", 10, &tfTreeClass::WorldToInsTransform, this);
}
