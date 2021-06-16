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

    // cout << map_roll << "\t" << map_pitch << "\t" << map_yaw << endl;
    // cout << map_quat.x() << "\t" << map_quat.y() << "\t" << map_quat.z() << "\t" << map_quat.w() << endl;

    static_WorldToMap.sendTransform(static_transformStamped);
}

void tfTreeClass::MapToInsTransform(const nav_msgs::Odometry::ConstPtr& msg)
{
    static tf2_ros::TransformBroadcaster MapToIns; 
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = map_frame;
    transformStamped.child_frame_id = ins_frame;

    transformStamped.transform.translation.x = msg->pose.pose.position.x - map_x;
    transformStamped.transform.translation.y = msg->pose.pose.position.y - map_y;
    transformStamped.transform.translation.z = msg->pose.pose.position.z - map_z;

    tf2::Quaternion map_quat;
    map_quat.setRPY(map_roll, map_pitch, map_yaw);
    //map_quat.inverse();
    //transformStamped.transform.rotation = msg->pose.pose.orientation * map_quat;
    transformStamped.transform.rotation.x = msg->pose.pose.orientation.x;// * map_quat.x();
    transformStamped.transform.rotation.y = msg->pose.pose.orientation.y;// * map_quat.y();
    transformStamped.transform.rotation.z = msg->pose.pose.orientation.z;// * map_quat.z();
    transformStamped.transform.rotation.w = msg->pose.pose.orientation.w;// * map_quat.w();

    cout << msg->pose.pose.orientation.x << "\t" << msg->pose.pose.orientation.y << "\t" << msg->pose.pose.orientation.z << "\t" << msg->pose.pose.orientation.w << endl; 

    WorldToMapTransform();
    MapToIns.sendTransform(transformStamped);
    InsToBaseLinkTransform();
    BaseLinkToLidarTransform();
    BaseLinkToCameraTransform();
    CameraToFrictionImageTransform();
}

void tfTreeClass::InsToBaseLinkTransform()
{
    static tf2_ros::StaticTransformBroadcaster static_InsToBaseLink;
    geometry_msgs::TransformStamped static_transformStamped;

    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = ins_frame;
    static_transformStamped.child_frame_id = base_link_frame;

    static_transformStamped.transform.translation.x = base_link_x;
    static_transformStamped.transform.translation.y = base_link_y;
    static_transformStamped.transform.translation.z = base_link_z;

    tf2::Quaternion base_link_quat;
    base_link_quat.setRPY(base_link_roll, base_link_pitch, base_link_yaw);
    static_transformStamped.transform.rotation.x = base_link_quat.x();
    static_transformStamped.transform.rotation.y = base_link_quat.y();
    static_transformStamped.transform.rotation.z = base_link_quat.z();
    static_transformStamped.transform.rotation.w = base_link_quat.w();

    static_InsToBaseLink.sendTransform(static_transformStamped);
}

void tfTreeClass::BaseLinkToLidarTransform()
{
    static tf2_ros::StaticTransformBroadcaster static_BaseLinkToLidar;
    geometry_msgs::TransformStamped static_transformStamped;

    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = base_link_frame;
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

    static_BaseLinkToLidar.sendTransform(static_transformStamped);
}

void tfTreeClass::BaseLinkToCameraTransform()
{
    static tf2_ros::StaticTransformBroadcaster static_BaseLinkToCamera;
    geometry_msgs::TransformStamped static_transformStamped;

    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = base_link_frame;
    static_transformStamped.child_frame_id = camera_frame;

    static_transformStamped.transform.translation.x = camera_x;
    static_transformStamped.transform.translation.y = camera_y;
    static_transformStamped.transform.translation.z = camera_z;

    tf2::Quaternion camera_quat;
    camera_quat.setRPY(camera_roll, camera_pitch, camera_yaw);
    static_transformStamped.transform.rotation.x = camera_quat.x();
    static_transformStamped.transform.rotation.y = camera_quat.y();
    static_transformStamped.transform.rotation.z = camera_quat.z();
    static_transformStamped.transform.rotation.w = camera_quat.w();

    static_BaseLinkToCamera.sendTransform(static_transformStamped);
}

void tfTreeClass::CameraToFrictionImageTransform()
{
    static tf2_ros::StaticTransformBroadcaster static_CameraToFrictionImage;
    geometry_msgs::TransformStamped static_transformStamped;

    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = camera_frame;
    static_transformStamped.child_frame_id = friction_frame;

    static_transformStamped.transform.translation.x = friction_x;
    static_transformStamped.transform.translation.y = friction_y;
    static_transformStamped.transform.translation.z = friction_z;

    tf2::Quaternion friction_quat;
    friction_quat.setRPY(friction_roll, friction_pitch, friction_yaw);
    static_transformStamped.transform.rotation.x = friction_quat.x();
    static_transformStamped.transform.rotation.y = friction_quat.y();
    static_transformStamped.transform.rotation.z = friction_quat.z();
    static_transformStamped.transform.rotation.w = friction_quat.w();

    static_CameraToFrictionImage.sendTransform(static_transformStamped);
}

void tfTreeClass::Initialize(int argc, char **argv)
{
    WorldToMapTransform();
    sub_ins = tf_node.subscribe("odometry_data", 10, &tfTreeClass::MapToInsTransform, this);
}
