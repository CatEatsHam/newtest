#include "tfTreeClass.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tf_tree_publisher");
    tfTreeClass *tf_tree = new tfTreeClass();

    ros::NodeHandle node("~");
    node.getParam("num_of_ins", tf_tree->ins_number);
    node.getParam("num_of_lidar", tf_tree->lidar_number);
    node.getParam("num_of_camera", tf_tree->camera_number);
    node.getParam("num_of_radar", tf_tree->radar_number);

    node.getParam("world_frame", tf_tree->world_frame);
    node.getParam("map_frame", tf_tree->map_frame);
    node.getParam("ins_frame", tf_tree->ins_frame);
    node.getParam("base_link_frame", tf_tree->base_link_frame);
    node.getParam("lidar_frame", tf_tree->lidar_frame);
    node.getParam("roof_lidar_frame", tf_tree->roof_lidar_frame);
    node.getParam("camera_frame", tf_tree->camera_frame);
    node.getParam("radar_frame", tf_tree->radar_frame);
    node.getParam("friction_frame", tf_tree->friction_frame);

    node.getParam("map_translation_x", tf_tree->map_x);
    node.getParam("map_translation_y", tf_tree->map_y);
    node.getParam("map_translation_z", tf_tree->map_z);
    node.getParam("map_rotation_roll", tf_tree->map_roll);
    node.getParam("map_rotation_pitch", tf_tree->map_pitch);
    node.getParam("map_rotation_yaw", tf_tree->map_yaw);

    node.getParam("base_link_translation_x", tf_tree->base_link_x);
    node.getParam("base_link_translation_y", tf_tree->base_link_y);
    node.getParam("base_link_translation_z", tf_tree->base_link_z);
    node.getParam("base_link_rotation_roll", tf_tree->base_link_roll);
    node.getParam("base_link_rotation_pitch", tf_tree->base_link_pitch);
    node.getParam("base_link_rotation_yaw", tf_tree->base_link_yaw);

    node.getParam("lidar_translation_x", tf_tree->lidar_x);
    node.getParam("lidar_translation_y", tf_tree->lidar_y);
    node.getParam("lidar_translation_z", tf_tree->lidar_z);
    node.getParam("lidar_rotation_roll", tf_tree->lidar_roll);
    node.getParam("lidar_rotation_pitch", tf_tree->lidar_pitch);
    node.getParam("lidar_rotation_yaw", tf_tree->lidar_yaw);

    node.getParam("roof_lidar_translation_x", tf_tree->roof_lidar_x);
    node.getParam("roof_lidar_translation_y", tf_tree->roof_lidar_y);
    node.getParam("roof_lidar_translation_z", tf_tree->roof_lidar_z);
    node.getParam("roof_lidar_rotation_roll", tf_tree->roof_lidar_roll);
    node.getParam("roof_lidar_rotation_pitch", tf_tree->roof_lidar_pitch);
    node.getParam("roof_lidar_rotation_yaw", tf_tree->roof_lidar_yaw);

    node.getParam("camera_translation_x", tf_tree->camera_x);
    node.getParam("camera_translation_y", tf_tree->camera_y);
    node.getParam("camera_translation_z", tf_tree->camera_z);
    node.getParam("camera_rotation_roll", tf_tree->camera_roll);
    node.getParam("camera_rotation_pitch", tf_tree->camera_pitch);
    node.getParam("camera_rotation_yaw", tf_tree->camera_yaw);

    node.getParam("friction_translation_x", tf_tree->friction_x);
    node.getParam("friction_translation_y", tf_tree->friction_y);
    node.getParam("friction_translation_z", tf_tree->friction_z);
    node.getParam("friction_rotation_roll", tf_tree->friction_roll);
    node.getParam("friction_rotation_pitch", tf_tree->friction_pitch);
    node.getParam("friction_rotation_yaw", tf_tree->friction_yaw);

    tf_tree->Initialize(argc, argv);
    ros::spin();

    delete tf_tree;
    return 0;
}
