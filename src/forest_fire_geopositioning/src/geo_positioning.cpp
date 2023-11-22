//
// Created by qin on 10/11/23.
//

#include <iostream>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <glog/logging.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <vector>
#include <sensor_msgs/ChannelFloat32.h>

std::vector<geometry_msgs::PoseStamped> camera_poses_SLAM;
std::vector<sensor_msgs::NavSatFix> camera_poses_GPS;

void callback(ros::Publisher *real_scale_pub,
                ros::Publisher *fire_spots_GPS_pub,
              const geometry_msgs::PoseArrayConstPtr &fire_spots_msg,
              const geometry_msgs::PoseStampedConstPtr &camera_pose_msg,
              const sensor_msgs::NavSatFixConstPtr &GPS_msg) {
//    LOG(INFO)<<"The number of fire spots: "<<fire_spots_msg->poses.size()<<".";
    // store camera pose
//    if (rest_map) {
//        camera_poses_SLAM.clear();
//        camera_poses_GPS.clear();
//        rest_map = false;
//    }
    camera_poses_SLAM.push_back(*camera_pose_msg);
    camera_poses_GPS.push_back(*GPS_msg);

    // calculate the real scale
    if (camera_poses_SLAM.size() > 1) {
        // calculate the distance between the camera poses
        double distance = sqrt(pow(camera_poses_SLAM.back().pose.position.x - camera_poses_SLAM.front().pose.position.x, 2)
                               + pow(camera_poses_SLAM.back().pose.position.y - camera_poses_SLAM.front().pose.position.y, 2)
                               + pow(camera_poses_SLAM.back().pose.position.z - camera_poses_SLAM.front().pose.position.z, 2));
        // calculate the distance between the GPS positions
        double distance_GPS = sqrt(pow(camera_poses_GPS.back().latitude - camera_poses_GPS.front().latitude, 2)
                                   + pow(camera_poses_GPS.back().longitude - camera_poses_GPS.front().longitude, 2)
                                   + pow(camera_poses_GPS.back().altitude - camera_poses_GPS.front().altitude, 2));
        // calculate the real scale
        double real_scale = distance / distance_GPS;
        // publish the real scale
        sensor_msgs::ChannelFloat32 real_scale_msg;
        real_scale_msg.name = "real_scale";
        real_scale_msg.values.push_back(real_scale);
        real_scale_pub->publish(real_scale_msg);
        // calculate the GPS positions of fire spots
        geometry_msgs::PoseArray fire_spots_GPS_msg;
        fire_spots_GPS_msg.header.stamp = fire_spots_msg->header.stamp;
        fire_spots_GPS_msg.header.frame_id = "map";
        for (const geometry_msgs::Pose &fire_spot: fire_spots_msg->poses) {
            geometry_msgs::Pose pose;
            pose.position.x = fire_spot.position.x * real_scale + camera_poses_GPS.front().latitude;
            pose.position.y = fire_spot.position.y * real_scale + camera_poses_GPS.front().longitude;
            pose.position.z = fire_spot.position.z * real_scale + camera_poses_GPS.front().altitude;
            fire_spots_GPS_msg.poses.push_back(pose);
            LOG(INFO)<<"Fire spot GPS: "<<pose.position.x<<", "<<pose.position.y<<", "<<pose.position.z<<".";
        }
        // publish the GPS positions of fire spots
        fire_spots_GPS_pub->publish(fire_spots_GPS_msg);
        // publish real scale
        real_scale_pub->publish(real_scale_msg);
    }
}

int main(int argc, char **argv) {
    // subscribe to the fire spots and the camera pose using message_filters
    ros::init(argc, argv, "geo_positioning");
    ros::NodeHandle nh;
    // publish the real scale SLAM using float data type
    ros::Publisher real_scale_pub = nh.advertise<sensor_msgs::ChannelFloat32>("/position/real_scale", 10);
    // publish GPS positions of fire spots
    ros::Publisher fire_spots_GPS_pub = nh.advertise<geometry_msgs::PoseArray>("/position/fire_spots_GPS", 10);

    message_filters::Subscriber<geometry_msgs::PoseArray> fire_spots_sub(nh, "/position/fire_spots", 10);
    message_filters::Subscriber<geometry_msgs::PoseStamped> camera_pose_sub(nh, "/position/camera_pose", 10);
    message_filters::Subscriber<sensor_msgs::NavSatFix> GPS_sub(nh, "/dji_osdk_ros/gps_position", 10);
    // synchronize the fire spots, the camera pose and the GPS
    message_filters::TimeSynchronizer<geometry_msgs::PoseArray, geometry_msgs::PoseStamped, sensor_msgs::NavSatFix>
            sync(fire_spots_sub, camera_pose_sub, GPS_sub, 10);
    sync.registerCallback(boost::bind(&callback, &real_scale_pub, &fire_spots_GPS_pub, _1, _2, _3));

    ros::spin();

    return 0;
}