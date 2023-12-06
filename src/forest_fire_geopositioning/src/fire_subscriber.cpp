//
// Created by qin on 05/12/23.
// This is an example of subscribing to GPS data.
//

#include <iostream>
#include <ros/ros.h>
#include <message_filters/time_synchronizer.h>
#include <glog/logging.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <vector>
#include <sensor_msgs/ChannelFloat32.h>

//std::vector<sensor_msgs::NavSatFix> camera_poses_GPS; // not recommended

void CamCallback(const sensor_msgs::NavSatFixConstPtr &m300_GPS) {
    // print the GPS position of the camera
    LOG(INFO) << "The GPS position of the camera: " << m300_GPS->latitude << ", " << m300_GPS->longitude << ", "
              << m300_GPS->altitude << ".";
}

void FireCallback(const geometry_msgs::PoseArrayConstPtr &fire_spots_GPS) {
    // print number of fire spots
    LOG(INFO) << "The number of fire spots: " << fire_spots_GPS->poses.size() << ".";
    // print the average GPS positions of fire spots
    double latitude = 0;
    double longitude = 0;
    double altitude = 0;
    for (const geometry_msgs::Pose &fire_spot: fire_spots_GPS->poses) {
        latitude += fire_spot.position.x;
        longitude += fire_spot.position.y;
        altitude += fire_spot.position.z;
    }
    latitude /= fire_spots_GPS->poses.size();
    longitude /= fire_spots_GPS->poses.size();
    altitude /= fire_spots_GPS->poses.size();
    LOG(INFO) << "The average GPS positions of fire spots: " << latitude << ", " << longitude << ", " << altitude
              << ".";
}

int main(int argc, char **argv) {
    // subscribe to the fire spots and the camera pose using message_filters
    ros::init(argc, argv, "fire_subscriber");
    ros::NodeHandle nh;

    // subscribe to the camera GPS
    ros::Subscriber m300_GPS_sub = nh.subscribe("/dji_osdk_ros/gps_position", 1, CamCallback);
    // subscribe to the fire GPS
    ros::Subscriber fire_spots_GPS_sub = nh.subscribe("/position/fire_spots_GPS", 1, FireCallback);

    ros::spin();

    return 0;
}