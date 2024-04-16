//
// Created by qin on 4/11/24.
//

#include <iostream>
#include <ros/ros.h>
#include <glog/logging.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/ChannelFloat32.h>

void callback(const sensor_msgs::NavSatFixConstPtr &GPS_msg) {
    // show the GPS position of the camera
    LOG(INFO) << "The GPS position of the camera: " << GPS_msg->latitude << ", " << GPS_msg->longitude << ", "
              << GPS_msg->altitude << ".";
}


int main(int argc, char **argv) {
    // subscribe to the fire spots and the camera pose using message_filters
    ros::init(argc, argv, "gps_sub");
    ros::NodeHandle nh;

    ros::Subscriber GPS_sub = nh.subscribe<sensor_msgs::NavSatFix>
            ("/dji_osdk_ros/gps_position", 10, callback);

    ros::spin();

    return 0;
}