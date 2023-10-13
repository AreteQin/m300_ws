//
// Created by qin on 12/10/23.
//

// use image_transport to subscribe to the topic /dji_osdk_ros/main_camera_images
// and compress the image, then publish it to // the topic /dji_osdk_ros/main_wide_RGB/

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using namespace std;
using namespace cv;

void imageCallback(const sensor_msgs::ImageConstPtr& msg, image_transport::Publisher *pub) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        Mat img = cv_ptr->image;
        Mat dst;
        resize(img, dst, Size(640, 480));
//        imshow("view", dst);
//        waitKey(1);
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", dst).toImageMsg();
        pub->publish(msg);

    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "compress_video_node");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("/dji_osdk_ros/main_wide_RGB/", 5);
    image_transport::Subscriber sub = it.subscribe("/dji_osdk_ros/main_camera_images", 5,
                                                   boost::bind(imageCallback, _1, &pub));

    ros::spin();
    return 0;
}