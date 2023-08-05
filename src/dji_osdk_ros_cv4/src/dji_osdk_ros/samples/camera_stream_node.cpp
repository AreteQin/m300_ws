//INCLUDE
#include <ros/ros.h>
#include <iostream>
#include <dji_camera_image.hpp>
#include <dji_osdk_ros/SetupCameraStream.h>
#include <dji_osdk_ros/SetupCameraH264.h>
#include <sensor_msgs/Image.h>

extern "C"{
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
}

#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace dji_osdk_ros;

void show_rgb(CameraRGBImage img, char* name)
{
  std::cout << "#### Got image from:\t" << std::string(name) << std::endl;
  cv::Mat mat(img.height, img.width, CV_8UC3, img.rawData.data(), img.width*3);
  cvtColor(mat, mat, cv::COLOR_RGB2BGR);
  imshow(name,mat);
  cv::waitKey(1);
}

void fpvCameraStreamCallBack(const sensor_msgs::Image& msg)
{
  CameraRGBImage img;
  img.rawData = msg.data;
  img.height  = msg.height;
  img.width   = msg.width;
//  char Name[] = "FPV_CAM";
//  show_rgb(img, Name);
  std::cout<<"height is"<<msg.height<<std::endl;
  std::cout<<"width is"<<msg.width<<std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera_stream_node");
    ros::NodeHandle nh;

    /*! H264 flow init and H264 decoder init */
//    auto setup_camera_h264_client = nh.serviceClient<dji_osdk_ros::SetupCameraH264>("setup_camera_h264");
//    dji_osdk_ros::SetupCameraH264 setupCameraH264_;
//    ffmpeg_init();

    /*! RGB flow init */
    auto setup_camera_stream_client = nh.serviceClient<dji_osdk_ros::SetupCameraStream>("setup_camera_stream");
    auto fpv_camera_stream_sub = nh.subscribe("dji_osdk_ros/fpv_camera_images", 10, fpvCameraStreamCallBack);
    dji_osdk_ros::SetupCameraStream setupCameraStream_;

    setupCameraStream_.request.cameraType = setupCameraStream_.request.FPV_CAM;
    setupCameraStream_.request.start = 0;
    setup_camera_stream_client.call(setupCameraStream_);

//    ros::AsyncSpinner spinner(1);
//    spinner.start();
//    ros::Duration(20).sleep();
//
//    setupCameraStream_.request.cameraType = setupCameraStream_.request.FPV_CAM;
//    setupCameraStream_.request.start = 0;
//    setup_camera_stream_client.call(setupCameraStream_);
//
//    ROS_INFO_STREAM("Finished. Press CTRL-C to terminate the node");
//    ros::waitForShutdown();

    return 0;
}
