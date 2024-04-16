//
// Created by qin on 4/11/24.
//

#include <iostream>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <glog/logging.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Image.h>
#include <vector>
#include <sensor_msgs/ChannelFloat32.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
//
//void callback(const sensor_msgs::NavSatFixConstPtr &GPS_msg,
//              const sensor_msgs::ImageConstPtr &camera_msg) {
//    // show the GPS position of the camera
//    LOG(INFO) << "The GPS position of the camera: " << GPS_msg->latitude << ", " << GPS_msg->longitude << ", "
//              << GPS_msg->altitude << ".";
//    // show the image
//    cv_bridge::CvImagePtr cv_ptr;
//    try {
//        cv_ptr = cv_bridge::toCvCopy(camera_msg, sensor_msgs::image_encodings::BGR8);
//    }
//    catch (cv_bridge::Exception &e) {
//        ROS_ERROR("cv_bridge exception: %s", e.what());
//        return;
//    }
//    cv::imshow("view", cv_ptr->image);
//    cv::waitKey(1);
//}
//
//
//int main(int argc, char **argv) {
//    // subscribe to the fire spots and the camera pose using message_filters
//    ros::init(argc, argv, "sync_gps_vision");
//    ros::NodeHandle nh;
//
//    message_filters::Subscriber<sensor_msgs::NavSatFix> GPS_sub
//            (nh, "/dji_osdk_ros/gps_position", 10);
//    message_filters::Subscriber<sensor_msgs::Image> camera_sub
//            (nh, "/dji_osdk_ros/main_wide_RGB", 10);
//    // synchronize the fire spots, the camera pose and the GPS
//    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::NavSatFix, sensor_msgs::Image> SyncPolicy;
//    message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), GPS_sub, camera_sub);
//    sync.setMaxIntervalDuration(ros::Duration(1));
////    message_filters::TimeSynchronizer<sensor_msgs::NavSatFix, sensor_msgs::Image>
////            sync(GPS_sub, camera_sub, 10);
//    sync.registerCallback(boost::bind(&callback, _1, _2));
//
//    ros::spin();
//
//    return 0;
//}

class MyClass {
    typedef message_filters::sync_policies::ApproximateTime
            <sensor_msgs::Image, sensor_msgs::NavSatFix> SyncPolicy;
    message_filters::Subscriber<sensor_msgs::Image> *visua_sub_;
    message_filters::Subscriber<sensor_msgs::NavSatFix> *gps_sub_;
//    message_filters::Synchronizer<SyncPolicy> *sync_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_{nullptr};
// method definitions needed here
public:
    MyClass();

private:
    void callbackMethod(const sensor_msgs::ImageConstPtr &visual_img_msg,
                        const sensor_msgs::NavSatFixConstPtr &gps_msg);
    ros::NodeHandle nh_;
};

MyClass::MyClass() {
    std::string visua_tp_("/dji_osdk_ros/main_wide_RGB");
    std::string gps_tp_("/dji_osdk_ros/gps_position");
    int q = 100; //queue size

    visua_sub_ = new message_filters::Subscriber<sensor_msgs::Image>(nh_, visua_tp_, q);
    gps_sub_ = new message_filters::Subscriber<sensor_msgs::NavSatFix>(nh_, gps_tp_, q);

    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(q), *visua_sub_, *gps_sub_);
    sync_->registerCallback(boost::bind(&MyClass::callbackMethod, this, _1, _2));
    ros::Duration(1.0).sleep();
    std::cout << "Created!" << std::endl;
}

//The callback method
void MyClass::callbackMethod(const sensor_msgs::ImageConstPtr &visual_img_msg,
                             const sensor_msgs::NavSatFixConstPtr &gps_msg) {
    //Your processing code
    LOG(INFO) << "The GPS position of the camera: " << gps_msg->latitude << ", " << gps_msg->longitude << ", "
              << gps_msg->altitude << ".";
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(visual_img_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::imshow("view", cv_ptr->image);
    cv::waitKey(1);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "sync_gps_vision");
    MyClass myClass;
    ros::spin();
    return 0;
}