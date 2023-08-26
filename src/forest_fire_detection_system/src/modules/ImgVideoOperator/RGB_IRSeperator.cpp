#include <glog/logging.h>
#include <modules/ImgVideoOperator/RGB_IRSeperator.hpp>
#include <dji_camera_image.hpp>

FFDS::MODULES::RGB_IRSeperator::RGB_IRSeperator() {
//    image_transport::ImageTransport it(nh);
//    image_transport::Subscriber sub_color = it.subscribe("dji_osdk_ros/main_camera_images", 1,
//                                                         RGB_IRSeperator::imageCallback);
//    auto main_camera_stream_sub = nh.subscribe("dji_osdk_ros/main_camera_images", 10, &RGB_IRSeperator::imageCallback, this);
    imgSub = nh.subscribe("dji_osdk_ros/main_camera_images", 1, &RGB_IRSeperator::imageCallback, this);
    imgIRPub = it.advertise("forest_fire_detection_system/main_camera_ir_image", 1);
    imgRGBPub = it.advertise("forest_fire_detection_system/main_camera_rgb_image", 1);
    resizeImgRGBPub = it.advertise("forest_fire_detection_system/main_camera_rgb_resize_image", 1);

    ros::Duration(2.0).sleep();
}

void FFDS::MODULES::RGB_IRSeperator::imageCallback(
        const sensor_msgs::Image::ConstPtr &img) {
    rawImgPtr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    rawImg = rawImgPtr->image;
}

void FFDS::MODULES::RGB_IRSeperator::run() {
    const std::string package_path =
            ros::package::getPath("dji_osdk_ros");
    const std::string config_path = package_path + "/config/H20T_Camera.yaml";
    PRINT_INFO("get camera params from %s", config_path.c_str());
    YAML::Node node = YAML::LoadFile(config_path);

    int irImgWid = FFDS::TOOLS::getParam(node, "pure_IR_width", 960);
    int irImgHet = FFDS::TOOLS::getParam(node, "pure_IR_height", 770);

    int rgbImgWid = FFDS::TOOLS::getParam(node, "pure_RGB_width", 960);
    int rgbImgHet = FFDS::TOOLS::getParam(node, "pure_RGB_height", 770);

    int upperBound = FFDS::TOOLS::getParam(node, "upper_bound", 336);
    int lowerBound = FFDS::TOOLS::getParam(node, "lower_bound", 1106);

    int irUpLeft_x = 0;
    int irUpLeft_y = upperBound;

    int rgbUpLeft_x = irImgWid;
    int rgbUpLeft_y = upperBound;

    /**
     * FIXED: the hh DJI change the video resolution after press the "RECORD" button
     * */

    while (ros::ok()) {
        ros::spinOnce();
        if (rawImg.empty()) {
            LOG(WARNING) << "raw image is empty!";
            ros::Duration(0.1).sleep();
            continue;
        }

        LOG(INFO) << "Org mixed image shape: rows: " << rawImg.rows << ", cols: " << rawImg.cols;
//        LOG(INFO) << "ir image position: rows: " << irUpLeft_x
//                  << ", cols: " << irUpLeft_y;
//        LOG(INFO) << "ir image shape: rows: " << irImgWid
//                  << ", cols: " << irImgHet;
//        LOG(INFO) << "rgb image position: rows: " << rgbUpLeft_x
//                  << ", cols: " << rgbUpLeft_y;
//        LOG(INFO) << "rgb image shape: rows: " << rgbImgWid
//                  << ", cols: " << rgbImgHet;

        cv::Mat irImg =
                rawImg(cv::Rect(irUpLeft_x, irUpLeft_y, irImgWid, irImgHet));

        cv::Mat rgbImg =
                rawImg(cv::Rect(rgbUpLeft_x, rgbUpLeft_y, rgbImgWid, rgbImgHet));

        cv::Mat resizeRgbImg;
        cv::resize(rgbImg, resizeRgbImg, cv::Size(resRGBWid, resRGBHet));

        sensor_msgs::ImagePtr irMsg =
                cv_bridge::CvImage(std_msgs::Header(), "bgr8", irImg).toImageMsg();
        sensor_msgs::ImagePtr rgbMsg =
                cv_bridge::CvImage(std_msgs::Header(), "bgr8", rgbImg).toImageMsg();
        sensor_msgs::ImagePtr reszieRgbMsg =
                cv_bridge::CvImage(std_msgs::Header(), "bgr8", resizeRgbImg)
                        .toImageMsg();

        irMsg->header.frame_id = "H20T_IR";
        irMsg->header.stamp = ros::Time::now();

        rgbMsg->header.frame_id = "H20T_RGB";
        rgbMsg->header.stamp = irMsg->header.stamp;

        reszieRgbMsg->header.frame_id = "H20T_RGB_RESIZE";
        reszieRgbMsg->header.stamp = irMsg->header.stamp;

        imgIRPub.publish(irMsg);
        imgRGBPub.publish(rgbMsg);
        resizeImgRGBPub.publish(reszieRgbMsg);

        ros::Rate(10).sleep();
    }
}
