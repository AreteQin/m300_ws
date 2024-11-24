//
// Created by qin on 18/11/24.
//

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <glog/logging.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <matplotlibcpp.h>
#include <boost/thread.hpp>

namespace plt = matplotlibcpp;

// Constants for WGS84 ellipsoid
const double a = 6378137.0; // Semi-major axis (meters)
const double e2 = 0.00669437999014; // Square of eccentricity
const double b = 6356752.314245; // Semi-minor axis (meters)

// positions in ECEF coordinate system
std::vector<Eigen::Vector3d> camera_poses_ecef;
// positions in inertial coordinate system
std::vector<Eigen::Vector3d> camera_poses_inertial;
// positions in GPS coordinate system
std::vector<sensor_msgs::NavSatFix> camera_poses_gps;
std::vector<sensor_msgs::NavSatFix> camera_poses_gps_calculated;

Eigen::Matrix3d r;
double scale;
Eigen::Vector3d translation;
const int origin_frame_index = 50;

boost::mutex data_mutex;

template <typename T>
void GPS2ECEF(const T lat, const T lon, const T alt, T& X, T& Y, T& Z)
{
    T phi = lat * M_PI / 180.0;
    T lambda = lon * M_PI / 180.0;
    T x_z_squar = a * a * b * b / (b * b + a * a * pow(tan(phi), 2));
    T x_z = sqrt(x_z_squar);
    T R = sqrt(x_z_squar * (1 + tan(phi) * tan(phi)));
    Z = (R + alt) * sin(phi);
    X = (R + alt) * cos(phi) * cos(lambda);
    Y = (R + alt) * cos(phi) * sin(lambda);
}

template <typename T>
void ECEF2GPS(const T X, const T Y, const T Z, T& lat, T& lon, T& alt)
{
    lon = atan2(Y, X) * 180 / M_PI;
    lat = atan2(Z, sqrt(X * X + Y * Y)) * 180 / M_PI;
    T phi = lat * M_PI / 180.0;
    T lambda = lon * M_PI / 180.0;
    T x_z_squar = a * a * b * b / (b * b + a * a * pow(tan(phi), 2));
    T R = sqrt(x_z_squar * (1 + tan(phi) * tan(phi)));
    alt = sqrt(X * X + Y * Y + Z * Z) - R;
}

void callback(const geometry_msgs::PoseStampedConstPtr& camera_pose_msg,
              const sensor_msgs::NavSatFixConstPtr& GPS_msg)
{
    boost::mutex::scoped_lock lock(data_mutex);
    // // print the latest camera pose and gps
    // LOG(INFO) << "The latest camera pose: " << camera_pose_msg->pose.position.x << ", "
    //     << camera_pose_msg->pose.position.y << ", " << camera_pose_msg->pose.position.z << ".";
    // LOG(INFO) << "The latest GPS position: " << GPS_msg->latitude << ", " << GPS_msg->longitude << ", "
    //     << GPS_msg->altitude << ".";
    // swap the x and y in SLAM coordinate system since it is todo: why
    Eigen::Vector3d camera_pose_inertial(camera_pose_msg->pose.position.y, camera_pose_msg->pose.position.x,
                                         camera_pose_msg->pose.position.z);
    camera_poses_inertial.push_back(camera_pose_inertial);
    camera_poses_gps.push_back(*GPS_msg);
    // LOG(INFO) << "The size of camera poses: " << camera_poses_inertial.size() << ".";
    // LOG(INFO) << "Last GPS: " << GPS_msg->latitude << ", " << GPS_msg->longitude << ", " << GPS_msg->altitude << ".";

    // convert the GPS position to ECEF coordinate system
    double x, y, z;
    GPS2ECEF(GPS_msg->latitude, GPS_msg->longitude, GPS_msg->altitude, x, y, z);
    Eigen::Vector3d camera_pose_ecef(x, y, z);
    camera_poses_ecef.push_back(camera_pose_ecef);
    // LOG(INFO) << "The size of camera poses in ECEF: " << camera_poses_ecef.size() << ".";
    // LOG(INFO) << "The last ECEF: "<< camera_poses_ecef.back() << ".";

    // double time_interval = GPS_msg->header.stamp.toSec() - camera_poses_gps[camera_poses_gps.size() - 2].header.stamp.
    //     toSec();
    // LOG(INFO) << "The time consumed by GPS: " << time_interval << " seconds.";

    if (camera_poses_inertial.size() < origin_frame_index+1)
    {
        return;
    }
    if (camera_poses_inertial.size() == origin_frame_index+1)
    {
        // LOG(INFO) << "size of gps: " << camera_poses_gps.size();
        // LOG(INFO) << "size of inertial: " << camera_poses_inertial.size();
        // LOG(INFO) << "size of ecef: " << camera_poses_ecef.size();
        // calculate the distance between the camera poses between the first and the last frame.
        double distance = (camera_poses_inertial[origin_frame_index] - camera_poses_inertial[0]).norm();
        // calculate the distance between the camera poses in ECEF coordinate system
        double distance_ecef = (camera_poses_ecef[origin_frame_index] - camera_poses_ecef[0]).norm();
        // LOG(INFO) << "ecef original and 0: " << camera_poses_gps[origin_frame_index] << ", " << camera_poses_gps[0];
        // LOG(INFO) << "ecef original and 0: " << camera_poses_ecef[origin_frame_index] << ", " << camera_poses_ecef[0];
        // calculate the scale
        scale = distance_ecef / distance;
        LOG(INFO) << "The scale: " << scale << ".";

        // calculate the translation from inertial coordinate system to ECEF coordinate system
        translation = camera_poses_ecef[origin_frame_index] - camera_poses_inertial[origin_frame_index];
        LOG(INFO) << "The translation: " << translation[0] << ", " << translation[1] << ", " << translation[2] << ".";

        // calculate the rotation matrix
        Eigen::Vector3d a = scale * (camera_poses_inertial[origin_frame_index] - camera_poses_inertial[0]);
        Eigen::Vector3d b = camera_poses_ecef[origin_frame_index] - camera_poses_ecef[0];
        // calculate the cross product of a and b
        Eigen::Vector3d v = a.cross(b);
        // Compute sine and cosine of the angle between a and b
        double s = v.norm();
        double c = a.dot(b);
        // Construct the skew-symmetric matrix
        Eigen::Matrix3d nx;
        nx << 0, -v[2], v[1],
            v[2], 0, -v[0],
            -v[1], v[0], 0;
        // Calculate the rotation matrix using the Rodrigues formula
        r = Eigen::Matrix3d::Identity(3, 3) + nx + nx * nx * ((1 - c) / (s * s));
    }
    // convert the camera pose in inertial coordinate system to ECEF coordinate system
    Eigen::Vector3d camera_pose_ecef_calculated = r * (camera_poses_inertial.back() - camera_poses_inertial[
        origin_frame_index]);
    camera_pose_ecef_calculated = camera_pose_ecef_calculated.normalized() * (camera_poses_inertial.back() -
        camera_poses_inertial[origin_frame_index]).norm() * scale + translation;
    // convert the camera pose in ECEF coordinate system to GPS coordinate system
    double lat, lon, alt;
    ECEF2GPS(camera_pose_ecef_calculated[0], camera_pose_ecef_calculated[1], camera_pose_ecef_calculated[2],
             lat, lon, alt);
    // LOG(INFO) << "The calculated GPS position: " << lat << ", " << lon << ", " << alt << ".";
    // calculate the difference between the calculated GPS position and the real GPS position
    // std::vector<double> diff = {lat - GPS_msg->latitude, lon - GPS_msg->longitude, alt - GPS_msg->altitude};
    sensor_msgs::NavSatFix camera_pose_gps_calculated;
    camera_pose_gps_calculated.latitude = lat;
    camera_pose_gps_calculated.longitude = lon;
    camera_pose_gps_calculated.altitude = alt;
    camera_poses_gps_calculated.push_back(camera_pose_gps_calculated);
}

void rosNodeFunction(int argc, char** argv)
{
    ros::init(argc, argv, "geo_positioning");
    ros::NodeHandle nh;

    message_filters::Subscriber<geometry_msgs::PoseStamped> camera_pose_sub;
    camera_pose_sub.subscribe(nh, "/position/camera_pose", 10);
    message_filters::Subscriber<sensor_msgs::NavSatFix> GPS_sub;
    GPS_sub.subscribe(nh, "/dji_osdk_ros/gps_position", 10);

    typedef message_filters::sync_policies::ApproximateTime<
        geometry_msgs::PoseStamped, sensor_msgs::NavSatFix> MySyncPolicy;
    boost::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync;

    sync.reset(new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), camera_pose_sub, GPS_sub));
    sync->registerCallback(boost::bind(&callback, _1, _2));

    ros::Rate loop_rate(30);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void matplotlibFunction()
{
    plt::ion(); // Turn on interactive mode for real-time updates

    while (ros::ok())
    {
        // Local copy of shared data
        std::vector<sensor_msgs::NavSatFix> camera_poses_gps_calculated_, camera_poses_gps_;
        {
            boost::mutex::scoped_lock lock(data_mutex);
            camera_poses_gps_calculated_ = camera_poses_gps_calculated;
            camera_poses_gps_ = camera_poses_gps;
        }

        // Check if there is data to plot
        if (!camera_poses_gps_calculated_.empty())
        {
            std::vector<double> latitudes, longitudes;

            // Extract latitude and longitude for plotting
            for (const auto& gps : camera_poses_gps_calculated_)
            {
                latitudes.push_back(gps.latitude);
                longitudes.push_back(gps.longitude);
            }

            // Clear and plot updated data
            plt::clf();
            plt::plot(longitudes, latitudes, "bo-");
            for (const auto& gps : camera_poses_gps_)
            {
                plt::plot({gps.longitude}, {gps.latitude}, "ro");
            }
            plt::title("Real-Time GPS Positions");
            plt::xlabel("Longitude");
            plt::ylabel("Latitude");
            plt::grid(true);

            // Pause to update the plot
            plt::pause(0.03);
        }
        else
        {
            // If no data yet, wait briefly before checking again
            plt::pause(0.03);
        }
    }

    plt::show(); // Keep the final plot displayed when exiting
}


int main(int argc, char** argv)
{
    // Create threads for ROS and matplotlibcpp
    boost::thread rosThread(rosNodeFunction, argc, argv);
    boost::thread matplotlibThread(matplotlibFunction);

    // Wait for both threads to finish
    rosThread.join();
    matplotlibThread.join();

    return 0;
}
