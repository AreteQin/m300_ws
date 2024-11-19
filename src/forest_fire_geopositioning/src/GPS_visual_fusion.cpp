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
    // print the latest camera pose and gps
    LOG(INFO) << "The latest camera pose: " << camera_pose_msg->pose.position.x << ", "
        << camera_pose_msg->pose.position.y << ", " << camera_pose_msg->pose.position.z << ".";
    LOG(INFO) << "The latest GPS position: " << GPS_msg->latitude << ", " << GPS_msg->longitude << ", "
        << GPS_msg->altitude << ".";
    Eigen::Vector3d camera_pose_inertial(camera_pose_msg->pose.position.x, camera_pose_msg->pose.position.y,
                                         camera_pose_msg->pose.position.z);
    camera_poses_inertial.push_back(camera_pose_inertial);
    camera_poses_gps.push_back(*GPS_msg);

    // convert the GPS position to ECEF coordinate system
    double x, y, z;
    GPS2ECEF(GPS_msg->latitude, GPS_msg->longitude, GPS_msg->altitude, x, y, z);
    Eigen::Vector3d camera_pose_ecef(x, y, z);
    camera_poses_ecef.push_back(camera_pose_ecef);

    double time_interval = GPS_msg->header.stamp.toSec() - camera_poses_gps[camera_poses_gps.size() - 2].header.stamp.
        toSec();
    LOG(INFO) << "The time consumed by GPS: " << time_interval << " seconds.";

    if (camera_poses_inertial.size() < 500)
    {
        return;
    }
    if (camera_poses_inertial.size() == 500)
    {
        // calculate the distance between the camera poses between the first and the last frame.
        double distance = (camera_poses_inertial.back() - camera_poses_inertial[0]).norm();
        // calculate the distance between the camera poses in ECEF coordinate system
        double distance_ecef = (camera_poses_ecef.back() - camera_poses_ecef[0]).norm();
        // calculate the scale
        scale = distance_ecef / distance;
        LOG(INFO) << "The scale: " << scale << ".";

        // calculate the translation from inertial coordinate system to ECEF coordinate system
        translation = camera_poses_ecef[0] - camera_poses_inertial[0];
        LOG(INFO) << "The translation: " << translation[0] << ", " << translation[1] << ", " << translation[2] << ".";

        // calculate the rotation matrix
        Eigen::Vector3d a = camera_poses_inertial.back() - camera_poses_inertial[0];
        Eigen::Vector3d b = camera_poses_ecef.back() - camera_poses_ecef[0];
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
    Eigen::Vector3d camera_pose_ecef_calculated = r * (camera_poses_inertial.back() - camera_poses_inertial[0]);
    camera_pose_ecef_calculated = camera_pose_ecef_calculated.normalized() * (camera_poses_ecef.back() -
        camera_poses_ecef[0]).norm() + translation;
    // convert the camera pose in ECEF coordinate system to GPS coordinate system
    double lat, lon, alt;
    ECEF2GPS(camera_pose_ecef_calculated[0], camera_pose_ecef_calculated[1], camera_pose_ecef_calculated[2],
             lat, lon, alt);
    LOG(INFO)<< "The calculated GPS position: " << lat << ", " << lon << ", " << alt << ".";
    // calculate the difference between the calculated GPS position and the real GPS position
    std::vector<double> diff = {lat - GPS_msg->latitude, lon - GPS_msg->longitude, alt - GPS_msg->altitude};
    LOG(INFO) << "The difference between the calculated GPS position and the real GPS position: " << diff[0] << ", "
        << diff[1] << ", " << diff[2] << ".";
    camera_poses_gps_calculated.push_back(*GPS_msg);
    // plot the calculated and real GPS positions
    std::vector<double> t;
    std::vector<double> latitudes;
    std::vector<double> longitudes;
    std::vector<double> altitudes;
    for (int i = 0; i < camera_poses_gps_calculated.size(); i++)
    {
        t.push_back(i);
        latitudes.push_back(camera_poses_gps_calculated[i].latitude);
        longitudes.push_back(camera_poses_gps_calculated[i].longitude);
        altitudes.push_back(camera_poses_gps_calculated[i].altitude);
    }
    plt::ion();
    plt::plot(t, latitudes, "r");
    plt::plot(t, longitudes, "g");
    plt::plot(t, altitudes, "b");
    plt::show();
    // Sleep for a 0.01 second
    ros::Duration(0.01).sleep();
    // Close the plot
    plt::close();
}

int main(int argc, char** argv)
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

    while (ros::ok())
    {
        ros::spinOnce();
    }

    return 0;
}
