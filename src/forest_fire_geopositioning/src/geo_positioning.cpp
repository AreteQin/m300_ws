//
// Created by qin on 10/11/23.
//

#include <matplotlibcpp.h>
namespace plt = matplotlibcpp;

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <glog/logging.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <pangolin/pangolin.h>

// Constants for WGS84 ellipsoid
const double a = 6378137.0; // Semi-major axis (meters)
const double e2 = 0.00669437999014; // Square of eccentricity
const double b = 6356752.314245; // Semi-minor axis (meters)

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

class GeoPositioning
{
public:
    GeoPositioning(ros::NodeHandle& nh)
    {
        real_scale_pub = nh.advertise<sensor_msgs::ChannelFloat32>("/position/real_scale", 10);
        fire_spots_GPS_pub = nh.advertise<geometry_msgs::PoseArray>("/position/fire_spots_GPS", 10);

        fire_spots_sub.subscribe(nh, "/position/fire_spots", 10);
        camera_pose_sub.subscribe(nh, "/position/camera_pose", 10);
        GPS_sub.subscribe(nh, "/dji_osdk_ros/gps_position", 10);

        sync.reset(new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), fire_spots_sub, camera_pose_sub,
                                                                   GPS_sub));
        sync->registerCallback(boost::bind(&GeoPositioning::callback, this, _1, _2, _3));
    }

    int size()
    {
        return camera_trajectory_SLAM.size();
    }

    std::vector<geometry_msgs::PoseStamped>* getCameraPosesSLAM()
    {
        return &camera_trajectory_SLAM;
    }

    std::vector<sensor_msgs::NavSatFix>* getCameraPosesGPS()
    {
        return &camera_trajectory_GPS;
    }

    geometry_msgs::PoseArray* getFireSpotsSLAM()
    {
        return &fire_spots_SLAM;
    }

    geometry_msgs::PoseArray* getFireSpotsGPS()
    {
        return &fire_spots_GPS;
    }

    std::vector<Eigen::Vector3d>* getCameraPosesGPSCalculated()
    {
        return &camera_trajectory_GPS_calculated;
    }

    std::vector<double>* getRealScales()
    {
        return &real_scales;
    }

private:
    ros::Publisher real_scale_pub, fire_spots_GPS_pub;
    message_filters::Subscriber<geometry_msgs::PoseArray> fire_spots_sub;
    message_filters::Subscriber<geometry_msgs::PoseStamped> camera_pose_sub;
    message_filters::Subscriber<sensor_msgs::NavSatFix> GPS_sub;
    typedef message_filters::sync_policies::ApproximateTime<
        geometry_msgs::PoseArray, geometry_msgs::PoseStamped, sensor_msgs::NavSatFix> MySyncPolicy;
    boost::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync;

    std::vector<geometry_msgs::PoseStamped> camera_trajectory_SLAM;
    std::vector<sensor_msgs::NavSatFix> camera_trajectory_GPS;
    geometry_msgs::PoseArray fire_spots_SLAM;
    geometry_msgs::PoseArray fire_spots_GPS;
    std::vector<Eigen::Vector3d> camera_trajectory_GPS_calculated;
    std::vector<double> real_scales;
    double scales_sum = 0;
    int origin_frame_index = 800;

    void callback(const geometry_msgs::PoseArrayConstPtr& fire_spots_msg,
                  const geometry_msgs::PoseStampedConstPtr& camera_pose_msg,
                  const sensor_msgs::NavSatFixConstPtr& GPS_msg)
    {
        LOG(INFO) << "The number of fire spots: " << fire_spots_msg->poses.size() << ".";
        // store camera pose
        //    if (rest_map) {
        //        camera_poses_SLAM.clear();
        //        camera_poses_GPS.clear();
        //        rest_map = false;
        //    }
        camera_trajectory_SLAM.push_back(*camera_pose_msg);
        camera_trajectory_GPS.push_back(*GPS_msg);

        LOG(INFO) << "Total number of camera poses: " << camera_trajectory_SLAM.size() << ".";
        LOG(INFO) << "Total number of GPS positions: " << camera_trajectory_GPS.size() << ".";

        // print the latest camera pose and gps
        std::cout << std::setprecision(std::numeric_limits<double>::digits10 + 1) << "The latest camera pose: " <<
            camera_pose_msg->pose.position.x << ", "
            << camera_pose_msg->pose.position.y << ", " << camera_pose_msg->pose.position.z << std::endl;
        std::cout << "The latest GPS position: " << GPS_msg->latitude << ", " << GPS_msg->longitude << ", "
            << GPS_msg->altitude << std::endl;

        // calculate the real scale
        if (camera_trajectory_SLAM.size() < origin_frame_index * 1.1)
        {
            return;
        }
        // calculate the distance between the camera poses between the 800th and the last frame.
        double distance = Eigen::Vector3d(
                camera_trajectory_SLAM.back().pose.position.y - camera_trajectory_SLAM[origin_frame_index].pose.position
                .y,
                camera_trajectory_SLAM.back().pose.position.x - camera_trajectory_SLAM[origin_frame_index].pose.position
                .x,
                camera_trajectory_SLAM.back().pose.position.z - camera_trajectory_SLAM[origin_frame_index].pose.position
                .z).
            norm();
        double x1, y1, z1, x2, y2, z2;
        GPS2ECEF(camera_trajectory_GPS[origin_frame_index].latitude,
                 camera_trajectory_GPS[origin_frame_index].longitude,
                 camera_trajectory_GPS[origin_frame_index].altitude, x1, y1, z1);
        GPS2ECEF(camera_trajectory_GPS.back().latitude, camera_trajectory_GPS.back().longitude,
                 camera_trajectory_GPS.back().altitude, x2, y2, z2);
        double distance_lat = x2 - x1;
        double distance_lon = y2 - y1;
        double distance_alt = z2 - z1;
        double distance_ECEF = sqrt(
            distance_lat * distance_lat + distance_lon * distance_lon + distance_alt * distance_alt);

        // calculate the real scale
        double real_scale = distance_ECEF / distance;
        LOG(INFO) << "The real scale: " << distance_ECEF << " / " << distance << " = " << real_scale << ".";
        // if (real_scale > 0.4)
        // {
        //     return;
        // }
        real_scales.push_back(real_scale);
        // calculate the average real scale
        scales_sum += real_scale;
        real_scale = scales_sum / real_scales.size();
        // publish the real scale
        sensor_msgs::ChannelFloat32 real_scale_msg;
        real_scale_msg.name = "real_scale";
        real_scale_msg.values.push_back(real_scale);
        real_scale_pub.publish(real_scale_msg);

        // calculate the rotation matrix
        // Calculate the cross product of a and b
        Eigen::Vector3d a = real_scale * Eigen::Vector3d(
            camera_trajectory_SLAM.back().pose.position.x - camera_trajectory_SLAM[origin_frame_index].pose.position.x,
            camera_trajectory_SLAM.back().pose.position.y - camera_trajectory_SLAM[origin_frame_index].pose.position.y,
            camera_trajectory_SLAM.back().pose.position.z - camera_trajectory_SLAM[origin_frame_index].pose.position.z);
        Eigen::Vector3d b = Eigen::Vector3d(distance_lat, distance_lon, distance_alt);

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
        Eigen::Matrix3d r = Eigen::Matrix3d::Identity(3, 3) + nx + nx * nx * ((1 - c) / (s * s));
        // calculate the camera pose in ecef coordinate system
        Eigen::Vector3d camera_pose_ECEF_calculated = r * a;
        camera_pose_ECEF_calculated = camera_pose_ECEF_calculated.normalized() * a.norm() + Eigen::Vector3d(x1, y1, z1);
        double drone_lon, drone_lat, drone_alt;
        ECEF2GPS(camera_pose_ECEF_calculated[0], camera_pose_ECEF_calculated[1], camera_pose_ECEF_calculated[2],
                 drone_lat, drone_lon, drone_alt);
        Eigen::Vector3d camera_pose_GPS_calculated(drone_lat, drone_lon, drone_alt);
        camera_trajectory_GPS_calculated.push_back(camera_pose_GPS_calculated);

        // calculate the positions of fire spots in GPS coordinate system
        for (const geometry_msgs::Pose& fire_spot : fire_spots_msg->poses)
        {
            Eigen::Vector3d fire_spot_SLAM(
                fire_spot.position.x - camera_trajectory_SLAM[origin_frame_index].pose.position.x,
                fire_spot.position.y - camera_trajectory_SLAM[origin_frame_index].pose.position.y,
                fire_spot.position.z - camera_trajectory_SLAM[origin_frame_index].pose.position.z);
            double fire_norm = fire_spot_SLAM.norm();
            Eigen::Vector3d fire_ecef = r * fire_spot_SLAM;
            fire_ecef = fire_ecef.normalized() * real_scales.back() * fire_norm + Eigen::Vector3d(x1, y1, z1);
            Eigen::Vector3d fire_gps;
            ECEF2GPS(fire_ecef[0], fire_ecef[1], fire_ecef[2], fire_gps[0], fire_gps[1], fire_gps[2]);
            geometry_msgs::Pose pose;
            pose.position.x = fire_gps[0] + 0.000035;
            pose.position.y = fire_gps[1] - 0.00004;
            pose.position.z = fire_gps[2] - 19;
            fire_spots_GPS.poses.push_back(pose);
        }
        // publish the GPS positions of fire spots
        fire_spots_GPS_pub.publish(fire_spots_GPS);
        fire_spots_SLAM = *fire_spots_msg;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "geo_positioning");
    ros::NodeHandle nh;
    GeoPositioning geoPositioning(nh);
    while (ros::ok())
    {
        ros::spinOnce();
        if (geoPositioning.size() > 1700)
        {
            break;
        }
        geoPositioning.getFireSpotsGPS()->poses.clear();
    }

    // draw the ground truth of fire spots in the 2D map in red and the drone in blue in GPS coordinate system
    std::vector<double> longitude, latitude;
    for (const sensor_msgs::NavSatFix& drone_GPS : *geoPositioning.getCameraPosesGPS())
    {
        longitude.push_back(drone_GPS.longitude);
        latitude.push_back(drone_GPS.latitude);
    }
    // set the boundary of the 2D map
    plt::plot(longitude, latitude, "bo");
    // plot the ground truth of fire spots
    longitude.clear();
    latitude.clear();
    latitude.push_back(45.458047453);
    longitude.push_back(-73.932875451);
    latitude.push_back(45.458046062);
    longitude.push_back(-73.932882159);
    latitude.push_back(45.458050887);
    longitude.push_back(-73.932884099);
    latitude.push_back(45.458049803);
    longitude.push_back(-73.932890364);
    plt::plot(longitude, latitude, "ro");

    // draw all the fire spots in the 2D map in red and the drone in blue
    longitude.clear();
    latitude.clear();
    int points_kept = 6000;
    for (int i = geoPositioning.getFireSpotsGPS()->poses.size() - points_kept; i < geoPositioning.getFireSpotsGPS()->poses.
         size()
         ; i++)
    {
        longitude.push_back(geoPositioning.getFireSpotsGPS()->poses[i].position.y);
        latitude.push_back(geoPositioning.getFireSpotsGPS()->poses[i].position.x);
    }
    // draw with green color
    plt::plot(longitude, latitude, "go");
    longitude.clear();
    latitude.clear();
    for (const Eigen::Vector3d& drone_GPS : *geoPositioning.getCameraPosesGPSCalculated())
    {
        longitude.push_back(drone_GPS[1]);
        latitude.push_back(drone_GPS[0]);
    }
    // draw with yellow color
    plt::plot(longitude, latitude, "yo");
    // plt::ylim(45.4578, 45.4581);
    // plt::xlim(-73.9329, -73.9326);
    plt::show();

    // store the all the fire spots position into a file with highest precision
    std::ofstream file;
    file.open("01_fire_spots_GPS.txt");
    for (int i = geoPositioning.getFireSpotsGPS()->poses.size() - points_kept; i < geoPositioning.getFireSpotsGPS()->poses.
         size()
         ; i++)
    {
        file << std::setprecision(std::numeric_limits<double>::digits10 + 1) << geoPositioning.getFireSpotsGPS()->poses
            [i].position.x << " " << geoPositioning.getFireSpotsGPS()->poses[i].position.y << " " <<
            geoPositioning.getFireSpotsGPS()->poses[i].position.z << std::endl;
    }
    file.close();

    // store all the camera poses in GPS coordinate system into a file
    file.open("02_camera_poses_GPS.txt");
    for (const sensor_msgs::NavSatFix& camera_GPS : *geoPositioning.getCameraPosesGPS())
    {
        file << std::setprecision(std::numeric_limits<double>::digits10 + 1) << camera_GPS.latitude << " " <<
            camera_GPS.longitude << " " << camera_GPS.altitude << std::endl;
    }
    file.close();

    // store all the camera poses in GPS coordinate system calculated into a file
    file.open("03_camera_poses_GPS_calculated.txt");
    for (const Eigen::Vector3d& camera_GPS_calculated : *geoPositioning.getCameraPosesGPSCalculated())
    {
        file << std::setprecision(std::numeric_limits<double>::digits10 + 1) << camera_GPS_calculated[0] << " " <<
            camera_GPS_calculated[1] << " " << camera_GPS_calculated[2] << std::endl;
    }
    file.close();

    // plot the distribution of the real scale
    LOG(INFO) << "The number of real scales: " << geoPositioning.getRealScales()->size() << ".";
    plt::hist(*geoPositioning.getRealScales(), 100);
    plt::show();
    // store all the scales into a file
    file.open("04_real_scales.txt");
    for (double scale : *geoPositioning.getRealScales())
    {
        file << std::setprecision(std::numeric_limits<double>::digits10 + 1) << scale << std::endl;
    }
    file.close();

    return 0;
}
