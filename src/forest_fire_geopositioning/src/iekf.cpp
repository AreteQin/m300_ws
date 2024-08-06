#include <opencv2/core/core.hpp>
#include <vector>
#include <matplotlibcpp.h>
#include <glog/logging.h>
#include <fstream>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace plt = matplotlibcpp;

std::vector<Eigen::Vector3d> gps, gps_calculated, gps_filtered;

Eigen::Vector3d f_process(Eigen::Vector3d x)
{
    return x;
}

Eigen::Vector3d h_measure(Eigen::Vector3d x)
{
    return x;
}

Eigen::Matrix3d F_jacobian(Eigen::Vector3d x)
{
    return Eigen::Matrix3d::Identity();
}

Eigen::Matrix3d H_jacobian(Eigen::Vector3d x)
{
    return Eigen::Matrix3d::Identity();
}

Eigen::Vector3d IEKF(const Eigen::Vector3d& x, const Eigen::Vector3d& y, const Eigen::Matrix3d& Q,
                     const Eigen::Matrix3d& R, Eigen::Matrix3d& P, int iteration_times)
{
    // predict
    // prior estimate
    Eigen::Vector3d est_prior = f_process(x);
    Eigen::Matrix3d df = F_jacobian(est_prior);
    Eigen::Matrix3d dh = H_jacobian(est_prior);
    // prior covariance
    P = df * P * df.transpose() + Q;
    // update
    // kalman gain
    Eigen::Matrix3d K = P * dh.transpose() * (dh * P * dh.transpose() + R).inverse();
    // posterior estimate
    Eigen::Vector3d est_posterior = est_prior + K * (y - h_measure(est_prior));
    for (int j = 0; j < iteration_times; j++)
    {
        dh = H_jacobian(est_posterior);
        K = P * dh.transpose() * (dh * P * dh.transpose() + R).inverse();
        est_posterior = est_prior + K * (y - h_measure(est_posterior)-dh*(est_prior-est_posterior));
    }
    // posterior covariance
    P = P - K * dh * P;
    // P = (Eigen::Matrix3d::Identity() - K * dh) * P * (Eigen::Matrix3d::Identity() - K * dh).transpose() +
    //     K * R * K.transpose();
    return est_posterior;
}

int main()
{
    // read the gps data from files
    std::ifstream gps_file("camera_poses_GPS.txt");
    // save latitude, longitude and altitude to the vector gps
    std::string line;
    while (std::getline(gps_file, line))
    {
        std::istringstream iss(line);
        double latitude, longitude, altitude;
        if (!(iss >> latitude >> longitude >> altitude))
        {
            break;
        }
        Eigen::Vector3d gps_temp;
        gps_temp << latitude, longitude, altitude;
        gps.push_back(gps_temp);
    }

    // read the calculated gps data from files
    std::ifstream gps_calculated_file("camera_poses_GPS_calculated.txt");
    // save latitude, longitude and altitude to the vector gps
    while (std::getline(gps_calculated_file, line))
    {
        std::istringstream iss(line);
        double latitude, longitude, altitude;
        if (!(iss >> latitude >> longitude >> altitude))
        {
            break;
        }
        Eigen::Vector3d gps_temp;
        gps_temp << latitude, longitude, altitude;
        gps_calculated.push_back(gps_temp);
    }
    LOG(INFO) << "The size of gps is: " << gps.size();
    LOG(INFO) << "The size of gps_calculated is: " << gps_calculated.size();

    int iteration_times = 5;
    Eigen::Matrix3d Q = Eigen::Matrix3d::Identity(); // state noise covariance
    Eigen::Matrix3d R = 5 * Eigen::Matrix3d::Identity(); // measurement noise covariance
    Eigen::Matrix3d P = Q;
    std::vector<Eigen::Vector3d> est_IEKF;
    std::vector<double> time_stamp;
    time_stamp.push_back(0);
    for (int i = 0; i < gps.size(); i++)
    {
        time_stamp.push_back(i);
        est_IEKF.push_back(IEKF(gps_calculated[i], gps[i], Q, R, P, iteration_times));
    }

    // plot the 2D map
    std::vector<double> longitude, latitude;
    for (const Eigen::Vector3d& gps_temp : gps)
    {
        longitude.push_back(gps_temp[1]);
        latitude.push_back(gps_temp[0]);
    }
    plt::plot(longitude, latitude, "bo");
    longitude.clear();
    latitude.clear();
    for (const Eigen::Vector3d& gps_temp : gps_calculated)
    {
        longitude.push_back(gps_temp[1]);
        latitude.push_back(gps_temp[0]);
    }
    plt::plot(longitude, latitude, "go");
    longitude.clear();
    latitude.clear();
    for (const Eigen::Vector3d& gps_temp : est_IEKF)
    {
        longitude.push_back(gps_temp[1]);
        latitude.push_back(gps_temp[0]);
    }
    plt::plot(longitude, latitude, "ro");
    plt::show();
    return 0;
}
