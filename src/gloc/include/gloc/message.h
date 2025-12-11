#ifndef GLOC_MESSAGE_H
#define GLOC_MESSAGE_H

#include <Eigen/Dense>

struct ImuData {
    double _timestamp;

    Eigen::Vector3d _accel;
    Eigen::Vector3d _gyro;
};

struct GpsData {
    double _timestamp;

    Eigen::Vector3d _lla;
    Eigen::Vector3d _position;
    Eigen::Vector3d _velocity;

    Eigen::Matrix3d _cov;
    int _status;
};

struct Heading {
    double _timestamp;

    double _heading; // rad
};

struct State {
    Eigen::Vector3d _position;
    Eigen::Vector3d _velocity;
    Eigen::Matrix3d _rotation;
    Eigen::Vector3d _bias_accel;
    Eigen::Vector3d _bias_gyro;

    Eigen::Matrix<double, 15, 15> _cov;
    double _timestamp;
};


#endif