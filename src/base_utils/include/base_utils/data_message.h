#ifndef DATA_MESSAGE_H
#define DATA_MESSAGE_H

#include <vector>
#include <Eigen/Dense>

#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

struct Point {
    double _x;
    double _y;
    double _z;
    double _timestamp;
    double _intensity;
    int _label;

};

struct PointFrame {
    std::vector<Point> _points;
    double _timestamp;
};



struct Pose {
    double timestamp;
    Eigen::Vector3d position;  // x, y, z
    Eigen::Quaterniond orientation;  // 四元数表示姿态
    
    Pose() : timestamp(0), position(Eigen::Vector3d::Zero()), 
             orientation(Eigen::Quaterniond::Identity()) {}
    
    Pose(double t, const Eigen::Vector3d& pos, const Eigen::Quaterniond& orient)
        : timestamp(t), position(pos), orientation(orient) {}


    Pose& operator=(const Pose& other) {
        
        if (this != &other) {  // 防止自赋值
            timestamp = other.timestamp;
            position = other.position;
            orientation = other.orientation;
        }
        
        return *this;
    }
};


struct Ins {
    double _longitude;
    double _latitude;
    double _altitude;
    float _roll;
    float _pitch;
    float _azimuth;
    int _status;
    double _timestamp;
    double _linear_velocity_x;
    double _linear_velocity_y;
    double _linear_velocity_z;

    Pose _pose;
};

struct Gps {

};

struct Imu {
    double _timestamp;
    double _linear_acceleration_x;
    double _linear_acceleration_y;
    double _linear_acceleration_z;
    double _angular_velocity_x;
    double _angular_velocity_y;
    double _angular_velocity_z;
};

struct wheelspeed {

};


struct Gloc {
    double _timestamp;
    double _easting;
    double _northing;
    double _height;

    Eigen::Quaterniond _orientation;


    Gloc() : _timestamp(0), _easting(0), _northing(0), _height(0), 
            _orientation(Eigen::Quaterniond::Identity()) {}
    
    Gloc(double timestamp, double easting, double northing, double height, const Eigen::Quaterniond orient) : 
    _timestamp(0), _easting(0), _northing(0), _height(0), _orientation(orient) {}
};



#endif
