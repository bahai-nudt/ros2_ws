#pragma once

#include <Eigen/Core>

namespace msf {

/** UTM 坐标（带带号/半球） */
struct UtmCoord {
    double _easting = 0.0;   // [m]
    double _northing = 0.0;  // [m]
    double _height = 0.0;    // [m]
    int _zone = 0;
    bool _north = true;
};

/**
 * 坐标转换统一入口（core 唯一实现，基于 GeographicLib）。
 * 约定：lla 一律 [lat(rad), lon(rad), h(m)]，xyz 为 ECEF [m]，enu 为地理 ENU [m]。
 * 度只允许出现在配置 / 输出边界（由 io / app 负责换算）。
 */
class Coordinate {
public:
    static Eigen::Vector3d lla2xyz(const Eigen::Vector3d& lla);
    static Eigen::Vector3d xyz2lla(const Eigen::Vector3d& xyz);
    static Eigen::Vector3d lla2enu(const Eigen::Vector3d& lla_origin, const Eigen::Vector3d& lla);
    static Eigen::Vector3d enu2lla(const Eigen::Vector3d& lla_origin, const Eigen::Vector3d& enu);
    static UtmCoord lla2utm(const Eigen::Vector3d& lla);
    static Eigen::Vector3d utm2lla(const UtmCoord& utm);
};

}  // namespace msf
