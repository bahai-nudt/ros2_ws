#include "math/coordinate.h"

#include <GeographicLib/GeoCoords.hpp>
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

#include "math/constants.h"

using namespace GeographicLib;

namespace msf {

namespace {

const Geocentric& Wgs84() {
    static const Geocentric geocentric(Constants::WGS84_a(), Constants::WGS84_f());
    return geocentric;
}

}  // namespace

Eigen::Vector3d Coordinate::lla2xyz(const Eigen::Vector3d& lla) {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    Wgs84().Forward(lla(0) * constants::_R2D, lla(1) * constants::_R2D, lla(2), x, y, z);
    return Eigen::Vector3d(x, y, z);
}

Eigen::Vector3d Coordinate::xyz2lla(const Eigen::Vector3d& xyz) {
    double lat = 0.0;
    double lon = 0.0;
    double h = 0.0;
    Wgs84().Reverse(xyz(0), xyz(1), xyz(2), lat, lon, h);
    return Eigen::Vector3d(lat * constants::_D2R, lon * constants::_D2R, h);
}

Eigen::Vector3d Coordinate::lla2enu(const Eigen::Vector3d& lla_origin, const Eigen::Vector3d& lla) {
    LocalCartesian local(lla_origin(0) * constants::_R2D,
                         lla_origin(1) * constants::_R2D, lla_origin(2));
    double east = 0.0;
    double north = 0.0;
    double up = 0.0;
    local.Forward(lla(0) * constants::_R2D, lla(1) * constants::_R2D, lla(2),
                  east, north, up);
    return Eigen::Vector3d(east, north, up);
}

Eigen::Vector3d Coordinate::enu2lla(const Eigen::Vector3d& lla_origin, const Eigen::Vector3d& enu) {
    LocalCartesian local(lla_origin(0) * constants::_R2D,
                         lla_origin(1) * constants::_R2D, lla_origin(2));
    double lat = 0.0;
    double lon = 0.0;
    double h = 0.0;
    local.Reverse(enu(0), enu(1), enu(2), lat, lon, h);
    return Eigen::Vector3d(lat * constants::_D2R, lon * constants::_D2R, h);
}

UtmCoord Coordinate::lla2utm(const Eigen::Vector3d& lla) {
    GeoCoords c(lla(0) * constants::_R2D, lla(1) * constants::_R2D);
    UtmCoord utm;
    utm._easting = c.Easting();
    utm._northing = c.Northing();
    utm._height = lla(2);
    utm._zone = c.Zone();
    utm._north = c.Northp();
    return utm;
}

Eigen::Vector3d Coordinate::utm2lla(const UtmCoord& utm) {
    GeoCoords c(utm._zone, utm._north, utm._easting, utm._northing);
    return Eigen::Vector3d(c.Latitude() * constants::_D2R,
                           c.Longitude() * constants::_D2R, utm._height);
}

}  // namespace msf
