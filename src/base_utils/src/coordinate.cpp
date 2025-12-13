#include "base_utils/coordinate.h"

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/GeoCoords.hpp>
 
#include <boost/algorithm/string.hpp>

using namespace GeographicLib;

std::vector<double> Coordinate::lla2utm(double lon, double lat, double height, bool north) {
    std::vector<double> utm_result;

    GeoCoords c(lat, lon);
    std::string str = c.UTMUPSRepresentation(north);

    std::vector<std::string> result;
    boost::split(result, str, boost::is_any_of(" "));

    utm_result.push_back(std::stod(result[1]));
    utm_result.push_back(std::stod(result[2]));
    utm_result.push_back(height);

    return utm_result;
}
std::vector<double> Coordinate::utm2lla(double utm_x, double utm_y, double utm_z, int utm_zone, bool north) {
    std::vector<double> lla_result;
    GeoCoords c(utm_zone, north, utm_x, utm_y);

    lla_result.push_back(c.Longitude());
    lla_result.push_back(c.Latitude());
    lla_result.push_back(utm_z);

    return lla_result;
}

std::vector<double> Coordinate::lla2enu(double lon0, double lat0, double height0, double lon, double lat, double height) {
    std::vector<double> enu;

    LocalCartesian local(lat0, lon0, height0);

    double east, north, up;
    local.Forward(lat, lon, height, east, north, up);

    enu.push_back(east);
    enu.push_back(north);
    enu.push_back(up);

    return enu;
}
std::vector<double> Coordinate::enu2lla(double lon0, double lat0, double height0, double x, double y, double z) {
    std::vector<double> lla;
    LocalCartesian local(lat0, lon0, height0);

    double lon = 0.0;
    double lat = 0.0;
    double alt = 0.0;
    local.Reverse(x, y, z, lat, lon, alt);

    lla.push_back(lon);
    lla.push_back(lat);
    lla.push_back(alt);

    return lla;
}


