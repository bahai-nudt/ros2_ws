#include <iostream>
#include <proj.h>
#include <vector>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry> 

class Coordinate {
public:
    static std::vector<double> lla2utm(double lon, double lat, double height, bool north = true);
    static std::vector<double> utm2lla(double utm_x, double utm_y, double utm_z, int utm_zone, bool north = true);
    static std::vector<double> lla2enu(double lon0, double lat0, double height0, double lon, double lat, double height);
    static std::vector<double> enu2lla(double lon0, double lat0, double height0, double x, double y, double z);


};
