#include "base_utils/coordinate.h"
#include <iomanip>





#include <iostream>
#include <exception>
#include <cmath>
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/GeoCoords.hpp>
 
using namespace std;
using namespace GeographicLib;
 
int main() {

  double lon = 116.4;
  double lat = 39.9;
  std::vector<double> utm_xyz = Coordinate::lla2utm(lon, lat, 0);

  std::cout << utm_xyz[0] << std::endl;
  std::cout << utm_xyz[1] << std::endl;
  std::cout << utm_xyz[2] << std::endl;

  // utm 转 lla
  std::vector<double> lla;
  lla = Coordinate::utm2lla(448709, 4416830, 0, 50);

  std::cout << lla[0] << std::endl;
  std::cout << lla[1] << std::endl;
  std::cout << lla[2] << std::endl;


  
  // lla 转 enu， enu 转 lla
  double lon0 = 116.41;
  double lat0 = 39.91;
  std::vector<double> enu = Coordinate::lla2enu(lon0, lat0, 0, lon, lat, 0);

  std::cout << enu[0] << std::endl;
  std::cout << enu[1] << std::endl;
  std::cout << enu[2] << std::endl;

    return 0;










}