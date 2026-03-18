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

  // double lon = 116.4;
  // double lat = 39.9;
  // std::vector<double> utm_xyz = Coordinate::lla2utm(lon, lat, 0);

  // std::cout << utm_xyz[0] << std::endl;
  // std::cout << utm_xyz[1] << std::endl;
  // std::cout << utm_xyz[2] << std::endl;

  // // utm 转 lla
  // std::vector<double> lla;
  // lla = Coordinate::utm2lla(448709, 4416830, 0, 50);

  // std::cout << lla[0] << std::endl;
  // std::cout << lla[1] << std::endl;
  // std::cout << lla[2] << std::endl;


  
  // lla 转 enu， enu 转 lla



    
    


  double lon0 = 118.216753093;
  double lat0 = 41.14418117;
  std::vector<double> enu = Coordinate::enu2lla(lon0, lat0, 686.148, -1173.54, 293.939, 12.808);

  std::cout << std::setprecision(10) << enu[0] << std::endl;
  std::cout << std::setprecision(10) << enu[1] << std::endl;
  std::cout << std::setprecision(10) << enu[2] << std::endl;

    return 0;










}