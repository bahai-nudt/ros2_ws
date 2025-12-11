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



    // utm 转 lla
   try {
    // Miscellaneous conversions
    double lat = 39.9, lon = 116.4;
    GeoCoords c(50, true, 448709, 4416831);
    cout << c.Latitude() << "\n";
    cout << c.Longitude() << "\n";
    // c.Reset("18TWN0050");
    // cout << c.DMSRepresentation() << "\n";
    // cout << c.Latitude() << " " << c.Longitude() << "\n";
    // c.Reset("1d38'W 55d30'N");
    // cout << c.GeoRepresentation() << "\n";
  }
  catch (const exception& e) {
    cerr << "Caught exception: " << e.what() << "\n";
    return 1;
  }


    // lla 转 utm
//   try {
//     // Miscellaneous conversions
//     double lat = 39.9, lon = 116.4;
//     GeoCoords c(lat, lon);
//     cout << c.UTMUPSRepresentation(true) << "\n";
//     // c.Reset("18TWN0050");
//     // cout << c.DMSRepresentation() << "\n";
//     // cout << c.Latitude() << " " << c.Longitude() << "\n";
//     // c.Reset("1d38'W 55d30'N");
//     // cout << c.GeoRepresentation() << "\n";
//   }
//   catch (const exception& e) {
//     cerr << "Caught exception: " << e.what() << "\n";
//     return 1;
//   }


// lla 转 enu， enu 转 lla

    // double lat0 = 39.9041; // 纬度
    // double lon0 = 116.4071; // 经度
    // double height0 = 0; // 高程（米）

    // // 创建 LocalCartesian 对象，表示 ENU 坐标系
    // LocalCartesian local(lat0, lon0, height0);

    // // ECEF 坐标（假设）
    // double lat = 39.9042; // 纬度
    // double lon = 116.4074; // 经度
    // double height = 0; // 高程（米）

    // // 转换 ECEF 到 ENU 坐标
    // double east, north, up;
    // local.Forward(lat, lon, height, east, north, up);

    // // 输出结果
    // std::cout << "ENU Coordinates: " << std::endl;
    // std::cout << "East: " << east << std::endl;
    // std::cout << "North: " << north << std::endl;
    // std::cout << "Up: " << up << std::endl;

    return 0;










}