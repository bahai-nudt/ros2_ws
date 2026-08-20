#include <cmath>
#include <iostream>

#include "math/coordinate.h"

namespace {

bool Vec3Near(const Eigen::Vector3d& actual, const Eigen::Vector3d& expected, double tolerance) {
    return (actual - expected).cwiseAbs().maxCoeff() <= tolerance;
}

bool TestLlaXyzRoundTrip() {
    const Eigen::Vector3d lla(0.7, 1.2, 100.0);  // [lat, lon, h] 弧度
    const Eigen::Vector3d back = msf::Coordinate::xyz2lla(msf::Coordinate::lla2xyz(lla));
    return Vec3Near(back, lla, 1e-9);
}

bool TestLlaEnuRoundTrip() {
    const Eigen::Vector3d origin(0.7, 1.2, 100.0);
    const Eigen::Vector3d lla(0.7005, 1.201, 150.0);
    const Eigen::Vector3d back = msf::Coordinate::enu2lla(origin, msf::Coordinate::lla2enu(origin, lla));
    return Vec3Near(back, lla, 1e-8);
}

bool TestUtmRoundTrip() {
    const Eigen::Vector3d lla(0.7, 1.2, 100.0);
    const msf::UtmCoord utm = msf::Coordinate::lla2utm(lla);
    const Eigen::Vector3d back = msf::Coordinate::utm2lla(utm);
    return utm._zone > 0 && Vec3Near(back, lla, 1e-6);
}

bool TestEnuEastIncreasesLongitude() {
    // 赤道附近东移 100 m，经度应增大
    const Eigen::Vector3d origin(0.0, 1.2, 0.0);
    const Eigen::Vector3d lla = msf::Coordinate::enu2lla(origin, Eigen::Vector3d(100.0, 0.0, 0.0));
    return lla(1) > origin(1);
}

}  // namespace

int main() {
    struct TestCase {
        const char* name;
        bool (*fn)();
    };

    const TestCase tests[] = {
        {"lla_xyz_round_trip", TestLlaXyzRoundTrip},
        {"lla_enu_round_trip", TestLlaEnuRoundTrip},
        {"utm_round_trip", TestUtmRoundTrip},
        {"enu_east_increases_longitude", TestEnuEastIncreasesLongitude},
    };

    int failed = 0;
    for (const auto& test : tests) {
        if (!test.fn()) {
            std::cerr << "[FAIL] " << test.name << std::endl;
            ++failed;
        } else {
            std::cout << "[PASS] " << test.name << std::endl;
        }
    }

    return failed == 0 ? 0 : 1;
}
