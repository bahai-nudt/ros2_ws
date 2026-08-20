#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "gnss_initializer.h"
#include "gnss_quality.h"
#include "math/constants.h"

namespace {

bool Near(double actual, double expected, double tolerance = 1e-6) {
    return std::abs(actual - expected) <= tolerance;
}

bool TestAllowedGnssPos() {
    msf::GnssPos ok;
    ok._pos_type = 50;
    ok._blh = msf::constants::_D2R * Eigen::Vector3d(30.0, 120.0, 0.0);
    ok._blh_std = Eigen::Vector3d::Ones();
    if (!msf::gnss::AllowedGnssPos(ok)) {
        return false;
    }

    msf::GnssPos bad_type = ok;
    bad_type._pos_type = 16;
    if (msf::gnss::AllowedGnssPos(bad_type)) {
        return false;
    }

    msf::GnssPos bad_std = ok;
    bad_std._blh_std = Eigen::Vector3d::Zero();
    if (msf::gnss::AllowedGnssPos(bad_std)) {
        return false;
    }

    msf::GnssPos bad_lat = ok;
    bad_lat._blh(0) = 0.5 * msf::constants::_D2R;  // 0.5 deg 超出范围
    return !msf::gnss::AllowedGnssPos(bad_lat);
}

bool TestAllowedGnssVel() {
    msf::GnssVel ok;
    ok._vel_type = 50;
    ok._hor_speed = 5.0;
    ok._trk_gnd = 1.0;
    ok._ver_speed = 0.0;
    if (!msf::gnss::AllowedGnssVel(ok)) {
        return false;
    }

    msf::GnssVel bad_type = ok;
    bad_type._vel_type = 0;
    if (msf::gnss::AllowedGnssVel(bad_type)) {
        return false;
    }

    msf::GnssVel bad_trk = ok;
    bad_trk._trk_gnd = 2.0 * msf::constants::_PI + 0.1;
    return !msf::gnss::AllowedGnssVel(bad_trk);
}

bool TestAllowedHeading() {
    msf::HeadingData ok;
    ok._pos_type = 50;
    ok._heading = 1.0;
    ok._heading_std = 0.1;
    ok._baseline_length = 2.0;
    if (!msf::gnss::AllowedHeading(ok)) {
        return false;
    }

    msf::HeadingData bad_std = ok;
    bad_std._heading_std = 0.0;
    if (msf::gnss::AllowedHeading(bad_std)) {
        return false;
    }

    msf::HeadingData bad_base = ok;
    bad_base._baseline_length = 0.0;
    return !msf::gnss::AllowedHeading(bad_base);
}

bool TestVelRejectReason() {
    msf::GnssVel vel;
    vel._vel_type = 0;
    vel._hor_speed = 5.0;
    vel._trk_gnd = 1.0;
    vel._ver_speed = 0.0;
    if (msf::gnss::GnssVelRejectReason(vel, 0.0) != "vel_type_no_solution") {
        return false;
    }

    vel._vel_type = 50;
    if (msf::gnss::GnssVelRejectReason(vel, 0.05) != "time_sync_failed") {
        return false;
    }
    return msf::gnss::GnssVelRejectReason(vel, 0.0) == "ok";
}

bool TestInitializeNominal() {
    std::vector<msf::ImuData> imu;
    for (int i = 0; i <= 100; ++i) {
        msf::ImuData d;
        d._timestamp = i * 0.01;
        d._dt = (i == 0) ? 0.0 : 0.01;
        d._gyro = Eigen::Vector3d(0.001, -0.002, 0.005);
        d._accel = Eigen::Vector3d(0.1, 0.2, 9.8);
        imu.push_back(d);
    }

    std::vector<msf::GnssPos> pos;
    for (int i = 0; i <= 10; ++i) {
        msf::GnssPos p;
        p._timestamp = i * 0.1;
        p._sol_status = 0;
        p._pos_type = 50;
        p._blh = msf::constants::_D2R * Eigen::Vector3d(30.0, 120.0, 0.0);
        p._blh(2) = 100.0;
        p._blh_std = Eigen::Vector3d::Ones();
        pos.push_back(p);
    }

    std::vector<msf::GnssVel> vel;
    msf::GnssVel v;
    v._timestamp = 1.0;
    v._vel_type = 50;
    v._hor_speed = 5.0;
    v._trk_gnd = 90.0 * msf::constants::_D2R;
    v._ver_speed = 0.0;
    vel.push_back(v);

    std::vector<msf::HeadingData> heading;
    for (int i = 0; i < 3; ++i) {
        msf::HeadingData h;
        h._timestamp = 0.8 + i * 0.1;
        h._sol_status = 0;
        h._pos_type = 50;
        h._baseline_length = 2.0;
        h._heading = 90.0 * msf::constants::_D2R;
        h._heading_std = 0.1 * msf::constants::_D2R;
        heading.push_back(h);
    }

    msf::NominalState nominal;
    msf::earth eth;
    if (!msf::gnss::InitializeNominal(imu, pos, vel, heading, Eigen::Vector3d::Zero(),
                                      msf::GlobalConfig::ImuCalib{}, nominal, eth)) {
        return false;
    }

    // heading=90° -> yaw=0；lever=0 -> pos=GNSS；v=5m/s 正东 -> vn=[5,0,0]
    return Near(nominal._t_cur, 1.0) &&
           Near(nominal._att(2), 0.0) &&
           Near(nominal._pos(0), 30.0 * msf::constants::_D2R) &&
           Near(nominal._pos(1), 120.0 * msf::constants::_D2R) &&
           Near(nominal._pos(2), 100.0) &&
           Near(nominal._vn(0), 5.0) &&
           Near(nominal._vn(1), 0.0, 1e-6) &&
           eth._pos.isApprox(nominal._pos);
}

}  // namespace

int main() {
    struct TestCase {
        const char* name;
        bool (*fn)();
    };

    const TestCase tests[] = {
        {"allowed_gnss_pos", TestAllowedGnssPos},
        {"allowed_gnss_vel", TestAllowedGnssVel},
        {"allowed_heading", TestAllowedHeading},
        {"vel_reject_reason", TestVelRejectReason},
        {"initialize_nominal", TestInitializeNominal},
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
