// INS 机械编排 / 粗对准 / 静止初始化快速试验。
#include <cmath>
#include <iostream>
#include <vector>

#include "config/global_config.h"
#include "ins_initializer.h"
#include "ins_propagator.h"
#include "math/earth.h"
#include "math/pose_converter.h"
#include "types/sensors/imu_message.h"

namespace {

constexpr double kDeg2Rad = M_PI / 180.0;

bool Near(double a, double b, double tol) { return std::abs(a - b) <= tol; }

bool TestAlignCoarse() {
    const Eigen::Vector3d pos0(30.0 * kDeg2Rad, 120.0 * kDeg2Rad, 0.0);

    msf::earth eth;
    eth.Update(pos0, Eigen::Vector3d::Zero());

    const Eigen::Vector3d wmm = eth._wnie;
    const Eigen::Vector3d vmm = -eth._gcc;

    msf::InsPropagator ins;
    msf::NominalState nominal;
    nominal._pos = pos0;
    const Eigen::Vector3d att = ins.AlignCoarse(nominal, wmm, vmm);
    std::cout << "[align ] att(deg) = " << (att / kDeg2Rad).transpose() << "\n";
    return Near(att(0), 0.0, 1e-3) && Near(att(1), 0.0, 1e-3) && Near(att(2), 0.0, 1e-3);
}

bool TestStaticPropagation() {
    const Eigen::Vector3d pos0(30.0 * kDeg2Rad, 120.0 * kDeg2Rad, 0.0);

    msf::earth eth;
    eth.Update(pos0, Eigen::Vector3d::Zero());
    const Eigen::Vector3d wnie = eth._wnie;
    const Eigen::Vector3d gcc = eth._gcc;

    msf::InsPropagator ins;
    msf::NominalState nominal;
    nominal._pos = pos0;
    const double dt = 0.01;
    const int steps = 500;

    for (int i = 0; i < steps; ++i) {
        ins.Propagate(nominal, eth, wnie, -gcc, wnie, -gcc, dt);
        if ((i + 1) % 100 == 0) {
            std::cout << "[t=" << nominal._t_cur << "s] "
                      << "vn=" << nominal._vn.transpose()
                      << " att(deg)=" << (nominal._att / kDeg2Rad).transpose()
                      << " dh=" << (nominal._pos(2) - pos0(2)) << "\n";
        }
    }

    const double dlat = std::abs(nominal._pos(0) - pos0(0));
    const double dlon = std::abs(nominal._pos(1) - pos0(1));
    const double dh = std::abs(nominal._pos(2) - pos0(2));
    std::cout << "[static] |vn|=" << nominal._vn.norm() << " m/s"
              << " dpos(rad)=" << dlat << "," << dlon << " dh=" << dh << " m\n";
    std::cout << "[ecef  ] pos_ecef=" << nominal._pos_ecef.transpose() << "\n";
    std::cout << "[ecef  ] ve=" << nominal._ve.transpose()
              << " web=" << nominal._web.transpose() << "\n";

    return nominal._vn.norm() < 1e-2 && dlat < 1e-7 && dlon < 1e-7 && dh < 0.5;
}

bool TestInitializeLIO() {
    const Eigen::Vector3d origin(30.0 * kDeg2Rad, 120.0 * kDeg2Rad, 10.0);
    msf::earth eth0;
    eth0.Update(origin, Eigen::Vector3d::Zero());
    const Eigen::Vector3d fb_level = -eth0._gcc;  // 水平静止比力 ≈ [0,0,g]
    const Eigen::Vector3d gyro_bias(0.01, -0.02, 0.03);

    std::vector<msf::ImuData> imu;
    imu.reserve(50);
    for (int i = 0; i < 50; ++i) {
        msf::ImuData d;
        d._timestamp = 100.0 + 0.01 * i;
        d._dt = 0.01;
        d._gyro = gyro_bias;
        d._accel = fb_level;
        imu.push_back(d);
    }

    msf::GlobalConfig::ImuCalib calib;
    msf::NominalState nominal;
    msf::earth eth;
    if (!msf::InitializeLIO(imu, origin, calib, 40, nominal, eth)) {
        return false;
    }

    const bool att_ok = Near(nominal._att(0), 0.0, 1e-3) && Near(nominal._att(1), 0.0, 1e-3) &&
                        Near(nominal._att(2), 0.0, 1e-12);
    const bool pos_ok = (nominal._pos - origin).norm() < 1e-12;
    const bool vel_ok = nominal._vn.norm() < 1e-12;
    const bool bias_ok = (nominal._eb - gyro_bias).norm() < 1e-12;
    const bool time_ok = Near(nominal._t_cur, imu[39]._timestamp, 1e-12);
    std::cout << "[init  ] att(deg)=" << (nominal._att / kDeg2Rad).transpose()
              << " eb=" << nominal._eb.transpose() << "\n";
    return att_ok && pos_ok && vel_ok && bias_ok && time_ok;
}

}  // namespace

int main() {
    bool ok = true;
    ok &= TestAlignCoarse();
    ok &= TestStaticPropagation();
    ok &= TestInitializeLIO();
    if (!ok) {
        std::cerr << "INS propagator test failed\n";
        return 1;
    }
    std::cout << "INS propagator test passed\n";
    return 0;
}
