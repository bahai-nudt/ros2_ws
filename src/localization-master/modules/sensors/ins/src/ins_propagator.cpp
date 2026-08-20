#include "../include/ins_propagator.h"

#include <cmath>

#include "math/pose_converter.h"
#include "math/t_quat.h"
#include "math/coordinate.h"

namespace msf {

Eigen::Vector3d InsPropagator::AlignCoarse(const NominalState& nominal,
                                           const Eigen::Vector3d& wmm,
                                           const Eigen::Vector3d& vmm) const {
    const double angular_norm = wmm.norm();
    const double velocity_norm = vmm.norm();
    if (angular_norm < 1.0e-12 || velocity_norm < 1.0e-12) {
        return nominal._att;
    }

    const double latitude = nominal._pos(0);
    const double cos_lat = std::cos(latitude);
    if (std::abs(cos_lat) < 1.0e-12) {
        return nominal._att;
    }
    const double tan_lat = std::tan(latitude);
    const Eigen::Vector3d wbib = wmm / angular_norm;
    const Eigen::Vector3d fb = vmm / velocity_norm;

    double t31 = fb(0);
    double t32 = fb(1);
    double t33 = fb(2);
    double t21 = wbib(0) / cos_lat - t31 * tan_lat;
    double t22 = wbib(1) / cos_lat - t32 * tan_lat;
    double t23 = wbib(2) / cos_lat - t33 * tan_lat;

    double norm = std::sqrt(t21 * t21 + t22 * t22 + t23 * t23);
    if (norm < 1.0e-12) {
        return nominal._att;
    }
    t21 /= norm;
    t22 /= norm;
    t23 /= norm;

    double t11 = t22 * t33 - t23 * t32;
    double t12 = t23 * t31 - t21 * t33;
    double t13 = t21 * t32 - t22 * t31;
    norm = std::sqrt(t11 * t11 + t12 * t12 + t13 * t13);
    if (norm < 1.0e-12) {
        return nominal._att;
    }
    t11 /= norm;
    t12 /= norm;
    t13 /= norm;

    Eigen::Matrix3d Cnb;
    Cnb << t11, t12, t13,
           t21, t22, t23,
           t31, t32, t33;
    return pose_converter::m2att(Cnb);
}

bool InsPropagator::Propagate(NominalState& nominal, earth& eth,
                           const Eigen::Vector3d& gyro_prev, const Eigen::Vector3d& accel_prev,
                           const Eigen::Vector3d& gyro_curr, const Eigen::Vector3d& accel_curr,
                           double dt) {
    nominal._nts = std::abs(dt);
    if (nominal._nts < 1.0e-9) {
        return false;
    }

    nominal._t_cur += nominal._nts;
    const double half_dt = nominal._nts / 2.0;

    const std::vector<Eigen::Vector3d> wm{(gyro_prev + gyro_curr) * half_dt};
    const std::vector<Eigen::Vector3d> vm{(accel_prev + accel_curr) * half_dt};
    imu_.Update(wm, vm);
    imu_._phim = nominal._Kg.asDiagonal() * imu_._phim - nominal._eb * nominal._nts;
    imu_._dvbm = nominal._Ka.asDiagonal() * imu_._dvbm - nominal._db * nominal._nts;

    const Eigen::Vector3d vn_half = nominal._vn + nominal._an * half_dt;
    const Eigen::Vector3d pos_half = nominal._pos + eth.v2dp(vn_half, half_dt);
    eth.Update(pos_half, vn_half);

    nominal._wib = imu_._phim / nominal._nts;
    const Eigen::Vector3d fb = imu_._dvbm / nominal._nts;
    const t_quat qnb = nominal._qnb;
    nominal._web = nominal._wib - nominal._Cbn * eth._wnie;
    nominal._wnb = nominal._wib -
                   t_quat::conj(qnb * pose_converter::rv2q(imu_._phim / 2.0)) * eth._wnin;

    const Eigen::Vector3d fn_nav = qnb * fb;
    nominal._fn = fn_nav;
    const Eigen::Vector3d an = pose_converter::rv2q(-eth._wnin * half_dt) * fn_nav + eth._gcc;
    nominal._an = an;

    const Eigen::Vector3d vn_next = nominal._vn + an * nominal._nts;
    nominal._pos = nominal._pos + eth.v2dp(nominal._vn + vn_next, half_dt);
    nominal._vn = vn_next;

    t_quat qnb_next = pose_converter::rv2q(-eth._wnin * nominal._nts) * qnb * pose_converter::rv2q(imu_._phim);
    t_quat::normlize(qnb_next);
    nominal._qnb = qnb_next;
    nominal._Cnb = pose_converter::q2mat(qnb_next);
    nominal._att = pose_converter::m2att(nominal._Cnb);
    nominal._Cbn = nominal._Cnb.transpose();

    nominal._pos_ecef = Coordinate::lla2xyz(nominal._pos);
    nominal._Ceb = eth._Cen * nominal._Cnb;
    nominal._Cbe = nominal._Ceb.transpose();
    nominal._qeb = pose_converter::m2qua(nominal._Ceb);
    nominal._ve = eth._Cen * nominal._vn;
    nominal._ae = eth._Cen * nominal._an;
    return true;
}

}  // namespace msf
