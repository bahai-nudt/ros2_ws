#include "lidar_pose_meas.h"

#include <algorithm>
#include <cmath>

#include "math/constants.h"
#include "math/pose_converter.h"
#include "math/t_quat.h"
#include "math/utility.h"
#include "pcd_preprocess.h"
#include "types/state_layout.h"

namespace msf {
namespace lidar {
namespace {

std::vector<LidarPlaneConstraint> CollectEffectivePlanes(
    const PointCloudXYZI::Ptr& cloud_imu,
    const PointCloudXYZI::Ptr& cloud_world,
    const std::vector<LidarPlaneConstraint>& matched,
    const GlobalConfig& config,
    LidarFeatureDiag& feature_diag) {
    const int min_effective = std::max(1, config._lidar._min_effective_features);
    std::vector<LidarPlaneConstraint> effective;
    effective.reserve(matched.size());
    for (const auto& constraint : matched) {
        if (!cloud_imu || !cloud_world ||
            constraint.point_index >= cloud_imu->size() ||
            constraint.point_index >= cloud_world->size()) {
            continue;
        }
        const PointType& point_world = cloud_world->points[constraint.point_index];
        const Eigen::Vector3d p_imu(cloud_imu->points[constraint.point_index].x,
                                    cloud_imu->points[constraint.point_index].y,
                                    cloud_imu->points[constraint.point_index].z);
        const float pd2 = static_cast<float>(constraint.normal_enu.x()) * point_world.x +
                          static_cast<float>(constraint.normal_enu.y()) * point_world.y +
                          static_cast<float>(constraint.normal_enu.z()) * point_world.z +
                          static_cast<float>(constraint.plane_d);
        const Eigen::Vector3d p_lidar =
            config._calibration._R_bl.transpose() * (p_imu - config._calibration._T_lb_m);
        const double point_norm = std::max(p_lidar.norm(), 1.0e-12);
        const double s = 1.0 - config._lidar._residual_score_scale * std::fabs(pd2) /
                                    std::sqrt(point_norm);
        if (s <= config._lidar._residual_score_min) {
            continue;
        }
        LidarPlaneConstraint valid = constraint;
        valid.point_imu = p_imu;
        effective.push_back(valid);
    }
    feature_diag.effective_features = static_cast<int>(effective.size());
    if (static_cast<int>(effective.size()) < min_effective) {
        effective.clear();
        feature_diag.effective_features = 0;
    }
    return effective;
}

bool BuildPlaneLinearSystem(const NominalState& nominal,
                            const earth& eth,
                            const GlobalConfig& config,
                            const StateLayout& layout,
                            const std::vector<LidarPlaneConstraint>& effective,
                            Eigen::MatrixXd& H,
                            Eigen::VectorXd& z) {
    if (effective.empty()) {
        return false;
    }
    const int nx = layout.Dim();
    const int rot_off = layout.Offset(BlockId::Rotation);
    const int pos_off = layout.Offset(BlockId::Position);
    Eigen::Matrix3d J_blh_to_enu;
    J_blh_to_enu << 0.0, 1.0 / eth._f_cbRNh, 0.0,
                    1.0 / eth._f_RMh, 0.0, 0.0,
                    0.0, 0.0, 1.0;
    const Eigen::Vector3d imu_position_enu = PositionEnu(nominal, config);

    H = Eigen::MatrixXd::Zero(static_cast<int>(effective.size()), nx);
    z = Eigen::VectorXd::Zero(static_cast<int>(effective.size()));
    int row = 0;
    for (const auto& constraint : effective) {
        const Eigen::Vector3d p_I = constraint.point_imu;
        Eigen::Vector3d n_map = constraint.normal_enu;
        const double norm_n = n_map.norm();
        if (!p_I.allFinite() || !n_map.allFinite() ||
            !std::isfinite(constraint.plane_d) || norm_n < 1.0e-12) {
            continue;
        }
        n_map /= norm_n;
        const Eigen::Vector3d Cnb_p_I = nominal._Cnb * p_I;
        const Eigen::Vector3d p_w = imu_position_enu + Cnb_p_I;
        const double pd2 = n_map.dot(p_w) + constraint.plane_d;
        if (!std::isfinite(pd2)) {
            continue;
        }
        H.block<1, 3>(row, rot_off) = (askew(Cnb_p_I) * n_map).transpose();
        H.block<1, 3>(row, pos_off) = (-J_blh_to_enu.transpose() * n_map).transpose();
        z(row) = -pd2;
        ++row;
    }
    if (row == 0) {
        return false;
    }
    H.conservativeResize(row, Eigen::NoChange);
    z.conservativeResize(row);
    return true;
}

Eigen::MatrixXd PoseJacobianXi(const Eigen::MatrixXd& H, const StateLayout& layout) {
    Eigen::MatrixXd H_xi(H.rows(), 6);
    H_xi.leftCols(3) = H.block(0, layout.Offset(BlockId::Rotation), H.rows(), 3);
    H_xi.rightCols(3) = H.block(0, layout.Offset(BlockId::Position), H.rows(), 3);
    return H_xi;
}

void ApplyAttPosError(NominalState& nominal, earth& eth,
                      const Eigen::Vector3d& dphi, const Eigen::Vector3d& dpos) {
    nominal._pos -= dpos;
    nominal._qnb = pose_converter::rv2q(dphi) * nominal._qnb;
    t_quat::normlize(nominal._qnb);
    nominal._Cnb = pose_converter::q2mat(nominal._qnb);
    nominal._Cbn = nominal._Cnb.transpose();
    nominal._att = pose_converter::m2att(nominal._Cnb);
    eth.Update(nominal._pos, nominal._vn);
}

Eigen::Matrix<double, 6, 6> OrthogonalizeEigenvectors(
    const Eigen::Matrix<double, 6, 6>& V) {
    Eigen::JacobiSVD<Eigen::Matrix<double, 6, 6>> svd(
        V, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix<double, 6, 6> U = svd.matrixU();
    const Eigen::Matrix<double, 6, 6> Vt = svd.matrixV().transpose();
    Eigen::Matrix<double, 6, 6> orth = U * Vt;
    if (orth.determinant() < 0.0) {
        U.col(5) *= -1.0;
        orth = U * Vt;
    }
    return orth;
}

}  // namespace

Eigen::Matrix<double, 6, 6> MakeLidarPoseCovariance(
    const Eigen::Matrix<double, 6, 6>& lambda,
    const GlobalConfig& config) {
    Eigen::Matrix<double, 6, 6> information = lambda;
    information.diagonal().array() += 1.0e-9;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(information);
    Eigen::Matrix<double, 6, 1> inv_eig = solver.eigenvalues();
    const double min_eigenvalue = std::max(config._lidar._pose_meas_min_eigenvalue, 0.0);
    constexpr double kDegenerateVariance = 1.0e6;
    for (int i = 0; i < 6; ++i) {
        if (!std::isfinite(inv_eig(i)) || inv_eig(i) < min_eigenvalue) {
            inv_eig(i) = 1.0 / kDegenerateVariance;
        } else {
            inv_eig(i) = 1.0 / inv_eig(i);
        }
    }
    const Eigen::Matrix<double, 6, 6> V_orth =
        OrthogonalizeEigenvectors(solver.eigenvectors());
    Eigen::Matrix<double, 6, 6> R = V_orth * inv_eig.asDiagonal() * V_orth.transpose();
    const double inflate = std::max(config._lidar._pose_meas_inflate, 1.0);
    R *= inflate;

    for (int i = 0; i < 3; ++i) {
        const double att_floor =
            config._lidar._pose_meas_att_std_floor_deg(i) * constants::_deg;
        const double pos_floor = config._lidar._pose_meas_pos_std_floor_m(i);
        R(i, i) = std::max(R(i, i), att_floor * att_floor);
        R(i + 3, i + 3) = std::max(R(i + 3, i + 3), pos_floor * pos_floor);
    }
    R = 0.5 * (R + R.transpose());
    return R;
}

LidarPoseEstimate EstimateLidarPose(LocalMap& map,
                                    const PointCloudXYZI::Ptr& cloud_imu,
                                    const LidarScanInfo& scan,
                                    const NominalState& nominal,
                                    earth eth,
                                    const GlobalConfig& config) {
    LidarPoseEstimate result;
    if (!cloud_imu || cloud_imu->empty() || !map.IsBuilt()) {
        return result;
    }

    const double R_point = config._lidar._iterated_update_noise_var > 0.0
        ? config._lidar._iterated_update_noise_var : 0.001;
    int maximum_iter = std::max(1, config._lidar._max_iterations);
    double convergence_threshold = config._lidar._iekf_convergence_threshold;
    if (convergence_threshold <= 0.0) {
        convergence_threshold = 0.001;
    }

    const StateLayout layout = StateLayout::FromStateDim(
        config._process_option._state_dim > 0 ? config._process_option._state_dim : 15);

    NominalState work = nominal;
    eth.Update(work._pos, work._vn);

    auto assemble = [&](bool rematch, Eigen::MatrixXd& H, Eigen::VectorXd& h,
                        LidarFeatureDiag& diag) -> bool {
        const PointCloudXYZI::Ptr cloud_world =
            TransformCloudImuToWorld(cloud_imu, work, config);
        if (!cloud_world || cloud_world->empty()) {
            return false;
        }
        std::vector<LidarPlaneConstraint> matched;
        map.Match(cloud_world, rematch, matched, diag);
        const auto effective =
            CollectEffectivePlanes(cloud_imu, cloud_world, matched, config, diag);
        return BuildPlaneLinearSystem(work, eth, config, layout, effective, H, h);
    };

    Eigen::MatrixXd H;
    Eigen::VectorXd h;
    LidarFeatureDiag local_diag;
    if (!assemble(true, H, h, local_diag)) {
        result._feature_diag = local_diag;
        return result;
    }

    bool converge = true;
    int converge_count = 0;
    int converge_at = -1;
    bool updated = false;
    int actual_iter = 0;
    double max_dx_last = 0.0;
    Eigen::MatrixXd last_H_xi;

    for (int iter = -1; iter < maximum_iter; ++iter) {
        if (iter > -1) {
            if (!assemble(converge, H, h, local_diag)) {
                continue;
            }
        }
        const Eigen::MatrixXd H_xi = PoseJacobianXi(H, layout);
        Eigen::Matrix<double, 6, 6> HTH = Eigen::Matrix<double, 6, 6>::Zero();
        HTH.noalias() = H_xi.transpose() * H_xi;
        HTH.diagonal().array() += 1.0e-6;
        Eigen::LDLT<Eigen::Matrix<double, 6, 6>> ldlt(HTH);
        if (!ldlt.isPositive()) {
            continue;
        }
        const Eigen::Matrix<double, 6, 1> dx_xi = ldlt.solve(H_xi.transpose() * h);
        if (!dx_xi.allFinite()) {
            continue;
        }

        ApplyAttPosError(work, eth, dx_xi.head<3>(), dx_xi.tail<3>());
        updated = true;
        last_H_xi = H_xi;

        converge = true;
        max_dx_last = 0.0;
        for (int j = 0; j < 6; ++j) {
            const double abs_dx = std::fabs(dx_xi(j));
            if (abs_dx > max_dx_last) {
                max_dx_last = abs_dx;
            }
            if (abs_dx > convergence_threshold) {
                converge = false;
            }
        }
        ++actual_iter;
        if (converge) {
            ++converge_count;
            if (converge_at < 0) {
                converge_at = actual_iter;
            }
        }
        if (!converge_count && iter == maximum_iter - 2) {
            converge = true;
        }
        if (converge_count > 1 || iter == maximum_iter - 1) {
            break;
        }
    }

    result._iterations = actual_iter;
    result._converge_iter = converge_at;
    result._final_converged = converge;
    result._max_dx_last = max_dx_last;
    result._feature_diag = local_diag;
    if (!updated || last_H_xi.size() == 0) {
        return result;
    }

    Eigen::Matrix<double, 6, 6> lambda = Eigen::Matrix<double, 6, 6>::Zero();
    lambda.noalias() = last_H_xi.transpose() * last_H_xi / R_point;
    result._C_meas = work._Cnb;
    result._p_meas_enu = PositionEnu(work, config);
    result._R_pose = MakeLidarPoseCovariance(lambda, config);
    result._valid = result._C_meas.allFinite() && result._p_meas_enu.allFinite() &&
                    result._R_pose.allFinite();
    return result;
}

}  // namespace lidar
}  // namespace msf
