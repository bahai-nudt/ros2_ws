#include "gloc/gps_processor.h"
#include "base_utils/coordinate.h"
#include "base_utils/tools.h"

GpsProcessor::GpsProcessor() {

}

void GpsProcessor::update(GpsData gps_data, State& state) {
    Eigen::Matrix<double, 3, 15> H;
    Eigen::Vector3d residual;
    compute_jacobian_residual(state._init_lla, gps_data, state, H, residual);
    const Eigen::Matrix3d& V = gps_data._cov;

    // EKF.
    const Eigen::MatrixXd P = state._cov;
    const Eigen::MatrixXd K = P * H.transpose() * (H * P * H.transpose() + V).inverse();
    const Eigen::VectorXd delta_x = K * residual;

    // Add delta_x to state.
    add_delta2state(delta_x, state);

    // Covarance.
    const Eigen::MatrixXd I_KH = Eigen::Matrix<double, 15, 15>::Identity() - K * H;
    state._cov = I_KH * P * I_KH.transpose() + K * V * K.transpose();
}


void GpsProcessor::compute_jacobian_residual(
    const Eigen::Vector3d& init_lla,
    GpsData gps_data,
    const State& state,
    Eigen::Matrix<double, 3, 15>& jacobian,
    Eigen::Vector3d& residual) {

    // Convert wgs84 to ENU frame.
    
    std::vector<double> local_coor = Coordinate::lla2enu(init_lla(0), init_lla(1), init_lla(2), gps_data._lla(0), gps_data._lla(1), gps_data._lla(2));

    Eigen::Vector3d local_enu;
    local_enu << local_coor[0], local_coor[1], local_coor[2];


    // Compute residual.
    residual = local_enu - (state._position  + state._rotation * _lever_arm);

    // Compute jacobian.
    jacobian.setZero();
    jacobian.block<3, 3>(0, 0)  = Eigen::Matrix3d::Identity();
    jacobian.block<3, 3>(0, 6)  = - state._rotation * BaseTools::GetSkewMatrix(_lever_arm);

}

void GpsProcessor::add_delta2state(const Eigen::Matrix<double, 15, 1>& delta_x, State& state) {
    state._position    += delta_x.block<3, 1>(0, 0);
    state._velocity    += delta_x.block<3, 1>(3, 0);
    state._bias_accel  += delta_x.block<3, 1>(9, 0);
    state._bias_gyro   += delta_x.block<3, 1>(12, 0);

    if (delta_x.block<3, 1>(6, 0).norm() > 1e-12) {
        state._rotation *= Eigen::AngleAxisd(delta_x.block<3, 1>(6, 0).norm(), delta_x.block<3, 1>(6, 0).normalized()).toRotationMatrix();
    }
}