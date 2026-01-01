#include "gloc/gps_processor.h"
#include "base_utils/coordinate.h"
#include "base_utils/tools.h"
#include <deque>
#include <iomanip>

GpsProcessor::GpsProcessor() {

}

bool GpsProcessor::update(GpsData gps_data) {
   /*
    std::deque<State> state_cache;
    if (1) {
        std::lock_guard<std::mutex> lock(SingletonDataBuffer::getInstance()._state_mtx);
        state_cache = SingletonDataBuffer::getInstance()._state_buffer.cache;
    }

    State state;
    int state_ind = 0;
    for (size_t i = state_cache.size() - 1; i >= 0; i++) {
        if (state_cache[i]._timestamp < gps_data._timestamp) {
            cur_state_ind = i;
            cur_state = state_cache[i];
            break;
        } 
    }

    if (state_ind == 0) {
        std::cout << "gps 时间戳与 state不匹配" << std::endl;
        return false;
    }

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
    state._cov = I_KH * P * I_KH.transpose() + K * V * K.transpose();*/
    return true;
}


bool GpsProcessor::update(GpsData gps_data, State& state) {

    if (gps_data._timestamp - state._timestamp > 0.02 || gps_data._timestamp < state._timestamp) {
        std::cout << "gps time is far away from state time or gps time before states" << std::endl;
        return false;
    }

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

    return true;
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
    local_enu = local_enu - state._velocity *(gps_data._timestamp - state._timestamp);

    Eigen::Vector3d euler_angles = state._rotation.eulerAngles(2, 0, 1);

    // Compute residual.
    residual = local_enu - (state._position  + state._rotation * _lever_arm);

        std::cout << "观测航向角度：  " <<euler_angles * 180 / 3.14<< std::endl;
    std::cout << "加速度零偏： " << state._bias_accel << std::endl;
    std::cout << "角速度零篇： " <<  state._bias_gyro << std::endl;

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