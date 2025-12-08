#include "gloc/imu_processor.h"
#include "base_utils/tools.h"

ImuProcessor::ImuProcessor() {
    ;
}

ImuProcessor::ImuProcessor(
    double acc_noise, 
    double gyro_noise,
    double acc_bias_noise, 
    double gyro_bias_noise,
    const Eigen::Vector3d& gravity) : 
    _acc_noise(acc_noise), _gyro_noise(gyro_noise), _acc_bias_noise(acc_bias_noise), _gyro_bias_noise(gyro_bias_noise), _gravity(gravity) {}

void ImuProcessor::predict(const ImuData& cur_imu, State& state) {
    
    const double delta_t = cur_imu._timestamp - state._timestamp;
    if (delta_t < 1e-12) { // 小于1ms
        // TODO add log
        return;
    }

    const double delta_t2 = delta_t * delta_t;

    // Set last state.
    State last_state = state;

    state._position = last_state._position + last_state._velocity * delta_t +
                   0.5 * (last_state._orientation * cur_imu._accel + _gravity) * delta_t2;
    state._velocity = last_state._velocity + (last_state._orientation * cur_imu._accel + _gravity) * delta_t;



    const Eigen::Vector3d delta_angle_axis = cur_imu._gyro * delta_t;
    if (delta_angle_axis.norm() > 1e-12) {
        state._orientation = last_state._orientation * Eigen::AngleAxisd(delta_angle_axis.norm(), delta_angle_axis.normalized()).toRotationMatrix();
    }
    // Error-state. Not needed.

    // Covariance of the error-state.   
    Eigen::Matrix<double, 15, 15> Fx = Eigen::Matrix<double, 15, 15>::Identity();
    Fx.block<3, 3>(0, 3)   = Eigen::Matrix3d::Identity() * delta_t;
    Fx.block<3, 3>(3, 6)   = - state._orientation.toRotationMatrix() * BaseTools::GetSkewMatrix(cur_imu._accel) * delta_t;
    Fx.block<3, 3>(3, 9)   = - state._orientation.toRotationMatrix() * delta_t;
    if (delta_angle_axis.norm() > 1e-12) {
        Fx.block<3, 3>(6, 6) = Eigen::AngleAxisd(delta_angle_axis.norm(), delta_angle_axis.normalized()).toRotationMatrix().transpose();
    } else {
        Fx.block<3, 3>(6, 6).setIdentity();
    }
    Fx.block<3, 3>(6, 12)  = - Eigen::Matrix3d::Identity() * delta_t;

    Eigen::Matrix<double, 15, 12> Fi = Eigen::Matrix<double, 15, 12>::Zero();
    Fi.block<12, 12>(3, 0) = Eigen::Matrix<double, 12, 12>::Identity();

    Eigen::Matrix<double, 12, 12> Qi = Eigen::Matrix<double, 12, 12>::Zero();
    Qi.block<3, 3>(0, 0) = delta_t2 * _acc_noise * Eigen::Matrix3d::Identity();
    Qi.block<3, 3>(3, 3) = delta_t2 * _gyro_noise * Eigen::Matrix3d::Identity();
    Qi.block<3, 3>(6, 6) = delta_t * _acc_bias_noise * Eigen::Matrix3d::Identity();
    Qi.block<3, 3>(9, 9) = delta_t * _gyro_bias_noise * Eigen::Matrix3d::Identity();

    state._cov = Fx * last_state._cov * Fx.transpose() + Fi * Qi * Fi.transpose();

    // Time and imu.
    state._timestamp = cur_imu._timestamp;

}