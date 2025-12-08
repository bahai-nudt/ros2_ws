#ifndef IMU_PROCESSOR_H
#define IMU_PROCESSOR_H

#include <Eigen/Dense>

#include "gloc/message.h"

class ImuProcessor {
public:

    ImuProcessor();
    ImuProcessor(
        double acc_noise, 
        double gyro_noise,
        double acc_bias_noise, 
        double gyro_bias_noise,
        const Eigen::Vector3d& gravity);

    void predict(const ImuData& cur_imu, State& state);

    double _acc_noise;
    double _gyro_noise;
    double _acc_bias_noise;
    double _gyro_bias_noise;

    const Eigen::Vector3d _gravity;


};


#endif