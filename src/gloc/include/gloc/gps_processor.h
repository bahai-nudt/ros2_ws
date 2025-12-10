#ifndef GPS_PROCESSOR_H
#define GPS_PROCESSOR_H

#include <Eigen/Dense>

#include "gloc/message.h"

class GpsProcessor {
public:
    GpsProcessor();
    GpsProcessor(const Eigen::Vector3d& pos);
    void update(GpsData gps_data, State& state);


    void compute_jacobian_residual(
        const Eigen::Vector3d& init_lla,
        GpsData gps_data,
        const State& state,
        Eigen::Matrix<double, 3, 15>& jacobian,
        Eigen::Vector3d& residual);

    void add_delta2state(const Eigen::Matrix<double, 15, 1>& delta_x, State& state);

    Eigen::Vector3d _pos;
};


#endif