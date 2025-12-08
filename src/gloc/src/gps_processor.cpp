#include "gloc/gps_processor.h"

GpsProcessor::GpsProcessor() {

}

GpsProcessor::GpsProcessor(const Eigen::Vector3d& pos) {
    ;
}
void GpsProcessor::update(const Eigen::Vector3d& init_lla, GpsData gps_data, State& state) {
    ;
}


void GpsProcessor::compute_jacobian_residual(
    const Eigen::Vector3d& init_lla,
    GpsData gps_data,
    const State& state,
    Eigen::Matrix<double, 3, 15>& jacobian,
    Eigen::Vector3d& residual) {

}

void GpsProcessor::add_delta2state(const Eigen::Matrix<double, 15, 1>& delta_x, State& state) {
    ;
}