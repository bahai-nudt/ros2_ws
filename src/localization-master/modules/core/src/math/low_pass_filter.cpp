#include "math/low_pass_filter.h"

#include <algorithm>
#include <cmath>

LowPassFilter::LowPassFilter(double cutoff_freq_hz, double sample_rate_hz)
    : alpha_(0.0), last_output_(0.0), initialized_(false) {
    const double tau = 1.0 / (2.0 * M_PI * cutoff_freq_hz);
    const double dt = 1.0 / sample_rate_hz;
    alpha_ = dt / (tau + dt);
    if (alpha_ <= 0.0) alpha_ = 0.001;
    if (alpha_ > 1.0) alpha_ = 1.0;
}

double LowPassFilter::update(double input_value) {
    if (!initialized_) {
        last_output_ = input_value;
        initialized_ = true;
        return input_value;
    }
    last_output_ = alpha_ * input_value + (1.0 - alpha_) * last_output_;
    return last_output_;
}

void LowPassFilter::reset(double current_value) {
    last_output_ = current_value;
    initialized_ = true;
}
