#pragma once

class LowPassFilter {
public:
    // 构造函数：传入截止频率 (Hz) 和 采样率 (Hz)
    LowPassFilter(double cutoff_freq_hz, double sample_rate_hz);

    double update(double input_value);
    void reset(double current_value);

private:
    double alpha_;
    double last_output_;
    bool initialized_;
};
