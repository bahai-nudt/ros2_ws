#pragma once

#include <iostream>

#include "config/global_config.h"

namespace msf {
namespace tools {

/**
 * CLI `--max-duration` 优先；未给时用 yaml `process_option.max_process_duration_sec`。
 * <=0 表示不截断。
 */
inline double ResolveMaxDuration(bool cli_set, double cli_value, const GlobalConfig& config) {
    const double duration = cli_set ? cli_value : config._process_option._max_process_duration_sec;
    if (duration > 0.0) {
        std::cout << "[process] max_duration=" << duration << " s" << std::endl;
    }
    return duration;
}

inline bool DurationExceeded(double event_ts, double init_time, double max_duration) {
    return max_duration > 0.0 && event_ts > init_time + max_duration;
}

}  // namespace tools
}  // namespace msf
