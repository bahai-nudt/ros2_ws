#pragma once

#include <string>

namespace msf {

/** LiDAR 扫描观测元数据（不依赖 PCL） */
struct LidarScanInfo {
    double _timestamp = 0.0;  // 帧头时间 [s]（来自文件名）
    double _end_time = 0.0;   // 帧尾时间 [s]
    std::string _path;        // PCD 文件路径
};

}  // namespace msf
