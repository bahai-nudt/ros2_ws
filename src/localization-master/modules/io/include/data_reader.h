#pragma once

#include <string>
#include <vector>

#include "config/global_config.h"
#include "types/sensors/gnss_message.h"
#include "types/sensors/imu_message.h"
#include "types/sensors/lidar_message.h"

namespace msf {

/** 离线加载选项：由组合层（demo）显式指定读哪些传感器。IMU 始终加载。 */
struct LoadOptions {
    bool load_gnss = true;   ///< GNSS pos/vel + heading
    bool load_lidar = false; ///< 点云目录（config._lidar_frame_id）
};

/** 离线数据读取：BADP IMU txt、CSV（GNSS/Heading）与点云目录（LiDAR）。 */
class DataReader {
public:
    DataReader() = default;
    ~DataReader() = default;

    // 按配置与 LoadOptions 加载离线传感器数据；不做 timeline 合并。
    bool LoadData(const GlobalConfig& config, const LoadOptions& options = {});

    bool LoadImu(const std::string& path, bool apply_bewis_axis_rotation);
    bool LoadGnssPos(const std::string& path);
    bool LoadGnssVel(const std::string& path);
    bool LoadHeading(const std::string& path, double heading_offset_deg);
    bool LoadLidar(const std::string& path);

    const std::vector<ImuData>& GetImuData() const { return imu_data_; }
    const std::vector<GnssPos>& GetGnssPosData() const { return gnsspos_data_; }
    const std::vector<GnssVel>& GetGnssVelData() const { return gnssvel_data_; }
    const std::vector<HeadingData>& GetHeadingData() const { return heading_data_; }
    const std::vector<LidarScanInfo>& GetLidarData() const { return lidar_data_; }

    size_t GetImuCount() const { return imu_data_.size(); }
    size_t GetGnssPosCount() const { return gnsspos_data_.size(); }
    size_t GetGnssVelCount() const { return gnssvel_data_.size(); }
    size_t GetHeadingCount() const { return heading_data_.size(); }
    size_t GetLidarCount() const { return lidar_data_.size(); }

    double GetStartTime() const;
    double GetEndTime() const;

    const std::vector<std::string>& GetLoadedPaths() const { return loaded_paths_; }

private:
    // GNSS 按时间戳去重（保留同时间戳最后一条）
    void DeduplicateGnssData();

    std::vector<ImuData> imu_data_;
    std::vector<GnssPos> gnsspos_data_;
    std::vector<GnssVel> gnssvel_data_;
    std::vector<HeadingData> heading_data_;
    std::vector<LidarScanInfo> lidar_data_;
    std::vector<std::string> loaded_paths_;

    std::vector<std::string> SplitCsvLine(const std::string& line) const;
    static double ParseTimestampFromPath(const std::string& path);
};

}  // namespace msf
