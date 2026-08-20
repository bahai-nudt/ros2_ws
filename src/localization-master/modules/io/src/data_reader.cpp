#include "data_reader.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <unordered_map>

#include "math/constants.h"

namespace msf {

namespace {

std::string ImuFileName(const std::string& imu_topic) {
    if (imu_topic.find("/bewis/") != std::string::npos) {
        return "bewis.imu.txt";
    }
    if (imu_topic.find("/shangyu/") != std::string::npos) {
        return "shangyu.imu.txt";
    }
    if (imu_topic.find("/bynav/") != std::string::npos) {
        return "bynav.imu.txt";
    }
    return "imu.txt";
}

std::vector<std::string> SplitWhitespace(const std::string& line) {
    std::vector<std::string> tokens;
    std::istringstream stream(line);
    std::string token;
    while (stream >> token) {
        tokens.push_back(token);
    }
    return tokens;
}

template <typename Row, typename TsGetter>
size_t DropRollbackData(std::vector<Row>& rows,
                        const std::string& sensor_name,
                        TsGetter ts_getter,
                        double tolerance_sec = 1.0e-6) {
    if (rows.size() <= 1) {
        return 0;
    }

    std::vector<Row> filtered;
    filtered.reserve(rows.size());
    filtered.push_back(rows.front());

    size_t rollback_count = 0;
    double last_kept_ts = ts_getter(rows.front());
    for (size_t index = 1; index < rows.size(); ++index) {
        const double current_ts = ts_getter(rows[index]);
        if (current_ts + tolerance_sec < last_kept_ts) {
            ++rollback_count;
            std::cerr << std::fixed << std::setprecision(6)
                      << "[DATA] " << sensor_name
                      << " 时间回跳: idx=" << index
                      << " prev_ts=" << last_kept_ts
                      << " curr_ts=" << current_ts
                      << " -> drop" << std::endl;
            continue;
        }
        filtered.push_back(rows[index]);
        last_kept_ts = current_ts;
    }

    if (rollback_count > 0) {
        std::cerr << "[DATA] " << sensor_name
                  << " 回跳剔除统计: dropped=" << rollback_count
                  << ", kept=" << filtered.size()
                  << ", raw=" << rows.size() << std::endl;
        rows.swap(filtered);
    }
    return rollback_count;
}

}  // namespace

bool DataReader::LoadData(const GlobalConfig& config, const LoadOptions& options) {
    loaded_paths_.clear();
    imu_data_.clear();
    gnsspos_data_.clear();
    gnssvel_data_.clear();
    heading_data_.clear();
    lidar_data_.clear();

    const std::string& data_dir = config._file._input_data_dir;
    const std::string imu_file = ImuFileName(config._topics._imu_topic);
    const std::string imu_path = data_dir + "/" + imu_file;
    const bool apply_bewis_axis_rotation =
        config._topics._imu_topic.find("/bewis/") != std::string::npos;
    if (!LoadImu(imu_path, apply_bewis_axis_rotation)) {
        std::cerr << "加载 IMU 数据失败" << std::endl;
        return false;
    }
    loaded_paths_.push_back(imu_path);

    if (options.load_gnss) {
        const std::string gnsspos_path = data_dir + "/gnsspos.txt";
        const std::string gnssvel_path = data_dir + "/gnssvel.txt";
        const std::string heading_path = data_dir + "/heading.txt";
        const bool gnss_pos_loaded = LoadGnssPos(gnsspos_path);
        const bool gnss_vel_loaded = LoadGnssVel(gnssvel_path);
        const bool heading_loaded =
            LoadHeading(heading_path, config._calibration._heading_offset_deg);
        if (!gnss_pos_loaded || !gnss_vel_loaded || !heading_loaded) {
            std::cerr << "加载 GNSS/Heading 初始化数据失败" << std::endl;
            return false;
        }
        loaded_paths_.push_back(gnsspos_path);
        loaded_paths_.push_back(gnssvel_path);
        loaded_paths_.push_back(heading_path);
    }

    if (options.load_lidar) {
        const std::string lidar_path = data_dir + "/" + config._lidar_frame_id;
        if (!std::filesystem::exists(lidar_path) || !LoadLidar(lidar_path)) {
            std::cerr << "加载 LiDAR 数据失败: " << lidar_path << std::endl;
            return false;
        }
        loaded_paths_.push_back(lidar_path);
    }

    const size_t imu_rollback_count = DropRollbackData(
        imu_data_, "IMU", [](const ImuData& row) { return row._timestamp; });
    size_t gnsspos_rollback_count = 0;
    size_t gnssvel_rollback_count = 0;
    size_t heading_rollback_count = 0;
    if (options.load_gnss) {
        gnsspos_rollback_count = DropRollbackData(
            gnsspos_data_, "GNSSPOS", [](const GnssPos& row) { return row._timestamp; });
        gnssvel_rollback_count = DropRollbackData(
            gnssvel_data_, "GNSSVEL", [](const GnssVel& row) { return row._timestamp; });
        heading_rollback_count = DropRollbackData(
            heading_data_, "HEADING", [](const HeadingData& row) { return row._timestamp; });
    }
    const size_t lidar_rollback_count = DropRollbackData(
        lidar_data_, "LIDAR", [](const LidarScanInfo& row) { return row._timestamp; });

    const size_t rollback_total = imu_rollback_count + gnsspos_rollback_count +
                                  gnssvel_rollback_count + heading_rollback_count +
                                  lidar_rollback_count;
    std::cerr << "[DATA] 时间回跳总计: IMU=" << imu_rollback_count
              << ", GNSSPOS=" << gnsspos_rollback_count
              << ", GNSSVEL=" << gnssvel_rollback_count
              << ", HEADING=" << heading_rollback_count
              << ", LIDAR=" << lidar_rollback_count
              << ", TOTAL=" << rollback_total << std::endl;

    DeduplicateGnssData();
    return true;
}

void DataReader::DeduplicateGnssData() {
    auto deduplicate_by_timestamp = [](auto& rows) {
        if (rows.empty()) {
            return;
        }
        std::unordered_map<double, size_t> last_index_by_time;
        last_index_by_time.reserve(rows.size());
        for (size_t i = 0; i < rows.size(); ++i) {
            last_index_by_time[rows[i]._timestamp] = i;
        }

        std::vector<std::remove_reference_t<decltype(rows.front())>> deduped;
        deduped.reserve(last_index_by_time.size());
        for (size_t i = 0; i < rows.size(); ++i) {
            const auto it = last_index_by_time.find(rows[i]._timestamp);
            if (it != last_index_by_time.end() && it->second == i) {
                deduped.push_back(rows[i]);
            }
        }
        rows.swap(deduped);
    };

    deduplicate_by_timestamp(gnsspos_data_);
    deduplicate_by_timestamp(gnssvel_data_);
    deduplicate_by_timestamp(heading_data_);
}

std::vector<std::string> DataReader::SplitCsvLine(const std::string& line) const {
    std::vector<std::string> tokens;
    std::stringstream stream(line);
    std::string token;
    while (std::getline(stream, token, ',')) {
        tokens.push_back(token);
    }
    return tokens;
}

// BADP `{brand}.imu.txt`：空格分隔
// timestamp_us qx qy qz qw gx gy gz ax ay az
bool DataReader::LoadImu(const std::string& path, bool apply_bewis_axis_rotation) {
    std::ifstream file(path);
    if (!file.is_open()) {
        std::cerr << "无法打开 IMU 文件: " << path << std::endl;
        return false;
    }

    imu_data_.clear();
    std::string line;

    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }

        const auto tokens = SplitWhitespace(line);
        if (tokens.size() < 11) {
            continue;
        }

        ImuData imu;
        imu._timestamp = std::stod(tokens[0]) * 1.0e-6;
        const double gyro_x = std::stod(tokens[5]);
        const double gyro_y = std::stod(tokens[6]);
        const double gyro_z = std::stod(tokens[7]);
        const double accel_x = std::stod(tokens[8]);
        const double accel_y = std::stod(tokens[9]);
        const double accel_z = std::stod(tokens[10]);

        if (apply_bewis_axis_rotation) {
            imu._gyro << -gyro_y, gyro_x, gyro_z;
            imu._accel << -accel_y, accel_x, accel_z;
        } else {
            imu._gyro << gyro_x, gyro_y, gyro_z;
            imu._accel << accel_x, accel_y, accel_z;
        }

        if (!imu_data_.empty()) {
            const auto& prev = imu_data_.back();
            imu._dt = imu._timestamp - prev._timestamp;
            imu._wm = (prev._gyro + imu._gyro) * imu._dt / 2.0;
            imu._vm = (prev._accel + imu._accel) * imu._dt / 2.0;
        }

        imu_data_.push_back(imu);
    }

    return !imu_data_.empty();
}

bool DataReader::LoadGnssPos(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        std::cerr << "无法打开 GNSS POS 文件: " << path << std::endl;
        return false;
    }

    gnsspos_data_.clear();
    std::string line;
    bool is_header = true;

    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }
        if (is_header) {
            is_header = false;
            continue;
        }

        const auto tokens = SplitCsvLine(line);
        if (tokens.size() < 9) {
            continue;
        }

        GnssPos gnsspos;
        gnsspos._timestamp = std::stod(tokens[0]);
        gnsspos._sol_status = std::stoi(tokens[1]);
        gnsspos._pos_type = std::stoi(tokens[2]);
        // 输入是度，转弧度（与 POST_MSF / localization_msf 对齐）
        gnsspos._blh << std::stod(tokens[3]) * constants::_D2R,
                        std::stod(tokens[4]) * constants::_D2R,
                        std::stod(tokens[5]);
        gnsspos._blh_std << std::stod(tokens[6]), std::stod(tokens[7]), std::stod(tokens[8]);

        gnsspos_data_.push_back(gnsspos);
    }

    return !gnsspos_data_.empty();
}

bool DataReader::LoadGnssVel(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        std::cerr << "无法打开 GNSS VEL 文件: " << path << std::endl;
        return false;
    }

    gnssvel_data_.clear();
    std::string line;
    bool is_header = true;

    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }
        if (is_header) {
            is_header = false;
            continue;
        }

        const auto tokens = SplitCsvLine(line);
        if (tokens.size() < 5) {
            continue;
        }

        GnssVel gnssvel;
        gnssvel._timestamp = std::stod(tokens[0]);
        gnssvel._vel_type = std::stoi(tokens[1]);
        gnssvel._hor_speed = std::stod(tokens[2]);
        gnssvel._trk_gnd = std::stod(tokens[3]) * constants::_D2R;  // 度转弧度
        while (gnssvel._trk_gnd < 0.0) {
            gnssvel._trk_gnd += 2.0 * constants::_PI;
        }
        while (gnssvel._trk_gnd >= 2.0 * constants::_PI) {
            gnssvel._trk_gnd -= 2.0 * constants::_PI;
        }
        gnssvel._ver_speed = std::stod(tokens[4]);

        gnssvel_data_.push_back(gnssvel);
    }

    return !gnssvel_data_.empty();
}

bool DataReader::LoadHeading(const std::string& path, double heading_offset_deg) {
    std::ifstream file(path);
    if (!file.is_open()) {
        std::cerr << "无法打开 Heading 文件: " << path << std::endl;
        return false;
    }

    heading_data_.clear();
    std::string line;
    bool is_header = true;

    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }
        if (is_header) {
            is_header = false;
            continue;
        }

        const auto tokens = SplitCsvLine(line);
        if (tokens.size() < 6) {
            continue;
        }

        HeadingData heading;
        heading._timestamp = std::stod(tokens[0]);
        heading._sol_status = std::stoi(tokens[1]);
        heading._pos_type = std::stoi(tokens[2]);
        heading._baseline_length = std::stod(tokens[3]);
        heading._heading = (std::stod(tokens[4]) - heading_offset_deg) * constants::_D2R;
        heading._heading_std = std::stod(tokens[5]) * constants::_D2R;

        heading_data_.push_back(heading);
    }

    return !heading_data_.empty();
}

double DataReader::ParseTimestampFromPath(const std::string& path) {
    const std::string stem = std::filesystem::path(path).stem().string();
    std::string digits;
    for (char ch : stem) {
        if (std::isdigit(static_cast<unsigned char>(ch))) {
            digits.push_back(ch);
        }
    }
    if (digits.empty()) {
        return 0.0;
    }

    const double raw = std::stod(digits);
    if (raw > 1.0e14) {
        return raw * 1.0e-6;
    }
    if (raw > 1.0e11) {
        return raw * 1.0e-3;
    }
    return raw;
}

bool DataReader::LoadLidar(const std::string& path) {
    lidar_data_.clear();
    if (path.empty() || !std::filesystem::exists(path)) {
        std::cerr << "LiDAR 路径不存在: " << path << std::endl;
        return false;
    }

    for (const auto& entry : std::filesystem::directory_iterator(path)) {
        if (!entry.is_regular_file()) {
            continue;
        }
        const std::string ext = entry.path().extension().string();
        if (ext != ".pcd" && ext != ".txt") {
            continue;
        }

        LidarScanInfo scan;
        scan._path = entry.path().string();
        scan._timestamp = ParseTimestampFromPath(scan._path);
        scan._end_time = scan._timestamp;
        if (scan._timestamp > 0.0) {
            lidar_data_.push_back(scan);
        }
    }

    std::sort(lidar_data_.begin(), lidar_data_.end(), [](const auto& lhs, const auto& rhs) {
        return lhs._timestamp < rhs._timestamp;
    });

    return !lidar_data_.empty();
}

double DataReader::GetStartTime() const {
    double timestamp = std::numeric_limits<double>::max();
    if (!imu_data_.empty()) {
        timestamp = std::min(timestamp, imu_data_.front()._timestamp);
    }
    if (!gnsspos_data_.empty()) {
        timestamp = std::min(timestamp, gnsspos_data_.front()._timestamp);
    }
    if (!gnssvel_data_.empty()) {
        timestamp = std::min(timestamp, gnssvel_data_.front()._timestamp);
    }
    if (!heading_data_.empty()) {
        timestamp = std::min(timestamp, heading_data_.front()._timestamp);
    }
    if (!lidar_data_.empty()) {
        timestamp = std::min(timestamp, lidar_data_.front()._timestamp);
    }
    return timestamp;
}

double DataReader::GetEndTime() const {
    double timestamp = std::numeric_limits<double>::lowest();
    if (!imu_data_.empty()) {
        timestamp = std::max(timestamp, imu_data_.back()._timestamp);
    }
    if (!gnsspos_data_.empty()) {
        timestamp = std::max(timestamp, gnsspos_data_.back()._timestamp);
    }
    if (!gnssvel_data_.empty()) {
        timestamp = std::max(timestamp, gnssvel_data_.back()._timestamp);
    }
    if (!heading_data_.empty()) {
        timestamp = std::max(timestamp, heading_data_.back()._timestamp);
    }
    if (!lidar_data_.empty()) {
        timestamp = std::max(timestamp, lidar_data_.back()._timestamp);
    }
    return timestamp;
}

}  // namespace msf
