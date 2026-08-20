#include <cmath>
#include <filesystem>
#include <iostream>
#include <memory>

#include <pcl/io/pcd_io.h>

#include "config/global_config.h"
#include "lidar_types.h"
#include "math/constants.h"
#include "math/coordinate.h"
#include "pcd_preprocess.h"
#include "lidar_visualizer.h"
#include "types/nav_state.h"
#include "types/sensors/lidar_message.h"

namespace {

using msf::lidar::PointCloudXYZI;
using msf::lidar::PointType;
using msf::lidar::PointImuToWorld;
using msf::lidar::PositionEnu;
using msf::lidar::TransformCloudImuToWorld;
using msf::lidar::TransformLidarCloudToImu;

bool Near(double actual, double expected, double tol = 1.0e-6) {
    return std::fabs(actual - expected) <= tol;
}

bool Near(const Eigen::Vector3d& actual, const Eigen::Vector3d& expected, double tol = 1.0e-6) {
    return Near(actual.x(), expected.x(), tol) &&
           Near(actual.y(), expected.y(), tol) &&
           Near(actual.z(), expected.z(), tol);
}

PointType MakePoint(float x, float y, float z) {
    PointType p;
    p.x = x;
    p.y = y;
    p.z = z;
    p.intensity = 0.0f;
    return p;
}

msf::lidar::PointXYZIRT MakePointIrt(float x, float y, float z, double time) {
    msf::lidar::PointXYZIRT p;
    p.x = x;
    p.y = y;
    p.z = z;
    p.intensity = 0.0f;
    p.ring = 0;
    p.time = time;
    return p;
}

std::filesystem::path MakeTempDir(const std::string& name) {
    const auto dir = std::filesystem::temp_directory_path() / name;
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir);
    return dir;
}

// 名义状态正好位于局部地图原点时，PositionEnu 应为 0
bool TestPositionEnuOrigin() {
    msf::GlobalConfig config;
    config._local_map_lla_deg = Eigen::Vector3d(30.0, 120.0, 100.0);

    msf::NominalState nominal;
    nominal._pos = Eigen::Vector3d(30.0 * msf::constants::_D2R,
                                   120.0 * msf::constants::_D2R,
                                   100.0);

    const Eigen::Vector3d enu = PositionEnu(nominal, config);
    return Near(enu, Eigen::Vector3d::Zero());
}

// LiDAR 系点云通过 R_bl / T_lb_m 变换到 IMU 系
bool TestTransformLidarToImu() {
    msf::GlobalConfig config;
    config._calibration._R_bl = Eigen::Matrix3d::Identity();
    config._calibration._T_lb_m = Eigen::Vector3d(0.1, 0.2, 0.3);

    PointCloudXYZI::Ptr cloud_lidar(new PointCloudXYZI());
    cloud_lidar->push_back(MakePoint(1.0f, 2.0f, 3.0f));

    const auto cloud_imu = TransformLidarCloudToImu(cloud_lidar, config);
    if (!cloud_imu || cloud_imu->size() != 1) {
        return false;
    }
    const auto& p = cloud_imu->points[0];
    return Near(p.x, 1.1f) && Near(p.y, 2.2f) && Near(p.z, 3.3f);
}

// 读取 PCD：过滤近点，相对时间写入 intensity，并推算帧尾时间
bool TestLoadLidarPcd() {
    const auto dir = MakeTempDir("msf_lidar_pcd");
    const auto path = dir / "scan.pcd";

    pcl::PointCloud<msf::lidar::PointXYZIRT> raw;
    raw.points.push_back(MakePointIrt(10.0f, 0.0f, 0.0f, 0.10));
    raw.points.push_back(MakePointIrt(20.0f, 0.0f, 0.0f, 0.20));
    raw.points.push_back(MakePointIrt(0.5f, 0.0f, 0.0f, 0.15));  // 距离过近，应被丢弃
    raw.width = static_cast<std::uint32_t>(raw.points.size());
    raw.height = 1;
    raw.is_dense = true;

    if (pcl::io::savePCDFileBinary(path.string(), raw) != 0) {
        return false;
    }

    PointCloudXYZI::Ptr cloud(new PointCloudXYZI());
    double scan_end_time = 0.0;
    if (!msf::lidar::LoadLidarPcd(path.string(), 100.0, cloud, &scan_end_time)) {
        return false;
    }
    if (!cloud || cloud->size() != 2) {
        return false;
    }
    const auto& p0 = cloud->points[0];
    const auto& p1 = cloud->points[1];
    return Near(p0.x, 10.0f) && Near(p0.intensity, 0.0f) &&
           Near(p1.x, 20.0f) && Near(p1.intensity, 100.0f) &&
           Near(scan_end_time, 100.1);
}

// 抽稀 + 体素滤波：point_filter_num=2 时应保留 0/2/4 三个点
bool TestFilterRawScan() {
    msf::GlobalConfig config;
    config._lidar._point_filter_num = 2;
    config._lidar._frame_voxel_size = 0.5;

    PointCloudXYZI::Ptr raw(new PointCloudXYZI());
    for (int i = 0; i < 5; ++i) {
        raw->push_back(MakePoint(static_cast<float>(i), 0.0f, 0.0f));
    }

    const auto filtered = msf::lidar::FilterRawScan(raw, config);
    return filtered && filtered->size() == 3;
}

// IMU 系点云通过 PositionEnu + Cnb 变换到世界 ENU 系
bool TestTransformImuToWorld() {
    msf::GlobalConfig config;
    config._local_map_lla_deg = Eigen::Vector3d(30.0, 120.0, 100.0);

    const Eigen::Vector3d origin_rad(30.0 * msf::constants::_D2R,
                                     120.0 * msf::constants::_D2R,
                                     100.0);
    msf::NominalState nominal;
    nominal._pos = msf::Coordinate::enu2lla(origin_rad, Eigen::Vector3d(10.0, 20.0, 30.0));
    nominal._Cnb = Eigen::Matrix3d::Identity();

    const Eigen::Vector3d expected_enu = PositionEnu(nominal, config) + Eigen::Vector3d(1.0, 0.0, 0.0);

    PointCloudXYZI::Ptr cloud_imu(new PointCloudXYZI());
    cloud_imu->push_back(MakePoint(1.0f, 0.0f, 0.0f));

    const auto cloud_world = TransformCloudImuToWorld(cloud_imu, nominal, config);
    if (!cloud_world || cloud_world->size() != 1) {
        return false;
    }
    const auto& p = cloud_world->points[0];
    return Near(Eigen::Vector3d(p.x, p.y, p.z), expected_enu);
}

// 输出路径：<output>/<lidar_frame_id>/<原文件名>
bool TestWorldPcdOutputPath() {
    msf::GlobalConfig config;
    config._file._output_data_dir = "/tmp/msf_world_pcd_out";
    config._lidar_frame_id = "lidar_front";

    msf::LidarScanInfo scan;
    scan._path = "/data/decode_cyber/lidar_front/scan_001.pcd";
    const std::string path = msf::lidar::WorldPcdOutputPath(config, scan);
    return path == "/tmp/msf_world_pcd_out/lidar_front/scan_001.pcd";
}

bool TestExportWorldPcdEmpty() {
    msf::GlobalConfig config;
    msf::NominalState nominal;
    PointCloudXYZI::Ptr empty(new PointCloudXYZI());
    std::string reason;
    return !msf::lidar::ExportWorldPcd(empty, nominal, config, "/tmp/unused.pcd", &reason) &&
           reason == "pcd_load_failed";
}

// 雷达系 → IMU → 世界 ENU，写 binary PCD 再读回
bool TestExportWorldPcdRoundtrip() {
    const auto dir = MakeTempDir("msf_world_pcd_export");
    const auto out_path = dir / "lidar_front" / "scan.pcd";

    msf::GlobalConfig config;
    config._file._output_data_dir = dir.string();
    config._lidar_frame_id = "lidar_front";
    config._local_map_lla_deg = Eigen::Vector3d(30.0, 120.0, 100.0);
    config._calibration._R_bl = Eigen::Matrix3d::Identity();
    config._calibration._T_lb_m = Eigen::Vector3d(1.0, 0.0, 0.0);

    msf::NominalState nominal;
    nominal._pos = Eigen::Vector3d(30.0 * msf::constants::_D2R,
                                   120.0 * msf::constants::_D2R,
                                   100.0);
    nominal._Cnb = Eigen::Matrix3d::Identity();

    PointCloudXYZI::Ptr cloud_lidar(new PointCloudXYZI());
    PointType pt;
    pt.x = 1.0f;
    pt.y = 2.0f;
    pt.z = 3.0f;
    pt.intensity = 12.5f;
    cloud_lidar->push_back(pt);

    std::string reason;
    if (!msf::lidar::ExportWorldPcd(cloud_lidar, nominal, config, out_path.string(), &reason)) {
        return false;
    }

    pcl::PointCloud<PointType> loaded;
    if (pcl::io::loadPCDFile<PointType>(out_path.string(), loaded) != 0 || loaded.size() != 1) {
        return false;
    }
    // p_imu = (2,2,3), 原点处 Cnb=I → 世界 (2,2,3)
    return Near(loaded.points[0].x, 2.0f) &&
           Near(loaded.points[0].y, 2.0f) &&
           Near(loaded.points[0].z, 3.0f) &&
           Near(loaded.points[0].intensity, 12.5f);
}

// 开关关闭时不写文件
bool TestMaybeExportWorldPcdDisabled() {
    const auto dir = MakeTempDir("msf_world_pcd_disabled");
    msf::GlobalConfig config;
    config._file._output_data_dir = dir.string();
    config._lidar_frame_id = "lidar_front";
    config._out._output_world_pcd = false;
    config._local_map_lla_deg = Eigen::Vector3d(30.0, 120.0, 100.0);

    msf::NominalState nominal;
    nominal._pos = Eigen::Vector3d(30.0 * msf::constants::_D2R,
                                   120.0 * msf::constants::_D2R,
                                   100.0);
    nominal._Cnb = Eigen::Matrix3d::Identity();

    msf::LidarScanInfo scan;
    scan._path = (dir / "src.pcd").string();
    PointCloudXYZI::Ptr cloud(new PointCloudXYZI());
    cloud->push_back(MakePoint(1.0f, 0.0f, 0.0f));

    if (!msf::lidar::EnsureWorldPcdOutputDir(config)) {
        return false;
    }
    msf::lidar::MaybeExportWorldPcd(scan, cloud, nominal, config);
    return !std::filesystem::exists(dir / "lidar_front");
}

bool TestEffectiveVisualizationMode() {
    return msf::EffectiveVisualizationMode(0, false) == 0 &&
           msf::EffectiveVisualizationMode(0, true) == 3 &&
           msf::EffectiveVisualizationMode(1, false) == 1 &&
           msf::EffectiveVisualizationMode(1, true) == 1 &&
           msf::EffectiveVisualizationMode(2, true) == 2 &&
           msf::EffectiveVisualizationMode(5, false) == 3 &&
           msf::EffectiveVisualizationMode(-2, true) == 3;
}

bool TestVisualizationCloudModes() {
    msf::GlobalConfig config;
    config._lidar._point_filter_num = 1;
    config._lidar._frame_voxel_size = 0.01;
    config._calibration._R_bl = Eigen::Matrix3d::Identity();
    config._calibration._T_lb_m = Eigen::Vector3d::Zero();
    config._local_map_lla_deg = Eigen::Vector3d(30.0, 120.0, 100.0);

    msf::NominalState nominal;
    nominal._Cnb = Eigen::Matrix3d::Identity();
    nominal._pos = Eigen::Vector3d(30.0 * msf::constants::_D2R,
                                   120.0 * msf::constants::_D2R,
                                   100.0);

    PointCloudXYZI::Ptr raw(new PointCloudXYZI());
    raw->push_back(MakePoint(1.0f, 0.0f, 0.0f));
    PointCloudXYZI::Ptr undistorted(new PointCloudXYZI());
    undistorted->push_back(MakePoint(2.0f, 0.0f, 0.0f));
    PointCloudXYZI::Ptr cloud_imu(new PointCloudXYZI());
    cloud_imu->push_back(MakePoint(5.0f, 0.0f, 0.0f));

    const auto mode1 = msf::lidar::VisualizationCloud(1, raw, undistorted, cloud_imu, nominal, config);
    const auto mode2 = msf::lidar::VisualizationCloud(2, raw, undistorted, cloud_imu, nominal, config);
    const auto mode3 = msf::lidar::VisualizationCloud(3, raw, undistorted, cloud_imu, nominal, config);
    const auto mode0 = msf::lidar::VisualizationCloud(0, raw, undistorted, cloud_imu, nominal, config);
    return mode1 && mode1->size() == 1 && Near(static_cast<double>(mode1->points[0].x), 1.0) &&
           mode2 && mode2->size() == 1 && Near(static_cast<double>(mode2->points[0].x), 2.0) &&
           mode3 && mode3->size() == 1 && Near(static_cast<double>(mode3->points[0].x), 5.0) &&
           mode0 && mode0->empty();
}

}  // namespace

int main() {
    struct TestCase {
        const char* name;
        bool (*fn)();
    };

    const TestCase tests[] = {
        {"position_enu_origin", TestPositionEnuOrigin},
        {"transform_lidar_to_imu", TestTransformLidarToImu},
        {"transform_imu_to_world", TestTransformImuToWorld},
        {"load_lidar_pcd", TestLoadLidarPcd},
        {"filter_raw_scan", TestFilterRawScan},
        {"world_pcd_output_path", TestWorldPcdOutputPath},
        {"export_world_pcd_empty", TestExportWorldPcdEmpty},
        {"export_world_pcd_roundtrip", TestExportWorldPcdRoundtrip},
        {"maybe_export_world_pcd_disabled", TestMaybeExportWorldPcdDisabled},
        {"effective_visualization_mode", TestEffectiveVisualizationMode},
        {"visualization_cloud_modes", TestVisualizationCloudModes},
    };

    int failed = 0;
    for (const auto& test : tests) {
        if (!test.fn()) {
            std::cerr << "[FAIL] " << test.name << std::endl;
            ++failed;
        } else {
            std::cout << "[PASS] " << test.name << std::endl;
        }
    }
    return failed == 0 ? 0 : 1;
}
