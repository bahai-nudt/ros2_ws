#include <cmath>
#include <iostream>
#include <memory>

#include "config/global_config.h"
#include "lidar_types.h"
#include "local_map.h"

namespace {

using msf::lidar::LidarFeatureDiag;
using msf::lidar::LidarPlaneConstraint;
using msf::lidar::LocalMap;
using msf::lidar::PointCloudXYZI;
using msf::lidar::PointType;

bool Near(double actual, double expected, double tol = 1.0e-4) {
    return std::fabs(actual - expected) <= tol;
}

PointType MakePoint(float x, float y, float z) {
    PointType p;
    p.x = x;
    p.y = y;
    p.z = z;
    p.intensity = 0.0f;
    return p;
}

void SetupConfig(msf::GlobalConfig& config) {
    config._lidar._nearest_points = 5;
    config._lidar._nearest_search_max_sq_dist = 10.0;
    config._lidar._plane_fit_threshold_m = 0.1;
    config._lidar._map_voxel_size = 0.5;
    config._lidar._frame_voxel_size = 0.5;
    config._lidar._cube_len = 1000.0;
    config._lidar._det_range = 100.0;
    config._lidar._iterated_update_noise_var = 0.001;
}

PointCloudXYZI::Ptr MakePlaneZ1() {
    PointCloudXYZI::Ptr cloud(new PointCloudXYZI());
    cloud->push_back(MakePoint(0.0f, 0.0f, 1.0f));
    cloud->push_back(MakePoint(1.0f, 0.0f, 1.0f));
    cloud->push_back(MakePoint(-1.0f, 0.0f, 1.0f));
    cloud->push_back(MakePoint(0.0f, 1.0f, 1.0f));
    cloud->push_back(MakePoint(0.0f, -1.0f, 1.0f));
    return cloud;
}

// 建图 + 平面匹配：z=1 平面应解出 n=(0,0,-1), d=1
bool TestBuildAndMatch() {
    msf::GlobalConfig config;
    SetupConfig(config);

    auto map = std::make_unique<LocalMap>();
    if (!map->Init(config, Eigen::Vector3d(30.0, 120.0, 100.0))) {
        return false;
    }
    if (map->IsBuilt()) {
        return false;
    }

    const auto cloud_world = MakePlaneZ1();
    map->Build(cloud_world);
    if (!map->IsBuilt()) {
        return false;
    }

    std::vector<LidarPlaneConstraint> planes;
    LidarFeatureDiag diag;
    map->Match(cloud_world, true, planes, diag);

    if (planes.size() != 5 || diag.candidate_features != 5 || diag.effective_features != 0) {
        return false;
    }
    for (const auto& plane : planes) {
        if (!Near(plane.normal_enu.x(), 0.0) ||
            !Near(plane.normal_enu.y(), 0.0) ||
            !Near(plane.normal_enu.z(), -1.0) ||
            !Near(plane.plane_d, 1.0)) {
            return false;
        }
    }
    return true;
}

// rematch=false 应复用缓存关联，仍能产出约束
bool TestMatchWithoutRematch() {
    msf::GlobalConfig config;
    SetupConfig(config);

    auto map = std::make_unique<LocalMap>();
    map->Init(config, Eigen::Vector3d(30.0, 120.0, 100.0));
    const auto cloud_world = MakePlaneZ1();
    map->Build(cloud_world);

    std::vector<LidarPlaneConstraint> planes;
    LidarFeatureDiag diag;
    map->Match(cloud_world, true, planes, diag);
    if (planes.empty()) {
        return false;
    }

    planes.clear();
    map->Match(cloud_world, false, planes, diag);
    return !planes.empty();
}

// FOV 滑动不应破坏地图状态
bool TestTrimFov() {
    msf::GlobalConfig config;
    SetupConfig(config);

    auto map = std::make_unique<LocalMap>();
    map->Init(config, Eigen::Vector3d(30.0, 120.0, 100.0));
    const auto cloud_world = MakePlaneZ1();
    map->Build(cloud_world);

    map->TrimFov(Eigen::Vector3d::Zero());
    map->TrimFov(Eigen::Vector3d(400.0, 0.0, 0.0));
    return map->IsBuilt();
}

// 增量插入不应破坏地图
bool TestInsert() {
    msf::GlobalConfig config;
    SetupConfig(config);

    auto map = std::make_unique<LocalMap>();
    map->Init(config, Eigen::Vector3d(30.0, 120.0, 100.0));
    const auto cloud_world = MakePlaneZ1();
    map->Build(cloud_world);
    map->Insert(cloud_world);
    return map->IsBuilt();
}

}  // namespace

int main() {
    struct TestCase {
        const char* name;
        bool (*fn)();
    };

    const TestCase tests[] = {
        {"build_and_match", TestBuildAndMatch},
        {"match_without_rematch", TestMatchWithoutRematch},
        {"trim_fov", TestTrimFov},
        {"insert", TestInsert},
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
