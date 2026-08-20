#pragma once

#include <memory>
#include <vector>

#include <Eigen/Eigen>

#include "config/global_config.h"
#include "lidar_types.h"

// 第三方 ikd-Tree 仅前置声明：完整头文件含宏和 `using namespace std;`，
// 不得出现在公共 include 链中。定义在 local_map.cpp。
template <typename PointType>
class KD_TREE;

namespace msf {
namespace lidar {

enum class MapState {
    Unbuilt,
    Ready
};

/**
 * 局部地图：纯 ENU 几何对象，不 include 任何导航状态类型。
 * 输入输出都是世界 ENU 点云 / 平面约束；位姿/外参由调用方负责。
 */
class LocalMap {
public:
    LocalMap();
    ~LocalMap();

    LocalMap(const LocalMap&) = delete;
    LocalMap& operator=(const LocalMap&) = delete;

    bool Init(const GlobalConfig& config, const Eigen::Vector3d& anchor_lla_deg);
    bool IsBuilt() const { return state_ == MapState::Ready; }

    /** 当前 ikd-Tree 有效点数（用于回归/调试）。 */
    int ValidSize();

    /** 首帧全量建图（对应 POST_MSF buildMap）。 */
    void Build(const PointCloudXYZI::Ptr& cloud_world);

    /** 以雷达 ENU 位置为中心滑动局部地图 FOV（对应 updateLocalMapFov）。 */
    void TrimFov(const Eigen::Vector3d& lidar_position_enu);

    /** 世界系点云 -> 平面约束（对应 matchWorldPoints；rematch 决定是否重做 kNN）。 */
    void Match(const PointCloudXYZI::Ptr& cloud_world, bool rematch,
               std::vector<LidarPlaneConstraint>& planes,
               LidarFeatureDiag& diag);

    /** 更新被接受后增量插入（对应 commitWorldPoints）。 */
    void Insert(const PointCloudXYZI::Ptr& cloud_world);

private:
    struct Bound {
        float vertex_min[3]{};
        float vertex_max[3]{};
    };

    GlobalConfig config_;
    Eigen::Vector3d anchor_lla_deg_ = Eigen::Vector3d::Zero();
    MapState state_ = MapState::Unbuilt;

    std::unique_ptr<KD_TREE<PointType>> ikdtree_;
    std::vector<PointVector> nearest_points_;
    std::vector<char> point_selected_surf_;
    Bound local_map_points_;
    std::vector<Bound> boxes_to_remove_;

    bool initialized_ = false;
    bool local_map_initialized_ = false;
    bool incremental_map_initialized_ = false;
    int mp_proc_num_ = 1;
};

}  // namespace lidar
}  // namespace msf
