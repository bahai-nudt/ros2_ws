#include "local_map.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>

#include "ikd_Tree.h"

#ifdef MP_EN
#include <omp.h>
#endif

namespace msf {
namespace lidar {
namespace {

int g_debug_insert_seq = []() {
    const char* s = std::getenv("MSF_DEBUG_INSERT_SEQ");
    return s ? std::atoi(s) : -1;
}();
int g_debug_insert_count = 0;

float calc_dist(const PointType& p1, const PointType& p2) {
    const float dx = p1.x - p2.x;
    const float dy = p1.y - p2.y;
    const float dz = p1.z - p2.z;
    return dx * dx + dy * dy + dz * dz;
}

bool esti_plane(Eigen::Matrix<float, 4, 1>& pca_result,
                const PointVector& points,
                int num_match_points,
                float threshold) {
    if (num_match_points <= 0 || static_cast<int>(points.size()) < num_match_points) {
        return false;
    }

    Eigen::Matrix<float, Eigen::Dynamic, 3> A(num_match_points, 3);
    Eigen::Matrix<float, Eigen::Dynamic, 1> b(num_match_points);
    b.setConstant(-1.0f);

    for (int j = 0; j < num_match_points; ++j) {
        A(j, 0) = points[j].x;
        A(j, 1) = points[j].y;
        A(j, 2) = points[j].z;
    }

    const Eigen::Vector3f normvec = A.colPivHouseholderQr().solve(b);
    const float norm = normvec.norm();
    if (!std::isfinite(norm) || norm <= 1e-12f) {
        return false;
    }

    pca_result.head<3>() = normvec / norm;
    pca_result(3) = 1.0f / norm;

    for (int j = 0; j < num_match_points; ++j) {
        const float distance = pca_result(0) * points[j].x +
                               pca_result(1) * points[j].y +
                               pca_result(2) * points[j].z +
                               pca_result(3);
        if (std::fabs(distance) > threshold) {
            return false;
        }
    }

    return true;
}

}  // namespace

LocalMap::LocalMap() : ikdtree_(std::make_unique<KD_TREE<PointType>>()) {}
LocalMap::~LocalMap() = default;

int LocalMap::ValidSize() {
    return state_ == MapState::Ready ? ikdtree_->validnum() : 0;
}

bool LocalMap::Init(const GlobalConfig& config, const Eigen::Vector3d& anchor_lla_deg) {
    config_ = config;
    anchor_lla_deg_ = anchor_lla_deg;

    if (config_._lidar._point_filter_num <= 0) config_._lidar._point_filter_num = 1;
    if (config_._lidar._frame_voxel_size <= 0.0) config_._lidar._frame_voxel_size = 0.5;
    if (config_._lidar._map_voxel_size <= 0.0) config_._lidar._map_voxel_size = 0.5;
    if (config_._lidar._iterated_update_noise_var <= 0.0) config_._lidar._iterated_update_noise_var = 0.001;
    if (config_._lidar._nearest_search_max_sq_dist <= 0.0) config_._lidar._nearest_search_max_sq_dist = 5.0;
    if (config_._lidar._plane_fit_threshold_m <= 0.0) config_._lidar._plane_fit_threshold_m = 0.1;
    if (config_._lidar._residual_score_scale <= 0.0) config_._lidar._residual_score_scale = 0.9;
    if (config_._lidar._residual_score_min <= 0.0) config_._lidar._residual_score_min = 0.9;

    ikdtree_->set_downsample_param(static_cast<float>(config_._lidar._map_voxel_size));

#ifdef MP_EN
    mp_proc_num_ = omp_get_num_procs();
    std::cout << "[LiDAR] OpenMP enabled, target threads: " << mp_proc_num_ << std::endl;
#else
    mp_proc_num_ = 1;
#endif

    initialized_ = true;
    state_ = MapState::Unbuilt;
    local_map_initialized_ = false;
    incremental_map_initialized_ = false;
    nearest_points_.clear();
    point_selected_surf_.clear();
    return true;
}

void LocalMap::Build(const PointCloudXYZI::Ptr& cloud_world) {
    if (!initialized_ || !cloud_world || cloud_world->empty()) {
        return;
    }

    ikdtree_->Build(cloud_world->points);
    state_ = MapState::Ready;
    incremental_map_initialized_ = false;
    nearest_points_.assign(cloud_world->size(), PointVector{});
    point_selected_surf_.assign(cloud_world->size(), 1);
}

void LocalMap::TrimFov(const Eigen::Vector3d& lidar_position_enu) {
    boxes_to_remove_.clear();

    if (!local_map_initialized_) {
        for (int i = 0; i < 3; ++i) {
            local_map_points_.vertex_min[i] = static_cast<float>(lidar_position_enu(i) - config_._lidar._cube_len / 2.0);
            local_map_points_.vertex_max[i] = static_cast<float>(lidar_position_enu(i) + config_._lidar._cube_len / 2.0);
        }
        local_map_initialized_ = true;
        return;
    }

    float dist_to_map_edge[3][2];
    bool need_move = false;
    constexpr double kMoveThreshold = 1.5;
    for (int i = 0; i < 3; ++i) {
        dist_to_map_edge[i][0] = static_cast<float>(std::fabs(lidar_position_enu(i) - local_map_points_.vertex_min[i]));
        dist_to_map_edge[i][1] = static_cast<float>(std::fabs(lidar_position_enu(i) - local_map_points_.vertex_max[i]));
        if (dist_to_map_edge[i][0] <= kMoveThreshold * config_._lidar._det_range ||
            dist_to_map_edge[i][1] <= kMoveThreshold * config_._lidar._det_range) {
            need_move = true;
        }
    }
    if (!need_move) {
        return;
    }

    Bound new_local_map_points = local_map_points_;
    Bound tmp_boxpoints;
    const double mov_dist = std::max((config_._lidar._cube_len - 2.0 * kMoveThreshold * config_._lidar._det_range) * 0.5 * 0.9,
                                     config_._lidar._det_range * (kMoveThreshold - 1.0));
    for (int i = 0; i < 3; ++i) {
        tmp_boxpoints = local_map_points_;
        if (dist_to_map_edge[i][0] <= kMoveThreshold * config_._lidar._det_range) {
            new_local_map_points.vertex_max[i] -= static_cast<float>(mov_dist);
            new_local_map_points.vertex_min[i] -= static_cast<float>(mov_dist);
            tmp_boxpoints.vertex_min[i] = local_map_points_.vertex_max[i] - static_cast<float>(mov_dist);
            boxes_to_remove_.push_back(tmp_boxpoints);
        } else if (dist_to_map_edge[i][1] <= kMoveThreshold * config_._lidar._det_range) {
            new_local_map_points.vertex_max[i] += static_cast<float>(mov_dist);
            new_local_map_points.vertex_min[i] += static_cast<float>(mov_dist);
            tmp_boxpoints.vertex_max[i] = local_map_points_.vertex_min[i] + static_cast<float>(mov_dist);
            boxes_to_remove_.push_back(tmp_boxpoints);
        }
    }
    local_map_points_ = new_local_map_points;

    PointVector points_history;
    ikdtree_->acquire_removed_points(points_history);
    if (!boxes_to_remove_.empty()) {
        std::vector<BoxPointType> vendor_boxes;
        vendor_boxes.reserve(boxes_to_remove_.size());
        for (const Bound& bound : boxes_to_remove_) {
            BoxPointType box;
            for (int i = 0; i < 3; ++i) {
                box.vertex_min[i] = bound.vertex_min[i];
                box.vertex_max[i] = bound.vertex_max[i];
            }
            vendor_boxes.push_back(box);
        }
        ikdtree_->Delete_Point_Boxes(vendor_boxes);
    }
}

void LocalMap::Match(const PointCloudXYZI::Ptr& cloud_world, bool rematch,
                     std::vector<LidarPlaneConstraint>& planes,
                     LidarFeatureDiag& diag) {
    planes.clear();
    diag.filtered_points = cloud_world ? static_cast<int>(cloud_world->size()) : 0;
    diag.iterated_update_noise_var = config_._lidar._iterated_update_noise_var;
    diag.candidate_features = 0;
    diag.effective_features = 0;

    if (!initialized_ || state_ != MapState::Ready || !cloud_world || cloud_world->empty()) {
        return;
    }

    if (nearest_points_.size() != cloud_world->size()) {
        nearest_points_.assign(cloud_world->size(), PointVector{});
    }
    if (point_selected_surf_.size() != cloud_world->size()) {
        point_selected_surf_.assign(cloud_world->size(), 1);
    }

    const size_t point_count = cloud_world->size();
    std::vector<LidarPlaneConstraint> constraint_buffer(point_count);
    std::vector<char> constraint_valid(point_count, 0);

#ifdef MP_EN
    omp_set_num_threads(mp_proc_num_);
#pragma omp parallel for
#endif
    for (int i = 0; i < static_cast<int>(point_count); ++i) {
        const PointType& point_world = cloud_world->points[static_cast<size_t>(i)];

        std::vector<float> point_search_sq_dis(static_cast<size_t>(config_._lidar._nearest_points));
        PointVector& points_near = nearest_points_[static_cast<size_t>(i)];
        if (rematch) {
            ikdtree_->Nearest_Search(point_world, config_._lidar._nearest_points,
                                     points_near, point_search_sq_dis);
            point_selected_surf_[static_cast<size_t>(i)] =
                (static_cast<int>(points_near.size()) >= config_._lidar._nearest_points &&
                 point_search_sq_dis[static_cast<size_t>(config_._lidar._nearest_points - 1)] <=
                     static_cast<float>(config_._lidar._nearest_search_max_sq_dist)) ? 1 : 0;
        }
        if (!point_selected_surf_[static_cast<size_t>(i)]) {
            continue;
        }

        Eigen::Matrix<float, 4, 1> plane;
        point_selected_surf_[static_cast<size_t>(i)] = 0;
        if (!esti_plane(plane, points_near, config_._lidar._nearest_points,
                        static_cast<float>(config_._lidar._plane_fit_threshold_m))) {
            continue;
        }

        point_selected_surf_[static_cast<size_t>(i)] = 1;

        LidarPlaneConstraint constraint;
        constraint.point_index = static_cast<size_t>(i);
        constraint.normal_enu = Eigen::Vector3d(plane(0), plane(1), plane(2));
        constraint.plane_d = plane(3);
        constraint_buffer[static_cast<size_t>(i)] = constraint;
        constraint_valid[static_cast<size_t>(i)] = 1;
    }

    planes.reserve(point_count);
    for (size_t i = 0; i < point_count; ++i) {
        if (constraint_valid[i]) {
            planes.push_back(constraint_buffer[i]);
        }
    }

    diag.candidate_features = static_cast<int>(planes.size());
}

void LocalMap::Insert(const PointCloudXYZI::Ptr& cloud_world) {
    if (!initialized_ || state_ != MapState::Ready || !cloud_world || cloud_world->empty()) {
        return;
    }

    const bool debug_this = (g_debug_insert_seq >= 0 &&
                             g_debug_insert_count++ == g_debug_insert_seq);
    PointVector points_to_add;
    PointVector points_no_need_downsample;
    points_to_add.reserve(cloud_world->size());
    points_no_need_downsample.reserve(cloud_world->size());

    for (size_t i = 0; i < cloud_world->size(); ++i) {
        const PointType& point_world = cloud_world->points[i];
        const char* branch = "else_add";
        float dist = 0.0f;
        PointType mid_point;
        if (i < nearest_points_.size() && !nearest_points_[i].empty() && incremental_map_initialized_) {
            const PointVector& points_near = nearest_points_[i];
            bool need_add = true;
            mid_point.x = std::floor(point_world.x / static_cast<float>(config_._lidar._map_voxel_size)) *
                          static_cast<float>(config_._lidar._map_voxel_size) + 0.5f * static_cast<float>(config_._lidar._map_voxel_size);
            mid_point.y = std::floor(point_world.y / static_cast<float>(config_._lidar._map_voxel_size)) *
                          static_cast<float>(config_._lidar._map_voxel_size) + 0.5f * static_cast<float>(config_._lidar._map_voxel_size);
            mid_point.z = std::floor(point_world.z / static_cast<float>(config_._lidar._map_voxel_size)) *
                          static_cast<float>(config_._lidar._map_voxel_size) + 0.5f * static_cast<float>(config_._lidar._map_voxel_size);
            dist = calc_dist(point_world, mid_point);
            if (std::fabs(points_near[0].x - mid_point.x) > 0.5f * static_cast<float>(config_._lidar._map_voxel_size) &&
                std::fabs(points_near[0].y - mid_point.y) > 0.5f * static_cast<float>(config_._lidar._map_voxel_size) &&
                std::fabs(points_near[0].z - mid_point.z) > 0.5f * static_cast<float>(config_._lidar._map_voxel_size)) {
                branch = "no_need";
                points_no_need_downsample.push_back(point_world);
                if (debug_this) {
                    std::cerr << "[OUR INSERT] i=" << i
                              << " branch=" << branch
                              << " p=" << point_world.x << "," << point_world.y << "," << point_world.z
                              << " mid=" << mid_point.x << "," << mid_point.y << "," << mid_point.z
                              << " dist=" << dist
                              << " near0=" << points_near[0].x << "," << points_near[0].y << "," << points_near[0].z
                              << " near_n=" << points_near.size()
                              << " near_d=";
                    for (const auto& np : points_near) {
                        std::cerr << calc_dist(np, mid_point) << ",";
                    }
                    std::cerr << std::endl;
                }
                continue;
            }
            for (int j = 0; j < config_._lidar._nearest_points; ++j) {
                if (static_cast<int>(points_near.size()) < config_._lidar._nearest_points) break;
                if (calc_dist(points_near[static_cast<size_t>(j)], mid_point) < dist) {
                    need_add = false;
                    break;
                }
            }
            if (need_add) {
                branch = "add";
                points_to_add.push_back(point_world);
            } else {
                branch = "skip";
            }
        } else {
            points_to_add.push_back(point_world);
            incremental_map_initialized_ = true;
        }
        if (debug_this) {
            std::cerr << "[OUR INSERT] i=" << i
                      << " branch=" << branch
                      << " p=" << point_world.x << "," << point_world.y << "," << point_world.z
                      << " mid=" << mid_point.x << "," << mid_point.y << "," << mid_point.z
                      << " dist=" << dist
                      << " near0=" << (nearest_points_[i].empty() ? -1.0f : nearest_points_[i][0].x)
                      << "," << (nearest_points_[i].empty() ? -1.0f : nearest_points_[i][0].y)
                      << "," << (nearest_points_[i].empty() ? -1.0f : nearest_points_[i][0].z)
                      << " near_n=" << nearest_points_[i].size()
                      << " near_d=";
                    for (const auto& np : nearest_points_[i]) {
                        std::cerr << calc_dist(np, mid_point) << ",";
                    }
                    std::cerr << std::endl;
        }
    }

    ikdtree_->Add_Points(points_to_add, true);
    ikdtree_->Add_Points(points_no_need_downsample, false);
}

}  // namespace lidar
}  // namespace msf
