// LIO demo：GNSS 仅用于初始化，滤波阶段使用 IMU + LiDAR。
// 时间推进、量测更新、输出格式与 POST_MSF fusion_mode=2 对齐，用于与 ref 输出逐列对比。
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "config/global_config.h"
#include "data_reader.h"
#include "result_writer.h"
#include "yaml_reader.h"
#include "ekf.h"
#include "gnss_initializer.h"
#include "gnss_pos_factor.h"
#include "gnss_vel_factor.h"
#include "heading_factor.h"
#include "ins_initializer.h"
#include "ins_propagator.h"
#include "lidar_plane_factor.h"
#include "lidar_visualizer.h"
#include "local_map.h"
#include "msf_algorithms/zupt.h"
#include "pcd_preprocess.h"
#include "scan_deskewer.h"
#include "math/constants.h"
#include "nominal_advance.h"
#include "process_limit.h"
#include "timeline.h"

namespace {

enum Tag : int { TAG_IMU = 0, TAG_LIDAR = 1 };

using msf::constants::_R2D;

void FillLidarFeature(msf::MeasLogEvent& rec, const msf::lidar::LidarFeatureDiag& diag) {
    rec.has_lidar_feature = true;
    rec.lidar_filtered = diag.filtered_points;
    rec.lidar_candidate = diag.candidate_features;
    rec.lidar_effective = diag.effective_features;
    rec.lidar_iter_noise_var = diag.iterated_update_noise_var;
}

void FillDeskewLog(msf::MeasLogEvent& rec, const msf::lidar::ScanDeskewer& deskewer,
                   const msf::lidar::DeskewDiag* diag) {
    rec.undistort_poses_size = deskewer.PoseCount();
    if (diag == nullptr) {
        return;
    }
    rec.has_undistort = true;
    rec.undistort_max_rot_diff_rad = diag->_max_rot_diff_rad;
    rec.undistort_max_rot_diff_rel_time_ms = diag->_max_rot_diff_rel_time_ms;
    rec.undistort_max_rot_diff_vec = diag->_max_rot_diff_vec;
    rec.undistort_max_compensation_m = diag->_max_compensation_m;
    rec.undistort_mean_compensation_m = diag->_mean_compensation_m;
}

void FillIekfLog(msf::MeasLogEvent& rec, const msf::IekfDiag& diag) {
    rec.has_iekf = true;
    rec.iekf_total_iter = diag.total_iter;
    rec.iekf_converge_iter = diag.converge_iter;
    rec.iekf_final_converged = diag.final_converged;
    rec.iekf_max_dx_last = diag.max_dx_last;
}

bool ParseArgs(int argc, char* argv[], std::string& config_file, std::string& output_dir,
               size_t& max_events, double& max_duration, bool& max_duration_cli, bool& visualize) {
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg.rfind("--config=", 0) == 0) {
            config_file = arg.substr(9);
        } else if (arg.rfind("--output=", 0) == 0) {
            output_dir = arg.substr(9);
        } else if (arg.rfind("--max-events=", 0) == 0) {
            max_events = static_cast<size_t>(std::stoull(arg.substr(13)));
        } else if (arg.rfind("--max-duration=", 0) == 0) {
            max_duration = std::stod(arg.substr(15));
            max_duration_cli = true;
        } else if (arg == "--visualize") {
            visualize = true;
        } else if (arg.rfind("--", 0) == 0) {
            std::cerr << "未知参数: " << arg << std::endl;
            return false;
        } else if (config_file.empty()) {
            config_file = arg;
        } else {
            std::cerr << "未知参数: " << arg << std::endl;
            return false;
        }
    }
    if (config_file.empty()) {
        std::cerr << "用法: lio --config=<yaml> [--output=<dir>] [--max-events=N] "
                  << "[--max-duration=SEC] [--visualize]" << std::endl;
        return false;
    }
    return true;
}

}  // namespace

// 输入参数：
//   --config=<yaml>              配置文件路径（必填；也可用位置参数传同一路径）
//   --output=<dir>               覆盖结果输出目录（可选）
//   --max-events=N               最多处理 N 个事件后退出，0 表示不限制（可选）
//   --max-duration=SEC           自初始化时刻起最多处理 SEC 秒；未给时用 yaml
//                                process_option.max_process_duration_sec（<=0 不限制）
//   --visualize                  强制打开可视化；yaml 为 0 时按世界系(3)打开
int main(int argc, char* argv[]) {
    std::string config_file;
    std::string output_dir;
    size_t max_events = 0;
    double max_duration = 0.0;
    bool max_duration_cli = false;
    bool visualize = false;
    if (!ParseArgs(argc, argv, config_file, output_dir, max_events, max_duration,
                   max_duration_cli, visualize)) {
        return 1;
    }

    msf::GlobalConfig config;
    if (!msf::LoadGlobalConfig(config_file, config)) {
        return 1;
    }
    if (!output_dir.empty()) {
        config._file._output_data_dir = output_dir;
    }
    max_duration = msf::tools::ResolveMaxDuration(max_duration_cli, max_duration, config);
    config._process_option._max_process_duration_sec = max_duration;
    const int viz_mode =
        msf::EffectiveVisualizationMode(config._out._visualization_mode, visualize);
    if (!msf::lidar::EnsureWorldPcdOutputDir(config)) {
        return 1;
    }
    std::cout << "[LiDAR] 可视化: " << msf::VisualizationModeNameZh(viz_mode) << std::endl;
    msf::ResultWriter result_writer;
    if (!result_writer.OpenResultFiles(config)) {
        return 1;
    }

    // init_mode: 0=GNSS+heading；1=IMU 静止均值。滤波阶段事件循环只使用 IMU + LiDAR。
    const bool use_gnss_init = (config._lidar._init_mode == 0);
    msf::DataReader reader;
    if (!reader.LoadData(config, msf::LoadOptions{use_gnss_init, true})) {
        std::cerr << "数据加载失败" << std::endl;
        return 1;
    }

    const auto& imu_data = reader.GetImuData();
    const auto& gnsspos_data = reader.GetGnssPosData();
    const auto& gnssvel_data = reader.GetGnssVelData();
    const auto& heading_data = reader.GetHeadingData();
    const auto& lidar_data = reader.GetLidarData();
    if (imu_data.empty()) {
        std::cerr << "IMU 数据为空" << std::endl;
        return 1;
    }
    if (lidar_data.empty()) {
        std::cerr << "LiDAR 数据为空" << std::endl;
        return 1;
    }
    if (use_gnss_init && (gnsspos_data.empty() || gnssvel_data.empty() || heading_data.empty())) {
        std::cerr << "GNSS/Heading 数据为空（init_mode=0 需要）" << std::endl;
        return 1;
    }

    // 初始化
    msf::NominalState nominal;
    msf::earth eth;
    if (use_gnss_init) {
        if (!msf::gnss::InitializeNominal(imu_data, gnsspos_data, gnssvel_data, heading_data,
                                          config._calibration._T_gb_m,
                                          config._calibration._imu_calib,
                                          nominal, eth)) {
            return 1;
        }
    } else {
        const Eigen::Vector3d origin_lla_rad =
            config._calibration._gloc_origin_lla_deg * msf::constants::_D2R;
        if (!msf::InitializeLIO(imu_data, origin_lla_rad, config._calibration._imu_calib,
                                config._lidar._imu_init_samples, nominal, eth)) {
            return 1;
        }
    }

    msf::Ekf ekf(config._process_option._state_dim);
    ekf.SetInitialState(nominal, eth);
    ekf.InitNoiseFromConfig(config);
    msf::InsPropagator propagator;

    // LiDAR 局部地图锚点 = 滤波初始化位置（LLA 度）
    config._local_map_lla_deg = Eigen::Vector3d(nominal._pos(0) * _R2D,
                                                nominal._pos(1) * _R2D,
                                                nominal._pos(2));

    auto lidar_map = std::make_unique<msf::lidar::LocalMap>();
    if (!lidar_map->Init(config, config._local_map_lla_deg)) {
        std::cerr << "LiDAR 地图初始化失败" << std::endl;
        return 1;
    }
    msf::lidar::LidarVisualizer lidar_visualizer;

    // 构建事件时间线：只包含 IMU 与 LiDAR（GNSS 仅用于初始化）
    std::vector<msf::tools::Event> events;
    msf::tools::AppendEvents(events, imu_data, TAG_IMU);
    msf::tools::AppendEvents(events, lidar_data, TAG_LIDAR);
    msf::tools::SortEvents(events);

    msf::LogPreamble preamble;
    preamble.config_file = config_file;
    preamble.fusion_mode_name = "LIO";
    preamble.has_lidar = true;
    preamble.loaded_paths = reader.GetLoadedPaths();
    preamble.imu_count = reader.GetImuCount();
    preamble.gnss_pos_count = reader.GetGnssPosCount();
    preamble.gnss_vel_count = reader.GetGnssVelCount();
    preamble.heading_count = reader.GetHeadingCount();
    preamble.lidar_count = reader.GetLidarCount();
    preamble.data_start_time = reader.GetStartTime();
    preamble.data_end_time = reader.GetEndTime();
    preamble.timeline_total = events.size();
    for (const auto& e : events) {
        if (e.tag == TAG_IMU) {
            ++preamble.timeline_imu;
        } else if (e.tag == TAG_LIDAR) {
            ++preamble.timeline_lidar;
        }
    }
    preamble.ekf_nominal = ekf.State()._nominal;
    preamble.ekf_eth = ekf.State()._eth;
    preamble.Pk = ekf.State()._error._Pk;
    preamble.Qt = ekf.State()._error._Qt;
    preamble.layout = ekf.State()._layout;
    result_writer.WriteLogPreamble(preamble, config);

    const double init_time = ekf.State()._nominal._t_cur;
    const size_t start_idx = msf::tools::FirstEventAfter(events, init_time);

    double unfused = 0.0;
    const size_t total_events = events.size() - start_idx;
    size_t output_count = 0;

    for (size_t ev_idx = start_idx; ev_idx < events.size(); ++ev_idx) {
        if (max_events > 0 && ev_idx - start_idx >= max_events) {
            break;
        }
        const msf::tools::Event& event = events[ev_idx];
        if (msf::tools::DurationExceeded(event.timestamp, init_time, max_duration)) {
            std::cout << "\n达到 max-duration，停止处理" << std::endl;
            break;
        }
        double advanced_dt = 0.0;
        if (!msf::tools::AdvanceNominal(ekf, propagator, imu_data, event.timestamp, &advanced_dt)) {
            continue;
        }
        unfused += advanced_dt;

        const size_t done_events = ev_idx - start_idx + 1;
        std::cout << '\r' << "[event " << done_events << "/" << total_events << "] "
                  << "event_ts=" << std::fixed << std::setprecision(4) << event.timestamp
                  << std::flush;
        msf::EkfState& s = ekf.State();

        bool updated = false;
        switch (event.tag) {
            case TAG_LIDAR: {
                msf::LidarScanInfo scan = lidar_data[event.index];
                msf::MeasLogEvent rec;
                rec.seq = done_events;
                rec.type = "LIDAR";
                rec.event_ts = event.timestamp;
                rec.sensor_ts = scan._timestamp;
                rec.map_initialized = lidar_map->IsBuilt();

                msf::lidar::PointCloudXYZI::Ptr raw_cloud(new msf::lidar::PointCloudXYZI());
                if (!msf::lidar::LoadLidarPcd(scan._path, scan._timestamp, raw_cloud, &scan._end_time)) {
                    rec.reject_reason = "pcd_load_failed";
                    result_writer.WriteMeasEvent(rec);
                    break;
                }

                // 参考实现用 ins_processor 副本做帧内前推；这里也必须复制 InsPropagator，
                // 避免去畸变推进污染主时间线使用的 IMU 内部状态。
                msf::InsPropagator deskew_propagator = propagator;
                msf::lidar::ScanDeskewer deskewer(config, deskew_propagator);
                msf::lidar::DeskewDiag deskew_diag;
                msf::lidar::PointCloudXYZI::Ptr undistorted = config._lidar._enable_undistort
                        ? deskewer.DeskewScan(raw_cloud, imu_data, scan, s._nominal, &deskew_diag)
                        : raw_cloud;
                if (!config._lidar._enable_undistort) {
                    deskewer.BuildPoseChain(s._nominal, imu_data, scan);
                }
                FillDeskewLog(rec, deskewer, config._lidar._enable_undistort ? &deskew_diag : nullptr);
                std::cerr << "[LiDAR deskew] scan=" << scan._timestamp
                          << " points=" << (undistorted ? undistorted->size() : 0)
                          << " max_comp_m=" << deskew_diag._max_compensation_m
                          << " mean_comp_m=" << deskew_diag._mean_compensation_m
                          << " max_rot_rad=" << deskew_diag._max_rot_diff_rad
                          << std::endl;
                const auto filtered = msf::lidar::FilterRawScan(undistorted, config);
                const auto cloud_imu = msf::lidar::TransformLidarCloudToImu(filtered, config);
                if (!cloud_imu || cloud_imu->empty()) {
                    rec.reject_reason = "pcd_empty";
                    result_writer.WriteMeasEvent(rec);
                    break;
                }

                const Eigen::Vector3d lidar_pos_enu = msf::lidar::PositionEnu(s._nominal, config) + s._nominal._Cnb * config._calibration._T_lb_m;
                lidar_map->TrimFov(lidar_pos_enu);

                if (!lidar_map->IsBuilt()) {
                    const auto cloud_world = msf::lidar::TransformCloudImuToWorld(cloud_imu, s._nominal, config);
                    lidar_map->Build(cloud_world);  // 首帧只建图
                    std::cerr << "[Map] build points=" << lidar_map->ValidSize() << std::endl;
                    rec.quality_passed = true;
                    rec.reject_reason = "initial_map_only";
                    if (viz_mode > 0) {
                        lidar_visualizer.ShowScan(raw_cloud, undistorted, cloud_imu, s._nominal,
                                                  config, viz_mode);
                    }
                    result_writer.WriteMeasEvent(rec);
                    break;
                }

                std::cerr << std::fixed << std::setprecision(9)
                          << "[LiDAR pre] t=" << s._nominal._t_cur
                          << " lat=" << s._nominal._pos(0) * _R2D
                          << " lon=" << s._nominal._pos(1) * _R2D
                          << " h=" << s._nominal._pos(2)
                          << " vn=" << s._nominal._vn.transpose()
                          << " att=" << s._nominal._att.transpose()
                          << std::endl;

                const int map_before = lidar_map->ValidSize();
                msf::lidar::LidarPlaneFactor factor(*lidar_map, cloud_imu, scan, config);
                rec.quality_passed = true;
                const msf::NominalState before = s._nominal;
                updated = ekf.Update(factor, config._lidar._max_iterations + 1, config._lidar._iekf_convergence_threshold);
                rec.fused = updated;
                rec.pre = msf::EvaluateResidual(factor.LastMeasBlock(), s._error._Pk);
                msf::FillLidarResidualSummary(rec, rec.pre.res, rec.pre.norm_res);
                FillLidarFeature(rec, factor.LastFeatureDiag());
                FillIekfLog(rec, ekf.LastIekfDiag());
                if (updated) {
                    msf::FillStateDelta(rec, before, s._nominal, s._eth);
                }
                if (!updated) {
                    rec.reject_reason = "lidar_update_rejected";
                }
                std::cerr << "[Map] before=" << map_before
                          << " after=" << lidar_map->ValidSize()
                          << " inserted=" << (lidar_map->ValidSize() - map_before)
                          << std::endl;

                std::cerr << std::fixed << std::setprecision(9)
                          << "[LiDAR post] t=" << s._nominal._t_cur
                          << " lat=" << s._nominal._pos(0) * _R2D
                          << " lon=" << s._nominal._pos(1) * _R2D
                          << " h=" << s._nominal._pos(2)
                          << " vn=" << s._nominal._vn.transpose()
                          << " att=" << s._nominal._att.transpose()
                          << " Pstd=" << s._error._Pk.diagonal().array().sqrt().transpose()
                          << std::endl;
                const auto& dbg_block = factor.LastMeasBlock();
                double res_sum = 0.0;
                double res_max = 0.0;
                const int sample_count = std::min<int>(3, static_cast<int>(dbg_block._z.size()));
                for (int ri = 0; ri < sample_count; ++ri) {
                    const double a = std::fabs(dbg_block._z(ri));
                    res_sum += a;
                    res_max = std::max(res_max, a);
                }
                const double res_mean = sample_count > 0 ? res_sum / sample_count : 0.0;
                std::cerr << "[LiDAR factor] scan=" << scan._timestamp
                          << " filtered=" << factor.LastFeatureDiag().filtered_points
                          << " candidate=" << factor.LastFeatureDiag().candidate_features
                          << " effective=" << factor.LastFeatureDiag().effective_features
                          << " updated=" << updated
                          << " iter=" << factor.BuildCount()
                          << " res_mean=" << res_mean
                          << " res_max=" << res_max
                          << std::endl;

                msf::lidar::MaybeExportWorldPcd(scan, undistorted, s._nominal, config);
                if (viz_mode > 0) {
                    lidar_visualizer.ShowScan(raw_cloud, undistorted, cloud_imu, s._nominal, config,
                                              viz_mode);
                }
                result_writer.WriteMeasEvent(rec);
                break;
            }
            case TAG_IMU: {
                int state_code = 2;
                if (unfused > config._quality_control._max_no_valid_measure_sec) {
                    state_code = 0;
                } else if (unfused > 1.0) {
                    state_code = 1;
                }
                result_writer.WriteGloc(imu_data[event.index], s._nominal, state_code, config);
                result_writer.WriteImuEvent(done_events, event.timestamp, imu_data[event.index],
                                            state_code, unfused, s._nominal, s._eth, s._error._Pk,
                                            s._layout);
                ++output_count;
                break;
            }
            default:
                break;
        }

        if (updated) {
            unfused = 0.0;
        }
    }

    std::cout << std::endl;
    result_writer.Close();
    std::cout << "处理完成: " << output_count << " 帧输出到 "
              << result_writer.GlocPath() << std::endl;
    return 0;
}
