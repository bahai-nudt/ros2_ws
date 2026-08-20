// GNSS/LIO/INS 组合导航 demo：GNSS 锁位置，LiDAR 只在副本上做点面 Gauss-Newton，
// 得到 C_meas / p_meas / R_pose 后再以 pose_meas 更新主滤波（默认只改正姿态）。
// 对应 POST_MSF fusion_mode=3 且 pose_meas_enable=true。点面 IEKF 见 GNSS_LIDAR_INS。
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "config/global_config.h"
#include "data_reader.h"
#include "ekf.h"
#include "gnss_initializer.h"
#include "gnss_pos_factor.h"
#include "gnss_quality.h"
#include "gnss_vel_factor.h"
#include "heading_factor.h"
#include "ins_propagator.h"
#include "lidar_pose_factor.h"
#include "lidar_pose_meas.h"
#include "lidar_visualizer.h"
#include "local_map.h"
#include "math/constants.h"
#include "msf_algorithms/zupt.h"
#include "pcd_preprocess.h"
#include "result_writer.h"
#include "scan_deskewer.h"
#include "nominal_advance.h"
#include "process_limit.h"
#include "timeline.h"
#include "yaml_reader.h"

namespace {

enum Tag : int {
    TAG_IMU = 0,
    TAG_GNSS_POS = 1,
    TAG_GNSS_VEL = 2,
    TAG_HEADING = 3,
    TAG_LIDAR = 4
};

using msf::constants::_R2D;

msf::ResidualDiag ResidualOf(msf::MeasFactor& factor, msf::EkfState& s) {
    return msf::EvaluateFactorResidual(factor, s._nominal, s._eth, s._layout, s._error._Pk);
}

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

void FillIekfLog(msf::MeasLogEvent& rec, const msf::lidar::LidarPoseEstimate& estimate) {
    rec.has_iekf = true;
    rec.iekf_total_iter = estimate._iterations;
    rec.iekf_converge_iter = estimate._converge_iter;
    rec.iekf_final_converged = estimate._final_converged;
    rec.iekf_max_dx_last = estimate._max_dx_last;
}

bool ParseArgs(int argc, char* argv[], std::string& config_file, std::string& output_dir,
               size_t& max_events, double& max_duration, bool& max_duration_cli,
               bool& visualize) {
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
        std::cerr << "用法: gnss_lio_ins --config=<yaml> [--output=<dir>] "
                  << "[--max-events=N] [--max-duration=SEC] [--visualize]" << std::endl;
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
    std::cout << "[LiDAR] pose_meas attitude_only="
              << (config._lidar._pose_meas_attitude_only ? "true" : "false")
              << "  可视化: " << msf::VisualizationModeNameZh(viz_mode) << std::endl;

    msf::ResultWriter result_writer;
    if (!result_writer.OpenResultFiles(config)) {
        return 1;
    }

    msf::DataReader reader;
    if (!reader.LoadData(config, msf::LoadOptions{true, true})) {
        std::cerr << "数据加载失败" << std::endl;
        return 1;
    }

    const auto& imu_data = reader.GetImuData();
    const auto& gnsspos_data = reader.GetGnssPosData();
    const auto& gnssvel_data = reader.GetGnssVelData();
    const auto& heading_data = reader.GetHeadingData();
    const auto& lidar_data = reader.GetLidarData();
    if (imu_data.empty() || gnsspos_data.empty() || gnssvel_data.empty() ||
        heading_data.empty()) {
        std::cerr << "IMU/GNSS/Heading 数据为空" << std::endl;
        return 1;
    }
    if (lidar_data.empty()) {
        std::cerr << "LiDAR 数据为空" << std::endl;
        return 1;
    }

    msf::NominalState nominal;
    msf::earth eth;
    if (!msf::gnss::InitializeNominal(imu_data, gnsspos_data, gnssvel_data, heading_data,
                                      config._calibration._T_gb_m,
                                      config._calibration._imu_calib,
                                      nominal, eth)) {
        return 1;
    }

    msf::Ekf ekf(config._process_option._state_dim);
    ekf.SetInitialState(nominal, eth);
    ekf.InitNoiseFromConfig(config);
    msf::InsPropagator propagator;

    config._local_map_lla_deg = Eigen::Vector3d(nominal._pos(0) * _R2D,
                                                nominal._pos(1) * _R2D,
                                                nominal._pos(2));
    auto lidar_map = std::make_unique<msf::lidar::LocalMap>();
    if (!lidar_map->Init(config, config._local_map_lla_deg)) {
        std::cerr << "LiDAR 地图初始化失败" << std::endl;
        return 1;
    }
    msf::lidar::LidarVisualizer lidar_visualizer;

    std::vector<msf::tools::Event> events;
    msf::tools::AppendEvents(events, imu_data, TAG_IMU);
    msf::tools::AppendEvents(events, gnsspos_data, TAG_GNSS_POS);
    msf::tools::AppendEvents(events, gnssvel_data, TAG_GNSS_VEL);
    msf::tools::AppendEvents(events, heading_data, TAG_HEADING);
    msf::tools::AppendEvents(events, lidar_data, TAG_LIDAR);
    msf::tools::SortEvents(events);

    msf::LogPreamble preamble;
    preamble.config_file = config_file;
    preamble.fusion_mode_name = "GNSS/INS/LiDAR";
    preamble.pose_meas_enable = true;
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
        } else if (e.tag == TAG_GNSS_POS) {
            ++preamble.timeline_gnss_pos;
        } else if (e.tag == TAG_GNSS_VEL) {
            ++preamble.timeline_gnss_vel;
        } else if (e.tag == TAG_HEADING) {
            ++preamble.timeline_heading;
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

    msf::algorithms::ZuptDetector zupt(config._quality_control._zupt);
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
            case TAG_GNSS_POS: {
                const msf::GnssPos& pos = gnsspos_data[event.index];
                msf::gnss::GnssPosFactor factor(pos, config);
                msf::MeasLogEvent rec;
                rec.seq = done_events;
                rec.type = "GNSS_POS";
                rec.event_ts = event.timestamp;
                rec.sensor_ts = pos._timestamp;
                rec.quality_passed = msf::gnss::AllowedGnssPos(pos);
                if (rec.quality_passed) {
                    rec.pre = ResidualOf(factor, s);
                    msf::ScaleGeodeticResidualToMeters(rec.pre, s._eth);
                    const msf::NominalState before = s._nominal;
                    updated = ekf.Update(
                        factor, 1, msf::constants::_iekf_convergence_threshold,
                        msf::constants::_gnss_norm_res_gate);
                    rec.fused = updated;
                    if (updated) {
                        rec.post = ResidualOf(factor, s);
                        msf::ScaleGeodeticResidualToMeters(rec.post, s._eth);
                        msf::FillStateDelta(rec, before, s._nominal, s._eth);
                    }
                }
                result_writer.WriteMeasEvent(rec);
                break;
            }
            case TAG_GNSS_VEL: {
                const msf::GnssVel& vel = gnssvel_data[event.index];
                msf::MeasLogEvent rec;
                rec.seq = done_events;
                rec.type = "GNSS_VEL";
                rec.event_ts = event.timestamp;
                rec.sensor_ts = vel._timestamp;
                rec.has_vel_raw = true;
                rec.hor_speed_mps = vel._hor_speed;
                rec.trk_deg = vel._trk_gnd * _R2D;
                rec.ver_speed_mps = vel._ver_speed;
                if (!config._process_option.VelocityFusionEnabled()) {
                    rec.reject_reason = "velocity_fusion_disabled";
                    result_writer.WriteMeasEvent(rec);
                    break;
                }
                rec.reject_reason =
                    msf::gnss::GnssVelRejectReason(vel, s._nominal._t_cur - vel._timestamp);
                rec.quality_passed = (rec.reject_reason == "ok");
                if (rec.quality_passed) {
                    if (config._quality_control._zupt._enable &&
                        zupt.Update(vel._timestamp, vel._hor_speed, vel._ver_speed, s._nominal)) {
                        msf::algorithms::ZeroVelocityFactor factor(config._quality_control._zupt);
                        rec.pre = ResidualOf(factor, s);
                        const msf::NominalState before = s._nominal;
                        updated = ekf.Update(
                            factor, 1, msf::constants::_iekf_convergence_threshold,
                            msf::constants::_gnss_norm_res_gate);
                        rec.fused = updated;
                        if (updated) {
                            rec.post = ResidualOf(factor, s);
                            msf::FillStateDelta(rec, before, s._nominal, s._eth);
                        }
                    } else {
                        msf::gnss::GnssVelFactor factor(vel, config);
                        rec.pre = ResidualOf(factor, s);
                        const msf::NominalState before = s._nominal;
                        updated = ekf.Update(
                            factor, 1, msf::constants::_iekf_convergence_threshold,
                            msf::constants::_gnss_norm_res_gate);
                        rec.fused = updated;
                        if (updated) {
                            rec.post = ResidualOf(factor, s);
                            msf::FillStateDelta(rec, before, s._nominal, s._eth);
                        }
                    }
                }
                result_writer.WriteMeasEvent(rec);
                break;
            }
            case TAG_HEADING: {
                const msf::HeadingData& h = heading_data[event.index];
                msf::MeasLogEvent rec;
                rec.seq = done_events;
                rec.type = "HEADING";
                rec.event_ts = event.timestamp;
                rec.sensor_ts = h._timestamp;
                if (!config._process_option.HeadingFusionEnabled()) {
                    rec.reject_reason = "heading_fusion_disabled";
                    result_writer.WriteMeasEvent(rec);
                    break;
                }
                rec.quality_passed = msf::gnss::AllowedHeading(h);
                if (rec.quality_passed) {
                    msf::gnss::HeadingFactor factor(h, config);
                    rec.pre = ResidualOf(factor, s);
                    const msf::NominalState before = s._nominal;
                    updated = ekf.Update(factor, 1);
                    rec.fused = updated;
                    if (updated) {
                        rec.post = ResidualOf(factor, s);
                        msf::FillStateDelta(rec, before, s._nominal, s._eth);
                    }
                }
                result_writer.WriteMeasEvent(rec);
                break;
            }
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
                const auto filtered = msf::lidar::FilterRawScan(undistorted, config);
                const auto cloud_imu = msf::lidar::TransformLidarCloudToImu(filtered, config);
                if (!cloud_imu || cloud_imu->empty()) {
                    rec.reject_reason = "pcd_empty";
                    result_writer.WriteMeasEvent(rec);
                    break;
                }

                const Eigen::Vector3d lidar_pos_enu =
                    msf::lidar::PositionEnu(s._nominal, config) +
                    s._nominal._Cnb * config._calibration._T_lb_m;
                lidar_map->TrimFov(lidar_pos_enu);

                if (!lidar_map->IsBuilt()) {
                    const auto cloud_world =
                        msf::lidar::TransformCloudImuToWorld(cloud_imu, s._nominal, config);
                    lidar_map->Build(cloud_world);
                    rec.quality_passed = true;
                    rec.reject_reason = "initial_map_only";
                    if (viz_mode > 0) {
                        lidar_visualizer.ShowScan(raw_cloud, undistorted, cloud_imu, s._nominal,
                                                  config, viz_mode);
                    }
                    result_writer.WriteMeasEvent(rec);
                    break;
                }

                const auto estimate = msf::lidar::EstimateLidarPose(
                    *lidar_map, cloud_imu, scan, s._nominal, s._eth, config);
                FillLidarFeature(rec, estimate._feature_diag);
                FillIekfLog(rec, estimate);
                if (estimate._valid) {
                    msf::lidar::LidarPoseFactor factor(*lidar_map, cloud_imu, scan, config, estimate);
                    rec.quality_passed = true;
                    rec.pre = ResidualOf(factor, s);
                    msf::FillLidarResidualSummary(rec, rec.pre.res, rec.pre.norm_res);
                    const msf::NominalState before = s._nominal;
                    updated = ekf.Update(factor, 1);
                    rec.fused = updated;
                    if (updated) {
                        msf::FillStateDelta(rec, before, s._nominal, s._eth);
                    }
                    if (!updated) {
                        rec.reject_reason = "lidar_update_rejected";
                    }
                } else {
                    rec.reject_reason = "pose_estimate_invalid";
                }

                // 旁路：量测更新后的名义状态 + 去畸变雷达系点云 → 分帧世界系 PCD
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
