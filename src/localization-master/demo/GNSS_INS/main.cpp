// GNSS/INS 组合导航 demo：复现 POST_MSF post_main 的 GNSS/INS 主流程。
// 时间推进、量测更新、输出格式均与 POST_MSF 对齐，用于与 ref 输出逐列对比。
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "config/global_config.h"
#include "data_reader.h"
#include "result_writer.h"
#include "yaml_reader.h"
#include "ekf.h"
#include "gnss_initializer.h"
#include "gnss_pos_factor.h"
#include "gnss_quality.h"
#include "gnss_vel_factor.h"
#include "heading_factor.h"
#include "ins_propagator.h"
#include "msf_algorithms/zupt.h"
#include "math/constants.h"
#include "nominal_advance.h"
#include "process_limit.h"
#include "timeline.h"

namespace {

enum Tag : int { TAG_IMU = 0, TAG_GNSS_POS = 1, TAG_GNSS_VEL = 2, TAG_HEADING = 3 };

msf::ResidualDiag ResidualOf(msf::MeasFactor& factor, msf::EkfState& s) {
    return msf::EvaluateFactorResidual(factor, s._nominal, s._eth, s._layout, s._error._Pk);
}

bool ParseArgs(int argc, char* argv[], std::string& config_file, std::string& output_dir,
               double& max_duration, bool& max_duration_cli) {
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg.rfind("--config=", 0) == 0) {
            config_file = arg.substr(9);
        } else if (arg.rfind("--output=", 0) == 0) {
            output_dir = arg.substr(9);
        } else if (arg.rfind("--max-duration=", 0) == 0) {
            max_duration = std::stod(arg.substr(15));
            max_duration_cli = true;
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
        std::cerr << "用法: gnss_ins --config=<yaml> [--output=<dir>] [--max-duration=SEC]"
                  << std::endl;
        return false;
    }
    return true;
}

}  // namespace

// 输入参数：
//   --config=<yaml>              配置文件路径（必填；也可用位置参数传同一路径）
//   --output=<dir>               覆盖结果输出目录（可选）
//   --max-duration=SEC           自初始化时刻起最多处理 SEC 秒；未给时用 yaml
//                                process_option.max_process_duration_sec（<=0 不限制）
int main(int argc, char* argv[]) {
    std::string config_file;
    std::string output_dir;
    double max_duration = 0.0;
    bool max_duration_cli = false;
    if (!ParseArgs(argc, argv, config_file, output_dir, max_duration, max_duration_cli)) {
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
    msf::ResultWriter result_writer;
    if (!result_writer.OpenResultFiles(config)) {
        return 1;
    }

    msf::DataReader reader;
    if (!reader.LoadData(config, msf::LoadOptions{true, false})) {
        std::cerr << "数据加载失败" << std::endl;
        return 1;
    }
    const auto& imu_data = reader.GetImuData();
    const auto& gnsspos_data = reader.GetGnssPosData();
    const auto& gnssvel_data = reader.GetGnssVelData();
    const auto& heading_data = reader.GetHeadingData();
    if (imu_data.empty() || gnsspos_data.empty() || gnssvel_data.empty() ||
        heading_data.empty()) {
        std::cerr << "IMU/GNSS/Heading 数据为空" << std::endl;
        return 1;
    }

    // 初始化
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
    config._local_map_lla_deg = Eigen::Vector3d(nominal._pos(0) * msf::constants::_R2D,
                                                nominal._pos(1) * msf::constants::_R2D,
                                                nominal._pos(2));

    // 构建事件时间线
    std::vector<msf::tools::Event> events;
    msf::tools::AppendEvents(events, imu_data, TAG_IMU);
    msf::tools::AppendEvents(events, gnsspos_data, TAG_GNSS_POS);
    msf::tools::AppendEvents(events, gnssvel_data, TAG_GNSS_VEL);
    msf::tools::AppendEvents(events, heading_data, TAG_HEADING);
    msf::tools::SortEvents(events);

    msf::LogPreamble preamble;
    preamble.config_file = config_file;
    preamble.fusion_mode_name = "GNSS/INS";
    preamble.loaded_paths = reader.GetLoadedPaths();
    preamble.imu_count = reader.GetImuCount();
    preamble.gnss_pos_count = reader.GetGnssPosCount();
    preamble.gnss_vel_count = reader.GetGnssVelCount();
    preamble.heading_count = reader.GetHeadingCount();
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
                rec.trk_deg = vel._trk_gnd * msf::constants::_R2D;
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
