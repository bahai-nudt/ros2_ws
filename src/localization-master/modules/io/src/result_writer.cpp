#include "result_writer.h"

#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>

#include <Eigen/Geometry>

#include "math/constants.h"
#include "math/coordinate.h"
#include "math/low_pass_filter.h"
#include "math/pose_converter.h"

namespace msf {
namespace {

using constants::_D2R;
using constants::_PI;
using constants::_R2D;

struct GlocRecord {
    double timestamp = 0.0;
    double obs_time = 0.0;
    int state = 0;
    double position_x = 0.0, position_y = 0.0, position_z = 0.0;
    double orientation_w = 1.0, orientation_x = 0.0, orientation_y = 0.0, orientation_z = 0.0;
    double linear_velocity_x = 0.0, linear_velocity_y = 0.0, linear_velocity_z = 0.0;
    double linear_acceleration_x = 0.0, linear_acceleration_y = 0.0, linear_acceleration_z = 0.0;
    double angular_velocity_x = 0.0, angular_velocity_y = 0.0, angular_velocity_z = 0.0;
    double latitude = 0.0, longitude = 0.0, altitude = 0.0;
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    double velocity = 0.0;
};

long long UnixSecToUs(double t) {
    const long long ts_sec = static_cast<long long>(t);
    const long long nanosec = static_cast<long long>((t - static_cast<double>(ts_sec)) * 1.0e9);
    return ts_sec * 1000000LL + nanosec / 1000LL;
}

Eigen::Vector3d First3(const Eigen::VectorXd& v) {
    Eigen::Vector3d out = Eigen::Vector3d::Zero();
    const int n = std::min(3, static_cast<int>(v.size()));
    for (int i = 0; i < n; ++i) {
        out(i) = v(i);
    }
    return out;
}

std::string FormatVec3(const Eigen::Vector3d& v, int precision = 6) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision) << "[" << v(0) << ", " << v(1) << ", " << v(2)
        << "]";
    return oss.str();
}

double WrapAzimuthDeg(double yaw_rad) {
    double azimuth_deg = -yaw_rad * _R2D;
    azimuth_deg = std::fmod(azimuth_deg, 360.0);
    if (azimuth_deg < 0.0) {
        azimuth_deg += 360.0;
    }
    return azimuth_deg;
}

// 对应 POST_MSF PoseImu2Vehicle：SINS 输出转车体系输出（含速度低通滤波）
void PoseImu2Vehicle(const Eigen::Vector3d& angular_velocity, const NominalState& sins,
                     const GlobalConfig& param, Eigen::Vector3d& position_enu,
                     Eigen::Quaterniond& q_vehicle, Eigen::Vector3d& velocity_enu) {
    static LowPassFilter speed_filter_x(1.0, 100.0);
    static LowPassFilter speed_filter_y(1.0, 100.0);
    static LowPassFilter speed_filter_z(1.0, 100.0);

    const Eigen::Vector3d& origin_lla_deg = param._calibration._gloc_origin_lla_deg;
    Eigen::Vector3d origin_rad;
    origin_rad << origin_lla_deg(0) * _D2R, origin_lla_deg(1) * _D2R, origin_lla_deg(2);

    const Eigen::Vector3d imu_pos_enu = Coordinate::lla2enu(origin_rad, sins._pos);
    const Eigen::Vector3d imu_vel_enu(sins._vn(0), sins._vn(1), sins._vn(2));

    const double pitch_rad = sins._att(0);
    const double roll_rad = sins._att(1);
    double yaw_rad = _PI / 2.0 + sins._att(2);
    while (yaw_rad > _PI) {
        yaw_rad -= 2.0 * _PI;
    }
    while (yaw_rad <= -_PI) {
        yaw_rad += 2.0 * _PI;
    }

    q_vehicle = Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(pitch_rad, Eigen::Vector3d::UnitX()) *
                Eigen::AngleAxisd(roll_rad, Eigen::Vector3d::UnitY());
    q_vehicle.normalize();
    const Eigen::Matrix3d r_vehicle_to_enu = q_vehicle.toRotationMatrix();

    const Eigen::Vector3d T_bv = param._calibration._T_bv_m;
    position_enu = imu_pos_enu - r_vehicle_to_enu * T_bv;

    const Eigen::Matrix3d R_vb = param._calibration._R_bv.transpose();
    const Eigen::Vector3d angular_vel_v = R_vb * angular_velocity;
    const Eigen::Vector3d lever_v = -T_bv;
    const Eigen::Vector3d vel_correction_v = angular_vel_v.cross(lever_v);
    const Eigen::Vector3d vehicle_vel_enu = imu_vel_enu + r_vehicle_to_enu * vel_correction_v;

    velocity_enu = Eigen::Vector3d(speed_filter_x.update(vehicle_vel_enu.x()),
                                   speed_filter_y.update(vehicle_vel_enu.y()),
                                   speed_filter_z.update(vehicle_vel_enu.z()));
}

GlocRecord BuildGloc(const ImuData& imu_data, const NominalState& publish_sins, int state_code,
                     const GlobalConfig& param) {
    GlocRecord gloc{};
    const bool state_error = (state_code == 0);
    const Eigen::Vector3d& origin_lla_deg = param._calibration._gloc_origin_lla_deg;

    const Eigen::Vector3d angular_velocity = state_error ? Eigen::Vector3d::Zero() : imu_data._gyro;
    const Eigen::Vector3d linear_acceleration =
        state_error ? Eigen::Vector3d::Zero() : imu_data._accel;

    Eigen::Vector3d vehicle_pos_enu = Eigen::Vector3d::Zero();
    Eigen::Vector3d vehicle_vel_enu = Eigen::Vector3d::Zero();
    Eigen::Quaterniond q_vehicle = Eigen::Quaterniond::Identity();
    if (!state_error) {
        PoseImu2Vehicle(angular_velocity, publish_sins, param, vehicle_pos_enu, q_vehicle,
                        vehicle_vel_enu);
    }

    gloc.timestamp = 0.0;
    gloc.obs_time = publish_sins._t_cur;
    gloc.state = state_code;
    gloc.position_x = vehicle_pos_enu.x();
    gloc.position_y = vehicle_pos_enu.y();
    gloc.position_z = vehicle_pos_enu.z();
    gloc.orientation_w = q_vehicle.w();
    gloc.orientation_x = q_vehicle.x();
    gloc.orientation_y = q_vehicle.y();
    gloc.orientation_z = q_vehicle.z();
    gloc.linear_velocity_x = vehicle_vel_enu.x();
    gloc.linear_velocity_y = vehicle_vel_enu.y();
    gloc.linear_velocity_z = vehicle_vel_enu.z();
    gloc.linear_acceleration_x = linear_acceleration.x();
    gloc.linear_acceleration_y = linear_acceleration.y();
    gloc.linear_acceleration_z = linear_acceleration.z();
    gloc.angular_velocity_x = angular_velocity.x();
    gloc.angular_velocity_y = angular_velocity.y();
    gloc.angular_velocity_z = angular_velocity.z();

    Eigen::Vector3d origin_rad;
    origin_rad << origin_lla_deg(0) * _D2R, origin_lla_deg(1) * _D2R, origin_lla_deg(2);
    const Eigen::Vector3d lla_rad = Coordinate::enu2lla(origin_rad, vehicle_pos_enu);
    gloc.latitude = lla_rad(0) * _R2D;
    gloc.longitude = lla_rad(1) * _R2D;
    gloc.altitude = lla_rad(2);

    const double pitch_rad = state_error ? 0.0 : publish_sins._att(0);
    const double roll_rad = state_error ? 0.0 : publish_sins._att(1);
    double yaw_rad = state_error ? 0.0 : _PI / 2.0 + publish_sins._att(2);
    while (!state_error && yaw_rad > _PI) {
        yaw_rad -= 2.0 * _PI;
    }
    while (!state_error && yaw_rad <= -_PI) {
        yaw_rad += 2.0 * _PI;
    }
    gloc.roll = roll_rad;
    gloc.pitch = pitch_rad;
    gloc.yaw = yaw_rad;

    const double speed_en = std::sqrt(gloc.linear_velocity_x * gloc.linear_velocity_x +
                                      gloc.linear_velocity_y * gloc.linear_velocity_y);
    const double proj = gloc.linear_velocity_x * std::cos(yaw_rad) +
                        gloc.linear_velocity_y * std::sin(yaw_rad);
    if (state_error) {
        gloc.velocity = 0.0;
    } else if (proj > 0.0) {
        gloc.velocity = +speed_en;
    } else if (proj < 0.0) {
        gloc.velocity = -speed_en;
    } else {
        gloc.velocity = 0.0;
    }
    return gloc;
}

void WriteGlocRecord(std::ofstream& file, const GlocRecord& gloc) {
    file << std::fixed << std::setprecision(6) << gloc.timestamp << "," << gloc.obs_time << ","
         << gloc.state << "," << gloc.position_x << "," << gloc.position_y << ","
         << gloc.position_z << "," << gloc.orientation_w << "," << gloc.orientation_x << ","
         << gloc.orientation_y << "," << gloc.orientation_z << "," << gloc.linear_velocity_x << ","
         << gloc.linear_velocity_y << "," << gloc.linear_velocity_z << ","
         << gloc.linear_acceleration_x << "," << gloc.linear_acceleration_y << ","
         << gloc.linear_acceleration_z << "," << gloc.angular_velocity_x << ","
         << gloc.angular_velocity_y << "," << gloc.angular_velocity_z << ","
         << std::setprecision(12) << gloc.latitude << "," << gloc.longitude << ","
         << std::setprecision(6) << gloc.altitude << "," << gloc.roll << "," << gloc.pitch << ","
         << gloc.yaw << "," << gloc.velocity << std::endl;
}

void WriteSinsRecord(std::ofstream& file, const NominalState& sins) {
    const long long ts_us = UnixSecToUs(sins._t_cur);
    file << std::setfill('0') << std::setw(16) << ts_us << ' ' << std::setfill(' ') << std::fixed
         << std::setprecision(9) << sins._pos(0) * _R2D << ' ' << sins._pos(1) * _R2D << ' '
         << std::setprecision(4) << sins._pos(2) << ' ' << std::setprecision(6)
         << sins._att(1) * _R2D << ' ' << sins._att(0) * _R2D << ' ' << WrapAzimuthDeg(sins._att(2))
         << ' ' << std::setprecision(4) << sins._vn(1) << ' ' << sins._vn(0) << ' ' << sins._vn(2)
         << ' ' << std::setfill('0') << std::setw(16) << ts_us << '\n';
    file << std::setfill(' ');
}

void WritePreRecord(std::ofstream& file, const NominalState& sins, const GlobalConfig& config) {
    const Eigen::Vector3d origin_rad(config._local_map_lla_deg(0) * _D2R,
                                     config._local_map_lla_deg(1) * _D2R,
                                     config._local_map_lla_deg(2));
    const Eigen::Vector3d position_imu_enu = Coordinate::lla2enu(origin_rad, sins._pos);
    const Eigen::Matrix3d R_lidar_map = sins._Cnb * config._calibration._R_bl;
    const Eigen::Vector3d position_lidar_map =
        position_imu_enu + sins._Cnb * config._calibration._T_lb_m;

    file << std::fixed << std::setprecision(6) << sins._t_cur;
    file << std::setw(20) << R_lidar_map(0, 0) << std::setw(20) << R_lidar_map(0, 1)
         << std::setw(20) << R_lidar_map(0, 2) << std::setw(20) << R_lidar_map(1, 0)
         << std::setw(20) << R_lidar_map(1, 1) << std::setw(20) << R_lidar_map(1, 2)
         << std::setw(20) << R_lidar_map(2, 0) << std::setw(20) << R_lidar_map(2, 1)
         << std::setw(20) << R_lidar_map(2, 2) << std::setw(20) << position_lidar_map(0)
         << std::setw(20) << position_lidar_map(1) << std::setw(20) << position_lidar_map(2)
         << std::endl;
}

void WriteEventBlockHeader(std::ofstream& file, const MeasLogEvent& rec) {
    file << "--------------------------------------------------------------------------------\n";
    file << "[EVENT " << std::setw(6) << std::setfill('0') << rec.seq << std::setfill(' ')
         << "] type=" << rec.type << " event_ts=" << std::fixed << std::setprecision(6)
         << rec.event_ts << " sensor_ts=" << rec.sensor_ts
         << " age_s=" << (rec.event_ts - rec.sensor_ts) << "\n";
}

void WriteResidualLine(std::ofstream& file, const char* tag, const ResidualDiag& diag,
                       const std::string& type) {
    file << "  " << tag << ": ";
    if (!diag.valid) {
        file << "invalid\n";
        return;
    }
    if (type == "GNSS_POS") {
        file << "res_m=" << FormatVec3(First3(diag.res), 6)
             << " norm_res=" << FormatVec3(First3(diag.norm_res), 6)
             << " sqrtS=" << FormatVec3(First3(diag.sqrt_S), 6) << "\n";
    } else if (type == "GNSS_VEL") {
        file << "res_mps=" << FormatVec3(First3(diag.res), 6)
             << " norm_res=" << FormatVec3(First3(diag.norm_res), 6)
             << " sqrtS=" << FormatVec3(First3(diag.sqrt_S), 6) << "\n";
    } else if (type == "HEADING") {
        const double res_rad = diag.res.size() > 0 ? diag.res(0) : 0.0;
        const double norm = diag.norm_res.size() > 0 ? diag.norm_res(0) : 0.0;
        const double sqrt_s = diag.sqrt_S.size() > 0 ? diag.sqrt_S(0) : 0.0;
        file << "res_deg=" << std::fixed << std::setprecision(6) << res_rad * _R2D
             << " res_rad=" << res_rad << " norm_res=" << norm << " sqrtS_deg=" << sqrt_s * _R2D
             << "\n";
    } else {
        file << "res=" << FormatVec3(First3(diag.res), 6)
             << " norm_res=" << FormatVec3(First3(diag.norm_res), 6) << "\n";
    }
}

std::string CurrentLocalTimestamp() {
    const std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::ostringstream oss;
    oss << std::put_time(std::localtime(&now), "%Y-%m-%d %H:%M:%S");
    return oss.str();
}

Eigen::Vector3d PositionDeltaMeters(const NominalState& before, const NominalState& after,
                                    const earth& eth) {
    Eigen::Vector3d dpos_m = Eigen::Vector3d::Zero();
    if (std::abs(eth._f_RMh) > 1e-12) {
        dpos_m(0) = (after._pos(0) - before._pos(0)) / eth._f_RMh;
    }
    if (std::abs(eth._f_cbRNh) > 1e-12) {
        dpos_m(1) = (after._pos(1) - before._pos(1)) / eth._f_cbRNh;
    }
    dpos_m(2) = after._pos(2) - before._pos(2);
    return dpos_m;
}

void WriteStateDelta(std::ofstream& file, const NominalState& before, const NominalState& after,
                     const earth& eth) {
    Eigen::Vector3d d_att = after._att - before._att;
    d_att(0) = pose_converter::WrapAngle(d_att(0));
    d_att(1) = pose_converter::WrapAngle(d_att(1));
    d_att(2) = pose_converter::WrapAngle(d_att(2));
    file << "  state_delta:\n";
    file << "    d_att_deg=" << FormatVec3(d_att * _R2D, 6) << "\n";
    file << "    d_vel_mps=" << FormatVec3(after._vn - before._vn, 6) << "\n";
    file << "    d_pos_m=" << FormatVec3(PositionDeltaMeters(before, after, eth), 6) << "\n";
    file << "    d_bg_dps=" << FormatVec3((after._eb - before._eb) / constants::_dps, 9) << "\n";
    file << "    d_ba_mps2=" << FormatVec3(after._db - before._db, 9) << "\n";
}

Eigen::Vector3d LayoutStd3(const Eigen::VectorXd& pk_std, const StateLayout& layout, BlockId id) {
    if (!layout.Has(id)) {
        return Eigen::Vector3d::Zero();
    }
    return pk_std.segment<3>(layout.Offset(id));
}

}  // namespace

ResidualDiag EvaluateResidual(const MeasFactor::MeasBlock& block, const Eigen::MatrixXd& Pk) {
    ResidualDiag diag;
    if (!block._valid || block._H.rows() == 0 || block._z.size() == 0) {
        return diag;
    }
    const int nr = static_cast<int>(block._z.size());
    const int nx = static_cast<int>(block._H.cols());
    if (block._H.rows() != nr || Pk.rows() != nx || Pk.cols() != nx) {
        return diag;
    }
    Eigen::MatrixXd R;
    if (block.HasFullR()) {
        R = block._R;
    } else if (block._R_diag.size() == nr) {
        R = block._R_diag.asDiagonal();
    } else {
        return diag;
    }
    const Eigen::MatrixXd S = block._H * Pk * block._H.transpose() + R;
    diag.res = block._z;
    diag.norm_res = Eigen::VectorXd::Zero(nr);
    diag.sqrt_S = Eigen::VectorXd::Zero(nr);
    for (int i = 0; i < nr; ++i) {
        const double sigma = std::sqrt(std::max(S(i, i), 0.0));
        diag.sqrt_S(i) = sigma;
        if (sigma > 1.0e-12) {
            diag.norm_res(i) = diag.res(i) / sigma;
        }
    }
    diag.valid = true;
    return diag;
}

void ScaleGeodeticResidualToMeters(ResidualDiag& diag, const earth& eth) {
    if (!diag.valid || diag.res.size() < 3) {
        return;
    }
    if (std::abs(eth._f_RMh) > 1e-12) {
        diag.res(0) /= eth._f_RMh;
        if (diag.sqrt_S.size() >= 3) {
            diag.sqrt_S(0) /= eth._f_RMh;
        }
    }
    if (std::abs(eth._f_cbRNh) > 1e-12) {
        diag.res(1) /= eth._f_cbRNh;
        if (diag.sqrt_S.size() >= 3) {
            diag.sqrt_S(1) /= eth._f_cbRNh;
        }
    }
}

void FillLidarResidualSummary(MeasLogEvent& ev, const Eigen::VectorXd& res,
                              const Eigen::VectorXd& norm_res) {
    ev.lidar_sample_count = static_cast<int>(res.size());
    if (res.size() == 0) {
        return;
    }
    double abs_sum = 0.0;
    double abs_max = 0.0;
    double norm_sq = 0.0;
    for (int i = 0; i < res.size(); ++i) {
        const double a = std::fabs(res(i));
        abs_sum += a;
        abs_max = std::max(abs_max, a);
        if (i < norm_res.size()) {
            norm_sq += norm_res(i) * norm_res(i);
        }
    }
    const double n = static_cast<double>(res.size());
    ev.lidar_res_abs_mean = abs_sum / n;
    ev.lidar_res_abs_max = abs_max;
    ev.lidar_norm_res_rms = std::sqrt(norm_sq / n);
}

void FillStateDelta(MeasLogEvent& rec, const NominalState& before, const NominalState& after,
                    const earth& eth_after) {
    rec.has_state_delta = true;
    rec.nominal_before = before;
    rec.nominal_after = after;
    rec.eth_after = eth_after;
}

ResultWriter::~ResultWriter() { Close(); }

bool ResultWriter::OpenResultFiles(const GlobalConfig& config) {
    Close();

    std::filesystem::create_directories(config._file._output_data_dir);
    const std::string& dir = config._file._output_data_dir;

    gloc_path_ = dir + "/gloc_result.txt";
    sins_path_ = dir + "/sins_result.txt";
    pre_path_ = dir + "/pre.txt";
    log_path_ = dir + "/log.txt";

    gloc_file_.open(gloc_path_);
    if (!gloc_file_.is_open()) {
        std::cerr << "无法打开 gloc_result.txt" << std::endl;
        gloc_path_.clear();
        return false;
    }
    sins_file_.open(sins_path_);
    if (!sins_file_.is_open()) {
        std::cerr << "无法打开 sins_result.txt" << std::endl;
        Close();
        return false;
    }
    pre_file_.open(pre_path_);
    if (!pre_file_.is_open()) {
        std::cerr << "无法打开 pre.txt" << std::endl;
        Close();
        return false;
    }
    log_file_.open(log_path_);
    if (!log_file_.is_open()) {
        std::cerr << "无法打开 log.txt" << std::endl;
        Close();
        return false;
    }

    gloc_file_ << "timestamp,obs_time,state,"
               << "position_x,position_y,position_z,"
               << "orientation_w,orientation_x,orientation_y,orientation_z,"
               << "linear_velocity_x,linear_velocity_y,linear_velocity_z,"
               << "linear_acceleration_x,linear_acceleration_y,linear_acceleration_z,"
               << "angular_velocity_x,angular_velocity_y,angular_velocity_z,"
               << "latitude,longitude,altitude,"
               << "roll,pitch,yaw,velocity" << std::endl;

    sins_file_ << "# timestamp latitude longitude height roll pitch azimuth "
               << "north_vel east_vel up_vel obs_timestamp" << std::endl;
    return true;
}

void ResultWriter::WriteLogPreamble(const LogPreamble& preamble, const GlobalConfig& config) {
    if (!log_file_.is_open()) {
        return;
    }
    auto& log = log_file_;
    log << std::fixed;
    log << "运行时间: " << CurrentLocalTimestamp() << "\n";
    log << "================================================================================\n";
    log << "MSF 运行日志\n";
    log << "================================================================================\n\n";
    log << "--------------------------------------------------------------------------------\n";

    log << "【一、运行配置】\n";
    log << "  配置文件: " << preamble.config_file << "\n";
    log << "  输入目录: " << config._file._input_data_dir << "\n";
    log << "  输出目录: " << config._file._output_data_dir << "\n";
    log << "  融合模式: " << preamble.fusion_mode_name << "\n";
    log << "  处理时长上限 max_process_duration_sec: "
        << config._process_option._max_process_duration_sec
        << (config._process_option._max_process_duration_sec > 0.0 ? " s\n" : "（不限制）\n");
    if (preamble.has_lidar) {
        log << "  世界系 PCD 导出: " << (config._out._output_world_pcd ? "开启" : "关闭");
        if (config._out._output_world_pcd) {
            log << "（目录=" << config._file._output_data_dir << "/" << config._lidar_frame_id
                << "）";
        }
        log << "\n";
    }
    log << "  雷达可视化: " << VisualizationModeNameZh(config._out._visualization_mode);
    if (config._out._visualization_mode > 0) {
        log << "（模式=" << config._out._visualization_mode << "，帧ID=" << config._lidar_frame_id
            << "）";
    }
    log << "\n";
    log << "  雷达位姿量测: " << (preamble.pose_meas_enable ? "开启" : "关闭（点到面 IEKF）");
    if (preamble.pose_meas_enable) {
        log << (config._lidar._pose_meas_attitude_only ? " attitude_only" : " full_pose")
            << " inflate=" << config._lidar._pose_meas_inflate
            << " pos_floor_m=" << config._lidar._pose_meas_pos_std_floor_m.transpose()
            << " att_floor_deg=" << config._lidar._pose_meas_att_std_floor_deg.transpose();
    }
    log << "\n";

    log << "  状态维度 state_dim: " << config._process_option._state_dim << "\n";
    log << "  量测维度 meas_dim: " << config._process_option._meas_dim << "\n";
    log << "  姿态初始标准差 att_err_deg: " << config._statistics._init._att_err_deg.transpose()
        << "\n";
    log << "  速度初始标准差 vel_err_mps: " << config._statistics._init._vel_err_mps.transpose()
        << "\n";
    log << "  位置初始标准差 pos_err_m: " << config._statistics._init._pos_err_m.transpose() << "\n";
    log << "  陀螺零偏初始标准差 gyro_bias_err_dps: "
        << config._statistics._init._gyro_bias_err_dps.transpose() << "\n";
    log << "  加计零偏初始标准差 acc_bias_err_mg: "
        << config._statistics._init._acc_bias_err_mg.transpose() << "\n";
    log << "  杆臂初始标准差 lever_arm_err_m: "
        << config._statistics._init._lever_arm_err_m.transpose() << "\n";

    log << "  陀螺零偏初值 gyro_bias_dps: "
        << config._calibration._imu_calib._gyro_bias_dps.transpose() << "\n";
    log << "  加计零偏初值 acc_bias_mg: "
        << config._calibration._imu_calib._acc_bias_mg.transpose() << "\n";
    log << "  陀螺比例因子 gyro_scale_factor: "
        << config._calibration._imu_calib._gyro_scale_factor.transpose() << "\n";
    log << "  加计比例因子 acc_scale_factor: "
        << config._calibration._imu_calib._acc_scale_factor.transpose() << "\n";

    log << "  角随机游走 ang_random_walk: "
        << config._statistics._process._ang_random_walk.transpose() << "\n";
    log << "  速度随机游走 vel_random_walk: "
        << config._statistics._process._vel_random_walk.transpose() << "\n";
    log << "  陀螺零偏驱动噪声 gyro_bias_noise: "
        << config._statistics._process._gyro_bias_noise.transpose() << "\n";
    log << "  加计零偏驱动噪声 acc_bias_noise: "
        << config._statistics._process._acc_bias_noise.transpose() << "\n";

    log << "  GNSS 固定解位置标准差下限 gnss_fixed_pos_std_floor_m: "
        << config._statistics._meas._gnss_fixed_pos_std_floor_m.transpose() << "\n";
    log << "  GNSS 浮点解位置标准差下限 gnss_float_pos_std_floor_m: "
        << config._statistics._meas._gnss_float_pos_std_floor_m.transpose() << "\n";
    log << "  GNSS 速度观测标准差 gnss_vel_meas_std_mps: "
        << config._statistics._meas._gnss_vel_meas_std_mps.transpose() << "\n";
    log << "  GNSS 速度融合最低速度 gnss_vel_min_speed_mps: "
        << config._quality_control._gnss_vel_min_speed_mps << "\n";
    log << "  航向观测标准差下限 heading_meas_std_floor_deg: "
        << config._statistics._meas._heading_meas_std_floor_deg << "\n";
    log << "  ZUPT 开关 zupt_enable: "
        << (config._quality_control._zupt._enable ? "true" : "false") << "\n";
    log << "  ZUPT 观测标准差 zupt_meas_std_mps: "
        << config._quality_control._zupt._meas_std_mps.transpose() << "\n";
    log << "  ZUPT 静止判别 hor/ver/ins_speed (m/s): "
        << config._quality_control._zupt._static_hor_speed_mps << " / "
        << config._quality_control._zupt._static_ver_speed_mps << " / "
        << config._quality_control._zupt._static_ins_speed_mps << "\n";
    log << "  ZUPT 静止判别 ang_rate (deg/s): "
        << config._quality_control._zupt._static_ang_rate_dps << "\n";
    log << "  ZUPT 最小时长 zupt_min_duration_sec: "
        << config._quality_control._zupt._min_duration_sec << "\n";

    log << std::setprecision(6) << "  gloc ENU 原点 LLA（度，与 LIO 无关）: "
        << config._calibration._gloc_origin_lla_deg.transpose() << "\n";
    log << "  IMU 相对车体平移 T_bv（米）: " << config._calibration._T_bv_m.transpose() << "\n";
    log << "  IMU 到 GNSS 天线杆臂 T_gb（米，b 系 RFU）: "
        << config._calibration._T_gb_m.transpose() << "\n";
    log << "  GNSS 杆臂长度（米）: " << config._calibration._T_gb_m.norm() << "\n";
    log << "  LiDAR-IMU 外参来源: "
        << (config._calibration._use_lidar_imu_yaml ? "YAML 覆盖" : "calib.json 推导") << "\n";
    log << "  雷达到 IMU 旋转 R_bl（p_imu = R_bl * p_lidar + T_lb）:\n"
        << config._calibration._R_bl << "\n";
    log << "  雷达到 IMU 平移 T_lb（米，IMU 系）: " << config._calibration._T_lb_m.transpose()
        << "\n\n";

    log << "--------------------------------------------------------------------------------\n";
    log << "【二、数据加载】\n";
    for (const auto& path : preamble.loaded_paths) {
        log << "  路径: " << path << "\n";
    }
    log << "  IMU:        " << preamble.imu_count << " 条\n";
    log << "  GNSS 位置:  " << preamble.gnss_pos_count << " 条\n";
    log << "  GNSS 速度:  " << preamble.gnss_vel_count << " 条\n";
    log << "  GNSS 航向:       " << preamble.heading_count << " 条\n";
    if (preamble.lidar_count > 0) {
        log << "  雷达点云:   " << preamble.lidar_count << " 帧\n";
    } else {
        log << "  雷达点云:   未加载\n";
    }
    if (preamble.data_end_time > preamble.data_start_time) {
        log << std::setprecision(3) << "  数据时间范围: " << preamble.data_start_time << " ~ "
            << preamble.data_end_time << "（"
            << (preamble.data_end_time - preamble.data_start_time) << " 秒）\n";
    }
    log << "\n";

    log << "--------------------------------------------------------------------------------\n";
    log << "【三、时间线】\n";
    log << "  解算模式: " << preamble.fusion_mode_name << "\n";
    log << "  运行期参与传感器: " << preamble.fusion_mode_name << "\n";
    log << "  事件总数: " << preamble.timeline_total << "\n";
    log << "    IMU:       " << preamble.timeline_imu << "\n";
    log << "    GNSS 位置: " << preamble.timeline_gnss_pos << "\n";
    log << "    GNSS 速度: " << preamble.timeline_gnss_vel << "\n";
    log << "    航向:      " << preamble.timeline_heading << "\n";
    log << "    雷达:      " << preamble.timeline_lidar << "\n\n";

    const auto& sins = preamble.ekf_nominal;
    const Eigen::VectorXd pk_std =
        preamble.Pk.size() > 0
            ? preamble.Pk.diagonal().array().max(0.0).sqrt().matrix()
            : Eigen::VectorXd();
    log << std::fixed;
    log << "--------------------------------------------------------------------------------\n";
    log << "【四、EKF 初始化】\n";
    log << "  --- 名义状态 (SINS) ---\n";
    log << std::setprecision(6) << "  obs_time: " << sins._t_cur << "\n";
    log << "  imu_dt: " << sins._nts << "\n";
    log << std::setprecision(4) << "  att pitch/roll/yaw (deg): " << sins._att(0) * _R2D << " "
        << sins._att(1) * _R2D << " " << sins._att(2) * _R2D << "\n";
    log << "  vn E/N/U (m/s): " << sins._vn.transpose() << "\n";
    log << std::setprecision(6) << "  pos lat/lon/alt (deg,deg,m): " << sins._pos(0) * _R2D << " "
        << sins._pos(1) * _R2D << " " << sins._pos(2) << "\n";
    log << std::setprecision(9) << "  eb gyro bias (rad/s): " << sins._eb.transpose() << "\n";
    log << "  db acc bias (m/s^2): " << sins._db.transpose() << "\n";
    log << std::setprecision(6) << "  lever arm (m): " << sins._lever.transpose() << "\n";
    log << "  Kg gyro scale: " << sins._Kg.transpose() << "\n";
    log << "  Ka acc scale: " << sins._Ka.transpose() << "\n";

    log << "  --- 误差状态 (EKF) ---\n";
    log << "  nx: " << preamble.layout.Dim() << "\n";
    if (pk_std.size() >= 15) {
        log << "  Xk: " << Eigen::VectorXd::Zero(preamble.layout.Dim()).transpose() << "\n";
        log << std::setprecision(6) << "  Pk std att (rad): "
            << LayoutStd3(pk_std, preamble.layout, BlockId::Rotation).transpose() << "\n";
        log << "  Pk std vn (m/s): "
            << LayoutStd3(pk_std, preamble.layout, BlockId::Velocity).transpose() << "\n";
        log << "  Pk std pos lat/lon(rad)/alt(m): "
            << LayoutStd3(pk_std, preamble.layout, BlockId::Position).transpose() << "\n";
        log << std::setprecision(9) << "  Pk std eb (rad/s): "
            << LayoutStd3(pk_std, preamble.layout, BlockId::GyroBias).transpose() << "\n";
        log << "  Pk std db (m/s^2): "
            << LayoutStd3(pk_std, preamble.layout, BlockId::AccBias).transpose() << "\n";
        if (preamble.layout.Has(BlockId::LeverArm)) {
            log << std::setprecision(6) << "  Pk std lever (m): "
                << LayoutStd3(pk_std, preamble.layout, BlockId::LeverArm).transpose() << "\n";
        }
    }
    if (preamble.Qt.size() >= 15) {
        log << "  Qt gyr: " << preamble.Qt.segment<3>(preamble.layout.Offset(BlockId::Rotation)).transpose()
            << "\n";
        log << "  Qt acc: " << preamble.Qt.segment<3>(preamble.layout.Offset(BlockId::Velocity)).transpose()
            << "\n";
        log << "  Qt b_gyr: "
            << preamble.Qt.segment<3>(preamble.layout.Offset(BlockId::GyroBias)).transpose() << "\n";
        log << "  Qt b_acc: "
            << preamble.Qt.segment<3>(preamble.layout.Offset(BlockId::AccBias)).transpose() << "\n";
        if (preamble.layout.Has(BlockId::LeverArm)) {
            log << "  Qt lever: "
                << preamble.Qt.segment<3>(preamble.layout.Offset(BlockId::LeverArm)).transpose()
                << "\n";
        }
    }
    log << "\n";

    log << "--------------------------------------------------------------------------------\n";
    log << "【五、事件流记录】\n";
    log << "说明：按时间线逐条记录，格式为事件块（非 CSV）。\n";
    log << "  - IMU 事件：记录时间、状态码、未融合时长、状态标准差与姿态速度摘要。\n";
    log << "  - 量测事件：记录时间、质量门限、是否融合、验前/验后残差、状态修正量。\n";
    log << "  - LiDAR 事件：补充特征统计、去畸变统计、IEKF 收敛信息。\n\n";
}

void ResultWriter::WriteGloc(const ImuData& imu_data, const NominalState& publish_sins,
                             int state_code, const GlobalConfig& config) {
    if (gloc_file_.is_open()) {
        WriteGlocRecord(gloc_file_, BuildGloc(imu_data, publish_sins, state_code, config));
    }
    if (sins_file_.is_open()) {
        WriteSinsRecord(sins_file_, publish_sins);
    }
    if (pre_file_.is_open()) {
        WritePreRecord(pre_file_, publish_sins, config);
    }
}

void ResultWriter::WriteImuEvent(size_t seq, double event_ts, const ImuData& imu, int state_code,
                                 double unfused_sec, const NominalState& nominal, const earth& eth,
                                 const Eigen::MatrixXd& Pk, const StateLayout& layout) {
    if (!log_file_.is_open()) {
        return;
    }
    MeasLogEvent header;
    header.seq = seq;
    header.type = "IMU";
    header.event_ts = event_ts;
    header.sensor_ts = imu._timestamp;
    WriteEventBlockHeader(log_file_, header);
    log_file_ << "  predict: state_code=" << state_code
              << " unfused_sec=" << std::fixed << std::setprecision(3) << unfused_sec
              << " dt=" << std::setprecision(6) << imu._dt << "\n";

    if (Pk.rows() >= 15 && Pk.cols() >= 15 && layout.Dim() >= 15) {
        const Eigen::VectorXd pk_std = Pk.diagonal().array().max(0.0).sqrt();
        Eigen::Vector3d att_std_deg = LayoutStd3(pk_std, layout, BlockId::Rotation) / constants::_deg;
        Eigen::Vector3d vel_std = LayoutStd3(pk_std, layout, BlockId::Velocity);
        Eigen::Vector3d pos_std_m = LayoutStd3(pk_std, layout, BlockId::Position);
        if (std::abs(eth._f_RMh) > 1e-12) {
            pos_std_m(0) /= eth._f_RMh;
        }
        if (std::abs(eth._f_cbRNh) > 1e-12) {
            pos_std_m(1) /= eth._f_cbRNh;
        }
        Eigen::Vector3d bg_std_dps = LayoutStd3(pk_std, layout, BlockId::GyroBias) / constants::_dps;
        Eigen::Vector3d ba_std = LayoutStd3(pk_std, layout, BlockId::AccBias);

        log_file_ << "  state_std:\n";
        log_file_ << "    att_deg=" << FormatVec3(att_std_deg, 4) << "\n";
        log_file_ << "    vel_mps=" << FormatVec3(vel_std, 4) << "\n";
        log_file_ << "    pos_m=" << FormatVec3(pos_std_m, 4) << "\n";
        log_file_ << "    bg_dps=" << FormatVec3(bg_std_dps, 6) << "\n";
        log_file_ << "    ba_mps2=" << FormatVec3(ba_std, 6) << "\n";
        if (layout.Has(BlockId::LeverArm)) {
            log_file_ << "    lever_m="
                      << FormatVec3(LayoutStd3(pk_std, layout, BlockId::LeverArm), 4) << "\n";
        }
    }

    const double yaw_deg = (_PI / 2.0 + nominal._att(2)) * _R2D;
    const double speed_hor =
        std::sqrt(nominal._vn(0) * nominal._vn(0) + nominal._vn(1) * nominal._vn(1));
    log_file_ << "  state_brief: yaw_deg=" << std::fixed << std::setprecision(3) << yaw_deg
              << " speed_hor_mps=" << speed_hor << " vu_mps=" << nominal._vn(2) << "\n";
}

void ResultWriter::WriteMeasEvent(const MeasLogEvent& rec) {
    if (!log_file_.is_open()) {
        return;
    }
    WriteEventBlockHeader(log_file_, rec);
    log_file_ << "  quality: passed=" << (rec.quality_passed ? 1 : 0);
    if (!rec.reject_reason.empty()) {
        log_file_ << " reason=" << rec.reject_reason;
    }
    log_file_ << "\n";
    log_file_ << "  update: fused=" << (rec.fused ? 1 : 0) << "\n";

    if (rec.type == "LIDAR") {
        log_file_ << "  scan_context: map_initialized=" << (rec.map_initialized ? 1 : 0)
                  << " undistort_poses_size=" << rec.undistort_poses_size << "\n";
    }

    WriteResidualLine(log_file_, "residual_pre", rec.pre, rec.type);
    if (rec.fused && rec.type != "LIDAR") {
        WriteResidualLine(log_file_, "residual_post", rec.post, rec.type);
    }

    if (rec.has_vel_raw) {
        log_file_ << "  meas_raw: hor_speed_mps=" << std::fixed << std::setprecision(6)
                  << rec.hor_speed_mps << " trk_deg=" << rec.trk_deg
                  << " ver_speed_mps=" << rec.ver_speed_mps << "\n";
    }

    if (rec.type == "LIDAR" && rec.lidar_sample_count > 0) {
        log_file_ << "  residual_summary: constraint_count=" << rec.lidar_effective
                  << " sample_count=" << rec.lidar_sample_count
                  << " res_abs_mean=" << std::fixed << std::setprecision(6) << rec.lidar_res_abs_mean
                  << " res_abs_max=" << rec.lidar_res_abs_max
                  << " norm_res_rms=" << rec.lidar_norm_res_rms << "\n";
    }
    if (rec.has_lidar_feature) {
        log_file_ << "  lidar_feature: filtered=" << rec.lidar_filtered
                  << " candidate=" << rec.lidar_candidate << " effective=" << rec.lidar_effective
                  << " iter_noise_var=" << std::fixed << std::setprecision(6)
                  << rec.lidar_iter_noise_var << "\n";
    }
    if (rec.has_undistort) {
        log_file_ << "  undistort: rot_diff_rad=" << std::fixed << std::setprecision(6)
                  << rec.undistort_max_rot_diff_rad
                  << " rot_diff_vec=" << FormatVec3(rec.undistort_max_rot_diff_vec, 6)
                  << " rot_diff_rel_time_ms=" << rec.undistort_max_rot_diff_rel_time_ms
                  << " max_comp_m=" << rec.undistort_max_compensation_m
                  << " mean_comp_m=" << rec.undistort_mean_compensation_m << "\n";
    }
    if (rec.has_iekf) {
        log_file_ << "  iekf: iter=" << rec.iekf_total_iter
                  << " converge_at=" << rec.iekf_converge_iter
                  << " converged=" << (rec.iekf_final_converged ? 1 : 0)
                  << " max_dx_last=" << std::fixed << std::setprecision(6) << rec.iekf_max_dx_last
                  << "\n";
    }
    if (rec.has_state_delta) {
        WriteStateDelta(log_file_, rec.nominal_before, rec.nominal_after, rec.eth_after);
    }
}

void ResultWriter::Close() {
    if (gloc_file_.is_open()) {
        gloc_file_.close();
    }
    if (sins_file_.is_open()) {
        sins_file_.close();
    }
    if (pre_file_.is_open()) {
        pre_file_.close();
    }
    if (log_file_.is_open()) {
        log_file_.close();
    }
}

}  // namespace msf
