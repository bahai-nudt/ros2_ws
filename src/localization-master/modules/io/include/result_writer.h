#pragma once

#include <fstream>
#include <string>
#include <vector>

#include <Eigen/Eigen>

#include "config/global_config.h"
#include "interfaces/meas_factor.h"
#include "math/earth.h"
#include "types/nav_state.h"
#include "types/sensors/imu_message.h"
#include "types/state_layout.h"

namespace msf {

/** 量测残差诊断（core 类型，供 log.txt 事件块使用）。 */
struct ResidualDiag {
    bool valid = false;
    Eigen::VectorXd res;
    Eigen::VectorXd norm_res;
    Eigen::VectorXd sqrt_S;
};

/**
 * 量测 / IMU 事件块（对齐 POST_MSF log.txt）。
 * plot_res.py 依赖 type / fused / residual_pre / residual_post / residual_summary。
 */
struct MeasLogEvent {
    size_t seq = 0;
    std::string type;  ///< GNSS_POS / GNSS_VEL / HEADING / LIDAR
    double event_ts = 0.0;
    double sensor_ts = 0.0;
    bool quality_passed = false;
    bool fused = false;
    std::string reject_reason;

    ResidualDiag pre;
    ResidualDiag post;

    bool has_vel_raw = false;
    double hor_speed_mps = 0.0;
    double trk_deg = 0.0;
    double ver_speed_mps = 0.0;

    bool has_lidar_feature = false;
    int lidar_filtered = 0;
    int lidar_candidate = 0;
    int lidar_effective = 0;
    double lidar_iter_noise_var = 0.0;
    bool map_initialized = false;
    int undistort_poses_size = 0;

    int lidar_sample_count = 0;
    double lidar_res_abs_mean = 0.0;
    double lidar_res_abs_max = 0.0;
    double lidar_norm_res_rms = 0.0;

    bool has_undistort = false;
    double undistort_max_rot_diff_rad = 0.0;
    double undistort_max_rot_diff_rel_time_ms = 0.0;
    Eigen::Vector3d undistort_max_rot_diff_vec = Eigen::Vector3d::Zero();
    double undistort_max_compensation_m = 0.0;
    double undistort_mean_compensation_m = 0.0;

    bool has_iekf = false;
    int iekf_total_iter = 0;
    int iekf_converge_iter = -1;
    bool iekf_final_converged = false;
    double iekf_max_dx_last = 0.0;

    bool has_state_delta = false;
    NominalState nominal_before;
    NominalState nominal_after;
    earth eth_after;
};

/** log.txt 开头四段：运行配置 / 数据加载 / 时间线 / EKF 初始化。 */
struct LogPreamble {
    std::string config_file;
    std::string fusion_mode_name;
    bool pose_meas_enable = false;
    bool has_lidar = false;

    std::vector<std::string> loaded_paths;
    size_t imu_count = 0;
    size_t gnss_pos_count = 0;
    size_t gnss_vel_count = 0;
    size_t heading_count = 0;
    size_t lidar_count = 0;
    double data_start_time = 0.0;
    double data_end_time = 0.0;

    size_t timeline_total = 0;
    size_t timeline_imu = 0;
    size_t timeline_gnss_pos = 0;
    size_t timeline_gnss_vel = 0;
    size_t timeline_heading = 0;
    size_t timeline_lidar = 0;

    NominalState ekf_nominal;
    earth ekf_eth;
    Eigen::MatrixXd Pk;
    Eigen::VectorXd Qt;
    StateLayout layout;
};

/** 由量测块与当前 P 计算新息残差 / 归一化残差 / sqrt(S)（假定误差状态为零）。 */
ResidualDiag EvaluateResidual(const MeasFactor::MeasBlock& block, const Eigen::MatrixXd& Pk);

inline ResidualDiag EvaluateFactorResidual(MeasFactor& factor, NominalState& nominal, earth& eth,
                                           const StateLayout& layout, const Eigen::MatrixXd& Pk) {
    IterationContext ctx;
    return EvaluateResidual(factor.BuildMeasBlock(nominal, eth, layout, ctx), Pk);
}

/** GNSS 位置残差：lat/lon 弧度 → 北/东/天米（sqrt_S 同步换算）。 */
void ScaleGeodeticResidualToMeters(ResidualDiag& diag, const earth& eth);

/** 用点面/位姿残差向量填充 LIDAR residual_summary。 */
void FillLidarResidualSummary(MeasLogEvent& ev, const Eigen::VectorXd& res,
                              const Eigen::VectorXd& norm_res);

/** 量测更新前后名义状态差（对齐 POST_MSF state_delta）。 */
void FillStateDelta(MeasLogEvent& rec, const NominalState& before, const NominalState& after,
                    const earth& eth_after);

/**
 * 结果 / 日志输出管理：创建输出目录、打开结果文件、写入导航解与量测事件。
 * 只消费 core 类型（NominalState / ImuData / GlobalConfig / MeasBlock）。
 */
class ResultWriter {
public:
    ResultWriter() = default;
    ~ResultWriter();

    ResultWriter(const ResultWriter&) = delete;
    ResultWriter& operator=(const ResultWriter&) = delete;

    /** 创建输出目录，打开 gloc / sins / pre / log，写入 gloc/sins 表头。 */
    bool OpenResultFiles(const GlobalConfig& config);

    /** 写 log.txt 配置 / 数据 / 时间线 / EKF 初始化 / 事件流说明。 */
    void WriteLogPreamble(const LogPreamble& preamble, const GlobalConfig& config);

    /** IMU 帧：写 gloc_result.txt、sins_result.txt、pre.txt。 */
    void WriteGloc(const ImuData& imu_data, const NominalState& publish_sins, int state_code,
                   const GlobalConfig& config);

    /** IMU 事件块（state_std / unfused）。 */
    void WriteImuEvent(size_t seq, double event_ts, const ImuData& imu, int state_code,
                       double unfused_sec, const NominalState& nominal, const earth& eth,
                       const Eigen::MatrixXd& Pk, const StateLayout& layout);

    /** 写一条量测事件块到 log.txt。 */
    void WriteMeasEvent(const MeasLogEvent& rec);

    const std::string& GlocPath() const { return gloc_path_; }
    const std::string& SinsPath() const { return sins_path_; }
    const std::string& PrePath() const { return pre_path_; }
    const std::string& LogPath() const { return log_path_; }

    void Close();

private:
    std::ofstream gloc_file_;
    std::ofstream sins_file_;
    std::ofstream pre_file_;
    std::ofstream log_file_;
    std::string gloc_path_;
    std::string sins_path_;
    std::string pre_path_;
    std::string log_path_;
};

}  // namespace msf
