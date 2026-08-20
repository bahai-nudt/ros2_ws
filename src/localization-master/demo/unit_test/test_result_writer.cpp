#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "config/global_config.h"
#include "math/constants.h"
#include "math/earth.h"
#include "result_writer.h"
#include "types/sensors/imu_message.h"
#include "types/state_layout.h"

namespace {

bool Near(double actual, double expected, double tolerance = 1e-9) {
    return std::abs(actual - expected) <= tolerance;
}

std::vector<std::string> SplitWs(const std::string& line) {
    std::istringstream iss(line);
    std::vector<std::string> tokens;
    std::string tok;
    while (iss >> tok) {
        tokens.push_back(tok);
    }
    return tokens;
}

bool TestSinsPreAndLog() {
    const auto dir = std::filesystem::temp_directory_path() / "msf_result_writer_out";
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir);

    msf::GlobalConfig config;
    config._file._output_data_dir = dir.string();
    config._local_map_lla_deg = Eigen::Vector3d(30.0, 120.0, 100.0);
    config._calibration._gloc_origin_lla_deg = Eigen::Vector3d(30.0, 120.0, 100.0);
    config._calibration._R_bl = Eigen::Matrix3d::Identity();
    config._calibration._T_lb_m = Eigen::Vector3d(1.0, 2.0, 3.0);
    config._calibration._T_bv_m = Eigen::Vector3d::Zero();

    msf::ResultWriter writer;
    if (!writer.OpenResultFiles(config)) {
        return false;
    }

    msf::NominalState sins;
    sins._t_cur = 100.5;
    sins._pos = Eigen::Vector3d(30.0 * msf::constants::_D2R, 120.0 * msf::constants::_D2R, 100.0);
    sins._att = Eigen::Vector3d(0.1, -0.2, msf::constants::_PI);  // yaw=180° → azimuth=180
    sins._vn = Eigen::Vector3d(1.0, 2.0, 3.0);                   // E,N,U
    sins._Cnb = Eigen::Matrix3d::Identity();

    msf::ImuData imu;
    imu._timestamp = 100.5;
    imu._dt = 0.01;
    imu._gyro = Eigen::Vector3d::Zero();
    imu._accel = Eigen::Vector3d::Zero();

    msf::earth eth;
    eth.Update(sins._pos, sins._vn);
    const msf::StateLayout layout = msf::StateLayout::FromStateDim(15);
    const Eigen::MatrixXd Pk = Eigen::MatrixXd::Identity(15, 15) * 0.01;
    const Eigen::VectorXd Qt = Eigen::VectorXd::Ones(15) * 1e-6;

    config._process_option._max_process_duration_sec = 12.5;
    msf::LogPreamble preamble;
    preamble.config_file = "test.yaml";
    preamble.fusion_mode_name = "GNSS/INS";
    preamble.imu_count = 1;
    preamble.ekf_nominal = sins;
    preamble.ekf_eth = eth;
    preamble.Pk = Pk;
    preamble.Qt = Qt;
    preamble.layout = layout;
    writer.WriteLogPreamble(preamble, config);

    writer.WriteGloc(imu, sins, 2, config);
    writer.WriteImuEvent(1, 100.5, imu, 2, 0.1, sins, eth, Pk, layout);

    msf::NominalState after = sins;
    after._vn += Eigen::Vector3d(0.1, 0.0, 0.0);

    msf::MeasLogEvent rec;
    rec.seq = 12;
    rec.type = "GNSS_POS";
    rec.event_ts = 100.51;
    rec.sensor_ts = 100.50;
    rec.quality_passed = true;
    rec.fused = true;
    rec.pre.valid = true;
    rec.pre.res = Eigen::Vector3d(0.01, -0.02, 0.03);
    rec.pre.norm_res = Eigen::Vector3d(0.5, -1.0, 1.5);
    rec.pre.sqrt_S = Eigen::Vector3d(0.02, 0.02, 0.02);
    rec.post = rec.pre;
    msf::FillStateDelta(rec, sins, after, eth);
    writer.WriteMeasEvent(rec);

    msf::MeasLogEvent lidar;
    lidar.seq = 13;
    lidar.type = "LIDAR";
    lidar.event_ts = 100.52;
    lidar.sensor_ts = 100.50;
    lidar.quality_passed = true;
    lidar.fused = true;
    lidar.map_initialized = true;
    lidar.undistort_poses_size = 8;
    lidar.has_undistort = true;
    lidar.undistort_max_rot_diff_rad = 0.001;
    lidar.has_iekf = true;
    lidar.iekf_total_iter = 3;
    lidar.iekf_converge_iter = 2;
    lidar.iekf_final_converged = true;
    lidar.iekf_max_dx_last = 1e-6;
    msf::FillStateDelta(lidar, sins, after, eth);
    writer.WriteMeasEvent(lidar);
    writer.Close();

    std::ifstream sins_file(writer.SinsPath());
    std::string header;
    std::string line;
    if (!std::getline(sins_file, header) || header.find("obs_timestamp") == std::string::npos) {
        return false;
    }
    if (!std::getline(sins_file, line)) {
        return false;
    }
    const auto cols = SplitWs(line);
    if (cols.size() != 11) {
        return false;
    }
    // 100.5 s → 100500000 us，16 位补零
    if (cols[0] != "0000000100500000" || cols[10] != "0000000100500000") {
        std::cerr << "sins timestamp mismatch: " << cols[0] << std::endl;
        return false;
    }
    if (!Near(std::stod(cols[1]), 30.0, 1e-8) || !Near(std::stod(cols[2]), 120.0, 1e-8) ||
        !Near(std::stod(cols[3]), 100.0, 1e-4)) {
        return false;
    }
    // roll=att(1), pitch=att(0)
    if (!Near(std::stod(cols[4]), -0.2 * msf::constants::_R2D, 1e-5) ||
        !Near(std::stod(cols[5]), 0.1 * msf::constants::_R2D, 1e-5) ||
        !Near(std::stod(cols[6]), 180.0, 1e-5)) {
        return false;
    }
    // north=vn(1), east=vn(0), up=vn(2)
    if (!Near(std::stod(cols[7]), 2.0, 1e-4) || !Near(std::stod(cols[8]), 1.0, 1e-4) ||
        !Near(std::stod(cols[9]), 3.0, 1e-4)) {
        return false;
    }

    std::ifstream pre_file(writer.PrePath());
    if (!std::getline(pre_file, line)) {
        return false;
    }
    const auto pre_cols = SplitWs(line);
    if (pre_cols.size() != 13) {
        std::cerr << "pre.txt column count=" << pre_cols.size() << std::endl;
        return false;
    }
    if (!Near(std::stod(pre_cols[0]), 100.5, 1e-6)) {
        return false;
    }
    // Identity Cnb * Identity R_bl → I；p = 0 + T_lb
    if (!Near(std::stod(pre_cols[1]), 1.0) || !Near(std::stod(pre_cols[5]), 1.0) ||
        !Near(std::stod(pre_cols[9]), 1.0) || !Near(std::stod(pre_cols[10]), 1.0) ||
        !Near(std::stod(pre_cols[11]), 2.0) || !Near(std::stod(pre_cols[12]), 3.0)) {
        return false;
    }

    std::ifstream log_file(writer.LogPath());
    std::string log_text((std::istreambuf_iterator<char>(log_file)),
                         std::istreambuf_iterator<char>());
    if (log_text.find("[EVENT 000012] type=GNSS_POS") == std::string::npos) {
        return false;
    }
    if (log_text.find("update: fused=1") == std::string::npos) {
        return false;
    }
    if (log_text.find("residual_pre: res_m=[0.010000, -0.020000, 0.030000]") == std::string::npos) {
        std::cerr << "log residual mismatch\n" << log_text << std::endl;
        return false;
    }
    if (log_text.find("【一、运行配置】") == std::string::npos ||
        log_text.find("max_process_duration_sec: 12.500000 s") == std::string::npos ||
        log_text.find("【五、事件流记录】") == std::string::npos) {
        std::cerr << "log preamble mismatch\n" << log_text << std::endl;
        return false;
    }
    if (log_text.find("[EVENT 000001] type=IMU") == std::string::npos ||
        log_text.find("state_std:") == std::string::npos) {
        std::cerr << "log IMU event mismatch\n" << log_text << std::endl;
        return false;
    }
    if (log_text.find("state_delta:") == std::string::npos ||
        log_text.find("d_vel_mps=") == std::string::npos) {
        std::cerr << "log state_delta mismatch\n" << log_text << std::endl;
        return false;
    }
    if (log_text.find("undistort_poses_size=8") == std::string::npos ||
        log_text.find("undistort:") == std::string::npos ||
        log_text.find("iekf: iter=3") == std::string::npos) {
        std::cerr << "log lidar extra mismatch\n" << log_text << std::endl;
        return false;
    }
    return true;
}

bool TestEvaluateResidualAndScale() {
    msf::MeasFactor::MeasBlock block;
    block._valid = true;
    block._H = Eigen::MatrixXd::Identity(3, 3);
    block._z = Eigen::Vector3d(2.0, 4.0, 6.0);
    block._R_diag = Eigen::Vector3d(1.0, 1.0, 1.0);
    Eigen::MatrixXd Pk = Eigen::MatrixXd::Zero(3, 3);
    const msf::ResidualDiag diag = msf::EvaluateResidual(block, Pk);
    if (!diag.valid || !Near(diag.res(0), 2.0) || !Near(diag.sqrt_S(0), 1.0) ||
        !Near(diag.norm_res(0), 2.0)) {
        return false;
    }

    msf::earth eth;
    eth.Update(Eigen::Vector3d(45.0 * msf::constants::_D2R, 0.0, 0.0), Eigen::Vector3d::Zero());
    msf::ResidualDiag geo = diag;
    msf::ScaleGeodeticResidualToMeters(geo, eth);
    if (!Near(geo.res(0), 2.0 / eth._f_RMh, 1e-6) ||
        !Near(geo.res(1), 4.0 / eth._f_cbRNh, 1e-6) || !Near(geo.res(2), 6.0)) {
        return false;
    }
    return true;
}

}  // namespace

int main() {
    struct TestCase {
        const char* name;
        bool (*fn)();
    };
    const TestCase tests[] = {
        {"sins_pre_log", TestSinsPreAndLog},
        {"evaluate_residual_scale", TestEvaluateResidualAndScale},
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
