#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

#include "config/global_config.h"
#include "yaml_reader.h"

namespace {

constexpr const char* kProjectRoot = CMAKE_SOURCE_DIR;

bool Near(double actual, double expected, double tolerance = 1e-9) {
    return std::abs(actual - expected) <= tolerance;
}

bool Vec3Near(const Eigen::Vector3d& actual, const Eigen::Vector3d& expected, double tolerance = 1e-9) {
    return (actual - expected).cwiseAbs().maxCoeff() <= tolerance;
}

bool TestDefaults() {
    const msf::GlobalConfig config;
    return config._process_option._state_dim == 18 &&
           config._process_option._meas_dim == 3 &&
           config._topics._imu_topic == "/bynav/imu/data_raw" &&
           config._topics._bestgnsspos_topic == "/bynav/bestgnsspos" &&
           config._topics._lidar_topic == "/rslidar/em4_front/raw" &&
           config._lidar_frame_id == "lidar_front" &&
           Near(config._calibration._heading_offset_deg, 0.0) &&
           Vec3Near(config._calibration._imu_calib._gyro_scale_factor, Eigen::Vector3d::Ones()) &&
           config._calibration._R_bv.isApprox(msf::FLU2RFU()) &&
           config._quality_control._zupt._enable &&
           Near(config._lidar._frame_voxel_size, 0.5);
}

bool TestVisualizationModePolicy() {
    return msf::EffectiveVisualizationMode(0, false) == 0 &&
           msf::EffectiveVisualizationMode(0, true) == 3 &&
           msf::EffectiveVisualizationMode(2, true) == 2 &&
           msf::EffectiveVisualizationMode(9, false) == 3;
}

bool TestMeasDimPolicy() {
    msf::GlobalConfig c1;
    c1._process_option._meas_dim = 1;
    msf::GlobalConfig c2;
    c2._process_option._meas_dim = 2;
    msf::GlobalConfig c3;
    c3._process_option._meas_dim = 3;
    return !c1._process_option.VelocityFusionEnabled() &&
           !c1._process_option.HeadingFusionEnabled() &&
           c2._process_option.VelocityFusionEnabled() &&
           !c2._process_option.HeadingFusionEnabled() &&
           c3._process_option.VelocityFusionEnabled() &&
           c3._process_option.HeadingFusionEnabled();
}

bool TestYamlLoad() {
    msf::GlobalConfig config;
    const std::string yaml_path = std::string(kProjectRoot) + "/config/parameter_default.yaml";
    if (!msf::LoadGlobalConfigFromYaml(yaml_path, config)) {
        return false;
    }

    return config._process_option._state_dim == 15 &&
           config._process_option._meas_dim == 3 &&
           config._topics._bestgnsspos_topic == "/bynav/bestgnsspos" &&
           config._lidar_frame_id == "lidar_front" &&
           config._out._visualization_mode == 3 &&
           Vec3Near(config._statistics._init._att_err_deg, Eigen::Vector3d(3.0, 3.0, 1.0)) &&
           Vec3Near(config._statistics._process._ang_random_walk, Eigen::Vector3d(0.10, 0.10, 0.13)) &&
           Vec3Near(config._statistics._meas._gnss_fixed_pos_std_floor_m,
                    Eigen::Vector3d(0.1, 0.05, 0.05)) &&
           config._quality_control._zupt._enable &&
           !config._file._output_data_dir.empty();
}

bool TestYamlYulinLoad() {
    msf::GlobalConfig config;
    const std::string yaml_path = std::string(kProjectRoot) + "/config/parameter_yulin.yaml";
    if (!msf::LoadGlobalConfigFromYaml(yaml_path, config)) {
        return false;
    }

    return config._topics._imu_topic == "/shangyu/imu/data_raw" &&
           Near(config._calibration._heading_offset_deg, 180.0) &&
           Near(config._statistics._meas._heading_meas_std_scale, 0.5) &&
           Near(config._quality_control._gnss_vel_min_speed_mps, 0.0) &&
           Near(config._quality_control._zupt._static_hor_speed_mps, 0.03);
}

bool TestMissingFile() {
    const std::filesystem::path missing =
        std::filesystem::temp_directory_path() / "msf_config_missing_file.yaml";
    msf::GlobalConfig config;
    return !msf::LoadGlobalConfig(missing.string(), config);
}

bool TestCalibFixture() {
    const std::filesystem::path temp_dir =
        std::filesystem::temp_directory_path() / "msf_config_test_calib";
    std::filesystem::remove_all(temp_dir);
    std::filesystem::create_directories(temp_dir);

    const std::filesystem::path yaml_path = temp_dir / "config.yaml";
    {
        std::ofstream file(yaml_path);
        file << "file:\n"
             << "  input_data_dir: \"" << temp_dir.string() << "\"\n"
             << "  output_data_dir: \"" << (temp_dir / "output").string() << "\"\n"
             << "topics:\n"
             << "  imu_topic: \"/bynav/imu/data_raw\"\n"
             << "  lidar_topic: \"/rslidar/em4_front/raw\"\n";
    }

    const std::filesystem::path calib_path = temp_dir / "calib.json";
    {
        std::ofstream file(calib_path);
        file << "lidar_params:\n"
             << "  - frameId: lidar_front\n"
             << "    transformMatrix:\n"
             << "      - [1.0, 0.0, 0.0, 1.0]\n"
             << "      - [0.0, 1.0, 0.0, 2.0]\n"
             << "      - [0.0, 0.0, 1.0, 3.0]\n"
             << "      - [0.0, 0.0, 0.0, 1.0]\n"
             << "imu_params:\n"
             << "  - frameId: bynav_imu\n"
             << "    transformMatrix:\n"
             << "      - [1.0, 0.0, 0.0, 4.0]\n"
             << "      - [0.0, 1.0, 0.0, 5.0]\n"
             << "      - [0.0, 0.0, 1.0, 6.0]\n"
             << "      - [0.0, 0.0, 0.0, 1.0]\n"
             << "arm_value_params:\n"
             << "  - bynav_arm_value: [0.1, 0.2, 0.3]\n";
    }

    msf::GlobalConfig config;
    if (!msf::LoadGlobalConfig(yaml_path.string(), config)) {
        return false;
    }

    return config._calibration._calib_loaded &&
           Vec3Near(config._calibration._T_bv_m, Eigen::Vector3d(4.0, 5.0, 6.0)) &&
           Vec3Near(config._calibration._T_lv_m, Eigen::Vector3d(1.0, 2.0, 3.0)) &&
           Vec3Near(config._calibration._T_gb_m, Eigen::Vector3d(0.1, 0.2, 0.3)) &&
           config._calibration._R_bl.isApprox(config._calibration._R_bv *
                                              config._calibration._R_vl);
}

bool WriteCalibFixture(const std::filesystem::path& dir) {
    const std::filesystem::path calib_path = dir / "calib.json";
    std::ofstream file(calib_path);
    if (!file) {
        return false;
    }
    file << "lidar_params:\n"
         << "  - frameId: lidar_front\n"
         << "    transformMatrix:\n"
         << "      - [1.0, 0.0, 0.0, 1.0]\n"
         << "      - [0.0, 1.0, 0.0, 2.0]\n"
         << "      - [0.0, 0.0, 1.0, 3.0]\n"
         << "      - [0.0, 0.0, 0.0, 1.0]\n"
         << "imu_params:\n"
         << "  - frameId: bynav_imu\n"
         << "    transformMatrix:\n"
         << "      - [1.0, 0.0, 0.0, 4.0]\n"
         << "      - [0.0, 1.0, 0.0, 5.0]\n"
         << "      - [0.0, 0.0, 1.0, 6.0]\n"
         << "      - [0.0, 0.0, 0.0, 1.0]\n"
         << "arm_value_params:\n"
         << "  - bynav_arm_value: [0.1, 0.2, 0.3]\n";
    return true;
}

bool TestLidarImuYamlOverride() {
    const std::filesystem::path temp_dir =
        std::filesystem::temp_directory_path() / "msf_config_test_lidar_imu_yaml";
    std::filesystem::remove_all(temp_dir);
    std::filesystem::create_directories(temp_dir);
    if (!WriteCalibFixture(temp_dir)) {
        return false;
    }

    const std::filesystem::path yaml_path = temp_dir / "config.yaml";
    {
        std::ofstream file(yaml_path);
        file << "file:\n"
             << "  input_data_dir: \"" << temp_dir.string() << "\"\n"
             << "  output_data_dir: \"" << (temp_dir / "output").string() << "\"\n"
             << "topics:\n"
             << "  imu_topic: \"/bynav/imu/data_raw\"\n"
             << "  lidar_topic: \"/rslidar/em4_front/raw\"\n"
             << "calibration:\n"
             << "  lidar_imu_extrinsic:\n"
             << "    enable: true\n"
             << "    R_bl: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]\n"
             << "    T_lb_m: [9.0, 8.0, 7.0]\n";
    }

    msf::GlobalConfig config;
    if (!msf::LoadGlobalConfig(yaml_path.string(), config)) {
        return false;
    }
    return config._calibration._use_lidar_imu_yaml &&
           config._calibration._R_bl.isApprox(Eigen::Matrix3d::Identity()) &&
           Vec3Near(config._calibration._T_lb_m, Eigen::Vector3d(9.0, 8.0, 7.0));
}

bool TestLidarImuYamlInvalid() {
    const std::filesystem::path temp_dir =
        std::filesystem::temp_directory_path() / "msf_config_test_lidar_imu_bad";
    std::filesystem::remove_all(temp_dir);
    std::filesystem::create_directories(temp_dir);

    const std::filesystem::path yaml_path = temp_dir / "config.yaml";
    {
        std::ofstream file(yaml_path);
        file << "file:\n"
             << "  input_data_dir: \"" << temp_dir.string() << "\"\n"
             << "topics:\n"
             << "  imu_topic: \"/bynav/imu/data_raw\"\n"
             << "  lidar_topic: \"/rslidar/em4_front/raw\"\n"
             << "calibration:\n"
             << "  lidar_imu_extrinsic:\n"
             << "    enable: true\n"
             << "    R_bl: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]\n"
             << "    T_lb_m: [1.0, 2.0, 3.0]\n";
    }

    msf::GlobalConfig config;
    return !msf::LoadGlobalConfigFromYaml(yaml_path.string(), config);
}

}  // namespace

int main() {
    struct TestCase {
        const char* name;
        bool (*fn)();
    };

    const TestCase tests[] = {
        {"defaults", TestDefaults},
        {"visualization_mode_policy", TestVisualizationModePolicy},
        {"meas_dim_policy", TestMeasDimPolicy},
        {"yaml_load", TestYamlLoad},
        {"yaml_yulin_load", TestYamlYulinLoad},
        {"missing_file", TestMissingFile},
        {"calib_fixture", TestCalibFixture},
        {"lidar_imu_yaml_override", TestLidarImuYamlOverride},
        {"lidar_imu_yaml_invalid", TestLidarImuYamlInvalid},
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
