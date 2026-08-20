#include "yaml_reader.h"

#include <algorithm>
#include <cmath>
#include <cctype>
#include <filesystem>
#include <iostream>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace msf {

namespace {

struct CalibTransform {
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t = Eigen::Vector3d::Zero();
};

std::vector<std::string> CandidateCalibLidarFrameIds(const std::string& frame_id) {
    std::vector<std::string> candidate_ids;
    candidate_ids.push_back(frame_id);
    if (frame_id == "front_lidar") {
        candidate_ids.push_back("lidar_front");
    } else if (frame_id == "rear_lidar") {
        candidate_ids.push_back("lidar_rear");
    } else if (frame_id == "left_lidar") {
        candidate_ids.push_back("lidar_left");
    } else if (frame_id == "right_lidar") {
        candidate_ids.push_back("lidar_right");
    } else if (frame_id == "lidar_front") {
        candidate_ids.push_back("front_lidar");
    } else if (frame_id == "lidar_rear") {
        candidate_ids.push_back("rear_lidar");
    } else if (frame_id == "lidar_left") {
        candidate_ids.push_back("left_lidar");
    } else if (frame_id == "lidar_right") {
        candidate_ids.push_back("right_lidar");
    }
    return candidate_ids;
}

bool LidarTopicToFrameId(const std::string& lidar_topic, std::string& frame_id) {
    if (lidar_topic == "/rslidar/em4_front/raw") {
        frame_id = "lidar_front";
        return true;
    }
    if (lidar_topic == "/rslidar/em4_rear/raw") {
        frame_id = "lidar_rear";
        return true;
    }
    if (lidar_topic == "/rslidar/m1p_left/raw") {
        frame_id = "lidar_left";
        return true;
    }
    if (lidar_topic == "/rslidar/m1p_right/raw") {
        frame_id = "lidar_right";
        return true;
    }
    return false;
}

bool ImuTopicToCalibIds(const std::string& imu_topic,
                        std::string& imu_frame_id,
                        std::string& arm_value_name) {
    if (imu_topic == "/bewis/imu/data_raw") {
        imu_frame_id = "bewis_imu_front";
        arm_value_name = "bewis_front_arm_value";
        return true;
    }
    if (imu_topic == "/bynav/imu/data_raw") {
        imu_frame_id = "bynav_imu";
        arm_value_name = "bynav_arm_value";
        return true;
    }
    if (imu_topic == "/shangyu/imu/data_raw") {
        imu_frame_id = "shangyu_imu";
        arm_value_name = "shangyu_arm_value";
        return true;
    }
    return false;
}

bool LoadCalibTransform(const YAML::Node& root,
                        const std::string& section,
                        const std::string& frame_id,
                        CalibTransform& transform) {
    if (!root[section] || !root[section].IsSequence()) {
        return false;
    }

    for (const auto& item : root[section]) {
        if (!item["frameId"] || item["frameId"].as<std::string>() != frame_id) {
            continue;
        }
        const YAML::Node matrix = item["transformMatrix"];
        if (!matrix || !matrix.IsSequence() || matrix.size() != 4) {
            return false;
        }

        for (int row = 0; row < 3; ++row) {
            if (!matrix[row].IsSequence() || matrix[row].size() != 4) {
                return false;
            }
            for (int col = 0; col < 3; ++col) {
                transform.R(row, col) = matrix[row][col].as<double>();
            }
            transform.t(row) = matrix[row][3].as<double>();
        }
        return true;
    }
    return false;
}

bool LoadCalibArmValue(const YAML::Node& root,
                       const std::string& arm_value_name,
                       Eigen::Vector3d& arm_value) {
    if (!root["arm_value_params"] || !root["arm_value_params"].IsSequence()) {
        return false;
    }

    for (const auto& item : root["arm_value_params"]) {
        const YAML::Node keyed_value = item[arm_value_name];
        if (keyed_value && keyed_value.IsSequence() && keyed_value.size() >= 3) {
            arm_value << keyed_value[0].as<double>(),
                         keyed_value[1].as<double>(),
                         keyed_value[2].as<double>();
            return true;
        }
        if (item["name"] && item["name"].as<std::string>() == arm_value_name) {
            const YAML::Node values = item["values"];
            if (values && values.IsSequence() && values.size() >= 3) {
                arm_value << values[0].as<double>(),
                             values[1].as<double>(),
                             values[2].as<double>();
                return true;
            }
        }
    }
    return false;
}

void ParseYamlMat3(const YAML::Node& node, const char* key, Eigen::Matrix3d& dst) {
    if (!node[key]) {
        return;
    }
    const YAML::Node arr = node[key];
    if (!arr.IsSequence() || arr.size() != 9) {
        std::cerr << "[WARN]  Invalid mat3 field: " << key << ", keep default" << std::endl;
        return;
    }
    dst << arr[0].as<double>(), arr[1].as<double>(), arr[2].as<double>(),
           arr[3].as<double>(), arr[4].as<double>(), arr[5].as<double>(),
           arr[6].as<double>(), arr[7].as<double>(), arr[8].as<double>();
}

void ParseYamlVec3(const YAML::Node& node, const char* key, Eigen::Vector3d& dst) {
    if (!node[key]) {
        return;
    }
    const YAML::Node arr = node[key];
    if (!arr.IsSequence() || arr.size() != 3) {
        std::cerr << "[WARN]  Invalid vec3 field: " << key << ", keep default" << std::endl;
        return;
    }
    dst << arr[0].as<double>(), arr[1].as<double>(), arr[2].as<double>();
}

void ParseYamlBool(const YAML::Node& node, const char* key, bool& dst) {
    if (!node[key]) {
        return;
    }
    const YAML::Node value = node[key];
    if (value.IsScalar()) {
        try {
            dst = value.as<bool>();
            return;
        } catch (...) {
        }
    }

    const std::string raw = value.as<std::string>();
    std::string lowered = raw;
    std::transform(lowered.begin(), lowered.end(), lowered.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });

    if (lowered == "1" || lowered == "true" || lowered == "on" || lowered == "yes") {
        dst = true;
        return;
    }
    if (lowered == "0" || lowered == "false" || lowered == "off" || lowered == "no") {
        dst = false;
        return;
    }
    std::cerr << "[WARN]  Invalid bool field: " << key << ", keep default" << std::endl;
}

void ParseYamlDouble(const YAML::Node& node, const char* key, double& dst) {
    if (!node[key]) {
        return;
    }
    dst = node[key].as<double>();
}

void ParseYamlString(const YAML::Node& node, const char* key, std::string& dst) {
    if (!node[key]) {
        return;
    }
    dst = node[key].as<std::string>();
}

void ParseYamlInt(const YAML::Node& node, const char* key, int& dst) {
    if (!node[key]) {
        return;
    }
    dst = node[key].as<int>();
}

}  // namespace

bool LoadGlobalConfigFromYaml(const std::string& config_file, GlobalConfig& config) {
    config = GlobalConfig();

    YAML::Node cfg;
    try {
        cfg = YAML::LoadFile(config_file);
    } catch (...) {
        std::cerr << "[ERROR] The format of config file " << config_file
                  << " is wrong. Please check (e.g. indentation)." << std::endl;
        return false;
    }

    try {
        if (cfg["process_option"]) {
            const YAML::Node process_option = cfg["process_option"];
            ParseYamlInt(process_option, "state_dim", config._process_option._state_dim);
            ParseYamlInt(process_option, "meas_dim", config._process_option._meas_dim);
            ParseYamlDouble(process_option, "max_process_duration_sec",
                            config._process_option._max_process_duration_sec);
        }

        if (cfg["file"]) {
            const YAML::Node file = cfg["file"];
            ParseYamlString(file, "input_data_dir", config._file._input_data_dir);
            ParseYamlString(file, "output_data_dir", config._file._output_data_dir);
        }

        if (cfg["topics"]) {
            const YAML::Node topics = cfg["topics"];
            ParseYamlString(topics, "imu_topic", config._topics._imu_topic);
            ParseYamlString(topics, "bestgnsspos_topic", config._topics._bestgnsspos_topic);
            ParseYamlString(topics, "bestvel_topic", config._topics._bestvel_topic);
            ParseYamlString(topics, "heading2_topic", config._topics._heading2_topic);
            ParseYamlString(topics, "lidar_topic", config._topics._lidar_topic);
        }

        if (cfg["calibration"]) {
            const YAML::Node calibration = cfg["calibration"];
            ParseYamlVec3(calibration, "gloc_origin_lla_deg",
                          config._calibration._gloc_origin_lla_deg);
            ParseYamlMat3(calibration, "R_bv", config._calibration._R_bv);
            ParseYamlDouble(calibration, "heading_offset_deg",
                            config._calibration._heading_offset_deg);
            if (calibration["lidar_imu_extrinsic"]) {
                const YAML::Node lidar_imu_extrinsic = calibration["lidar_imu_extrinsic"];
                ParseYamlBool(lidar_imu_extrinsic, "enable",
                              config._calibration._use_lidar_imu_yaml);
                if (config._calibration._use_lidar_imu_yaml) {
                    const YAML::Node rotation = lidar_imu_extrinsic["R_bl"];
                    const YAML::Node translation = lidar_imu_extrinsic["T_lb_m"];
                    if (!rotation || !rotation.IsSequence() || rotation.size() != 9 ||
                        !translation || !translation.IsSequence() || translation.size() != 3) {
                        std::cerr << "[ERROR] calibration.lidar_imu_extrinsic 开启时，"
                                  << "R_bl 必须为 9 元素数组，"
                                  << "T_lb_m 必须为 3 元素数组" << std::endl;
                        config = GlobalConfig();
                        return false;
                    }
                    ParseYamlMat3(lidar_imu_extrinsic, "R_bl", config._calibration._R_bl);
                    ParseYamlVec3(lidar_imu_extrinsic, "T_lb_m", config._calibration._T_lb_m);
                    const double orthogonal_error =
                        (config._calibration._R_bl.transpose() * config._calibration._R_bl -
                         Eigen::Matrix3d::Identity())
                            .norm();
                    const double determinant_error =
                        std::abs(config._calibration._R_bl.determinant() - 1.0);
                    if (!config._calibration._R_bl.allFinite() ||
                        !config._calibration._T_lb_m.allFinite() || orthogonal_error > 1.0e-3 ||
                        determinant_error > 1.0e-3) {
                        std::cerr << "[ERROR] calibration.lidar_imu_extrinsic 非法："
                                  << "旋转矩阵必须满足 R^T*R=I 且 det(R)=1，平移必须为有限值"
                                  << std::endl;
                        config = GlobalConfig();
                        return false;
                    }
                }
            }
            if (calibration["imu_calib"]) {
                const YAML::Node imu_calib = calibration["imu_calib"];
                ParseYamlVec3(imu_calib, "gyro_bias_dps",
                              config._calibration._imu_calib._gyro_bias_dps);
                ParseYamlVec3(imu_calib, "acc_bias_mg",
                              config._calibration._imu_calib._acc_bias_mg);
                ParseYamlVec3(imu_calib, "gyro_scale_factor",
                              config._calibration._imu_calib._gyro_scale_factor);
                ParseYamlVec3(imu_calib, "acc_scale_factor",
                              config._calibration._imu_calib._acc_scale_factor);
            }
        }

        if (cfg["statistics"]) {
            const YAML::Node statistics = cfg["statistics"];
            if (statistics["init"]) {
                const YAML::Node init_std = statistics["init"];
                ParseYamlVec3(init_std, "att_err_deg", config._statistics._init._att_err_deg);
                ParseYamlVec3(init_std, "vel_err_mps", config._statistics._init._vel_err_mps);
                ParseYamlVec3(init_std, "pos_err_m", config._statistics._init._pos_err_m);
                ParseYamlVec3(init_std, "gyro_bias_err_dps",
                              config._statistics._init._gyro_bias_err_dps);
                ParseYamlVec3(init_std, "acc_bias_err_mg",
                              config._statistics._init._acc_bias_err_mg);
                ParseYamlVec3(init_std, "lever_arm_err_m",
                              config._statistics._init._lever_arm_err_m);
            }
            if (statistics["process"]) {
                const YAML::Node process = statistics["process"];
                ParseYamlVec3(process, "ang_random_walk",
                              config._statistics._process._ang_random_walk);
                ParseYamlVec3(process, "vel_random_walk",
                              config._statistics._process._vel_random_walk);
                ParseYamlVec3(process, "gyro_bias_noise",
                              config._statistics._process._gyro_bias_noise);
                ParseYamlVec3(process, "acc_bias_noise",
                              config._statistics._process._acc_bias_noise);
                ParseYamlVec3(process, "lever_random_walk",
                              config._statistics._process._lever_random_walk);
            }
            if (statistics["meas"]) {
                const YAML::Node meas = statistics["meas"];
                ParseYamlVec3(meas, "gnss_fixed_pos_std_floor_m",
                              config._statistics._meas._gnss_fixed_pos_std_floor_m);
                ParseYamlVec3(meas, "gnss_float_pos_std_floor_m",
                              config._statistics._meas._gnss_float_pos_std_floor_m);
                ParseYamlVec3(meas, "gnss_vel_meas_std_mps",
                              config._statistics._meas._gnss_vel_meas_std_mps);
                ParseYamlDouble(meas, "heading_meas_std_floor_deg",
                                config._statistics._meas._heading_meas_std_floor_deg);
                ParseYamlDouble(meas, "heading_meas_std_scale",
                                config._statistics._meas._heading_meas_std_scale);
            }
        }

        if (cfg["quality_control"]) {
            const YAML::Node quality_control = cfg["quality_control"];
            ParseYamlDouble(quality_control, "gnss_vel_min_speed_mps",
                            config._quality_control._gnss_vel_min_speed_mps);
            ParseYamlDouble(quality_control, "heading_meas_std_max_deg",
                            config._quality_control._heading_meas_std_max_deg);
            ParseYamlDouble(quality_control, "gnss_vel_u_only_hor_speed_mps",
                            config._quality_control._gnss_vel_u_only_hor_speed_mps);
            ParseYamlDouble(quality_control, "max_no_valid_measure_sec",
                            config._quality_control._max_no_valid_measure_sec);
            if (quality_control["zupt"]) {
                const YAML::Node zupt = quality_control["zupt"];
                ParseYamlBool(zupt, "enable", config._quality_control._zupt._enable);
                ParseYamlVec3(zupt, "meas_std_mps", config._quality_control._zupt._meas_std_mps);
                ParseYamlDouble(zupt, "static_hor_speed_mps",
                                config._quality_control._zupt._static_hor_speed_mps);
                ParseYamlDouble(zupt, "static_ver_speed_mps",
                                config._quality_control._zupt._static_ver_speed_mps);
                ParseYamlDouble(zupt, "static_ins_speed_mps",
                                config._quality_control._zupt._static_ins_speed_mps);
                ParseYamlDouble(zupt, "static_ang_rate_dps",
                                config._quality_control._zupt._static_ang_rate_dps);
                ParseYamlDouble(zupt, "min_duration_sec",
                                config._quality_control._zupt._min_duration_sec);
            }
        }

        if (cfg["lidar"]) {
            const YAML::Node lidar = cfg["lidar"];
            ParseYamlBool(lidar, "enable_undistort", config._lidar._enable_undistort);
            ParseYamlInt(lidar, "init_mode", config._lidar._init_mode);
            if (config._lidar._init_mode < 0 || config._lidar._init_mode > 1) {
                config._lidar._init_mode = 0;
            }
            ParseYamlInt(lidar, "imu_init_samples", config._lidar._imu_init_samples);
            if (config._lidar._imu_init_samples < 10) {
                config._lidar._imu_init_samples = 10;
            }
            ParseYamlDouble(lidar, "frame_voxel_size", config._lidar._frame_voxel_size);
            ParseYamlDouble(lidar, "map_voxel_size", config._lidar._map_voxel_size);
            ParseYamlDouble(lidar, "det_range", config._lidar._det_range);
            ParseYamlDouble(lidar, "cube_len", config._lidar._cube_len);
            ParseYamlDouble(lidar, "iterated_update_noise_var",
                            config._lidar._iterated_update_noise_var);
            ParseYamlDouble(lidar, "iekf_convergence_threshold",
                            config._lidar._iekf_convergence_threshold);
            ParseYamlDouble(lidar, "nearest_search_max_sq_dist",
                            config._lidar._nearest_search_max_sq_dist);
            ParseYamlDouble(lidar, "plane_fit_threshold_m", config._lidar._plane_fit_threshold_m);
            ParseYamlDouble(lidar, "residual_score_scale", config._lidar._residual_score_scale);
            ParseYamlDouble(lidar, "residual_score_min", config._lidar._residual_score_min);
            ParseYamlInt(lidar, "point_filter_num", config._lidar._point_filter_num);
            ParseYamlInt(lidar, "nearest_points", config._lidar._nearest_points);
            ParseYamlInt(lidar, "min_effective_features", config._lidar._min_effective_features);
            ParseYamlInt(lidar, "max_iterations", config._lidar._max_iterations);
            ParseYamlBool(lidar, "pose_meas_attitude_only",
                          config._lidar._pose_meas_attitude_only);
            ParseYamlDouble(lidar, "pose_meas_inflate", config._lidar._pose_meas_inflate);
            ParseYamlVec3(lidar, "pose_meas_pos_std_floor_m",
                          config._lidar._pose_meas_pos_std_floor_m);
            ParseYamlVec3(lidar, "pose_meas_att_std_floor_deg",
                          config._lidar._pose_meas_att_std_floor_deg);
            ParseYamlDouble(lidar, "pose_meas_min_eigenvalue",
                            config._lidar._pose_meas_min_eigenvalue);
        }

        if (cfg["out"]) {
            const YAML::Node out = cfg["out"];
            ParseYamlBool(out, "output_world_pcd", config._out._output_world_pcd);
            ParseYamlInt(out, "visualization_mode", config._out._visualization_mode);
            if (config._out._visualization_mode < 0) {
                std::cerr << "[WARN] out.visualization_mode < 0，已钳为 0" << std::endl;
                config._out._visualization_mode = 0;
            } else if (config._out._visualization_mode > 3) {
                std::cerr << "[WARN] out.visualization_mode > 3，已钳为 3" << std::endl;
                config._out._visualization_mode = 3;
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Failed to parse parameter yaml: " << e.what() << std::endl;
        config = GlobalConfig();
        return false;
    }

    if (config._topics._lidar_topic.empty()) {
        std::cerr << "[ERROR] Missing topics.lidar_topic" << std::endl;
        config = GlobalConfig();
        return false;
    }
    if (!LidarTopicToFrameId(config._topics._lidar_topic, config._lidar_frame_id)) {
        std::cerr << "[ERROR] Unsupported topics.lidar_topic: " << config._topics._lidar_topic
                  << std::endl;
        config = GlobalConfig();
        return false;
    }

    const std::filesystem::path output_dir = config._file._output_data_dir.empty()
        ? (std::filesystem::path(config._file._input_data_dir).parent_path() / "output")
        : std::filesystem::path(config._file._output_data_dir);
    config._file._output_data_dir = output_dir.string();
    return true;
}

bool LoadGlobalConfig(const std::string& config_file, GlobalConfig& config) {
    if (!LoadGlobalConfigFromYaml(config_file, config)) {
        return false;
    }

    const std::string calib_path = config._file._input_data_dir + "/calib.json";
    if (!std::filesystem::exists(calib_path)) {
        std::cerr << "缺少必需的标定文件: " << calib_path << std::endl;
        return false;
    }
    if (!LoadCalib(calib_path, config)) {
        std::cerr << "calib.json 加载失败" << std::endl;
        return false;
    }
    config._calibration._calib_loaded = true;

    if (!config._calibration._use_lidar_imu_yaml) {
        config._calibration._R_bl = config._calibration._R_bv * config._calibration._R_vl;
        config._calibration._T_lb_m =
            config._calibration._R_bv * (config._calibration._T_lv_m - config._calibration._T_bv_m);
    }
    return true;
}

bool LoadCalib(const std::string& calib_path, GlobalConfig& config) {
    YAML::Node root;
    try {
        root = YAML::LoadFile(calib_path);
    } catch (const std::exception& e) {
        std::cerr << "[calib] 解析失败: " << calib_path << ", " << e.what() << std::endl;
        return false;
    }

    CalibTransform T_vehicle_lidar;
    CalibTransform T_vehicle_imu;
    std::string imu_frame_id;
    std::string arm_value_name;
    if (!ImuTopicToCalibIds(config._topics._imu_topic, imu_frame_id, arm_value_name)) {
        std::cerr << "[calib] 不支持的 imu_topic: " << config._topics._imu_topic << std::endl;
        return false;
    }

    bool found_lidar_extrinsic = false;
    if (!config._calibration._use_lidar_imu_yaml) {
        const auto lidar_frame_ids = CandidateCalibLidarFrameIds(config._lidar_frame_id);
        for (const auto& lidar_frame_id : lidar_frame_ids) {
            if (LoadCalibTransform(root, "lidar_params", lidar_frame_id, T_vehicle_lidar)) {
                found_lidar_extrinsic = true;
                break;
            }
        }
        if (!found_lidar_extrinsic) {
            std::cerr << "[calib] 缺少雷达外参，尝试过 frameId: ";
            for (size_t i = 0; i < lidar_frame_ids.size(); ++i) {
                if (i > 0) {
                    std::cerr << ", ";
                }
                std::cerr << lidar_frame_ids[i];
            }
            std::cerr << std::endl;
            return false;
        }
    }
    if (!LoadCalibTransform(root, "imu_params", imu_frame_id, T_vehicle_imu)) {
        std::cerr << "[calib] 缺少 " << imu_frame_id << " 外参" << std::endl;
        return false;
    }
    if (!LoadCalibArmValue(root, arm_value_name, config._calibration._T_gb_m)) {
        std::cerr << "[calib] 缺少 " << arm_value_name << " 杆臂参数" << std::endl;
        return false;
    }

    if (!config._calibration._use_lidar_imu_yaml) {
        config._calibration._R_vl = T_vehicle_lidar.R;
        config._calibration._T_lv_m = T_vehicle_lidar.t;
    }
    config._calibration._T_bv_m = T_vehicle_imu.t;
    return true;
}

}  // namespace msf
