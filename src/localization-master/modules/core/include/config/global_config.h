#pragma once

#include <string>

#include <Eigen/Eigen>

#include "math/utility.h"

namespace msf {

/**
 * 全局配置（core 唯一配置载体）：对应 yaml 根节点下全部小节，
 * 外加 calib.json 外参/杆臂。由 main 加载后显式传给各模块。
 */
struct GlobalConfig {
    struct ProcessOption {
        int _state_dim = 18;   ///< 误差状态维数：15 或 18
        int _meas_dim = 3;     ///< 量测维数：1=pos, 2=pos+vel, 3=pos+vel+heading
        /// 自初始化时刻起最长处理时长 [s]；<=0 表示跑完整包。
        double _max_process_duration_sec = 0.0;

        /// meas_dim>=2 时速度（含 ZUPT）才参与融合。
        bool VelocityFusionEnabled() const { return _meas_dim >= 2; }

        /// meas_dim>=3 时双天线航向才参与融合。
        bool HeadingFusionEnabled() const { return _meas_dim >= 3; }
    };

    struct File {
        std::string _input_data_dir;   ///< raw_data 输入目录（BADP 解码产物）
        std::string _output_data_dir;  ///< 结果输出目录
    };

    struct Topics {
        std::string _imu_topic = "/bynav/imu/data_raw";
        std::string _bestgnsspos_topic = "/bynav/bestgnsspos";
        std::string _bestvel_topic = "/bynav/bestvel";
        std::string _heading2_topic = "/bynav/heading2";
        std::string _lidar_topic = "/rslidar/em4_front/raw";
    };

    /** IMU 内参标定（b 系，RFU） */
    struct ImuCalib {
        Eigen::Vector3d _gyro_bias_dps = Eigen::Vector3d::Zero();
        Eigen::Vector3d _acc_bias_mg = Eigen::Vector3d::Zero();
        Eigen::Vector3d _gyro_scale_factor = Eigen::Vector3d::Ones();
        Eigen::Vector3d _acc_scale_factor = Eigen::Vector3d::Ones();
    };

    struct Calibration {
        /// gloc_result ENU 输出原点 WGS84 [lat, lon, h]（度）
        Eigen::Vector3d _gloc_origin_lla_deg = Eigen::Vector3d::Zero();
        /// v->b：V_b = R_bv * V_v（默认 v=FLU → b=RFU）；b->v 用 _R_bv.transpose()
        Eigen::Matrix3d _R_bv = FLU2RFU();
        /// 双天线航向补偿：heading_meas = heading_raw - offset（度）
        double _heading_offset_deg = 0.0;
        ImuCalib _imu_calib;

        /// calib.json 是否已加载
        bool _calib_loaded = false;
        /// 车体←雷达（calib.json）
        Eigen::Matrix3d _R_vl = Eigen::Matrix3d::Identity();
        /// 雷达原点在车体系下的坐标 [m]
        Eigen::Vector3d _T_lv_m = Eigen::Vector3d::Zero();
        /// IMU 原点在车体系下的坐标 [m]
        Eigen::Vector3d _T_bv_m = Eigen::Vector3d::Zero();
        /// GNSS 天线相位中心相对 IMU 的杆臂（calib.json arm_value_params）[m]
        Eigen::Vector3d _T_gb_m = Eigen::Vector3d::Zero();
        /// true：用 yaml calibration.lidar_imu_extrinsic 的 R_bl / T_lb_m 覆盖 calib.json
        bool _use_lidar_imu_yaml = false;
        /// l→b（LoadCalib 后写入，或 yaml 覆盖）：V_b = R_bl * V_l，T_lb = l 系原点在 b 系下坐标
        Eigen::Matrix3d _R_bl = Eigen::Matrix3d::Identity();
        Eigen::Vector3d _T_lb_m = Eigen::Vector3d::Zero();
    };

    struct Statistics {
        struct Init {  // P0
            Eigen::Vector3d _att_err_deg = Eigen::Vector3d(1.0, 1.0, 1.0);
            Eigen::Vector3d _vel_err_mps = Eigen::Vector3d(0.1, 0.1, 0.1);
            Eigen::Vector3d _pos_err_m = Eigen::Vector3d(1.0, 1.0, 3.0);
            Eigen::Vector3d _gyro_bias_err_dps = Eigen::Vector3d(0.5, 0.5, 0.5);
            Eigen::Vector3d _acc_bias_err_mg = Eigen::Vector3d(20.0, 20.0, 20.0);
            Eigen::Vector3d _lever_arm_err_m = Eigen::Vector3d(0.1, 0.1, 0.1);
        };
        struct Process {  // Q
            Eigen::Vector3d _ang_random_walk = Eigen::Vector3d(0.10, 0.10, 0.13);   // deg/sqrt(h)
            Eigen::Vector3d _vel_random_walk = Eigen::Vector3d(60.0, 60.0, 60.0);   // ug/sqrt(Hz)
            Eigen::Vector3d _gyro_bias_noise = Eigen::Vector3d(1.8, 1.8, 1.4);      // deg/h/sqrt(h)
            Eigen::Vector3d _acc_bias_noise = Eigen::Vector3d(15.0, 15.0, 15.0);    // ug/sqrt(h)
            Eigen::Vector3d _lever_random_walk = Eigen::Vector3d(2.0e-4, 2.0e-4, 5.0e-5);
        };
        struct Meas {  // R / 观测噪声下限（GNSS / 航向）
            Eigen::Vector3d _gnss_fixed_pos_std_floor_m = Eigen::Vector3d(0.1, 0.05, 0.05);  // [E,N,U] m
            Eigen::Vector3d _gnss_float_pos_std_floor_m = Eigen::Vector3d(0.5, 0.5, 1.0);    // [E,N,U] m
            Eigen::Vector3d _gnss_vel_meas_std_mps = Eigen::Vector3d(0.15, 0.15, 0.15);      // [E,N,U] m/s
            double _heading_meas_std_floor_deg = 0.3;
            double _heading_meas_std_scale = 1.0;
        };
        Init _init;
        Process _process;
        Meas _meas;
    };

    struct QualityControl {
        struct Zupt {
            bool _enable = true;
            Eigen::Vector3d _meas_std_mps = Eigen::Vector3d(0.03, 0.03, 0.03);  // [E,N,U] m/s
            double _static_hor_speed_mps = 0.05;
            double _static_ver_speed_mps = 0.05;
            double _static_ins_speed_mps = 0.10;
            double _static_ang_rate_dps = 1.0;
            double _min_duration_sec = 0.5;
        };
        double _gnss_vel_min_speed_mps = 2.0;         ///< 低于此速度跳过整条 GNSS 速度量测
        double _heading_meas_std_max_deg = 5.0;
        double _gnss_vel_u_only_hor_speed_mps = 0.0;  ///< >0 时低速放大 E/N 速度噪声；0=关
        double _max_no_valid_measure_sec = 10.0;
        Zupt _zupt;
    };

    struct Lidar {
        bool _enable_undistort = true;
        std::string _frame_id = "front_lidar";
        /// 0=GNSS+heading 初始化；1=前若干帧 IMU 静止均值初始化（参考 FAST-LIO2）
        int _init_mode = 0;
        /// init_mode=1 时使用的 IMU 样本数
        int _imu_init_samples = 100;
        double _frame_voxel_size = 0.5;
        double _map_voxel_size = 0.5;
        double _det_range = 100.0;
        double _cube_len = 1000.0;
        double _iterated_update_noise_var = 0.001;
        double _iekf_convergence_threshold = 0.001;
        double _nearest_search_max_sq_dist = 5.0;
        double _plane_fit_threshold_m = 0.1;
        double _residual_score_scale = 0.9;
        double _residual_score_min = 0.9;
        int _point_filter_num = 70;
        int _nearest_points = 5;
        int _min_effective_features = 5;
        int _max_iterations = 5;
        /// GNSS/LIO 位姿量测：true 时只改正姿态（3 维），false 改正姿态+位置（6 维）。
        bool _pose_meas_attitude_only = true;
        double _pose_meas_inflate = 50.0;
        Eigen::Vector3d _pose_meas_pos_std_floor_m = Eigen::Vector3d(0.20, 0.20, 0.20);
        Eigen::Vector3d _pose_meas_att_std_floor_deg = Eigen::Vector3d(0.30, 0.30, 0.50);
        double _pose_meas_min_eigenvalue = 20.0;
    };

    struct Out {
        bool _output_world_pcd = false;
        /// 0:关  1:雷达系原始帧  2:雷达系去畸变帧  3:世界系累积+轨迹
        int _visualization_mode = 0;
    };

    ProcessOption _process_option;
    File _file;
    Topics _topics;
    Calibration _calibration;
    Statistics _statistics;
    QualityControl _quality_control;
    Lidar _lidar;
    Out _out;

    /// 由 lidar_topic 推导的数据目录名（解码约定，io 层使用）
    std::string _lidar_frame_id = "lidar_front";
    /// local_map 原点 LLA（度，滤波初始化后写入）；地图轴向为地理 ENU
    Eigen::Vector3d _local_map_lla_deg = Eigen::Vector3d::Zero();
};

/** yaml 为 0 且 CLI `--visualize` 时视为世界系(3)；非法值钳到 [0, 3]。 */
inline int EffectiveVisualizationMode(int yaml_mode, bool visualize_cli) {
    int mode = yaml_mode;
    if (mode < 0) {
        mode = 0;
    }
    if (mode > 3) {
        mode = 3;
    }
    if (mode > 0) {
        return mode;
    }
    return visualize_cli ? 3 : 0;
}

inline const char* VisualizationModeNameZh(int mode) {
    switch (mode) {
        case 1:
            return "原始雷达系";
        case 2:
            return "去畸变雷达系";
        case 3:
            return "世界系";
        default:
            return "关闭";
    }
}

}  // namespace msf
