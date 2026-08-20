# GNSS/INS/LiDAR 组合导航 demo：滤波阶段同时使用 GNSS 与 LiDAR 点面 IEKF。
# 对应 POST_MSF fusion_mode=3 且 pose_meas_enable=false。pose_meas 见 GNSS_LIO_INS。

add_executable(gnss_lidar_ins main.cpp)
target_link_libraries(gnss_lidar_ins PRIVATE
    msf_io msf_gnss msf_ekf msf_ins msf_lidar msf_tools)
msf_apply_compiler_options(gnss_lidar_ins)
msf_app(gnss_lidar_ins)
