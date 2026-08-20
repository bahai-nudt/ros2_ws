# GNSS/LIO/INS 组合导航 demo：GNSS 锁位置，LiDAR 用副本点面 GN 产出 pose_meas 再更新主滤波。
# 对应 POST_MSF fusion_mode=3 且 pose_meas_enable=true（默认 attitude_only）。

add_executable(gnss_lio_ins main.cpp)
target_link_libraries(gnss_lio_ins PRIVATE
    msf_io msf_gnss msf_ekf msf_ins msf_lidar msf_tools)
msf_apply_compiler_options(gnss_lio_ins)
msf_app(gnss_lio_ins)
