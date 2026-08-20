# LIO demo：GNSS 仅用于初始化，滤波阶段使用 IMU + LiDAR 点面 IEKF。
# 对应 POST_MSF fusion_mode=2；输出 gloc_result.txt 用于回归对比。

add_executable(lio main.cpp)
target_link_libraries(lio PRIVATE msf_io msf_gnss msf_ekf msf_ins msf_lidar msf_tools)
msf_apply_compiler_options(lio)
msf_app(lio)
