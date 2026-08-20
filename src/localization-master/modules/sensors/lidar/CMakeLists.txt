# LiDAR sensor module: frontend / map / plane factor / visualizer。
# Iron rule: msf_lidar 只依赖 msf_core 与第三方库（PCL/OpenMP/Threads），不依赖 estimator 与其它 sensor。
#
# ikd-Tree 保持 FAST-LIO 原文件不改，仅作为 PRIVATE 源码编进本库，不进入公共 include。
# 可视化放在本模块：它消费点云与外参；io 不能依赖 msf_lidar。

find_package(PCL REQUIRED COMPONENTS visualization)

add_library(msf_lidar
    third_party/ikd-Tree/ikd_Tree.cpp
    src/pcd_preprocess.cpp
    src/scan_deskewer.cpp
    src/local_map.cpp
    src/lidar_plane_factor.cpp
    src/lidar_pose_meas.cpp
    src/lidar_pose_factor.cpp
    src/lidar_visualizer.cpp
)

target_include_directories(msf_lidar
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${PCL_INCLUDE_DIRS}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/third_party/ikd-Tree
)

target_link_libraries(msf_lidar PUBLIC
    msf_core
    Eigen3::Eigen
    ${PCL_LIBRARIES}
    OpenMP::OpenMP_CXX
    Threads::Threads
)

target_compile_definitions(msf_lidar PRIVATE ${PCL_DEFINITIONS} MP_EN)

msf_apply_compiler_options(msf_lidar)
msf_module(msf_lidar DEPS msf_core Eigen3::Eigen ${PCL_LIBRARIES} OpenMP::OpenMP_CXX Threads::Threads)
