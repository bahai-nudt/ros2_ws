# Sensor modules: GNSS / INS（配置已并入 core GlobalConfig）。

# --- GNSS ---
add_library(msf_gnss
    gnss/src/gnss_quality.cpp
    gnss/src/gnss_initializer.cpp
    gnss/src/gnss_pos_factor.cpp
    gnss/src/gnss_vel_factor.cpp
    gnss/src/heading_factor.cpp
)
target_include_directories(msf_gnss PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/gnss/include
)
target_link_libraries(msf_gnss PUBLIC
    msf_core
)
msf_apply_compiler_options(msf_gnss)
msf_module(msf_gnss DEPS msf_core)

# --- INS ---
add_library(msf_ins
    ins/src/imu_processor.cpp
    ins/src/ins_propagator.cpp
    ins/src/ins_initializer.cpp
)
target_include_directories(msf_ins PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/ins/include
)
target_link_libraries(msf_ins PUBLIC
    msf_core
)
msf_apply_compiler_options(msf_ins)
msf_module(msf_ins DEPS msf_core)

# --- LiDAR ---
add_subdirectory(lidar)
