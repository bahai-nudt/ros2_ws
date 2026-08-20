# Shared core: math constants / earth model / quaternion / pose / coordinate conversions,
# cross-module data contracts, interfaces, and global config.
# Public headers in include/ (grouped by math/types/interfaces/config), sources in src/.

add_library(msf_core
    src/math/coordinate.cpp
    src/math/earth.cpp
    src/math/low_pass_filter.cpp
    src/math/pose_converter.cpp
    src/math/t_quat.cpp
    src/types/state_layout.cpp
    src/msf_algorithms/zupt.cpp
)

target_include_directories(msf_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(msf_core PUBLIC
    Eigen3::Eigen
    ${GEOGRAPHIC_LIB}
)

msf_apply_compiler_options(msf_core)
msf_module(msf_core DEPS Eigen3::Eigen ${GEOGRAPHIC_LIB})
