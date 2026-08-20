# Estimator: loosely-coupled navigation EKF base (EkfState + filter algebra).
# Iron rule: ekf must not include concrete sensor measurement modules; INS may consume
# shared nominal-state definitions, but fusion logic stays out of sensors.

add_library(msf_ekf
    ekf/src/ekf.cpp
    ekf/src/ekf_state.cpp
)

target_include_directories(msf_ekf PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/ekf/include
)

target_link_libraries(msf_ekf PUBLIC msf_core)

msf_apply_compiler_options(msf_ekf)
msf_module(msf_ekf DEPS msf_core)
