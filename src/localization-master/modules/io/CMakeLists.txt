# IO: YAML config reader, offline DataReader, result / log writers.

add_library(msf_io
    src/data_reader.cpp
    src/yaml_reader.cpp
    src/result_writer.cpp
)

target_include_directories(msf_io PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(msf_io PUBLIC
    msf_core
    Eigen3::Eigen
    yaml-cpp
)

msf_apply_compiler_options(msf_io)
msf_module(msf_io DEPS msf_core Eigen3::Eigen yaml-cpp)
