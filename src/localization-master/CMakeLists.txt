cmake_minimum_required(VERSION 3.14)
project(localization VERSION 0.1.0 LANGUAGES CXX C)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(CompilerOptions)
include(ModuleGuard)

option(LOC_BUILD_DEMO "Build demo apps and/or unit tests under demo/" OFF)
option(LOC_BUILD_DEMO_APPS "Build demo executables (requires LOC_BUILD_DEMO)" ON)
option(LOC_BUILD_DEMO_TESTS "Build demo unit tests (requires LOC_BUILD_DEMO)" ON)

# 所有 demo 可执行文件统一输出到 build/bin
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(Threads REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters kdtree)
find_package(OpenMP REQUIRED COMPONENTS CXX)
find_library(GEOGRAPHIC_LIB GeographicLib REQUIRED)

# Dependency order: core → tools → sensors → estimator → io
add_subdirectory(modules/core)
add_subdirectory(modules/tools)
add_subdirectory(modules/sensors)
add_subdirectory(modules/estimator)
add_subdirectory(modules/io)

if(LOC_BUILD_DEMO)
    if(LOC_BUILD_DEMO_APPS)
        add_subdirectory(demo/GNSS_INS)
        add_subdirectory(demo/LIO)
        add_subdirectory(demo/GNSS_LIDAR_INS)
        add_subdirectory(demo/GNSS_LIO_INS)
    endif()
    if(LOC_BUILD_DEMO_TESTS)
        enable_testing()
        add_subdirectory(demo/unit_test)
    endif()
endif()
