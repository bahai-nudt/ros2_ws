cmake_minimum_required(VERSION 3.8)
project(compensation_gpu)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-std=c++17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(novatel_oem7_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(base_utils REQUIRED)

find_package(PCL REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(PROJ REQUIRED proj)

find_package(CUDA REQUIRED)


# 包含PCL头文件和库路径
include_directories(
  ${PCL_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIRS}
  include 
  ${catkin_INCLUDE_DIRS})


link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# 添加可执行文件并链接PCL库
cuda_add_executable(lidar_compensation src/compensation.cpp)
target_link_libraries(lidar_compensation ${PCL_LIBRARIES} ${PROJ_LIBRARIES})

ament_target_dependencies(lidar_compensation 
rclcpp  
std_msgs 
sensor_msgs 
pcl_conversions 
base_utils
novatel_oem7_msgs)

# Install executable
install(TARGETS
  lidar_compensation
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()