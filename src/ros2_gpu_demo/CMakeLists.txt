cmake_minimum_required(VERSION 3.5)
project(ros2_gpu_demo)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(CUDA REQUIRED)

cuda_add_executable(gpu_matrix_multiply_node
  src/gpu_matrix_multiply_node.cu
)

# 包含PCL头文件和库路径
include_directories(
  include 
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS})

target_link_libraries(gpu_matrix_multiply_node
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
)

ament_target_dependencies(gpu_matrix_multiply_node
rclcpp 
std_msgs 
)


install(TARGETS
  gpu_matrix_multiply_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()