cmake_minimum_required(VERSION 3.8)
project(gloc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(novatel_oem7_msgs REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(base_utils REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(PCL REQUIRED)
pkg_check_modules(PROJ REQUIRED proj)


include_directories(
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  include 
  ${catkin_INCLUDE_DIRS})

add_executable(gloc_node src/gloc.cpp
src/gps_processor.cpp
src/imu_processor.cpp
src/initializer.cpp
)

target_link_libraries(gloc_node 
${PROJ_LIBRARIES} 
${PCL_LIBRARIES}
) 

ament_target_dependencies(gloc_node
rclcpp 
std_msgs 
sensor_msgs 
nav_msgs
base_utils
novatel_oem7_msgs
)

# Install executable
install(TARGETS
  gloc_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files (optional)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
