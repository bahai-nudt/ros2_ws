cmake_minimum_required(VERSION 3.8)
project(base_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(PROJ REQUIRED proj)

include_directories(
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  include 
  ${catkin_INCLUDE_DIRS})

# 创建一个库
add_library(base_utils SHARED
  src/tools.cpp
  src/coordinate.cpp
)

target_link_libraries(base_utils ${PROJ_LIBRARIES} ${PCL_LIBRARIES} GeographicLib
)
ament_target_dependencies(base_utils rclcpp std_msgs geometry_msgs)

install(TARGETS base_utils
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS base_utils
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

add_executable(test_demo
  src/test.cpp
  src/tools.cpp
  src/coordinate.cpp
)

target_link_libraries(test_demo ${PROJ_LIBRARIES} ${PCL_LIBRARIES} GeographicLib
)
ament_target_dependencies(test_demo rclcpp std_msgs geometry_msgs)

install(TARGETS test_demo
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS test_demo
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)




ament_export_libraries(base_utils)
ament_export_include_directories(include)

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
