cmake_minimum_required(VERSION 3.8)
project(sampling_planners_demo)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBFCL REQUIRED "fcl>=0.7.0")
# replace LIBFCL_LIBRARIES with full paths to the libraries
set(LIBFCL_LIBRARIES_FULL "")
foreach(LIBFCL_LIBRARY ${LIBFCL_LIBRARIES})
  find_library(${LIBFCL_LIBRARY}_LIB ${LIBFCL_LIBRARY} ${LIBFCL_LIBRARY_DIRS})
  list(APPEND LIBFCL_LIBRARIES_FULL ${${LIBFCL_LIBRARY}_LIB})
endforeach()
set(LIBFCL_LIBRARIES "${LIBFCL_LIBRARIES_FULL}")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ompl REQUIRED)
find_package(OCTOMAP REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(octomap_server REQUIRED)


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

install(PROGRAMS
  scripts/navigator_rrt.py
  scripts/navigator_rrt_star.py
  scripts/online_navigator.py #? REGISTER NODE EXECUTABLE THIS WAY SINCE THIS IS A C++ ROS2 PACKAGE
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})


set(dependencies
  rclcpp
  octomap_msgs
  nav_msgs
  std_msgs
  std_srvs
  OCTOMAP
  LIBFCL
  OMPL
  geometry_msgs
  tf2_geometry_msgs
  visualization_msgs
)

include_directories(
include
SYSTEM
)

# Add links to PCL definitons and directories
include_directories(${FCL_INCLUDE_DIRS})

add_executable(ompl_planning_demo 
src/planning_framework_main.cpp
src/state_validity_checker_octomap_fcl_R2.cpp
)
ament_target_dependencies(ompl_planning_demo
  ${dependencies}
  ${fcl_LIBRARIES}
)
target_link_libraries(ompl_planning_demo ${LIBFCL_LIBRARIES} ${OCTOMAP_LIBRARIES} ${OMPL_LIBRARIES})

install(TARGETS
  ompl_planning_demo
  DESTINATION lib/${PROJECT_NAME})

ament_package()
