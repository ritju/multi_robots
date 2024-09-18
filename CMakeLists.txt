cmake_minimum_required(VERSION 3.8)
project(nav2_multi_robots_avoidance)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(capella_ros_msg REQUIRED)
find_package(rclcpp_components REQUIRED)

nav2_package()

include_directories(
  include
)

add_library(multi_robots_avoidance_lib SHARED src/multi_robots_avoidance_action.cpp)

set(dependencies
  ament_cmake
  rclcpp
  tf2_geometry_msgs
  tf2
  tf2_ros
  nav2_msgs
  geometry_msgs
  builtin_interfaces
  capella_ros_msg
  rclcpp_components
)
target_include_directories(multi_robots_avoidance_lib PUBLIC include)

ament_target_dependencies(multi_robots_avoidance_lib
  ${dependencies}
)

rclcpp_components_register_node(multi_robots_avoidance_lib
    PLUGIN "multi_robots_avoidance_action::MultiRobotsAvoidanceAction"
    EXECUTABLE multi_robots_avoidance)

install(TARGETS
  multi_robots_avoidance_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch param
DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(multi_robots_avoidance_lib)
ament_export_dependencies(${dependencies})

ament_package()
