cmake_minimum_required(VERSION 3.8)
project(servo_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()




# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
include_directories(${tf2_ros_INCLUDE_DIRS})
find_package(laser_geometry REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
#find_package(PCL REQUIRED COMPONENTS common filters segmentation search features)
#find_package(pcl_conversions REQUIRED)
#find_package(PCL REQUIRED COMPONENTS people)
#find_package(pcl_conversions REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(nav_msgs REQUIRED)




#add_executable(people_detector_node src/people_detector_node.cpp)
#_dependencies(rclcpp sensor_msgs visualization_msgs pcl_conversions)
#target_link_libraries(people_detector_node
#  pcl_people pcl_common pcl_io)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetLocalDimensions.srv"
)


# Dynamic TF Broadcaster Node
add_executable(dynamic_tf_broadcaster src/dynamic_tf_broadcaster.cpp)
ament_target_dependencies(dynamic_tf_broadcaster
  rclcpp
  std_msgs
  geometry_msgs
  tf2_ros
  tf2
)

# Laser Scan to PointCloud Node
add_executable(lidar_pointcloud_node src/lidar_pointcloud_node.cpp)
ament_target_dependencies(lidar_pointcloud_node
  rclcpp
  sensor_msgs
  laser_geometry
  tf2_ros
  tf2_sensor_msgs
  tf2
)

# Scan Sequence Node
add_executable(scan_sequence_node src/scan_sequence_node.cpp)
ament_target_dependencies(scan_sequence_node
  rclcpp
  std_msgs
  sensor_msgs
)

#target_link_libraries(scan_sequence_node ${PCL_LIBRARIES})

#Tracker Node
add_executable(target_tracker_node src/target_tracker_node.cpp)
ament_target_dependencies(target_tracker_node
  rclcpp
  sensor_msgs
  std_msgs
  visualization_msgs
)

add_executable(mavros_tf_node src/mavros_tf_node.cpp)
ament_target_dependencies(mavros_tf_node
  rclcpp
  geometry_msgs
  tf2_ros
)

add_executable(local_dimensions_node src/local_dimensions_node.cpp)
ament_target_dependencies(local_dimensions_node
  rclcpp
  octomap
  octomap_msgs
)

add_executable(px4_takeoff_manager_node src/px4_takeoff_manager_node.cpp)
ament_target_dependencies(px4_takeoff_manager_node
  rclcpp
  geometry_msgs
  mavros_msgs
  nav_msgs
)

rosidl_target_interfaces(px4_takeoff_manager_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

rosidl_target_interfaces(local_dimensions_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

add_executable(octomap_planner_node src/octomap_planner_node.cpp)
ament_target_dependencies(octomap_planner_node
  rclcpp
  geometry_msgs
  nav_msgs
  octomap
  octomap_msgs
)

# Install executables
install(TARGETS
  dynamic_tf_broadcaster
  lidar_pointcloud_node
  scan_sequence_node
  target_tracker_node
  mavros_tf_node
  local_dimensions_node
  px4_takeoff_manager_node
  octomap_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(servo_control)

install(PROGRAMS
  scripts/servo_motor_node.py
  scripts/lidar_bbox_open3d.py
  DESTINATION lib/${PROJECT_NAME}
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
