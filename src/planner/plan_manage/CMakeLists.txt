cmake_minimum_required(VERSION 3.8)
project(scan_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(bspline_opt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(path_searching REQUIRED)
find_package(plan_env REQUIRED)
find_package(rclcpp REQUIRED)
find_package(scan_planner_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(traj_utils REQUIRED)
find_package(visualization_msgs REQUIRED)

set(scan_planner_dependencies
  ament_index_cpp bspline_opt geometry_msgs nav_msgs path_searching plan_env
  rclcpp scan_planner_msgs sensor_msgs std_msgs tf2 tf2_geometry_msgs tf2_ros
  traj_utils visualization_msgs)

add_executable(scan_planner_node
  src/scan_planner_node.cpp
  src/scan_replan_fsm.cpp
  src/planner_manager.cpp)
target_include_directories(scan_planner_node PRIVATE include)
ament_target_dependencies(scan_planner_node ${scan_planner_dependencies})
target_link_libraries(scan_planner_node Eigen3::Eigen)

foreach(target open_loop_controller closed_loop_controller go2_kinematic_sim go2_gait_publisher)
  add_executable(${target} src/${target}.cpp)
  target_include_directories(${target} PRIVATE include)
  ament_target_dependencies(${target}
    bspline_opt geometry_msgs nav_msgs rclcpp scan_planner_msgs sensor_msgs
    std_msgs tf2 tf2_geometry_msgs tf2_ros)
  target_link_libraries(${target} Eigen3::Eigen)
endforeach()

install(TARGETS
  scan_planner_node open_loop_controller closed_loop_controller
  go2_kinematic_sim go2_gait_publisher
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME} OPTIONAL)
install(FILES launch/default.rviz DESTINATION share/${PROJECT_NAME}/rviz OPTIONAL)
install(PROGRAMS
  scripts/keypoint_recorder.py
  scripts/reference_path_publisher.py
  DESTINATION lib/${PROJECT_NAME} OPTIONAL)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_add_gtest(test_reference_path_utils test/test_reference_path_utils.cpp)
  target_include_directories(test_reference_path_utils PRIVATE include)
  ament_target_dependencies(test_reference_path_utils nav_msgs)
  target_link_libraries(test_reference_path_utils Eigen3::Eigen)
  set(_pytest_disable_plugin_autoload "$ENV{PYTEST_DISABLE_PLUGIN_AUTOLOAD}")
  set(ENV{PYTEST_DISABLE_PLUGIN_AUTOLOAD} "1")
  ament_add_pytest_test(
    test_reference_path_publisher_unit
    test/test_reference_path_publisher_unit.py
    PYTHON_EXECUTABLE /usr/bin/python3
    ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1)
  set(ENV{PYTEST_DISABLE_PLUGIN_AUTOLOAD} "${_pytest_disable_plugin_autoload}")
  add_launch_test(test/test_planner_startup.py TIMEOUT 30)
  add_launch_test(test/test_reference_path_publisher_launch.py TIMEOUT 30)
  add_launch_test(test/test_waypoint_parameters.py TIMEOUT 30)
endif()

ament_package()
