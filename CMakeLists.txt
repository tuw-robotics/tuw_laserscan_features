cmake_minimum_required(VERSION 3.5)
project(tuw_laserscan_features)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tuw_geometry_msgs REQUIRED)
find_package(tuw_geometry REQUIRED)

include_directories(include)

# This package installs libraries without exporting them.
# Export the library path to ensure that the installed libraries are available.
if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()


add_library(linedetection_node SHARED
            src/linedetection_node.cpp src/linedetection_parameter.cpp)
target_compile_definitions(linedetection_node
  PRIVATE "TUW2_EXAMPLE_PACKAGE_DLL")
ament_target_dependencies(linedetection_node rclcpp rclcpp_components std_msgs sensor_msgs tuw_geometry_msgs tuw_geometry)


add_executable(linedetection src/standalone_linedetection.cpp)
target_link_libraries(linedetection linedetection_node)
ament_target_dependencies(linedetection
  rclcpp)

add_library(feature_viz_node SHARED
            src/feature_viz_node.cpp)
target_compile_definitions(feature_viz_node
  PRIVATE "TUW2_EXAMPLE_PACKAGE_DLL")
ament_target_dependencies(feature_viz_node rclcpp rclcpp_components std_msgs sensor_msgs tuw_geometry_msgs tuw_geometry)

add_executable(feature_viz src/standalone_feature_viz.cpp)
target_link_libraries(feature_viz feature_viz_node)
ament_target_dependencies(feature_viz
  rclcpp)

add_executable(composed src/composed.cpp)
target_link_libraries(composed linedetection_node feature_viz_node)
ament_target_dependencies(composed rclcpp class_loader)

install(TARGETS
  linedetection_node
  feature_viz_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  composed
  linedetection
  feature_viz
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(pub_test test/test_subscriber.cpp)  # Need to link .cpp file under test
  target_include_directories(pub_test PUBLIC src/)  # Need to specify directories in order to #include from them in the test
  target_link_libraries(pub_test linedetection_node)
  ament_target_dependencies(pub_test rclcpp rclcpp_components std_msgs sensor_msgs tuw_geometry_msgs tuw_geometry)  # Gtest requires rclcpp to instantiate the Node

endif()

ament_package()
