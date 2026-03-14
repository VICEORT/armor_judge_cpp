cmake_minimum_required(VERSION 3.13)
project(cloud_integrated)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenVINO REQUIRED)
find_package(OpenCV REQUIRED)
find_package(pcl_conversions REQUIRED)

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
)

# 1. Cloud Integrator Node
add_executable(cloud_integrator_node
  src/cloud_integrated_node.cpp
)
ament_target_dependencies(cloud_integrator_node
  rclcpp sensor_msgs pcl_conversions
)

# 2. Armor Judge Node (Vision + Cloud Processing)
add_executable(armor_judge_node
  src/main_ros2.cpp
  src/node.cpp
  src/Inference.cpp
  src/Detection.cpp
  src/Visualization.cpp
  src/mode_manager.cpp
  src/YawController.cpp
  src/extrinsic_projection.cpp
  src/time_associator.cpp
  src/cloud_refine.cpp
  src/armor_bbox_counter.cpp
  src/pitch.cpp
  src/yaw.cpp
)
target_link_libraries(armor_judge_node
  openvino::runtime
  ${OpenCV_LIBS}
)
ament_target_dependencies(armor_judge_node
  rclcpp sensor_msgs cv_bridge image_transport pcl_conversions
)

# 3. Non-ROS2 original node (maybe we can still build it)
add_executable(armor_judge
  src/main.cpp
  src/Inference.cpp
  src/Detection.cpp
  src/Visualization.cpp
  src/mode_manager.cpp
  src/YawController.cpp
)
target_link_libraries(armor_judge
  openvino::runtime
  ${OpenCV_LIBS}
)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(armor_judge PRIVATE -Wall -Wextra -O3)
    target_compile_options(armor_judge_node PRIVATE -Wall -Wextra -O3)
    target_compile_options(cloud_integrator_node PRIVATE -Wall -Wextra -O3)
endif()

# install
install(TARGETS
  armor_judge_node
  cloud_integrator_node
  armor_judge
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
