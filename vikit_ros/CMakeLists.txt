cmake_minimum_required(VERSION 3.5)
project(vikit_ros2)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Set build flags
add_compile_options(-Wall -D_LINUX -D_REENTRANT -march=native -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unknown-pragmas)

# Find ROS 2 packages and other dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(geometry_msgs REQUIRED)

# list(APPEND CMAKE_PREFIX_PATH "$ENV{HOME}/ros2_ws/src/Sophus/install/lib")
# list(APPEND CMAKE_PREFIX_PATH "$ENV{HOME}/ros2_ws/src/rpg_vikit/vikit_common/install/lib")
# set(Sophus_INCLUDE_DIR "$ENV{HOME}/ros2_ws/src/Sophus/install/include")
# set(vikit_common_INCLUDE_DIR "$ENV{HOME}/ros2_ws/src/rpg_vikit/vikit_common/install/include")
find_package(Sophus REQUIRED)
find_package(vikit_common REQUIRED)

# Include dirs
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIR}
  ${vikit_common_INCLUDE_DIR}
)
message("Sophus_INCLUDE_DIR: ${Sophus_INCLUDE_DIR}")
message("vikit_common_INCLUDE_DIR: ${vikit_common_INCLUDE_DIR}")
# Set Sourcefiles
set(SOURCEFILES 
  src/output_helper.cpp
)

# Create vikit library
add_library(${PROJECT_NAME} SHARED ${SOURCEFILES})
target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${Sophus_LIBRARIES}
  ${vikit_common_LIBRARIES}
)
message("Sophus_LIBRARIES: ${Sophus_LIBRARIES}")
message("vikit_common_LIBRARIES: ${vikit_common_LIBRARIES}")
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  tf2_ros
  visualization_msgs
  ament_cmake
  geometry_msgs
)
# Ament package generation and installation
# install(TARGETS ${PROJECT_NAME}
#   DESTINATION lib/${PROJECT_NAME}
# )
# install(DIRECTORY include/
#   DESTINATION include
# )
install(DIRECTORY ${vikit_common_INCLUDE_DIR}/
  DESTINATION include
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  tf2_ros
  visualization_msgs
  Eigen3
  OpenCV
  Sophus
  vikit_common
  geometry_msgs
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
ament_package()