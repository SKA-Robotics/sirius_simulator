cmake_minimum_required(VERSION 3.16.3...3.28.1)
project(sirius_gazebo_plugins)

# Find catkin macros and libraries
find_package(
  catkin REQUIRED
  COMPONENTS gazebo_ros
             nav_msgs
             roscpp
             sensor_msgs
             tf2)
# Find gazebo
find_package(gazebo REQUIRED)
include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

# Catkin specific configuration
catkin_package(
  CATKIN_DEPENDS
  nav_msgs
  roscpp
  sensor_msgs
  tf2
  INCLUDE_DIRS
  include
  LIBRARIES
  sirius_gazebo_ros_gps_sensor
  sirius_gazebo_ros_p3d)

# Build
add_library(sirius_gazebo_ros_gps_sensor src/sirius_gazebo_ros_gps_sensor.cpp)
add_library(sirius_gazebo_ros_imu_sensor src/sirius_gazebo_ros_imu_sensor.cpp)
add_library(sirius_gazebo_ros_magnetometer_sensor src/sirius_gazebo_ros_magnetometer_sensor.cpp)
add_library(sirius_gazebo_ros_p3d src/sirius_gazebo_ros_p3d.cpp)

target_link_libraries(sirius_gazebo_ros_gps_sensor ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(sirius_gazebo_ros_imu_sensor ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(sirius_gazebo_ros_magnetometer_sensor ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(sirius_gazebo_ros_p3d ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

# Install
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.hpp")

install(
  TARGETS sirius_gazebo_ros_gps_sensor
          sirius_gazebo_ros_imu_sensor
          sirius_gazebo_ros_magnetometer_sensor
          sirius_gazebo_ros_p3d
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
