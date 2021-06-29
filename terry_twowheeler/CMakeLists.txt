cmake_minimum_required(VERSION 2.8.3)
project(terry_twowheeler)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  roscpp
  sensor_msgs
  geometry_msgs)
  
# Find Gazebo
find_package(gazebo REQUIRED)

find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
  CATKIN_DEPENDS 
    roscpp
    std_msgs
    sensor_msgs
    geometry_msgs
)

include_directories(${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
include_directories(SYSTEM ${Boost_INCLUDE_DIR})
link_directories(${GAZEBO_LIBRARY_DIRS} ${catkin_LIBRARY_DIRS})

add_executable(terry_controller src/terry_controller.cpp)
target_link_libraries(terry_controller ${catkin_LIBRARIES})

add_executable(terry_velocity_controller src/terry_velocity_controller.cpp)
target_link_libraries(terry_velocity_controller ${catkin_LIBRARIES})

# Build 3D lidar plugin *BONUS*
add_library(velodyne_plugin SHARED src/velodyne_plugin.cc)
target_link_libraries(velodyne_plugin ${GAZEBO_LIBRARIES})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(TARGETS terry_controller DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
