# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
project(vision_system)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg cv_bridge nav_msgs geometry_msgs)

## Declare ROS messages and services
#add_message_files(FILES Num.msg)
#add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs nav_msgs geometry_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})
include_directories($(GRIDMAP_INCLUDE_DIRS))

## Link libraries
#link_libraries()

add_executable(camera_interface src/CameraInterface.cpp)
target_link_libraries(camera_interface ${catkin_LIBRARIES})
add_dependencies(camera_interface vision_system_generate_messages_cpp)

add_executable(get_locations src/GetLocations.cpp)
target_link_libraries(get_locations ${catkin_LIBRARIES})
add_dependencies(get_locations vision_system_generate_messages_cpp)

# %EndTag(FULLTEXT)%
