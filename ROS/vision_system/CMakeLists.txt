# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
project(vision_system)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg cv_bridge grid_map grid_map_msgs)

## Declare ROS messages and services
#add_message_files(FILES Num.msg)
#add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(GenerateLocationsFromImage src/GenerateLocationsFromImage.cpp)
target_link_libraries(GenerateLocationsFromImage ${catkin_LIBRARIES})
add_dependencies(GenerateLocationsFromImage vision_system_generate_messages_cpp)

add_executable(GetLocations src/GetLocations.cpp)
target_link_libraries(GetLocations ${catkin_LIBRARIES})
add_dependencies(GetLocations vision_system_generate_messages_cpp)

# %EndTag(FULLTEXT)%
