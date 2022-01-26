cmake_minimum_required(VERSION 3.0.2)
project(gps_optimize_tool)
set(EXECUTABLE_OUTPUT_PATH "bin")

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  message_generation
)

add_message_files(
   FILES
   Info.msg
   Link.msg
   MapGraph.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
#  INCLUDE_DIRS include
#  LIBRARIES gps_optimize_tool
#  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)





add_executable(gps_distance 
               src/gps_distance.cpp)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(gps_distance
${catkin_LIBRARIES}
)
