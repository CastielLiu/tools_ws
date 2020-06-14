cmake_minimum_required(VERSION 2.8.3)
project(pcd_tools)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  sensor_msgs
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pcd2ros
#  CATKIN_DEPENDS pcl_ros sensor_msgs std_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library

add_executable(pcd2ros_node src/pcd2ros/pcd2ros.cpp)
add_dependencies(pcd2ros_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pcd2ros_node ${catkin_LIBRARIES})

add_executable(ros2pcd_node src/ros2pcd/ros2pcd.cpp)
add_dependencies(ros2pcd_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ros2pcd_node ${catkin_LIBRARIES})


add_executable(pcdfilter_node src/pcdfilter.cpp)
add_dependencies(pcdfilter_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pcdfilter_node ${catkin_LIBRARIES})

add_executable(register_demo_node src/register_demo/register_demo.cpp)
add_dependencies(register_demo_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(register_demo_node ${catkin_LIBRARIES})

