cmake_minimum_required(VERSION 2.8.3)
project(ros2kitti)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	pcl_ros
	sensor_msgs
	cv_bridge
	image_transport
	std_msgs
)
find_package(OpenCV REQUIRED)

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

add_executable(ros2kitti_node src/ros2kitti_2.cpp)
add_dependencies(ros2kitti_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ros2kitti_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

