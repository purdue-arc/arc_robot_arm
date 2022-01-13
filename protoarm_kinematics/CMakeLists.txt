set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

cmake_minimum_required(VERSION 3.0.2)

project(protoarm_kinematics)

find_package(catkin REQUIRED COMPONENTS
 geometry_msgs
 moveit_core
 moveit_ros_planning_interface
 moveit_visual_tools
 std_msgs
 roscpp
 tf2_ros
)

find_package(Eigen3 REQUIRED)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS std_msgs roscpp rospy geometry_msgs tf2_ros
)

include_directories(
	include
  ${catkin_INCLUDE_DIRS}
)

add_executable(test_kinematics src/test_kinematics.cpp src/kinematics.cpp)

target_link_libraries(test_kinematics ${catkin_LIBRARIES})