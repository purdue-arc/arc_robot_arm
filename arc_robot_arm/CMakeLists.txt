set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

cmake_minimum_required(VERSION 3.0.2)

project(arc_robot_arm)

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
   CATKIN_DEPENDS std_msgs roscpp rospy geometry_msgs tf2_ros 
)

include_directories(
	include
  ${catkin_INCLUDE_DIRS}
	SYSTEM
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(sensors_tf_broadcaster src/tf/sensors_tf_broadcaster.cpp)

target_link_libraries(sensors_tf_broadcaster ${catkin_LIBRARIES})

add_executable(move_to_pose src/kinematics/move_to_pose_node.cpp src/kinematics/kinematics.cpp)

target_link_libraries(move_to_pose ${catkin_LIBRARIES})

add_executable(test_kinematics src/kinematics/test_kinematics.cpp)

target_link_libraries(test_kinematics ${catkin_LIBRARIES})
