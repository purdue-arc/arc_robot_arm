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

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS std_msgs roscpp rospy geometry_msgs tf2_ros 
)

include_directories(
	include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS src/kinematics/Other/kinematics_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(sensors_tf_broadcaster src/tf/sensors_tf_broadcaster.cpp)

target_link_libraries(sensors_tf_broadcaster ${catkin_LIBRARIES})

add_executable(kinematics src/kinematics/test_kinematics.cpp src/kinematics/kinematics.cpp)

target_link_libraries(kinematics ${catkin_LIBRARIES})
