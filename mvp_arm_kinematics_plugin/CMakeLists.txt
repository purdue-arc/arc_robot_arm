cmake_minimum_required(VERSION 3.0.2)
project(mvp_arm_kinematics_plugin)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  moveit_core
	rostest
	rosunit
  pluginlib
)
## Generate added messages and services with any dependencies listed here
catkin_package(
	LIBRARIES
  INCLUDE_DIRS include 
  CATKIN_DEPENDS 
		moveit_core 
		pluginlib 
		roscpp
  DEPENDS 
		EIGEN3 
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)

include_directories(
	${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/mvp_arm_kinematics_plugin.cpp
	src/inverse_kinematics_5dof.cpp
)
target_link_libraries(${PROJECT_NAME}
	${catkin_LIBRARIES}	
)

## Add gtest based cpp test target and link libraries
if(CATKIN_ENABLE_TESTING)
	find_package(rostest REQUIRED)
	find_package(moveit_ros_planning REQUIRED)
  find_package(xmlrpcpp REQUIRED)

  include_directories(SYSTEM ${xmlrpcpp_INCLUDE_DIRS})
	include_directories(${moveit_ros_planning_INCLUDE_DIRS})

	catkin_add_executable_with_gtest(test_mvp_arm_kinematics_plugin test/test_mvp_arm_kinematics_plugin.cpp)
	target_link_libraries(test_mvp_arm_kinematics_plugin ${catkin_LIBRARIES} ${moveit_ros_planning_LIBRARIES} ${boost_LIBRARIES})

	add_rostest(test/test_kinematics.test)

endif()
