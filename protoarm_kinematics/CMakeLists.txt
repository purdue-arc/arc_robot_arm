set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

cmake_minimum_required(VERSION 3.0.2)

project(protoarm_kinematics)

find_package(catkin REQUIRED COMPONENTS
 geometry_msgs
 moveit_core
 moveit_ros_planning_interface
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

add_library(${PROJECT_NAME}
  src/kinematics.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

include_directories(
	include
  ${catkin_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(test_kinematics src/test_kinematics.cpp src/kinematics.cpp)

target_link_libraries(test_kinematics ${catkin_LIBRARIES})

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)