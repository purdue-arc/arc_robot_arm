cmake_minimum_required(VERSION 3.0.2)
project(arc_robot_arm)

# Find catkin macros
find_package(catkin REQUIRED)

# generates cmake config files and set variables for installation
catkin_package()

catkin_install_python(PROGRAMS nodes/ikine_4dof_solver.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
