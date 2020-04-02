cmake_minimum_required(VERSION 2.8.3)
project(open_arms_interface)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

catkin_install_python(PROGRAMS nodes/move_one_motor.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    USE_SOURCE_PERMISSIONS
)