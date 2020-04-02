cmake_minimum_required(VERSION 2.8.3)
project(open_arms_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

catkin_install_python(PROGRAMS nodes/degtorev_relay.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    USE_SOURCE_PERMISSIONS
)

add_service_files(
  FILES
  DegToRev.srv
  MoveMotorArduino.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)