cmake_minimum_required(VERSION 2.8.3)
project(open_arms_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

include_directories(${catkin_INCLUDE_DIRS})

catkin_install_python(PROGRAMS nodes/degtorev_relay.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    USE_SOURCE_PERMISSIONS
)

add_service_files(
  FILES
  RobotAngleRelay.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

add_executable(joint_states_relay src/joint_states_relay.cpp)
target_link_libraries(joint_states_relay ${catkin_LIBRARIES})