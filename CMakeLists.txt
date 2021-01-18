cmake_minimum_required(VERSION 3.1.3)
project(ds4_driver VERSION 0.2.1)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  sensor_msgs
)

add_message_files(
  DIRECTORY msg
  FILES Feedback.msg Report.msg Status.msg Trackpad.msg
)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package()

catkin_install_python(
  PROGRAMS
    nodes/ds4_driver_node.py
    nodes/ds4_twist_node.py
    nodes/demo.py
  DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    launch/
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(
  DIRECTORY
    config/
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
