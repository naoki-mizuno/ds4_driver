cmake_minimum_required(VERSION 2.8.3)
project(ds4_driver)

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

# catkin_install_python(PROGRAMS
#   nodes/ds4_driver_node.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
