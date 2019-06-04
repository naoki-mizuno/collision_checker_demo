cmake_minimum_required(VERSION 2.8.3)
project(collision_checker_demo)

find_package(catkin REQUIRED)

# add_message_files(
#   DIRECTORY msg
#   FILES File1.msg
# )

catkin_python_setup()

# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

catkin_package()

# catkin_install_python(PROGRAMS
#   nodes/collision_checker_demo_node.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
