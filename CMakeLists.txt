cmake_minimum_required(VERSION 2.8.3)
project(one_easy_protocol_pkg)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs rospy)

catkin_python_setup()

add_service_files(DIRECTORY srv FILES Move.srv Light.srv Gripper.srv ExtMotor.srv Connect.srv Disconnect.srv)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS std_msgs rospy)

catkin_install_python(PROGRAMS scripts/one_ctrl_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
