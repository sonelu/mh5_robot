cmake_minimum_required(VERSION 3.0.2)
project(mh5_controllers)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  hardware_interface
  combined_robot_hw
  joint_trajectory_controller
  forward_command_controller
  dynamixel_sdk
  pluginlib
  mh5_hardware
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS controller_manager 
                 hardware_interface
                 combined_robot_hw
                 dynamixel_sdk
                 joint_trajectory_controller
                 mh5_hardware
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(mh5_controllers
    src/extended_trajectory_controller.cpp
    src/active_joint_controller.cpp
)
target_link_libraries(mh5_controllers ${catkin_LIBRARIES})

add_executable(combo_control_node
    src/combo_control_node.cpp
)
target_link_libraries(combo_control_node ${catkin_LIBRARIES})