cmake_minimum_required(VERSION 3.0.2)
project(mh5_hardware_control)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  hardware_interface
  combined_robot_hw
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS controller_manager hardware_interface combined_robot_hw
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(mh5_hardware_interfaces 
    src/mh5_dynamixel_interface.cpp
)
target_link_libraries(mh5_hardware_interfaces ${catkin_LIBRARIES})

add_executable(combo_control_node
    src/combo_control_node.cpp
)
target_link_libraries(combo_control_node ${catkin_LIBRARIES})