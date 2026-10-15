cmake_minimum_required(VERSION 3.8)
project(justina_sim)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/simple_justina_sim_node.py
  scripts/kinect_tilt_motor_node.py
  scripts/door_sim_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY docs DESTINATION share/${PROJECT_NAME})

ament_package()
