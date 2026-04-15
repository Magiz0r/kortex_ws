cmake_minimum_required(VERSION 3.5)
project(kortex_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python executables
install(PROGRAMS
  scripts/isaac_to_real_trajectory_bridge.py
  scripts/kinova_read_only_joint_state_publisher.py
  scripts/real_to_isaac_joint_state_bridge.py
  scripts/test_huge_joint_space_jump.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
