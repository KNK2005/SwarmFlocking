cmake_minimum_required(VERSION 3.8)
project(swarm_flocking_gazebo)

find_package(ament_cmake REQUIRED)

# Install world files
install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds)

# Install model files
install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
