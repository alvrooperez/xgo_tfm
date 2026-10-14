cmake_minimum_required(VERSION 3.16)
project(xgo_sim)

find_package(ament_cmake REQUIRED)

# Install directories
install(DIRECTORY
  config
  launch
  maps
  rviz
  scripts
  urdf
  worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
