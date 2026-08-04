cmake_minimum_required(VERSION 3.8)
project(agri_nav2_config)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch maps rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
