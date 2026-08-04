cmake_minimum_required(VERSION 3.8)
project(agri_vslam_bringup)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS
    scripts/imu_bias_corrector.py
    scripts/imu_visualizer.py
    scripts/record_rgbd_imu_bag.sh
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
