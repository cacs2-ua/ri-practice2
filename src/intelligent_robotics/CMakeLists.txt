cmake_minimum_required(VERSION 3.1.3)
project(intelligent_robotics)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
catkin_install_python(PROGRAMS
  scripts/show_camera_robot.py
  scripts/capture_video.py
  scripts/pose_estimation.py
  scripts/obstacle_avoidance.py
  scripts/obstacle_detection.py
  scripts/blue_ackermann_low_level_bridge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)