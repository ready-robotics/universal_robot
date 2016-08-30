cmake_minimum_required(VERSION 2.8.3)
project(ur10_moveit_config)

find_package(catkin REQUIRED COMPONENTS
  ready_motion_planning
  convex_decomposition
)

catkin_package()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
