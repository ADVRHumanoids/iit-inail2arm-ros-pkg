cmake_minimum_required(VERSION 3.0)
project(inail2arm_config)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY hal
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES inail2arm_basic.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
