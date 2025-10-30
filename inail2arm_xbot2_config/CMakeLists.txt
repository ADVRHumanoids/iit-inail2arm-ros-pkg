cmake_minimum_required(VERSION 3.8)
project(inail2arm_xbot2_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY hal config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()