cmake_minimum_required(VERSION 3.8)
project(husky_control)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/prepare_and_spawn_husky.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
