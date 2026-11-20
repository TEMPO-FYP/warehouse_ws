cmake_minimum_required(VERSION 3.8)
project(b1z1_control)

find_package(ament_cmake REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-plugin2 REQUIRED)

add_library(dynamic_attach_system SHARED
  src/dynamic_attach_system.cpp
)
target_link_libraries(dynamic_attach_system
  gz-sim8::gz-sim8
  gz-transport13::gz-transport13
  gz-plugin2::gz-plugin2
)

install(
  TARGETS dynamic_attach_system
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/gz_sim_plugin_path.dsv.in")

ament_package()
