cmake_minimum_required(VERSION 3.8)
project(sp_vision_launch)

find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Create symlink to source configs (persists across builds)
install(CODE "execute_process(
  COMMAND ${CMAKE_COMMAND} -E create_symlink
  /home/infantry/infantry_ws/src/sp_vision_25_rosy/configs
  ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/configs
)")

ament_package()
