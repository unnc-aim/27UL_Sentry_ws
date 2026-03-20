cmake_minimum_required(VERSION 3.8)
project(sp_vision_launch)

find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Create symlink to source configs (persists across builds)
# Use a path relative to this package so it works in any workspace location.
set(SP_VISION_25_ROSY_CONFIGS_DIR "${CMAKE_CURRENT_LIST_DIR}/../sp_vision_25_rosy/configs")
if(NOT EXISTS "${SP_VISION_25_ROSY_CONFIGS_DIR}")
  message(FATAL_ERROR "sp_vision_25_rosy configs directory not found: ${SP_VISION_25_ROSY_CONFIGS_DIR}")
endif()
install(CODE "execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink \"${SP_VISION_25_ROSY_CONFIGS_DIR}\" \"${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/configs\")")

ament_package()
