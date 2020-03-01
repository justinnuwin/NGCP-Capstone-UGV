cmake_minimum_required(VERSION 2.8.7)
project(bugv)

find_package(catkin REQUIRED COMPONENTS mavros)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(bugv_logic src/bugv_logic.cpp src/bugv_control.cpp src/bugv_nav.cpp)
target_link_libraries(bugv_logic ${catkin_LIBRARIES})

#Add all files in subdirectories of the project in
# a dummy_target so qtcreator have access to all files
FILE(GLOB_RECURSE extra_files ${CMAKE_SOURCE_DIR}/*)
add_custom_target(dummy_${PROJECT_NAME} SOURCES ${extra_files})

###############################################################################
# INSTALL

install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
