cmake_minimum_required(VERSION 2.8.3)
project(bugv_nav)

find_package(catkin REQUIRED COMPONENTS
    mavros
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(offboard_px4 src/offboard_px4.cpp)
target_link_libraries(offboard_px4 ${catkin_LIBRARIES})
