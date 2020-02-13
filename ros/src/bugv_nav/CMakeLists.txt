cmake_minimum_required(VERSION 2.8.3)
project(bugv_nav)

find_package(catkin REQUIRED COMPONENTS
    mavros
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(manual_rover src/manual_rover.cpp)
target_link_libraries(manual_rover ${catkin_LIBRARIES})
