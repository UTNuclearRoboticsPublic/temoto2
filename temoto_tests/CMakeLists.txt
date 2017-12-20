cmake_minimum_required(VERSION 2.8.3)
project(temoto_tests)

add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES temoto_tests
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories( ${catkin_INCLUDE_DIRS})


# # # # # # # # # # # #
# ALGORITHM 0
# # # # # # # # # # # #

add_executable(algorithm_0 src/algorithm_0.cpp)
add_dependencies(algorithm_0 ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(algorithm_0 ${catkin_LIBRARIES} )
