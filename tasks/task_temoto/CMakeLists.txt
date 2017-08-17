cmake_minimum_required(VERSION 2.8.3)
project(task_temoto)

# use c++11 standard
add_compile_options(-std=c++11)

set(TEMOTO_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/../../temoto_2/include/)

find_package(catkin REQUIRED COMPONENTS
  class_loader
  roscpp
  rospy
  std_msgs
  temoto_2
	leap_motion_controller
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES task_3
#  CATKIN_DEPENDS class_loader roscpp rospy std_msgs temoto_2
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include/
  ${catkin_INCLUDE_DIRS}
  ${TEMOTO_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)
set_target_properties(${PROJECT_NAME}
  PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib/
)
target_link_libraries(${PROJECT_NAME} ${class_loader_LIBRARIES})
class_loader_hide_library_symbols(${PROJECT_NAME})
