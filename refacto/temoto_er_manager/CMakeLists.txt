cmake_minimum_required(VERSION 2.8.3)
project(temoto_er_manager)
add_compile_options(-std=c++1y)

find_package(catkin REQUIRED
  roscpp
  genmsg
  rospy
  std_msgs
  temoto_2
)

add_service_files(
  FILES
  LoadExtResource.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  temoto_2
)

catkin_package(
  INCLUDE_DIRS include
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(temoto_er_manager src/temoto_er_manager.cpp
	                               src/temoto_er_manager_node.cpp
)
add_dependencies(temoto_er_manager ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries( temoto_er_manager 
  ${catkin_LIBRARIES}
  temoto_core
)