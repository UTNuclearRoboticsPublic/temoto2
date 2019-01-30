cmake_minimum_required(VERSION 2.8.3)
project(temoto_context_manager)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++1y -Wno-reorder -Wno-unused-function)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  message_generation
  geometry_msgs
  shape_msgs
  human_msgs
  ar_track_alvar_msgs
  visualization_msgs
  diagnostic_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  temoto_component_manager
  temoto_core
  temoto_nlp
)

add_message_files(FILES 
  SpeechSpecifier.msg
  ObjectContainer.msg
)

add_service_files(
  FILES
  GetNumber.srv
  LoadTracker.srv
  LoadSpeech.srv
  AddObjects.srv
  TrackObject.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  shape_msgs
  visualization_msgs
  diagnostic_msgs
  temoto_core
  temoto_nlp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES yaml-cpp
  CATKIN_DEPENDS roscpp roslib temoto_component_manager temoto_core temoto_er_manager temoto_nlp
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(temoto_context_manager 
  src/context_manager_node.cpp
  src/context_manager.cpp
  src/context_manager_containers.cpp
)

add_dependencies(temoto_context_manager
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
target_link_libraries(temoto_context_manager
  ${catkin_LIBRARIES}
  yaml-cpp
  temoto_nlp_components
  temoto_core_components
)
