cmake_minimum_required(VERSION 2.8.3)
project(temoto_sensor_manager)

add_compile_options(-std=c++1y -Wno-reorder -Wno-unused-function)

find_package(catkin REQUIRED COMPONENTS
  temoto_2
  roscpp
  genmsg
  std_msgs
  diagnostic_msgs
	message_generation
)

add_service_files(FILES
  ListDevices.srv
  LoadSensor.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  diagnostic_msgs
  temoto_2
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES yaml-cpp
  CATKIN_DEPENDS roscpp std_msgs diagnostic_msgs
  DEPENDS 
    TinyXML
)

include_directories(SYSTEM ${Boost_INCLUDE_DIR})
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(temoto_sensor_manager 
  src/sensor_manager_node.cpp
  src/sensor_manager_servers.cpp
  src/sensor_snooper.cpp
  src/sensor_info_registry.cpp
  src/sensor_info.cpp
)
add_dependencies(temoto_sensor_manager
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
target_link_libraries(temoto_sensor_manager
  ${catkin_LIBRARIES} 
  yaml-cpp 
  ttp
  temoto_core
)

