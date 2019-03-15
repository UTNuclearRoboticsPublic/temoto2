cmake_minimum_required(VERSION 2.8.3)
project(temoto_2)

# SET (CMAKE_C_COMPILER "/usr/bin/clang-4.0")
# SET (CMAKE_CXX_COMPILER "/usr/bin/clang++-4.0")
# add_compile_options(-std=c++11 -Wno-deprecated-declarations)
add_compile_options(-std=c++1y -Wno-reorder -Wno-unused-function)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  genmsg
  rospy
  std_msgs
  geometry_msgs
  shape_msgs
  human_msgs
  ar_track_alvar_msgs
  visualization_msgs
  diagnostic_msgs
  cmake_modules
	message_generation
  file_template_parser
  temoto_action_assistant
  moveit_ros_planning_interface
  tf2
  tf2_ros
  tf2_geometry_msgs
  temoto_core
  temoto_nlp
  temoto_context_manager
  temoto_er_manager
  temoto_output_manager
  roslib
)

find_package(TinyXML REQUIRED)
find_package(Eigen3 REQUIRED)

# THE FOLLOWING FILES CANNOT BE PLACED UNDER SUBDIRS!

add_service_files(
  FILES

  # Sensor Manager
  sensor_manager/ListDevices.srv
  sensor_manager/LoadSensor.srv

  # Algorithm Manager
  algorithm_manager/LoadAlgorithm.srv

  # Robot Manager
  robot_manager/RobotLoad.srv
  robot_manager/RobotPlan.srv
  robot_manager/RobotExecute.srv
  robot_manager/RobotSetTarget.srv
  robot_manager/RobotSetMode.srv
  robot_manager/RobotGetVizInfo.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  shape_msgs
  visualization_msgs
  diagnostic_msgs
  temoto_2
  temoto_core
  temoto_nlp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES yaml-cpp ft_parser
  # LIBRARIES temoto_2
  CATKIN_DEPENDS 
    roscpp
    rospy 
    std_msgs 
    geometry_msgs
    message_runtime
    human_msgs
    diagnostic_msgs
    shape_msgs
    visualization_msgs
    temoto_core
    temoto_nlp
    temoto_context_manager
    temoto_er_manager
  DEPENDS 
    TinyXML
    EIGEN3
)

include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS}
)

# # # # # # # # # # # #   TEMOTO SYSTEM NODES   # # # # # # # # # # # # # # #

# # # # # # # # # # # #
# SENSOR MANAGER
# # # # # # # # # # # #
add_executable(sensor_manager 
  src/sensor_manager/sensor_manager_node.cpp
  src/sensor_manager/sensor_manager_servers.cpp
  src/sensor_manager/sensor_snooper.cpp
  src/sensor_manager/sensor_info_registry.cpp
  src/sensor_manager/sensor_info.cpp
)
add_dependencies(sensor_manager ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(sensor_manager ${catkin_LIBRARIES} yaml-cpp temoto_nlp_components temoto_core_components)

# # # # # # # # # # # #
# ALGORITHM MANAGER
# # # # # # # # # # # #
add_executable(algorithm_manager 
  src/algorithm_manager/algorithm_manager_node.cpp
  #src/algorithm_manager/algorithm_manager.cpp
  src/algorithm_manager/algorithm_manager_servers.cpp
  src/algorithm_manager/algorithm_snooper.cpp
  src/algorithm_manager/algorithm_info_registry.cpp 
  src/algorithm_manager/algorithm_info.cpp
)
add_dependencies(algorithm_manager ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(algorithm_manager ${catkin_LIBRARIES} yaml-cpp temoto_nlp_components temoto_core_components)

# # # # # # # # # # # #
# ROBOT MANAGER
# # # # # # # # # # # #
add_executable(robot_manager 
  src/robot_manager/robot_manager.cpp
  src/robot_manager/robot_manager_node.cpp
  src/robot_manager/robot.cpp
  src/robot_manager/robot_config.cpp
  src/robot_manager/robot_features.cpp
)

add_dependencies(robot_manager ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(robot_manager ${catkin_LIBRARIES} yaml-cpp temoto_core_components)


# # # # # # # # # # # #
# TEST TOOLS
# # # # # # # # # # # #
add_executable(language_input src/tools/language_input.cpp)
add_dependencies(language_input ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(language_input ${catkin_LIBRARIES} temoto_core_components)

#add_executable(tracking_pipes_tests src/tools/tracking_pipes_tests.cpp)
#add_dependencies(tracking_pipes_tests ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
#target_link_libraries(tracking_pipes_tests ${catkin_LIBRARIES} yaml-cpp)

#add_executable(algorithm_info_tests src/tools/algorithm_info_tests.cpp
#                                    src/algorithm_manager/algorithm_info.cpp)
#add_dependencies(algorithm_info_tests ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
#target_link_libraries(algorithm_info_tests ${catkin_LIBRARIES} yaml-cpp)

#add_executable(dummy_sensor src/tools/dummy_sensor.cpp)
#add_dependencies(dummy_sensor ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
#target_link_libraries(dummy_sensor ${catkin_LIBRARIES} )

#add_executable(broken_node_1 src/tools/broken_node_1.cpp)
#add_dependencies(broken_node_1 ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
#target_link_libraries(broken_node_1 ${catkin_LIBRARIES} )

#add_executable(pos_tagger src/tools/pos_tagger.cpp)
#add_dependencies(pos_tagger ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
#target_link_libraries(pos_tagger ${catkin_LIBRARIES} meta-classify meta-sequence-analyzers meta-parser-analyzers)

#link_directories(/tmp/usr/local/lib/)
#add_executable(blinker src/tools/blinker.cpp)
#target_link_libraries(blinker
#   ${catkin_LIBRARIES}
#   serial
#)
