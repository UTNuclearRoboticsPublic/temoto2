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
  diagnostic_msgs
  class_loader
  cmake_modules
	message_generation
  rviz_plugin_manager
  moveit_ros_planning_interface
  leap_motion_controller
)

find_package(TinyXML REQUIRED)
find_package(TBB REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(MeTA REQUIRED)

# THE FOLLOWING FILES CANNOT BE PLACED UNDER SUBDIRS!
add_message_files(FILES 
  Error.msg
  ErrorStack.msg
  StopTaskMsg.msg

  # Resource Management
  RMPRequest.msg
  RMPResponse.msg

  # Context Manager 
  SpeechSpecifier.msg
  GestureSpecifier.msg
  ObjectContainer.msg

  # Configuration Synchronizer
  ConfigSync.msg

  # Task manager
  TTP/task_manager/IndexTasks.msg
)

add_service_files(
  FILES
  # Resouce Management
  RMP/UnloadResource.srv
  RMP/ResourceStatus.srv

  # Process Manager
  process_manager/LoadProcess.srv

  # Sensor Manager
  sensor_manager/ListDevices.srv
  sensor_manager/LoadSensor.srv

  # Algorithm Manager
  algorithm_manager/LoadAlgorithm.srv

  # Context Manager
  context_manager/LoadTracker.srv
  context_manager/LoadGesture.srv
  context_manager/LoadSpeech.srv
  context_manager/AddObjects.srv
  context_manager/TrackObject.srv

  # Output Manager
  output_manager/rviz_manager/LoadRvizPlugin.srv

  # Robot Manager
  robot_manager/RobotLoad.srv
  robot_manager/RobotPlan.srv
  robot_manager/RobotExecute.srv
  robot_manager/RobotSetTarget.srv
  robot_manager/RobotSetMode.srv
  robot_manager/RobotGetVizInfo.srv

  # Task manager
  TTP/task_manager/StopTask.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  shape_msgs
  diagnostic_msgs
  temoto_2
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES yaml-cpp
  # LIBRARIES temoto_2
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs message_runtime human_msgs diagnostic_msgs
  DEPENDS 
    TinyXML
    EIGEN3
)

include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
include_directories(
  include/
  /tmp/usr/local/include/
  ${catkin_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS}
  ${TBB_INCLUDE_DIRS}
)

# # # # # # # # # # # #   TEMOTO SYSTEM NODES   # # # # # # # # # # # # # # #

# # # # # # # # # # # #
# CONTEXT MANAGER
# # # # # # # # # # # # 
add_executable(context_manager src/context_manager/context_manager_node.cpp
				                       src/context_manager/context_manager.cpp
                               src/context_manager/context_manager_containers.cpp
                               src/temoto_error/temoto_error.cpp)

add_dependencies(context_manager ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(context_manager ${catkin_LIBRARIES} yaml-cpp ttp)

# # # # # # # # # # # #
# SENSOR MANAGER
# # # # # # # # # # # #
add_executable(sensor_manager src/sensor_manager/sensor_manager.cpp 
	                            src/sensor_manager/sensor_manager_node.cpp
	                            src/sensor_manager/sensor_info.cpp
                              src/temoto_error/temoto_error.cpp)
add_dependencies(sensor_manager ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(sensor_manager ${catkin_LIBRARIES} yaml-cpp )

# # # # # # # # # # # #
# ALGORITHM MANAGER
# # # # # # # # # # # #
add_executable(algorithm_manager src/algorithm_manager/algorithm_manager.cpp 
	                               src/algorithm_manager/algorithm_manager_node.cpp
	                               src/algorithm_manager/algorithm_info.cpp
                                 src/temoto_error/temoto_error.cpp)
add_dependencies(algorithm_manager ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(algorithm_manager ${catkin_LIBRARIES} yaml-cpp )

# # # # # # # # # # # #
# OUTPUT MANAGER
# # # # # # # # # # # #
add_executable(output_manager src/output_manager/output_manager.cpp
															src/output_manager/rviz_manager/rviz_manager.cpp
															src/output_manager/rviz_manager/plugin_info.cpp
                              src/temoto_error/temoto_error.cpp )
add_dependencies(output_manager ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(output_manager ${catkin_LIBRARIES})

# # # # # # # # # # # #
# TTP
# # # # # # # # # # # #
add_library(ttp STATIC src/TTP/language_processors/meta/meta_lp.cpp
                src/TTP/language_processors/meta/branch_finder.cpp
                src/TTP/task_manager.cpp
                src/TTP/task_descriptor_processor.cpp
                src/TTP/io_descriptor.cpp
                src/TTP/task_descriptor.cpp
                src/TTP/task_tree_node.cpp
                src/TTP/task_tree.cpp
                src/TTP/task_container.cpp
                src/temoto_error/temoto_error.cpp)
add_dependencies(ttp ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(ttp ${catkin_LIBRARIES}
                          ${TinyXML_LIBRARIES}
                          ${TBB_LIBRARIES}
                          meta-classify meta-sequence-analyzers meta-parser-analyzers )

# # # # # # # # # # # #
# TEMOTO AGENT
# # # # # # # # # # # #
add_executable(temoto_agent src/core/temoto_agent.cpp)
add_dependencies(temoto_agent ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(temoto_agent ttp)


# # # # # # # # # # # #
# PROCESS MANAGER
# # # # # # # # # # # #
add_executable(process_manager src/process_manager/process_manager.cpp
	                             src/process_manager/process_manager_node.cpp
                               src/temoto_error/temoto_error.cpp)
add_dependencies(process_manager ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(process_manager ${catkin_LIBRARIES} )


# # # # # # # # # # # #
# ROBOT MANAGER
# # # # # # # # # # # #
add_executable(robot_manager src/robot_manager/robot_manager.cpp
	                           src/robot_manager/robot_manager_node.cpp
                             src/robot_manager/robot.cpp
                             src/robot_manager/robot_config.cpp
                             src/robot_manager/robot_feature.cpp
                             src/common/reliability.cpp
                             src/temoto_error/temoto_error.cpp)

add_dependencies(robot_manager ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(robot_manager ${catkin_LIBRARIES} yaml-cpp)


# # # # # # # # # # # #
# TEST TOOLS
# # # # # # # # # # # #
add_executable(language_input src/tools/language_input.cpp
                              src/temoto_error/temoto_error.cpp)
add_dependencies(language_input ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(language_input ${catkin_LIBRARIES} )

add_executable(tracking_pipes_tests src/tools/tracking_pipes_tests.cpp)
add_dependencies(tracking_pipes_tests ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(tracking_pipes_tests ${catkin_LIBRARIES} yaml-cpp)

#add_executable(algorithm_info_tests src/tools/algorithm_info_tests.cpp
#                                    src/algorithm_manager/algorithm_info.cpp)
#add_dependencies(algorithm_info_tests ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
#target_link_libraries(algorithm_info_tests ${catkin_LIBRARIES} yaml-cpp)

#add_executable(blinker_publisher src/tools/blinker_publisher.cpp)
#add_dependencies(blinker_publisher ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
#target_link_libraries(blinker_publisher ${catkin_LIBRARIES} )

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
