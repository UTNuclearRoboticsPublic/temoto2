cmake_minimum_required(VERSION 2.8.3)
project(temoto_2)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  genmsg
  rospy
  std_msgs
  class_loader
  cmake_modules
	message_generation
  rviz_plugin_manager
)

find_package(TinyXML REQUIRED)
find_package(TBB REQUIRED)

add_message_files(FILES 
	speechSpecifier.msg
	gestureSpecifier.msg
  BaseError.msg
  ErrorStack.msg
  StopTaskMsg.msg
)

add_service_files(
  FILES
  nodeSpawnKill.srv
	listDevices.srv
	startSensorRequest.srv
	stopSensorRequest.srv
	stopAllocatedServices.srv
	resourceStatus.srv
	task_handler/stopTask.srv
	task_handler/indexTasks.srv
	context_manager/human_context/getGestures.srv
  context_manager/human_context/getSpeech.srv
  output_manager/ShowInRviz.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES temoto_2
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime human_msgs
  DEPENDS system_lib TinyXML
)

include_directories(SYSTEM ${Boost_INCLUDE_DIR})

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
add_executable(context_manager src/context_manager/context_manager.cpp
				src/context_manager/human_context/human_context.cpp)

add_dependencies(context_manager ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(context_manager ${catkin_LIBRARIES} )

# # # # # # # # # # # #
# SENSOR MANAGER
# # # # # # # # # # # #
add_executable(sensor_manager src/sensor_manager/sensor_manager.cpp
															src/package_info/package_info.cpp)
add_dependencies(sensor_manager ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(sensor_manager ${catkin_LIBRARIES} )

# # # # # # # # # # # #
# OUTPUT MANAGER
# # # # # # # # # # # #
add_executable(output_manager src/output_manager/output_manager.cpp
															src/output_manager/rviz_manager/rviz_manager.cpp
															src/output_manager/rviz_manager/plugin_info.cpp
                              src/base_error/base_error.cpp )
add_dependencies(output_manager ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(output_manager ${catkin_LIBRARIES})

# # # # # # # # # # # #
# CORE
# # # # # # # # # # # #
add_executable(agent_core src/core/agent_core.cpp 
			  src/core/language_processor/language_processor.cpp
			  src/core/task_handler/task_handler.cpp
			  src/core/task_handler/description_processor.cpp
			  src/core/task_handler/task_info.cpp
			  src/base_error/base_error.cpp
)
add_dependencies(agent_core ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(agent_core
   ${catkin_LIBRARIES}
   ${class_loader_LIBRARIES}
   ${TinyXML_LIBRARIES}
   ${TBB_LIBRARIES} )
add_dependencies(agent_core ${catkin_EXPORTED_TARGETS})

# # # # # # # # # # # #
# NODE MANAGER
# # # # # # # # # # # #
add_executable(node_manager src/node_manager/node_manager.cpp src/node_manager/node_manager_node.cpp )
add_dependencies(node_manager ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(node_manager ${catkin_LIBRARIES} )



# # # # # # # # # # # #   TESTING TOOLS   # # # # # # # # # # # # # # #


add_executable(language_input src/tools/language_input.cpp
                              src/base_error/base_error.cpp)
add_dependencies(language_input ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(language_input ${catkin_LIBRARIES} )

add_executable(blinker_publisher src/tools/blinker_publisher.cpp)
add_dependencies(blinker_publisher ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(blinker_publisher ${catkin_LIBRARIES} )

add_executable(dummy_sensor src/tools/dummy_sensor.cpp)
add_dependencies(dummy_sensor ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(dummy_sensor ${catkin_LIBRARIES} )

add_executable(broken_node_1 src/tools/broken_node_1.cpp)
add_dependencies(broken_node_1 ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(broken_node_1 ${catkin_LIBRARIES} )

#link_directories(/tmp/usr/local/lib/)
#add_executable(blinker src/tools/blinker.cpp)
#target_link_libraries(blinker
#   ${catkin_LIBRARIES}
#   serial
#)
