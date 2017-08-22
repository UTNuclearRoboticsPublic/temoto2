cmake_minimum_required(VERSION 2.8.3)
project(temoto_2)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  class_loader
  cmake_modules
	message_generation
)

find_package(TinyXML REQUIRED)

add_message_files(FILES 
	context_manager/human_context/speechSpecifier.msg
	context_manager/human_context/gestureSpecifier.msg
  error_msgs/BaseError.msg
  error_msgs/ErrorStack.msg
)

add_service_files(
  FILES
  nodeSpawnKill.srv
	listDevices.srv
	startSensorRequest.srv
	stopSensorRequest.srv
	stopAllocatedServices.srv
	task_handler/stopTask.srv
	task_handler/indexTasks.srv
	context_manager/human_context/getGestures.srv
  context_manager/human_context/getSpeech.srv
  output_manager/loadPlugin.srv
  output_manager/showInRviz.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES temoto_2
CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib TinyXML
)

include_directories(SYSTEM ${Boost_INCLUDE_DIR})

include_directories(
  include/
  /tmp/usr/local/include/
  ${catkin_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS}
)

# # # # # # # # # # # #
# CONTEXT MANAGER
# # # # # # # # # # # #
add_executable(context_manager src/context_manager/context_manager.cpp
				src/context_manager/human_context/human_context.cpp)

target_link_libraries(context_manager
   ${catkin_LIBRARIES}
)

# # # # # # # # # # # #
# SENSOR MANAGER
# # # # # # # # # # # #
add_executable(sensor_manager src/sensor_manager/sensor_manager.cpp
															src/package_info/package_info.cpp)
target_link_libraries(sensor_manager
   ${catkin_LIBRARIES}
)

# # # # # # # # # # # #
# OUTPUT MANAGER
# # # # # # # # # # # #
add_executable(output_manager src/output_manager/output_manager.cpp
															src/output_manager/rviz_manager/rviz_manager.cpp
															src/output_manager/rviz_manager/plugin_info.cpp
                              src/base_error/base_error.cpp )
target_link_libraries(output_manager
   ${catkin_LIBRARIES})
add_dependencies(output_manager temoto_2_generate_messages_cpp)

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
target_link_libraries(agent_core
   ${catkin_LIBRARIES}
   ${class_loader_LIBRARIES}
   ${TinyXML_LIBRARIES} )
add_dependencies(agent_core temoto_2_generate_messages_cpp)


###################
#    testing tools
###################

add_executable(language_input src/tools/language_input.cpp)
target_link_libraries(language_input
   ${catkin_LIBRARIES}
)

add_executable(blinker_publisher src/tools/blinker_publisher.cpp)
target_link_libraries(blinker_publisher
   ${catkin_LIBRARIES}
)

add_executable(node_manager src/tools/node_manager.cpp)
target_link_libraries(node_manager
   ${catkin_LIBRARIES}
)

add_executable(dummy_sensor src/tools/dummy_sensor.cpp)
target_link_libraries(dummy_sensor
   ${catkin_LIBRARIES}
)

#link_directories(/tmp/usr/local/lib/)
#add_executable(blinker src/tools/blinker.cpp)
#target_link_libraries(blinker
#   ${catkin_LIBRARIES}
#   serial
#)
