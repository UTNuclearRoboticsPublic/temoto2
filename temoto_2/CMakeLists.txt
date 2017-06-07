cmake_minimum_required(VERSION 2.8.3)
project(temoto_2)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  class_loader
  cmake_modules
)

find_package(TinyXML REQUIRED)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES temoto_2
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib TinyXML
)

include_directories(SYSTEM ${Boost_INCLUDE_DIR})

include_directories(
  include/
  /tmp/usr/local/include/
  ${catkin_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS}
)

add_executable(context_manager src/context_manager.cpp)
target_link_libraries(context_manager
   ${catkin_LIBRARIES}
)

add_executable(agent_core src/agent_core.cpp 
			  src/language_processor/language_processor.cpp
			  src/task_handler/task_handler.cpp
			  src/task_handler/description_processor.cpp
)

target_link_libraries(agent_core
   ${catkin_LIBRARIES}
   ${class_loader_LIBRARIES}
   ${TinyXML_LIBRARIES}
)

# testing tools
add_executable(language_input src/tools/language_input.cpp)
target_link_libraries(language_input
   ${catkin_LIBRARIES}
)

add_executable(blinker_publisher src/tools/blinker_publisher.cpp)
target_link_libraries(blinker_publisher
   ${catkin_LIBRARIES}
)

link_directories(/tmp/usr/local/lib/)
add_executable(blinker src/tools/blinker.cpp)
target_link_libraries(blinker
   ${catkin_LIBRARIES}
   serial
)
