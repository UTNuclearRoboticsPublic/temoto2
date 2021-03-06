cmake_minimum_required(VERSION 2.8.12)
project(temoto_action_assistant)

add_compile_options(-std=c++11)

# definition needed for boost/math/constants/constants.hpp included by Ogre to compile
add_definitions(-DBOOST_MATH_DISABLE_FLOAT128)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

include_directories(include)

find_package(Boost REQUIRED COMPONENTS thread filesystem system program_options)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

find_package(catkin REQUIRED COMPONENTS
  rosconsole
  roscpp
  roslib
  std_msgs
)
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

# Qt Stuff
find_package(Qt5 REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)


# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)
add_definitions(-DQT_NO_KEYWORDS)

# Support new yaml-cpp API.
find_package(PkgConfig)
pkg_check_modules(YAMLCPP REQUIRED yaml-cpp>=0.5)
include_directories(${YAMLCPP_INCLUDE_DIRS})
link_directories(${YAMLCPP_LIBRARY_DIRS})

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    #${PROJECT_NAME}_tools
  CATKIN_DEPENDS
)

# Header files that need Qt Moc pre-processing for use with Qt signals, etc:
set(HEADERS
  src/widgets/navigation_widget.h
  src/widgets/header_widget.h
  src/widgets/action_assistant_widget.h
  src/widgets/start_screen_widget.h
  src/widgets/sf_editor_widget.h
  src/widgets/interface_edit_widget.h
  src/widgets/object_edit_widget.h
  src/widgets/data_instance_edit_widget.h
  src/widgets/generate_package_widget.h
  #src/widgets/configuration_files_widget.h
  src/widgets/setup_screen_widget.h
  #src/widgets/author_information_widget.h
)

# Tools Library
#add_library(${PROJECT_NAME}_tools
#  src/tools/compute_default_collisions.cpp
#  src/tools/moveit_config_data.cpp
#  src/tools/collision_linear_model.cpp
#  src/tools/collision_matrix_model.cpp
#  src/tools/rotated_header_view.cpp
#)
#set_target_properties(${PROJECT_NAME}_tools PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
#target_link_libraries(${PROJECT_NAME}_tools
#  ${YAMLCPP_LIBRARIES}
#  ${catkin_LIBRARIES}
#  ${Boost_LIBRARIES}
#  ${QT_LIBRARIES}
#)

# Main Widgets Library - all screens (navigation options)
add_library(${PROJECT_NAME}_widgets
  src/widgets/start_screen_widget.cpp
  src/widgets/sf_editor_widget.cpp
  src/widgets/interface_edit_widget.cpp
  src/widgets/object_edit_widget.cpp
  src/widgets/data_instance_edit_widget.cpp
  src/widgets/generate_package_widget.cpp
  #src/widgets/configuration_files_widget.cpp
  src/widgets/navigation_widget.cpp
  src/widgets/header_widget.cpp
  src/widgets/action_assistant_widget.cpp
  src/widgets/setup_screen_widget.cpp
  #src/widgets/author_information_widget.cpp
  ${HEADERS}
)
set_target_properties(${PROJECT_NAME}_widgets PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
target_link_libraries(${PROJECT_NAME}_widgets
 # ${PROJECT_NAME}_tools
  ${QT_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

# executables
add_executable(${PROJECT_NAME} src/setup_assistant_main.cpp)
target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_widgets 
  #${PROJECT_NAME}_tools
  ${QT_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES} log4cxx yaml-cpp)

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_widgets 
  #${PROJECT_NAME}_tools
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION include)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY resources DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY templates DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
