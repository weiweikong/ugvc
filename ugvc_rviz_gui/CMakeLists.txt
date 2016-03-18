cmake_minimum_required(VERSION 2.8.3)
project(ugvc_rviz_gui)

#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY /home/rdu/ugvc_ws/devel/lib/ugvc_rviz_gui)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rviz
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
)
## Configuration of Qt 
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)

include(${QT_USE_FILE})
add_definitions(-DQT_NO_KEYWORDS)
add_definitions(${QT_DEFINITIONS})


set(HEADER_FILES 
  src/ugvc_gui.h
)

set(SOURCE_FILES
  src/main.cpp
  src/ugvc_gui.cpp
  ${MOC_FILES}
)

set(FORM_FILES 
  src/ugvc_gui.ui
)

QT4_WRAP_CPP(MOC_SRC_H
  ${HEADER_FILES}
)

QT4_WRAP_UI(MOC_GUI_H 
  ${FORM_FILES}
)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ugvc_rviz_gui
#  CATKIN_DEPENDS roscpp rviz sensor_msgs std_msgs
#  DEPENDS system_lib
)

###################################
## catkin specific configuration ##
###################################



###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(ugvc_rviz_gui
#   src/${PROJECT_NAME}/ugvc_rviz_gui.cpp
# )

add_executable(ugvc_rviz_gui ${SOURCE_FILES} ${MOC_SRC_H} ${MOC_GUI_H})
target_link_libraries(ugvc_rviz_gui ${QT_LIBRARIES} ${catkin_LIBRARIES})


## Declare a cpp executable
# add_executable(ugvc_rviz_gui_node src/ugvc_rviz_gui_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(ugvc_rviz_gui_node ugvc_rviz_gui_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(ugvc_rviz_gui_node
#   ${catkin_LIBRARIES}
# )




## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)




## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   sensor_msgs#   std_msgs
# )


