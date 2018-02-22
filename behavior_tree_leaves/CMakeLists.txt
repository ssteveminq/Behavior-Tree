cmake_minimum_required(VERSION 2.8.3)
project(behavior_tree_leaves)

#SET(SOURCES example_nodes)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  genmsg
  actionlib_msgs
  actionlib
  message_generation
  message_runtime
  roscpp
  roslib
  rospy
  std_msgs
  std_srvs
  geometry_msgs
  behavior_tree_core
  roslaunch
  gaze_service
  navi_service
)
roslaunch_add_file_check(launch)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
# System dependencies are found with CMake's conventions
#find_package(Boost REQUIRED COMPONENTS system thread program_options)


# Settings
add_definitions(-Wall -g -O0 -Wno-deprecated -static -Bstatic -std=gnu++0x)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

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
#add_action_files(
#  DIRECTORY action
#  FILES BT.action
#)


# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here


#generate_messages(
#  DEPENDENCIES actionlib_msgs std_msgs behavior_tree_core# Or other packages containing msgs
#)


# generate_messages(
#   DEPENDENCIES
#   actionlib_msgs#   std_msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES learning_actionlib
#  CATKIN_DEPENDS actionlib actionlib_msgs message_generation roscpp rospy std_msgs
#  DEPENDS system_lib
#)

find_package(gaze_service REQUIRED)
catkin_package(
  CATKIN_DEPENDS actionlib_msgs
  roscpp
  geometry_msgs
  std_msgs
  std_srvs
  actionlib
  tf
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  src
  exmaple_nodes
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(learning_actionlib
#   src/${PROJECT_NAME}/learning_actionlib.cpp
# )

set(COMMON_LIB ${catkin_LIBRARIES} ${Boost_LIBRARIES})


add_executable(action_client src/action_client.cpp)
target_link_libraries(action_client ${catkin_LIBRARIES})
add_dependencies(
 action_client
  ${behavior_tree_leaves_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  behavior_tree_core_generate_messages_cpp
)

add_executable(open_action_client src/open_action_client.cpp)
target_link_libraries(open_action_client ${catkin_LIBRARIES})
add_dependencies(
 open_action_client
  ${behavior_tree_leaves_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  behavior_tree_core_generate_messages_cpp
)

add_executable(condition_client src/condition_client.cpp)
target_link_libraries(condition_client  ${catkin_LIBRARIES})
add_dependencies(
 condition_client
  ${behavior_tree_leaves_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  behavior_tree_core_generate_messages_cpp

)

add_executable(finddoor_condition_client src/finddoor_condition_client.cpp)
target_link_libraries(finddoor_condition_client  ${catkin_LIBRARIES})
add_dependencies(
 finddoor_condition_client
  ${behavior_tree_leaves_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  behavior_tree_core_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
)

add_executable(action_example example_nodes/cpp/action_example.cpp)

target_link_libraries(
  action_example
  ${catkin_LIBRARIES}
)

add_dependencies(
 action_example
  ${behavior_tree_leaves_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  behavior_tree_core_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
)

add_executable(action_open example_nodes/cpp/action_open.cpp)

target_link_libraries(
  action_open
  ${catkin_LIBRARIES}
)

add_dependencies(
 action_open
  ${behavior_tree_leaves_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  behavior_tree_core_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
)

add_executable(action_lookperson example_nodes/cpp/action_lookperson.cpp)

target_link_libraries(
  action_lookperson
  ${catkin_LIBRARIES}
)

add_dependencies(
 action_lookperson
  ${behavior_tree_leaves_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  behavior_tree_core_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
)

add_executable(condition_example example_nodes/cpp/condition_example.cpp)

target_link_libraries(
  condition_example
  ${catkin_LIBRARIES}
)

add_dependencies(
 condition_example
  ${behavior_tree_leaves_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  behavior_tree_core_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
)

add_executable(condition_finddoor example_nodes/cpp/condition_finddoor.cpp)

target_link_libraries(
  condition_finddoor
  ${catkin_LIBRARIES}
)

add_dependencies(
 condition_finddoor
  ${behavior_tree_leaves_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  behavior_tree_core_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
)

add_executable(condition_findperson example_nodes/cpp/condition_findperson.cpp)
target_link_libraries(
  condition_findperson 
  ${catkin_LIBRARIES}
)

add_dependencies(
  condition_findperson
  ${behavior_tree_leaves_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  behavior_tree_core_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
)

add_executable(findperson_condition_client src/findperson_condition_client.cpp)
target_link_libraries(findperson_condition_client ${catkin_LIBRARIES})
add_dependencies(
 findperson_condition_client 
  ${behavior_tree_leaves_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  behavior_tree_core_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
)




