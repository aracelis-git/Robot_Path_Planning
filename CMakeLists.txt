cmake_minimum_required(VERSION 2.8.3)
project(robot_path_planning)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  turtlebot_gazebo
  gazebo_ros
  tf
)

find_package(gazebo REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


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
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  DEPENDS 
  gazebo_ros
  INCLUDE_DIRS include
#  LIBRARIES robot_path_planning
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs sensor_msgs gazebo_ros message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(robot_path_planning
#   src/${PROJECT_NAME}/robot_path_planning.cpp
# )

add_library(envlib
  src/Env/tworooms.cc
  src/Env/taxi.cc
  src/Env/MountainCar.cc
  src/Env/FuelRooms.cc
  src/Env/CartPole.cc
  src/Env/fourrooms.cc
  src/Env/RobotCarVel.cc
  src/Env/energyrooms.cc
  src/Env/gridworld.cc
  src/Env/stocks.cc
  src/Env/LightWorld.cc
  src/corridor.cpp
)

add_library(agentlib
  src/Agent/DiscretizationAgent.cc
  src/Agent/QLearner.cc
  src/Agent/ModelBasedAgent.cc
  src/Agent/SavedPolicy.cc
  src/Agent/Dyna.cc
  src/Agent/Sarsa.cc
  src/Models/FactoredModel.cc
  src/Models/M5Tree.cc
  src/Models/LinearSplitsTree.cc
  src/Models/C45Tree.cc
  src/Models/Stump.cc
  src/Models/MultipleClassifiers.cc
  src/Models/ExplorationModel.cc
  src/Models/RMaxModel.cc
  src/Models/SepPlanExplore.cc
  src/Planners/ValueIteration.cc
  src/Planners/PolicyIteration.cc
  src/Planners/PrioritizedSweeping.cc
  src/Planners/ETUCT.cc
  src/Planners/PO_ParallelETUCT.cc
  src/Planners/ParallelETUCT.cc
  src/Planners/PO_ETUCT.cc
  src/Planners/MBS.cc
  src/newmat/newmat1.cc
  src/newmat/newmat2.cc
  src/newmat/newmat3.cc
  src/newmat/newmat4.cc
  src/newmat/newmat5.cc
  src/newmat/newmat6.cc
  src/newmat/newmat7.cc
  src/newmat/newmat8.cc
  src/newmat/newmatex.cc
  src/newmat/bandmat.cc
  src/newmat/submat.cc
  src/newmat/myexcept.cc
  src/newmat/cholesky.cc
  src/newmat/evalue.cc
  src/newmat/fft.cc
  src/newmat/hholder.cc
  src/newmat/jacobi.cc
  src/newmat/newfft.cc
  src/newmat/sort.cc
  src/newmat/svd.cc
  src/newmat/newmatrm.cc
  src/newmat/newmat9.cc
)

## Declare a cpp executable
add_executable(algorithm src/algorithm.cpp)
target_link_libraries(algorithm ${catkin_LIBRARIES})
add_executable(env src/env.cpp)
target_link_libraries(env envlib ${catkin_LIBRARIES})
add_executable(agent src/agent.cpp)
target_link_libraries(agent agentlib ${catkin_LIBRARIES})

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(robot_path_planning_node robot_path_planning_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(robot_path_planning_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS robot_path_planning robot_path_planning_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

find_package(GTest REQUIRED)
include_directories(
  include/${robot_path_planning}
  ${catkin_INCLUDE_DIRS}
  ${GTEST_INCLUDE_DIRS} 
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(utest test/utest.cpp)
  target_link_libraries(utest envlib ${catkin_LIBRARIES})
endif()

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  


#  add_rostest_gtest (
#  atest
#  test/utest.test
#  test/utest.cpp
#  )
#  target_link_libraries (
#  atest
#  #src/algorithm.cpp
#  ${catkin_LIBRARIES}
#  )
#  add_dependencies(
#  atest
#  algorithm
#  envlib
#  ${PROJECT_NAME}_generate_messages_cpp
#  ${catkin_EXPORTED_TARGETS}
#  )
endif()

