cmake_minimum_required(VERSION 3.0.2)
project(autonomy_simulator)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)   #  LSP needs this
set(CMAKE_CXX_STANDARD 17)    # for some c++ libs (like optional)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  std_msgs
  roscpp
)

add_service_files(
  FILES
  GetMap.srv
)

add_message_files(
  FILES
  RoverMap.msg
  RoverPose.msg
  SetGoal.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
  src/GoalSetter
  src/PathSolver
  src/Brain/include
)

add_executable(autonomy_simulator
  src/main.cpp
  src/autonomy_simulator.cpp
  src/map_generation.cpp
)

add_dependencies(autonomy_simulator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(map_generation src/map_generation.cpp)
add_library(GoalSetter src/GoalSetter/GoalSetter.cpp)
add_library(PathSolver src/PathSolver/PathSolver.cpp)
add_library(BrainLib
  src/Brain/SmallBrain.cpp
  src/Brain/BigBrain.cpp
  src/Brain/BiggerBrain.cpp
)

target_link_libraries(autonomy_simulator ${catkin_LIBRARIES} map_generation)

add_executable(goal_setter src/GoalSetter/gs_run.cpp)
target_link_libraries(goal_setter
  ${catkin_LIBRARIES}
  GoalSetter
)

add_executable(path_solver src/PathSolver/ps_run.cpp)
target_link_libraries(path_solver
  ${catkin_LIBRARIES}
  PathSolver
  BrainLib
)

add_subdirectory(test)
