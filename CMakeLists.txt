cmake_minimum_required(VERSION 2.8.3)
project(mc_pos_control)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  mavros_msgs
  mavros
)

find_package(cmake_modules)
find_package(Eigen3 REQUIRED)
find_package(eigen_conversions REQUIRED)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

# include headers
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${mavros_INCLUDE_DIRS}
  include/mc_pos_control
)

# build shared library
add_library(PositionControl
  src/PositionControl.cpp
  src/FlightTasks.cpp
  src/FlightTaskManual.cpp
  src/FlightTaskManualAltitude.cpp
  src/FlightTaskManualPosition.cpp
  src/BlockLowPass.cpp
  src/BlockDerivative.cpp
)

set(RELEVANT_LIBRARIES
  PositionControl
  ${catkin_LIBRARIES}
  ${mavros_LIBRARIES}
)

# rigorous error checking
add_definitions("-Wall -Werror -O3 -std=c++11")

# build main filter
add_executable(${PROJECT_NAME}
  src/mc_pos_control.cpp
  src/Node.cpp
)
target_link_libraries(${PROJECT_NAME} ${RELEVANT_LIBRARIES} ${eigen_conversions_LIBRARIES})

# ensures messages are generated before hand
add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)