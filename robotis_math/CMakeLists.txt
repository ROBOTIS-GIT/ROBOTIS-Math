cmake_minimum_required(VERSION 2.8.3)
project(robotis_math)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cmake_modules
)

find_package(Eigen REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES robotis_math
  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_library(robotis_math
   src/robotis_math_base.cpp
   src/robotis_trajectory_calculator.cpp
   src/robotis_linear_algebra.cpp
)

add_dependencies(robotis_math ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(robotis_math
   ${catkin_LIBRARIES}
)
