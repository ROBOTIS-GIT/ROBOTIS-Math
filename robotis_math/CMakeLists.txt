################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(robotis_math)

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
)

find_package(Eigen3 REQUIRED)

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Catkin specific configuration
################################################################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES robotis_math
  CATKIN_DEPENDS roscpp
  DEPENDS EIGEN3
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(robotis_math
   src/robotis_math_base.cpp
   src/robotis_trajectory_calculator.cpp
   src/fifth_order_polynomial_trajectory.cpp
   src/simple_trapezoidal_velocity_profile.cpp
   src/robotis_linear_algebra.cpp
   src/bezier_curve.cpp
   src/step_data_define.cpp
)

add_dependencies(robotis_math ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(robotis_math
   ${catkin_LIBRARIES}
)

################################################################################
# Install
################################################################################
install(TARGETS robotis_math
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

################################################################################
# Test
################################################################################
