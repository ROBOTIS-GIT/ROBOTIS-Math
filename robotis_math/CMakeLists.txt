cmake_minimum_required(VERSION 3.5)
project(robotis_math)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_compile_options(-fPIC)

add_library(${PROJECT_NAME}
   src/robotis_math_base.cpp
   src/robotis_trajectory_calculator.cpp
   src/minimum_jerk_trajectory.cpp
   src/minimum_jerk_trajectory_with_via_point.cpp
   src/fifth_order_polynomial_trajectory.cpp
   src/simple_trapezoidal_velocity_profile.cpp
   src/robotis_linear_algebra.cpp
   src/bezier_curve.cpp
   src/step_data_define.cpp
   src/preview_control.cpp
)
ament_target_dependencies(${PROJECT_NAME} rclcpp Eigen3)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp Eigen3)
ament_package()
