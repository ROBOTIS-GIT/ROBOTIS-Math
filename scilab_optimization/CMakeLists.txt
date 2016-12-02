cmake_minimum_required(VERSION 2.8.3)
project(scilab_optimization)

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
)

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
  LIBRARIES scilab_optimization
  CATKIN_DEPENDS roscpp
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  /usr/lib/scilab
)

add_library(scilab_optimization
   src/scilab_optimization.cpp
)

add_dependencies(scilab_optimization ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(scilab_optimization
  /usr/lib/scilab/libsciui_data.so
  /usr/lib/scilab/libsciumfpack.so
  /usr/lib/scilab/libsciscicos-cli.so
  /usr/lib/scilab/libscihdf5.so
  /usr/lib/scilab/libsciaction_binding.so
  /usr/lib/scilab/libsciscicos.so
  /usr/lib/scilab/libscigui-disable.so
  /usr/lib/scilab/libsciui_data-disable.so
  #/usr/lib/scilab/libsciconsole-minimal.so
  /usr/lib/scilab/libscihistory_browser.so
  /usr/lib/scilab/libsciexternal_objects_java.so
  /usr/lib/scilab/libscigraphic_export-disable.so
  #/usr/lib/scilab/libscigraphic_objects-disable.so
  /usr/lib/scilab/libscicall_scilab.so
  /usr/lib/scilab/libscigui.so
  /usr/lib/scilab/libsciscicos_blocks.so
  /usr/lib/scilab/libsciinterpolation.so
  /usr/lib/scilab/libscigraphics-disable.so
  /usr/lib/scilab/libscihistory_manager.so
  /usr/lib/scilab/libsciexternal_objects.so
  /usr/lib/scilab/libscigraphics.so
  /usr/lib/scilab/libsciparallel.so
  /usr/lib/scilab/libscispecial_functions.so
  #/usr/lib/scilab/libscigraphic_objects.so
  /usr/lib/scilab/libscixml.so
  /usr/lib/scilab/libscisignal_processing.so
  /usr/lib/scilab/libscicommons.so
  /usr/lib/scilab/libscifunctions.so
  /usr/lib/scilab/libscirenderer.so
  /usr/lib/scilab/libscipreferences-cli.so
  /usr/lib/scilab/libscilab-cli.so
  /usr/lib/scilab/libscijvm.so
  /usr/lib/scilab/libscimatio.so
  /usr/lib/scilab/libscisymbolic.so
  /usr/lib/scilab/libscispreadsheet.so
  #/usr/lib/scilab/libsciconsole.so
  /usr/lib/scilab/libscitypes.so
  /usr/lib/scilab/libscilocalization.so
  /usr/lib/scilab/libscirandlib.so
  /usr/lib/scilab/libscicommons-disable.so
  /usr/lib/scilab/libsciarnoldi.so
  /usr/lib/scilab/libsciscinotes-disable.so
  /usr/lib/scilab/libscistatistics.so
  /usr/lib/scilab/libscilab.so
  /usr/lib/scilab/libscitclsci.so
  /usr/lib/scilab/libscixcos-disable.so
  /usr/lib/scilab/libsciaction_binding-disable.so
  /usr/lib/scilab/libscijvm-disable.so
  /usr/lib/scilab/libscisound.so
  /usr/lib/scilab/libscipreferences.so
  /usr/lib/scilab/libscioptimization.so
  /usr/lib/scilab/libscigraphic_export.so
  /usr/lib/scilab/libscicompletion.so
  /usr/lib/scilab/libscisundials.so
  /usr/lib/scilab/libsciscinotes.so
  /usr/lib/scilab/libsciscicos_blocks-cli.so
  /usr/lib/scilab/libscihistory_browser-disable.so
  /usr/lib/scilab/libscihelptools.so
  /usr/lib/scilab/libscixcos.so
   ${catkin_LIBRARIES}
)

################################################################################
# Install
################################################################################
install(TARGETS scilab_optimization
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
