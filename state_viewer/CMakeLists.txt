cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

# - Try to find Graphviz
# Once done this will define
#
#  GRAPHVIZ_FOUND - system has Graphviz
#  GRAPHVIZ_INCLUDE_DIR - the Graphviz include directory
#  GRAPHVIZ_LIBRARY - Link these to use Graphviz
#  GRAPHVIZ_DEFINITIONS - Compiler switches required for using Graphviz
# Copyright (c) 2009, Adrien Bustany, <madcat@mymadcat.com>
#
# Redistribution and use is allowed according to the terms of the GPLv3+ license.
IF (GRAPHVIZ_INCLUDE_DIR AND GRAPHVIZ_CDT_LIBRARY AND GRAPHVIZ_CGRAPH_LIBRARY AND GRAPHVIZ_GRAPH_LIBRARY AND GRAPHVIZ_PATHPLAN_LIBRARY)
    SET(GRAPHVIZ_FIND_QUIETLY TRUE)
ENDIF (GRAPHVIZ_INCLUDE_DIR AND GRAPHVIZ_CDT_LIBRARY AND GRAPHVIZ_CGRAPH_LIBRARY AND GRAPHVIZ_GRAPH_LIBRARY AND GRAPHVIZ_PATHPLAN_LIBRARY)
FIND_PATH( GRAPHVIZ_INCLUDE_DIR graphviz/graph.h)
FIND_LIBRARY( GRAPHVIZ_CDT_LIBRARY NAMES cdt )
FIND_LIBRARY( GRAPHVIZ_GVC_LIBRARY NAMES gvc )
FIND_LIBRARY( GRAPHVIZ_CGRAPH_LIBRARY NAMES cgraph )
FIND_LIBRARY( GRAPHVIZ_GRAPH_LIBRARY NAMES graph )
FIND_LIBRARY( GRAPHVIZ_PATHPLAN_LIBRARY NAMES pathplan )
IF (GRAPHVIZ_INCLUDE_DIR AND GRAPHVIZ_CDT_LIBRARY AND GRAPHVIZ_GVC_LIBRARY AND GRAPHVIZ_CGRAPH_LIBRARY AND GRAPHVIZ_GRAPH_LIBRARY AND GRAPHVIZ_PATHPLAN_LIBRARY)
   SET(GRAPHVIZ_FOUND TRUE)
ELSE (GRAPHVIZ_INCLUDE_DIR AND GRAPHVIZ_CDT_LIBRARY AND GRAPHVIZ_GVC_LIBRARY AND GRAPHVIZ_CGRAPH_LIBRARY AND GRAPHVIZ_GRAPH_LIBRARY AND GRAPHVIZ_PATHPLAN_LIBRARY)
   SET(GRAPHVIZ_FOUND FALSE)
ENDIF (GRAPHVIZ_INCLUDE_DIR AND GRAPHVIZ_CDT_LIBRARY AND GRAPHVIZ_GVC_LIBRARY AND GRAPHVIZ_CGRAPH_LIBRARY AND GRAPHVIZ_GRAPH_LIBRARY AND GRAPHVIZ_PATHPLAN_LIBRARY)
IF (GRAPHVIZ_FOUND)
  IF (NOT GRAPHVIZ_FIND_QUIETLY)
    MESSAGE(STATUS "Found Graphviz: ${GRAPHVIZ_CDT_LIBRARY} ${GRAPHVIZ_GVC_LIBRARY} ${GRAPHVIZ_CGRAPH_LIBRARY} ${GRAPHVIZ_GRAPH_LIBRARY} ${GRAPHVIZ_PATHPLAN_LIBRARY}")
  ENDIF (NOT GRAPHVIZ_FIND_QUIETLY)
ELSE (GRAPHVIZ_FOUND)
  IF (GRAPHVIZ_FIND_REQUIRED)
    MESSAGE(FATAL_ERROR "Could NOT find Graphivz")
  ENDIF (GRAPHVIZ_FIND_REQUIRED)
ENDIF (GRAPHVIZ_FOUND)

include_directories(GRAPHVIZ_INCLUDE_DIR)

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(state_viewer src/state_viewer.cpp)
#rosbuild_add_executable(rdf_graphviz src/rdf_graphviz.cpp)
#target_link_libraries(rdf_graphviz ${GRAPHVIZ_GVC_LIBRARY} ${GRAPHVIZ_GRAPH_LIBRARY})
target_link_libraries(state_viewer gvc graph)

