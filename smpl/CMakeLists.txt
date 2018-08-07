cmake_minimum_required(VERSION 2.8.12)
project(smpl)

###################################
# overall build mode and features #
###################################

set(SMPL_CONSOLE_ROS ON CACHE BOOL "Replace smpl log macros with rosconsole macros")
set(SMPL_SV_VISUALIZATION_MSGS ON CACHE BOOL "Include visualization_msgs in visualizer interface")
set(SMPL_BUILD_SHARED ON CACHE BOOL "Build smpl as a shared library")
set(SMPL_BUILD_STATIC ON CACHE BOOL "Build smpl as a static library")

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

#######################
# version information #
#######################

set(SMPL_MAJOR_VERSION 0)
set(SMPL_MINOR_VERSION 0)
set(SMPL_PATCH_VERSION 0)
set(SMPL_VERSION ${SMPL_MAJOR_VERSION}.${SMPL_MINOR_VERSION}.${SMPL_PATCH_VERSION})

###########################################################
# allow overriding install directories for system install #
###########################################################

set(SMPL_INSTALL_LIB_DIR     lib             CACHE PATH "Installation directory for libraries")
set(SMPL_INSTALL_INCLUDE_DIR include         CACHE PATH "Installation directory for header files")
set(SMPL_INSTALL_CMAKE_DIR   lib/cmake/smpl  CACHE PATH "Installation directory for CMake files")
set(SMPL_INSTALL_BIN_DIR     bin             CACHE PATH "Installation directory for executables")
foreach(path LIB INCLUDE CMAKE BIN)
    set(var SMPL_INSTALL_${path}_DIR)
    if(NOT IS_ABSOLUTE "${${var}}")
        set(${var} "${CMAKE_INSTALL_PREFIX}/${${var}}")
    endif()
endforeach()

########################
# resolve dependencies #
########################

find_package(Boost REQUIRED COMPONENTS filesystem program_options system)
find_package(Eigen3 REQUIRED)
find_package(sbpl REQUIRED)
if(SMPL_CONSOLE_ROS)
    find_package(roscpp REQUIRED)
endif()
if(SMPL_SV_VISUALIZATION_MSGS)
    find_package(visualization_msgs REQUIRED)
endif()

####################################################
# generate our own .cmake files for system install #
####################################################

file(RELATIVE_PATH REL_INCLUDE_DIR "${SMPL_INSTALL_CMAKE_DIR}" "${SMPL_INSTALL_INCLUDE_DIR}")
set(CONF_INCLUDE_DIRS "\${SMPL_CMAKE_DIR}/${REL_INCLUDE_DIR}")

configure_file(smpl-config.cmake.in "${PROJECT_BINARY_DIR}/smpl-config.cmake" @ONLY)
configure_file(smpl-config-version.cmake.in "${PROJECT_BINARY_DIR}/smpl-config-version.cmake" @ONLY)

#################################
# generate configuration header #
#################################

set(SMPL_CLOCK_API SMPL_CLOCK_CHRONO_HIGH_RESOLUTION)

configure_file(
    ${PROJECT_SOURCE_DIR}/include/smpl/config.h.in
    ${CMAKE_CURRENT_BINARY_DIR}/include/smpl/config.h)

########################
# compile time options #
########################

if(NOT MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()

if(SMPL_CONSOLE_ROS)
    add_definitions(-DSMPL_CONSOLE_ROS)
endif()

add_definitions(-DSV_PACKAGE_NAME="smpl")

set(SMPL_LIBRARY_SOURCES
    src/csv_parser.cpp
    src/collision_checker.cpp
    src/console/ansi.cpp
    src/console/console.cpp
    src/occupancy_grid.cpp
    src/planning_params.cpp
    src/post_processing.cpp
    src/robot_model.cpp
    src/bfs3d/bfs3d.cpp
    src/debug/colors.cpp
    src/debug/marker_utils.cpp
    src/debug/visualize.cpp
    src/distance_map/chessboard_distance_map.cpp
    src/distance_map/distance_map_common.cpp
    src/distance_map/edge_euclid_distance_map.cpp
    src/distance_map/euclid_distance_map.cpp
    src/distance_map/sparse_distance_map.cpp
    src/geometry/bounding_spheres.cpp
    src/geometry/intersect.cpp
    src/geometry/mesh_utils.cpp
    src/geometry/voxelize.cpp
    src/graph/action_space.cpp
    src/graph/adaptive_workspace_lattice.cpp
    src/graph/experience_graph.cpp
    src/graph/manip_lattice.cpp
    src/graph/manip_lattice_egraph.cpp
    src/graph/manip_lattice_action_space.cpp
    src/graph/robot_planning_space.cpp
    src/graph/workspace_lattice.cpp
    src/graph/workspace_lattice_base.cpp
    src/graph/workspace_lattice_egraph.cpp
    src/graph/simple_workspace_lattice_action_space.cpp
    src/heuristic/attractor_heuristic.cpp
    src/heuristic/bfs_heuristic.cpp
    src/heuristic/egraph_bfs_heuristic.cpp
    src/heuristic/generic_egraph_heuristic.cpp
    src/heuristic/euclid_dist_heuristic.cpp
    src/heuristic/robot_heuristic.cpp
    src/heuristic/joint_dist_heuristic.cpp
    src/heuristic/multi_frame_bfs_heuristic.cpp
    src/heuristic/sparse_egraph_dijkstra_heuristic.cpp
    src/heuristic/zero_heuristic.cpp
    src/search/fmhastar.cpp
    src/search/meta_mhastar_dts.cpp
    src/search/mhastarpp.cpp
    src/search/umhastar.cpp
    src/search/arastar.cpp
    src/search/experience_graph_planner.cpp
    src/search/adaptive_planner.cpp
    src/search/lazy_arastar.cpp
    src/search/lazy_mhastar.cpp
    src/search/smhastar.cpp
    src/search/awastar.cpp
    src/unicycle/unicycle.cpp)

set(CMAKE_DEBUG_POSTFIX "_d")

# Gather include directories
list(APPEND PRIVATE_HEADERS ${CMAKE_CURRENT_BINARY_DIR}/include)
list(APPEND PRIVATE_HEADERS include)

if(SMPL_CONSOLE_ROS)
    list(APPEND PUBLIC_HEADERS ${roscpp_INCLUDE_DIRS})
endif()
if(SMPL_SV_VISUALIZATION_MSGS)
    list(APPEND PUBLIC_HEADERS ${visualization_msgs_INCLUDE_DIRS})
endif()

list(APPEND PUBLIC_HEADERS ${EIGEN3_INCLUDE_DIR})
list(APPEND PUBLIC_HEADERS ${Boost_INCLUDE_DIRS})
list(APPEND PUBLIC_HEADERS ${SBPL_INCLUDE_DIRS})

list(APPEND INTERFACE_HEADERS ${SMPL_INSTALL_INCLUDE_DIR})

# Gather libraries
list(APPEND PRIVATE_LIBRARIES ${Boost_FILESYSTEM_LIBRARY})
list(APPEND PRIVATE_LIBRARIES ${Boost_PROGRAM_OPTIONS_LIBRARY})
list(APPEND PRIVATE_LIBRARIES ${Boost_SYSTEM_LIBRARY})

if(SMPL_CONSOLE_ROS)
    list(APPEND PUBLIC_LIBRARIES ${roscpp_LIBRARIES})
endif()
list(APPEND PUBLIC_LIBRARIES ${SBPL_LIBRARIES})

macro(include_common_directories name)
    target_include_directories(${name} PRIVATE ${PRIVATE_HEADERS})
    target_include_directories(${name} SYSTEM PUBLIC ${PUBLIC_HEADERS})
    target_include_directories(${name} INTERFACE ${INTERFACE_HEADERS})
endmacro()
macro(link_common_libraries name)
    target_link_libraries(${name} PRIVATE ${PRIVATE_LIBRARIES})
    target_link_libraries(${name} PUBLIC ${PUBLIC_LIBRARIES})
    if(NOT Boost_USE_STATIC_LIBS)
        target_compile_definitions(${name} PRIVATE -DBOOST_ALL_DYN_LINK)
    endif()

    if(MSVC)
        target_compile_definitions(${name} PRIVATE -DBOOST_ALL_NO_LIB)
    endif()
endmacro()
macro(add_common_compile_definitions name)
    if(NOT Boost_USE_STATIC_LIBS)
        target_compile_definitions(${name} PRIVATE -DBOOST_ALL_DYN_LINK)
    endif()

    if(MSVC)
        target_compile_definitions(${name} PRIVATE -DBOOST_ALL_NO_LIB)
    endif()
endmacro()

add_library(smpl-objects OBJECT ${SMPL_LIBRARY_SOURCES})
set_target_properties(smpl-objects PROPERTIES POSITION_INDEPENDENT_CODE 1)
include_common_directories(smpl-objects)
add_common_compile_definitions(smpl-objects)

####################################
# Build the shared library version #
####################################

if(SMPL_BUILD_SHARED)
#    add_library(smpl SHARED ${SMPL_LIBRARY_SOURCES})
    add_library(smpl SHARED $<TARGET_OBJECTS:smpl-objects>)
    set_target_properties(
        smpl
        PROPERTIES
            VERSION ${SMPL_VERSION}
            SOVERSION ${SMPL_MAJOR_VERSION}.${SMPL_MINOR_VERSION})

    include_common_directories(smpl)
    link_common_libraries(smpl)
    add_common_compile_definitions(smpl)
endif()

####################################
# Build the static library version #
####################################

if(SMPL_BUILD_STATIC)
#    add_library(smpl-static STATIC ${SMPL_LIBRARY_SOURCES})
    add_library(smpl-static STATIC $<TARGET_OBJECTS:smpl-objects>)

    set_target_properties(smpl-static PROPERTIES OUTPUT_NAME smpl)
    if(MSVC)
        set_target_properties(smpl-static PROPERTIES PREFIX lib)
    endif()

    include_common_directories(smpl-static)
    link_common_libraries(smpl-static)
    add_common_compile_definitions(smpl-static)
endif()

################
# Installation #
################

set(SMPL_INCLUDE_DESTINATION ${SMPL_INSTALL_INCLUDE_DIR}/smpl)
set(SMPL_ARCHIVE_DESTINATION ${SMPL_INSTALL_LIB_DIR})
set(SMPL_LIBRARY_DESTINATION ${SMPL_INSTALL_LIB_DIR})
set(SMPL_RUNTIME_DESTINATION ${SMPL_INSTALL_BIN_DIR})

install(
    DIRECTORY include/smpl/
    DESTINATION ${SMPL_INCLUDE_DESTINATION}
    COMPONENT dev
    FILES_MATCHING PATTERN *.h PATTERN *.hpp)
install(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/include/smpl/config.h
    DESTINATION ${SMPL_INCLUDE_DESTINATION}
    COMPONENT dev)

if(SMPL_BUILD_SHARED)
    list(APPEND BUILT_LIBRARIES smpl)
endif()
if(SMPL_BUILD_STATIC)
    list(APPEND BUILT_LIBRARIES smpl-static)
endif()

install(
    TARGETS ${BUILT_LIBRARIES}
    EXPORT smpl-targets
    ARCHIVE DESTINATION ${SMPL_ARCHIVE_DESTINATION}
    LIBRARY DESTINATION ${SMPL_LIBRARY_DESTINATION}
    RUNTIME DESTINATION ${SMPL_RUNTIME_DESTINATION})

install(
    EXPORT smpl-targets
    DESTINATION ${SMPL_INSTALL_CMAKE_DIR}
    NAMESPACE smpl::
    COMPONENT dev)
install(
    FILES
        "${PROJECT_BINARY_DIR}/smpl-config.cmake"
        "${PROJECT_BINARY_DIR}/smpl-config-version.cmake"
    DESTINATION ${SMPL_INSTALL_CMAKE_DIR}
    COMPONENT dev)
