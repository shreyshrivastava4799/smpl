cmake_minimum_required(VERSION 2.8.12)
project(smpl_urdf_robot_model)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(SMPL_URDF_ROBOT_MODEL_MAJOR_VERSION 0)
set(SMPL_URDF_ROBOT_MODEL_MINOR_VERSION 0)
set(SMPL_URDF_ROBOT_MODEL_PATCH_VERSION 0)
set(SMPL_URDF_ROBOT_MODEL_VERSION ${SMPL_URDF_ROBOT_MODEL_MAJOR_VERSION}.${SMPL_URDF_ROBOT_MODEL_MINOR_VERSION}.${SMPL_URDF_ROBOT_MODEL_PATCH_VERSION})

set(SMPL_URDF_ROBOT_MODEL_INSTALL_LIB_DIR     lib             CACHE PATH "Installation directory for libraries")
set(SMPL_URDF_ROBOT_MODEL_INSTALL_INCLUDE_DIR include         CACHE PATH "Installation directory for header files")
set(SMPL_URDF_ROBOT_MODEL_INSTALL_CMAKE_DIR   lib/cmake/smpl_urdf_robot_model  CACHE PATH "Installation directory for CMake files")
set(SMPL_URDF_ROBOT_MODEL_INSTALL_BIN_DIR     bin             CACHE PATH "Installation directory for executables")
foreach(path LIB INCLUDE CMAKE BIN)
    set(var SMPL_URDF_ROBOT_MODEL_INSTALL_${path}_DIR)
    if(NOT IS_ABSOLUTE "${${var}}")
        set(${var} "${CMAKE_INSTALL_PREFIX}/${${var}}")
    endif()
endforeach()

find_package(Eigen3 REQUIRED)
find_package(smpl REQUIRED)
find_package(urdfdom REQUIRED)
find_package(urdfdom_headers REQUIRED)

file(
    RELATIVE_PATH
    REL_INCLUDE_DIR
    "${SMPL_URDF_ROBOT_MODEL_INSTALL_CMAKE_DIR}"
    "${SMPL_URDF_ROBOT_MODEL_INSTALL_INCLUDE_DIR}")
set(CONF_INCLUDE_DIRS "\${SMPL_URDF_ROBOT_MODEL_CMAKE_DIR}/${REL_INCLUDE_DIR}")

configure_file(
    smpl_urdf_robot_model-config.cmake.in
    "${PROJECT_BINARY_DIR}/smpl_urdf_robot_model-config.cmake"
    @ONLY)
configure_file(
    smpl_urdf_robot_model-config-version.cmake.in
    "${PROJECT_BINARY_DIR}/smpl_urdf_robot_model-config-version.cmake"
    @ONLY)

add_library(
    smpl_urdf_robot_model
    SHARED
    src/robot_model.cpp
    src/robot_state.cpp
    src/robot_state_bounds.cpp
    src/robot_state_visualization.cpp
    src/urdf_robot_model.cpp)
target_compile_options(smpl_urdf_robot_model PUBLIC -std=c++11)
target_include_directories(smpl_urdf_robot_model PRIVATE include)
target_include_directories(smpl_urdf_robot_model INTERFACE ${SMPL_URDF_ROBOT_MODEL_INSTALL_INCLUDE_DIR})
target_include_directories(smpl_urdf_robot_model SYSTEM PRIVATE ${urdfdom_headers_INCLUDE_DIRS})
target_include_directories(smpl_urdf_robot_model SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIR})
target_link_libraries(smpl_urdf_robot_model PUBLIC smpl::smpl)
target_link_libraries(smpl_urdf_robot_model PRIVATE ${urdfdom_LIBRARIES})

set(SMPL_URDF_ROBOT_MODEL_INCLUDE_DESTINATION ${SMPL_URDF_ROBOT_MODEL_INSTALL_INCLUDE_DIR}/smpl_urdf_robot_model)
set(SMPL_URDF_ROBOT_MODEL_ARCHIVE_DESTINATION ${SMPL_URDF_ROBOT_MODEL_INSTALL_LIB_DIR})
set(SMPL_URDF_ROBOT_MODEL_LIBRARY_DESTINATION ${SMPL_URDF_ROBOT_MODEL_INSTALL_LIB_DIR})
set(SMPL_URDF_ROBOT_MODEL_RUNTIME_DESTINATION ${SMPL_URDF_ROBOT_MODEL_INSTALL_BIN_DIR})

install(
    DIRECTORY include/smpl_urdf_robot_model/
    DESTINATION ${SMPL_URDF_ROBOT_MODEL_INCLUDE_DESTINATION}
    COMPONENT dev)
install(
    TARGETS smpl_urdf_robot_model
    EXPORT smpl_urdf_robot_model-targets
    RUNTIME DESTINATION ${SMPL_URDF_ROBOT_MODEL_RUNTIME_DESTINATION}
    ARCHIVE DESTINATION ${SMPL_URDF_ROBOT_MODEL_ARCHIVE_DESTINATION}
    LIBRARY DESTINATION ${SMPL_URDF_ROBOT_MODEL_LIBRARY_DESTINATION})

install(
    EXPORT smpl_urdf_robot_model-targets
    DESTINATION ${SMPL_URDF_ROBOT_MODEL_INSTALL_CMAKE_DIR}
    NAMESPACE smpl::
    COMPONENT dev)
install(
    FILES
        "${PROJECT_BINARY_DIR}/smpl_urdf_robot_model-config.cmake"
        "${PROJECT_BINARY_DIR}/smpl_urdf_robot_model-config-version.cmake"
        DESTINATION ${SMPL_URDF_ROBOT_MODEL_INSTALL_CMAKE_DIR}
        COMPONENT dev)

