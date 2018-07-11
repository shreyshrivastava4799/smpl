cmake_minimum_required(VERSION 2.8.12)
project(smpl_ompl_interface)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS smpl)
find_package(Boost REQUIRED)
find_package(ompl REQUIRED)

catkin_package(
    INCLUDE_DIRS include ${OMPL_INCLUDE_DIRS}
    LIBRARIES smpl_ompl_interface ${OMPL_LIBRARIES})

add_library(smpl_ompl_interface src/ompl_interface.cpp)
target_compile_options(smpl_ompl_interface PUBLIC -std=c++11)
target_include_directories(smpl_ompl_interface PUBLIC include)
target_include_directories(smpl_ompl_interface SYSTEM PRIVATE ${smpl_INCLUDE_DIRS})
target_include_directories(smpl_ompl_interface SYSTEM PUBLIC ${OMPL_INCLUDE_DIRS})
target_include_directories(smpl_ompl_interface SYSTEM PRIVATE ${Boost_INCLUDE_DIRS})
target_link_libraries(smpl_ompl_interface PRIVATE ${smpl_LIBRARIES})
target_link_libraries(smpl_ompl_interface PUBLIC ${OMPL_LIBRARIES})
target_link_libraries(smpl_ompl_interface PRIVATE ${Boost_LIBRARIES})

add_executable(smpl_ompl_test src/smpl_ompl.cpp)
if($ENV{ROS_DISTRO} EQUAL "indigo")
    target_compile_definitions(smpl_ompl_interface PRIVATE -DROS_INDIGO)
endif()

target_compile_options(smpl_ompl_test PRIVATE -std=c++11)
target_include_directories(smpl_ompl_test SYSTEM PRIVATE ${OMPL_INCLUDE_DIRS})
target_include_directories(smpl_ompl_test SYSTEM PRIVATE ${Boost_INCLUDE_DIRS})
target_include_directories(smpl_ompl_test SYSTEM PRIVATE ${smpl_INCLUDE_DIRS})
target_link_libraries(smpl_ompl_test PRIVATE smpl_ompl_interface)
target_link_libraries(smpl_ompl_test PRIVATE ${OMPL_LIBRARIES})
target_link_libraries(smpl_ompl_test PRIVATE ${Boost_LIBRARIES})
target_link_libraries(smpl_ompl_test PRIVATE ${smpl_LIBRARIES})

install(
    TARGETS smpl_ompl_interface
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(
    DIRECTORY include/smpl_ompl_interface/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

