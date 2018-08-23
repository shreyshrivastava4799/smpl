cmake_minimum_required(VERSION 2.8.12)
project(sbpl_collision_checking_test)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin
    REQUIRED
    COMPONENTS
        geometric_shapes
        geometry_msgs
        leatherman
        moveit_msgs
        roscpp
        sbpl_collision_checking
        urdf
        visualization_msgs)

find_package(smpl REQUIRED)

catkin_package()

include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

add_executable(test_collision_model src/test_collision_model.cpp)
target_link_libraries(test_collision_model ${catkin_LIBRARIES})

add_executable(test_space src/test_collision_space.cpp)
target_link_libraries(test_space ${catkin_LIBRARIES})
target_link_libraries(test_space smpl::smpl)

add_executable(benchmark src/benchmark_cc.cpp)
target_link_libraries(benchmark ${catkin_LIBRARIES})
target_link_libraries(benchmark smpl::smpl)
