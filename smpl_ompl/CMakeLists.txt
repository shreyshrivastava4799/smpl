cmake_minimum_required(VERSION 2.8.12)
project(smpl_ompl)

find_package(catkin REQUIRED COMPONENTS smpl)

find_package(ompl REQUIRED)

catkin_package()

add_executable(smpl_ompl src/smpl_ompl.cpp)
target_compile_options(smpl_ompl PRIVATE -std=c++11)
target_include_directories(smpl_ompl SYSTEM PRIVATE ${smpl_INCLUDE_DIRS} ${OMPL_INCLUDE_DIRS})
target_link_libraries(smpl_ompl PRIVATE ${smpl_LIBRARIES} ${OMPL_LIBRARIES})
