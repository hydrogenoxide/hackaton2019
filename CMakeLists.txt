cmake_minimum_required(VERSION 3.1...3.15)

find_package(Eigen3 REQUIRED)

if(${CMAKE_VERSION} VERSION_LESS 3.12)
    cmake_policy(VERSION ${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION})
endif()


include_directories(lib/hackatum-2019/src/ ${EIGEN3_INCLUDE_DIR})
add_subdirectory(lib/hackatum-2019/src/)
link_directories(lib/hackatum-2019/src/)

project(RealRoads VERSION 0.0.1
  DESCRIPTION "Convert and render XODR files"
  LANGUAGES CXX)
add_executable(rr src/main.cpp)
target_link_libraries(rr xodr tinyxml)
