# Machine Learning hole (mlhole)

cmake_minimum_required(VERSION 2.8.12 FATAL_ERROR)

project(mlhole)

set (CMAKE_BUILD_TYPE Debug)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")


find_package(DART 6.1.1 REQUIRED
                  PATHS /home/fritz/dart/install
                  COMPONENTS gui utils-urdf)

add_executable(mlhole mlhole.cpp controller.cpp)
target_include_directories(mlhole PUBLIC ${DART_INCLUDE_DIRS})
target_link_libraries(mlhole PUBLIC ${DART_LIBRARIES})
