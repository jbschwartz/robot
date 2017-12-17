cmake_minimum_required(VERSION 3.7)
project(Robot)

set(VERSION 1)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/bin)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})

option(DEFINE_DEBUG "Build in Debug Mode" ON)
if(DEFINE_DEBUG)
  message("Building in debug mode")
  add_definitions(-DDEBUG)
ENDIF(DEFINE_DEBUG)

include_directories(include)
add_subdirectory(test)

file(GLOB HEADERS "include/*.h")
file(GLOB SOURCES "src/*.cpp")
list(REMOVE_ITEM SOURCES "${CMAKE_SOURCE_DIR}/src/main.cpp")

add_library(RobotLib ${HEADERS} ${SOURCES})
add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} RobotLib)
