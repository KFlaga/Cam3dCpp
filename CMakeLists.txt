cmake_minimum_required(VERSION 3.0)
project(Cam3d)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -std=c++14")

set(BINARIES_DIR ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARIES_DIR ${PROJECT_SOURCE_DIR}/lib)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${LIBRARIES_DIR})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${LIBRARIES_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BINARIES_DIR})

include_directories( ${PROJECT_SOURCE_DIR} )

IF(CMAKE_BUILD_TYPE MATCHES Debug)
message(STATUS "Debug build.")
add_definitions(-D_DEBUG)
ELSEIF(CMAKE_BUILD_TYPE MATCHES Release)
message(STATUS "Release build.")
add_definitions(-D_RELEASE)
ELSE()
message(FATAL_ERROR "Bad build type.")
ENDIF()

find_package(Boost 1.61.0 REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

add_subdirectory(CamCommon)
add_subdirectory(CamImageProcessing)
add_subdirectory(CamTests)
