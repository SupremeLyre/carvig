project(carvig)

cmake_minimum_required(VERSION 2.8)
cmake_policy(SET CMP0015 NEW)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_CXX_FLAGS "-std=c++11 -O0 -fPIC")
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
set(CMAKE_CXX_FLAGS "-msse3")
set(CMAKE_C_FLAGS "-msse3")
set(CMAKE_C_FLAGS "-mavx")

if("${CMAKE_BUILD_TYPE}" STREQUAL "")
    SET(CMAKE_BUILD_TYPE "Release")
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(OpenCV QUIET)
if(NOT OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV not found.")
endif()
find_package(Eigen3 3.1.0 REQUIRED)
find_package(PythonLibs REQUIRED)
find_package(Boost REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/src/ins-gnss/common
        ${PROJECT_SOURCE_DIR}/src/ins-gnss/gnss
        ${PROJECT_SOURCE_DIR}/src/ins-gnss/gnss/rcv
        ${PROJECT_SOURCE_DIR}/src/ins-gnss/ins
        ${PROJECT_SOURCE_DIR}/src/ins-gnss/ins/rcv
        ${PROJECT_SOURCE_DIR}/src/ins-gnss/vision
        ${PROJECT_SOURCE_DIR}/src/ins-gnss/vision/io
        ${PROJECT_SOURCE_DIR}/src/ins-gnss/vision/rcv
        ${PROJECT_SOURCE_DIR}/src/libviso
        ${EIGEN3_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
)
include_directories(${PYTHON_INCLUDE_DIRS})

file(GLOB_RECURSE COMMON_SRC_FILES
        "src/ins-gnss/common/*.c" "src/ins-gnss/common/*.cc")
file(GLOB_RECURSE GNSS_SRC_FILES
        "src/ins-gnss/gnss/*.c" "src/ins-gnss/gnss/*.cc")
file(GLOB_RECURSE INS_SRC_FILES
        "src/ins-gnss/ins/*.c" "src/ins-gnss/ins/*.cc")
file(GLOB_RECURSE VISION_SRC_FILES
        "src/ins-gnss/vision/*.c" "src/ins-gnss/vision/*.cc")

set(SRC_FILES
        ${COMMON_SRC_FILES}
        ${GNSS_SRC_FILES}
        ${INS_SRC_FILES}
        ${VISION_SRC_FILES})

add_library(carvig ${SRC_FILES})
target_link_libraries(carvig blas lapack z pthread
                      png
                      jpeg)

set_target_properties(carvig PROPERTIES ARCHIVE_OUTPUT_DIRECTORY
                      ${PROJECT_SOURCE_DIR}/lib)

# add test
if(EXISTS "${PROJECT_SOURCE_DIR}/test/CMakeLists.txt")
    add_subdirectory(test)
endif()

# add relative pose estimate library
add_subdirectory(src/relative-pose)

# add libvisio
add_subdirectory(src/libviso)

add_executable(carvignav app/igvonav.c)
target_link_libraries(carvignav carvig pthread
                      ${OpenCV_LIBS}
                      five-point-nister
                      viso)
set_target_properties(carvignav PROPERTIES LINKER_LANGUAGE CXX)

add_executable(tc-postpos app/tc-postpos.c)
target_link_libraries(tc-postpos
                      carvig pthread
                      ${OpenCV_LIBS}
                      five-point-nister
                      viso)
set_target_properties(tc-postpos PROPERTIES LINKER_LANGUAGE CXX)

add_executable(rnx2rtkp app/rnx2rtkp.c)
target_link_libraries(rnx2rtkp
                      carvig pthread
                      ${OpenCV_LIBS}
                      five-point-nister
                      viso)
set_target_properties(rnx2rtkp PROPERTIES LINKER_LANGUAGE CXX)
