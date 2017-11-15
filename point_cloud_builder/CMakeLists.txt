cmake_minimum_required(VERSION 2.8.7)

project(point_cloud_builder)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

IF(NOT ROS_BUILD_TYPE)
  SET(ROS_BUILD_TYPE Release)
ENDIF()

#find_package(OpenCV 3.1.0 REQUIRED)
find_package(OpenCV 2.4.3 REQUIRED)
if(NOT OpenCV_FOUND)
	message("OpenCV Not found!")
endif()

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF(NOT CMAKE_BUILD_TYPE)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(PCL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    visualization_msgs
    tf
) 

catkin_package(
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    visualization_msgs
    tf
    tf2_ros
    pcl_conversions
)

add_library(
  ${PROJECT_NAME}
  src/Rendering.cc
  src/KeyFrame.cc
  src/MapPoint.cc
)

include_directories(
    ${PROJECT_SOURCE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_DIRS}
    ${PROJECT_SOURCE_DIR}/include
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

link_directories(
${PCL_LIBRARY_DIRS}
${OpenCV_DIRS})

rosbuild_add_executable(generate_pc
    src/Main.cc
)

target_link_libraries(
    generate_pc
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${PROJECT_NAME}
    ${PCL_LIBRARIES}
)
