cmake_minimum_required(VERSION 3.0)

project(OpenHSML)
set(EXECUTABLE_OUTPUT_PATH ../bin)

set(VREP_CFLAGS "-DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255 -D__linux")

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -fPIC")

file( GLOB_RECURSE lib_source_files include/lib/* src/* )

include_directories( include/lib/ )
include_directories( include/lib/Eigen/ )
include_directories( include/lib/yaml-cpp/ )
include_directories( /usr/include/python3.6/ )
include_directories( include/lib/matplotlib-cpp/ )
include_directories( include/lib/opencv/ )

set( OpenCV_DIR "include/lib/opencv/share/OpenCV" )
find_package( OpenCV REQUIRED )

link_libraries(stdc++fs python3.6m boost_filesystem boost_system opencv_calib3d opencv_core opencv_highgui opencv_imgproc opencv_imgcodecs)

find_package( yaml-cpp REQUIRED )
find_package ( Eigen3 REQUIRED )

add_library( library_source ${lib_source_files} )
target_link_libraries( library_source ${CMAKE_THREAD_LIBS_INIT} ${OpenCV_LIBS} yaml-cpp Eigen3::Eigen )

file( GLOB_RECURSE exemple_source_files apps/* )
add_executable(demo ${exemple_source_files})
target_link_libraries(demo library_source)
