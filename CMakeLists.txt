cmake_minimum_required(VERSION 3.12)

project(OpenHSML)
set(EXECUTABLE_OUTPUT_PATH ../bin)

find_package( OpenCV REQUIRED )
find_package( Threads REQUIRED )
find_package( yaml-cpp REQUIRED )
find_package( Eigen3 3.3 REQUIRED NO_MODULE )
find_package( Python REQUIRED COMPONENTS Development NumPy )

file( GLOB_RECURSE lib_source_files include/lib/* src/* )
add_library( library_source ${lib_source_files} )
target_link_libraries(
    library_source 
    PUBLIC
        ${CMAKE_THREAD_LIBS_INIT} 
        opencv_calib3d 
        opencv_core 
        opencv_highgui 
        opencv_imgproc 
        opencv_imgcodecs 
        yaml-cpp 
        Eigen3::Eigen
)
if(CMAKE_CXX_COMPILER_ID MATCHES GNU)
    target_link_libraries(library_source PRIVATE stdc++fs)
endif()
target_include_directories(library_source PUBLIC include/lib)
target_compile_features(library_source PUBLIC cxx_std_14)
target_compile_options(library_source PRIVATE -Wall -Wextra)

file( GLOB_RECURSE exemple_source_files apps/* )
add_executable(demo ${exemple_source_files})
target_link_libraries(demo library_source Python::Python Python::NumPy)