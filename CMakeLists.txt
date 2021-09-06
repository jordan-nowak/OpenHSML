CMAKE_MINIMUM_REQUIRED(VERSION 3.0.2)
set(WORKSPACE_DIR ${CMAKE_SOURCE_DIR}/../.. CACHE PATH "root of the PID workspace directory")
list(APPEND CMAKE_MODULE_PATH ${WORKSPACE_DIR}/share/cmake/system) # using generic scripts/modules of the workspace
include(Package_Definition NO_POLICY_SCOPE)

add_compile_options(-O2 -Wall -Wextra -Werror -Wunused -Wpedantic)

link_libraries(stdc++fs python3.6m)
include_directories(/usr/include/python3.6)


project(openhsml)

PID_Package(
			AUTHOR      Nowak Jordan
			ADDRESS	    https://github.com/jordan-nowak/OpenHSML.git
			YEAR        2020
			LICENSE     GNULGPL
			DESCRIPTION A simple library for finding the point correspondence between an RGB and a depth image.
			VERSION     0.1.0
		)



PID_Dependency(yaml-cpp VERSION 0.6.3)
PID_Dependency(opencv VERSION 4.0.1)
PID_Dependency(eigen)

build_PID_Package()
