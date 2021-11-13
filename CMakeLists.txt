cmake_minimum_required(VERSION 3.0)
project(reM)
set(CMAKE_CXX_STANDARD 23)

#NOTE(Zaklaus): setup build path
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR})

if(MSVC)
    add_definitions(-D_CRT_SECURE_NO_WARNINGS)
endif()

include_directories( 
    src
	src/engine
)

add_library(reM SHARED 
	src/reversiblehooks/SSimpleReversibleHook.cpp
	src/reversiblehooks/SVirtualReversibleHook.cpp
	src/reversiblehooks/ReversibleHooks.cpp

	src/engine/I3d_math.cpp

	src/main.cpp
	src/C_Vehicle.cpp
)

target_link_libraries(reM)