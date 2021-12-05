cmake_minimum_required(VERSION 3.10)

# set the project name
project(boids LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_RELEASE} -Ofast")

# set(CMAKE_CXX_FLAGS "-Wall -Wextra")
# set(CMAKE_CXX_FLAGS_DEBUG "-g")
# set(CMAKE_CXX_FLAGS_RELEASE "-O3")

add_definitions( -DCMAKE_EXPORT_COMPILE_COMMANDS=ON )
add_definitions( -DGL_SILENCE_DEPRECATION )
add_definitions( -Wall )
add_definitions( -Wextra )
add_definitions( -ffast-math )


# add_subdirectory(lib/rtmidi)



find_package (Eigen3 3.4 REQUIRED NO_MODULE)

find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIR})

find_package(glfw3 3 REQUIRED)
include_directories(${GLFW3_INCLUDE_DIR})

find_package(RtMidi REQUIRED)

find_package(OpenMP REQUIRED)



# add the executable
add_executable(boids src/main.cpp)

target_link_libraries( boids Eigen3::Eigen )
target_link_libraries( boids glfw)
target_link_libraries( boids OpenGL::GL )
target_link_libraries( boids RtMidi::rtmidi )
target_link_libraries( boids OpenMP::OpenMP_CXX)
