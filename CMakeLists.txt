cmake_minimum_required(VERSION 3.17)
set( CMAKE_VERBOSE_MAKEFILE ON )


# set the project name
project(boids LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_RELEASE} -Ofast")

# set(CMAKE_CXX_FLAGS "-Wall -Wextra")
# set(CMAKE_CXX_FLAGS_DEBUG "-g")
# set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set( HIPSYCL_TARGETS  "cuda:sm_72" )

# if (CMAKE_BUILD_TYPE EQUAL "DEBUG")
    # set( HIPSYCL_TARGETS  "omp" )
# endif (CMAKE_BUILD_TYPE EQUAL "DEBUG")

# set( HIPSYCL_DRYRUN  "yes" )
# set( HIPSYCL_EXPLICIT_MULTIPASS "yes" )
# set( OMP_LINK_LINE "-Xclang -fopenmp" )


add_definitions( -DCMAKE_EXPORT_COMPILE_COMMANDS=ON )
add_definitions( -DGL_SILENCE_DEPRECATION )
add_definitions( -Wall )
add_definitions( -Wextra )
# add_definitions( -ffast-math )
# add_definitions( -march=native )
# add_definitions( -DEIGEN_USE_SYCL=1 )
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")

# string( REPLACE "-DNDEBUG" "" CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}")



find_package (Eigen3 3.3 REQUIRED NO_MODULE)

find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIR})

find_package(glfw3 3 REQUIRED)
include_directories(${GLFW3_INCLUDE_DIR})


find_package(RtMidi REQUIRED)

find_package(OpenMP REQUIRED)

find_package(hipSYCL REQUIRED)

find_package(Threads REQUIRED)

include_directories("lib/json/single_include/nlohmann")

# add the executable
add_executable(boids "src/main.cpp")


target_link_libraries( boids PUBLIC Eigen3::Eigen )
target_link_libraries( boids PUBLIC glfw)
target_link_libraries( boids PUBLIC OpenGL::GL )
target_link_libraries( boids PUBLIC RtMidi::rtmidi )
target_link_libraries( boids PUBLIC OpenMP::OpenMP_CXX )
target_link_libraries( boids PUBLIC hipSYCL::hipSYCL-rt )
add_sycl_to_target( TARGET boids SOURCES "src/Boid.hpp")


# target_link_libraries( boids hipSYCL::rt-backend-omp )

# target_link_libraries( boids sycl-gtx )

