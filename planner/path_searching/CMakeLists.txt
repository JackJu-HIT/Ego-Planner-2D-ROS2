cmake_minimum_required(VERSION 3.28)
project(path_searching)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
include_directories(include)
include_directories(../GridMap2D/include)


add_library( path_searching STATIC
    src/dyn_a_star.cpp
    )
