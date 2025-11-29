
set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

include_directories(../path_searching/include)
include_directories(../plan_env/include)
include_directories(include)

add_library(    bspline_opt STATIC
                src/uniform_bspline.cpp 
                src/bspline_optimizer.cpp 
                src/gradient_descent_optimizer.cpp
 )

