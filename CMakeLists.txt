project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources src/main.cpp 
lib/json.hpp lib/spline.h 
src/helpers.cpp src/helpers.h
src/map.cpp src/map.h 
src/parameters.h 

src/vehicle.cpp src/vehicle.h 
src/trajectory.cpp src/trajectory.h 
src/trajectory_generator.cpp src/trajectory_generator.h 
src/planning.cpp src/planning.h 
src/prediction.cpp src/prediction.h 
)


if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(src/)
include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 


add_executable(path_planning ${sources})
target_include_directories(path_planning PUBLIC lib/)

target_link_libraries(path_planning z ssl uv uWS)
