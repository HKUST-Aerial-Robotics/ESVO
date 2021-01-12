cmake_minimum_required(VERSION 2.8.3)
project(esvo_time_surface)

# To be consistent with the configuration in esvo_core
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")
set(CMAKE_CXX_FLAGS "-O3")

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

find_package(OpenCV REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS})

# make the executable
cs_add_executable(esvo_time_surface
    src/TimeSurface.cpp
    src/TimeSurface_node.cpp
)

# link the executable to the necesarry libs
target_link_libraries(esvo_time_surface
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)

cs_install()