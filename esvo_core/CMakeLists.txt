cmake_minimum_required(VERSION 2.8.3)
project(esvo_core)
# explicitly set std=c++14 to remove errors from pcl library
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -O3")

find_package(OpenCV REQUIRED)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)

set(HEADERS
  include/esvo_core/core/DepthFusion.h
  include/esvo_core/core/DepthRegularization.h
  include/esvo_core/core/DepthProblem.h
  include/esvo_core/core/DepthProblemSolver.h
  include/esvo_core/core/EventBM.h
  include/esvo_core/core/RegProblemLM.h
  include/esvo_core/core/RegProblemSolverLM.h
  include/esvo_core/optimization/OptimizationFunctor.h
  include/esvo_core/container/CameraSystem.h
  include/esvo_core/container/DepthPoint.h
  include/esvo_core/container/EventMatchPair.h
  include/esvo_core/container/SmartGrid.h
  include/esvo_core/container/DepthMap.h
  include/esvo_core/container/ResidualItem.h
  include/esvo_core/container/TimeSurfaceObservation.h
  include/esvo_core/tools/Visualization.h
  include/esvo_core/tools/utils.h
  include/esvo_core/tools/TicToc.h
  include/esvo_core/tools/sobel.h
  include/esvo_core/tools/cayley.h
  include/esvo_core/tools/params_helper.h)

set(SOURCES
  src/core/DepthFusion.cpp
  src/core/DepthRegularization.cpp
  src/core/DepthProblem.cpp
  src/core/DepthProblemSolver.cpp
  src/core/EventBM.cpp
  src/core/RegProblemLM.cpp
  src/core/RegProblemSolverLM.cpp
  src/container/CameraSystem.cpp
  src/container/DepthPoint.cpp
  src/container/ResidualItem.cpp
  src/tools/Visualization.cpp
  src/tools/sobel.cpp
  src/tools/cayley.cpp)

cs_add_library(${PROJECT_NAME}_LIB ${SOURCES} ${HEADERS})

# Node esvo_Mapping
cs_add_executable(esvo_Mapping src/esvo_MappingNode.cpp
    src/esvo_Mapping.cpp include/esvo_core/esvo_Mapping.h)
target_link_libraries(esvo_Mapping ${PROJECT_NAME}_LIB
    ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} yaml-cpp)

# Node esvo_Tracking
cs_add_executable(esvo_Tracking src/esvo_TrackingNode.cpp
    src/esvo_Tracking.cpp include/esvo_core/esvo_Tracking.h)
target_link_libraries(esvo_Tracking ${PROJECT_NAME}_LIB
    ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} yaml-cpp)
