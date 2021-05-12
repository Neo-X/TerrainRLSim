#-------------------------------------------------------------------------------
#
# Agile and Dexterous Robotics Lab & Autonomous Systems Lab, ETH Zurich
#
# Author(s): Jemin Hwangbo
#
#-------------------------------------------------------------------------------
cmake_minimum_required (VERSION 2.6)  

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})
find_package(Eigen3 REQUIRED)
SET(CMAKE_CXX_FLAGS "-std=c++0x")
# Set Build Type
#   * None (CMAKE_C_FLAGS or CMAKE_CXX_FLAGS used)
#   * Debug (CMAKE_C_FLAGS_DEBUG or CMAKE_CXX_FLAGS_DEBUG)
#   * Release (CMAKE_C_FLAGS_RELEASE or CMAKE_CXX_FLAGS_RELEASE)
#   * RelWithDebInfo (CMAKE_C_FLAGS_RELWITHDEBINFO or CMAKE_CXX_FLAGS_RELWITHDEBINFO
#   * MinSizeRel (CMAKE_C_FLAGS_MINSIZEREL or CMAKE_CXX_FLAGS_MINSIZEREL) 
SET(CMAKE_BUILD_TYPE RelWithDebInfo)

set(EXAMPLE_SRC
	example_Optimization_main.cpp
)

include_directories(${EIGEN3_INCLUDE_DIR})
link_directories(
  /usr/local/lib
  /usr/lib
)

add_executable(example_Optimization ${EXAMPLE_SRC})
target_link_libraries(example_Optimization gsl gslcblas ${EIGEN3_LIBRARIES})