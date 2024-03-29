cmake_minimum_required(VERSION 3.15)
project(sim)
set(CMAKE_CXX_STANDARD 17)
set(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/lib)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)


find_package(Eigen3 REQUIRED)
find_package(matplotlib_cpp REQUIRED)
find_package(Python3 COMPONENTS Development NumPy)

include_directories(./src/task ./src/pathPlan)
file(GLOB SIM_SOURCEFILE ./src/task/*.cc ./src/pathPlan/*.cc)
add_library(PathFinder STATIC ${SIM_SOURCEFILE})

add_executable(test test.cc)
target_include_directories(b_test PRIVATE ${Python2_INCLUDE_DIRS} ${Python2_NumPy_INCLUDE_DIRS} lib)
target_link_libraries(b_test traj_lib UvUtility Python3::Python Python3::NumPy)
