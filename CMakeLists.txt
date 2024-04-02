cmake_minimum_required(VERSION 3.15)
project(sim)
set(CMAKE_CXX_STANDARD 17)
set(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/lib)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)


find_package(Eigen3 REQUIRED)
find_package(matplotlib_cpp REQUIRED)
find_package(Python3 COMPONENTS Development NumPy)

include_directories(./src/task ./src/planner ./src/robot ./src/map ./src/ring_vector)
file(GLOB SIM_SOURCEFILE ./src/task/*.cpp ./src/planner/*.cpp ./src/robot/*.cpp ./src/map/*.cpp ./src/ring_vector/*.cpp)
add_library(SIM_LIB STATIC ${SIM_SOURCEFILE})


add_executable(test ./src/test.cc)
target_include_directories(test PRIVATE ${Python2_INCLUDE_DIRS} ${Python2_NumPy_INCLUDE_DIRS} lib)
target_link_libraries(test Python3::Python Python3::NumPy SIM_LIB)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS}) 
add_executable(search_test ./src/planner/search_test.cc)
target_link_libraries(search_test ${OpenCV_LIBS} SIM_LIB)

add_executable(map_test ./src/map/map_test.cc)
target_link_libraries(map_test SIM_LIB)
