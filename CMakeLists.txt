cmake_minimum_required(VERSION 3.15)
project(sim)
set(CMAKE_CXX_STANDARD 17)
set(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/lib)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)


# find_package(Eigen3 REQUIRED)
find_package(matplotlib_cpp REQUIRED)
find_package(Python3 COMPONENTS Development NumPy)

include_directories(./src/task ./src/planner ./src/robot ./src/map ./src/ring_vector ./src/kalman)
include_directories("/usr/local/include/eigen3")
include_directories("/usr/include/eigen3")
file(GLOB SIM_SOURCEFILE ./src/task/*.cpp ./src/planner/*.cpp ./src/robot/*.cpp ./src/map/*.cpp ./src/ring_vector/*.cpp ./src/kalman/*.cpp)
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

add_executable(vector_test ./src/ring_vector/vector_test.cc)
target_link_libraries(vector_test SIM_LIB)

add_executable(task_test ./src/task/task_test.cc)
target_link_libraries(task_test SIM_LIB)

add_executable(kalman_test ./src/kalman/kalman_test.cc)
target_include_directories(kalman_test PRIVATE ${Python2_INCLUDE_DIRS} ${Python2_NumPy_INCLUDE_DIRS} lib)
target_link_libraries(kalman_test Python3::Python Python3::NumPy SIM_LIB)
target_link_libraries(kalman_test SIM_LIB)

add_executable(test_2_1 ./src/test_2_1.cc)
target_include_directories(test_2_1 PRIVATE ${Python2_INCLUDE_DIRS} ${Python2_NumPy_INCLUDE_DIRS} lib)
target_link_libraries(test_2_1 Python3::Python Python3::NumPy SIM_LIB)
target_link_libraries(test_2_1 SIM_LIB)

add_executable(test_3 ./src/test_3.cc)
target_include_directories(test_3 PRIVATE ${Python2_INCLUDE_DIRS} ${Python2_NumPy_INCLUDE_DIRS} lib)
target_link_libraries(test_3 Python3::Python Python3::NumPy SIM_LIB)
target_link_libraries(test_3 SIM_LIB)