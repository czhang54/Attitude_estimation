cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project("Attitude Estimation")

# Require Eigen
#find_package( Eigen REQUIRED ) # find_package() is NOT needed since Eigen is a header-only library
include_directories(/usr/local/Cellar/eigen/3.3.1/include) # Replace with actual include path of Eigen

set (CMAKE_CXX_STANDARD 11) # Choose C++ version to compile

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(include) # Optional, maybe because this include directory is added in src/CMakeLists.txt

add_subdirectory(src)

add_executable(attitude_estimation test/test.cpp)

target_link_libraries(attitude_estimation attitude_estimation_source)

#enable_testing()

#add_test(attitude_test attitude_estimation)


set(CONFIGURED_ONCE TRUE CACHE INTERNAL
    "A flag showing that CMake has configured at least once.")
