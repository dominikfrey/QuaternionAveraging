cmake_minimum_required(VERSION 3.14)
project(QuaternionAveraging)

set(CMAKE_CXX_STANDARD 14)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)
add_executable(QuaternionAveraging main.cpp)

target_link_libraries (QuaternionAveraging Eigen3::Eigen)