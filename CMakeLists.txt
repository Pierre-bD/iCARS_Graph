cmake_minimum_required(VERSION 3.0.0)
project(iCARS_FactorGraph VERSION 0.1.0)

#include(CTest)
enable_testing()

find_package(GTSAM REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME} gtsam)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
