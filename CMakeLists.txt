cmake_minimum_required(VERSION 3.0.0)
project(dfs VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED YES)

include(CTest)
enable_testing()

find_package(Threads)

add_executable(dfs main.cpp)
target_link_libraries(dfs PRIVATE Threads::Threads)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
