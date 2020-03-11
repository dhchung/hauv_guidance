cmake_minimum_required(VERSION 3.0)
project(hauv_guidance)

set(CMAKE_CXX_COMPILER g++)
set(SOURCES src/test.cpp)
set(CLASS_SOURCES guidance/hauv_guidance.cpp)

add_executable(${PROJECT_NAME} ${SOURCES} ${CLASS_SOURCES})
include_directories(guidance/)
target_link_libraries(${PROJECT_NAME})