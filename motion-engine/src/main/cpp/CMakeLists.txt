cmake_minimum_required(VERSION 3.22.1)

project("motionengine")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(
    -O3
    -Wall
    -Wextra
    -ffast-math
    -fvisibility=hidden
)

add_library(
    motionengine
    SHARED
    motion-engine.cpp
)

target_include_directories(
    motionengine
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

find_library(log-lib log)
find_library(android-lib android)
find_library(jnigraphics-lib jnigraphics)

target_link_libraries(
    motionengine
    ${log-lib}
    ${android-lib}
    ${jnigraphics-lib}
    m
)
