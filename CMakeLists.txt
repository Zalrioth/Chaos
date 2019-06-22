cmake_minimum_required(VERSION 3.12)
project(Chaos VERSION 0.1.0)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 14)

set(CMAKE_DEBUG_POSTFIX "d")

file(GLOB_RECURSE chaos_SRC
        include/*.h
        src/*.c
        include/*.hpp
        src/*.cpp)

add_library(Chaos STATIC ${chaos_SRC})

SET(includeList
        'include'
        lib/cglm/include)

target_include_directories(Chaos PUBLIC ${includeList})
