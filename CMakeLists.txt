cmake_minimum_required(VERSION 3.12)
project(Chaos VERSION 0.1.0)

set(CMAKE_C_STANDARD 11)
set(CMAKE_DEBUG_POSTFIX "d")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-unused-function")

file(GLOB_RECURSE chaos_SRC
        include/chaos/*.h
        src/chaos/*.c)

add_library(chaos STATIC ${chaos_SRC})
set(includeList 
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/lib/uber-math/include)
target_include_directories(chaos PUBLIC ${includeList})
