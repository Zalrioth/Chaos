cmake_minimum_required(VERSION 3.12)
project(Chaos VERSION 0.1.0)

#https://stackoverflow.com/questions/24570916/add-external-libraries-to-cmakelist-txt-c
#https://pybullet.org/Bullet/phpBB3/viewtopic.php?t=4947

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 14)

set(CMAKE_DEBUG_POSTFIX "")

if (WIN32)
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Xclang -fopenmp")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Xclang -fopenmp")
endif (WIN32)

# NOTE: Something like this might be needed for OpenMP on Linux/Mac
if(APPLE)
        find_package(OpenMP)
        if (OPENMP_FOUND)
                set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
                set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
        endif()
endif()

if(UNIX AND NOT APPLE)
        # for Linux, BSD, Solaris, Minix
        find_package(OpenMP)
        if (OPENMP_FOUND)
                set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
                set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
        endif()
endif()

file(GLOB_RECURSE chaos_SRC
        include/*.h
        src/*.c
        include/*.hpp
        src/*.cpp)

add_library(Chaos STATIC ${chaos_SRC})

SET(includePath ${CMAKE_CURRENT_SOURCE_DIR}/../../)

SET(includeList
        ${includePath}grindstone/Chaos/include
        ${includePath}cglm-master/include
        ${includePath}lua-master)

target_include_directories(Chaos PUBLIC ${includeList})
