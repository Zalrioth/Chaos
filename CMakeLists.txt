cmake_minimum_required(VERSION 3.12)
project(Grindstone VERSION 0.1.0)

#https://stackoverflow.com/questions/24570916/add-external-libraries-to-cmakelist-txt-c
#https://pybullet.org/Bullet/phpBB3/viewtopic.php?t=4947

set(CMAKE_CXX_STANDARD 14)

file(GLOB_RECURSE grindstone_SRC
        include/*.h
        src/*.c
        include/*.hpp
        src/*.cpp)

add_executable(Grindstone ${grindstone_SRC})

SET(includePath ${CMAKE_CURRENT_SOURCE_DIR}/../../)

target_link_libraries(Grindstone ${includePath}cpython-3.7.2/PCbuild/amd64/python37_d.lib)

SET(includeList ${includePath}cpython-3.7.2/PC ${includePath}cpython-3.7.2/Include)

target_include_directories(Grindstone PUBLIC ${includeList})
