# CMake build file for the ECE 4122 / 6122 final project.
# This file assumes that an OpenGL and GLUT development environment is installed.

cmake_minimum_required(VERSION 3.10)

project(ECE4122_FinalProject LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

add_executable(FinalProject
    main.cpp
    Vec3.h
    ECE_UAV.h
    ECE_UAV.cpp
    OBJLoader.h
    OBJLoader.cpp
)

target_include_directories(FinalProject PRIVATE
    ${OPENGL_INCLUDE_DIRS}
    ${GLUT_INCLUDE_DIRS}
)

target_link_libraries(FinalProject PRIVATE
    ${OPENGL_LIBRARIES}
    ${GLUT_LIBRARIES}
)

if(UNIX)
    target_link_libraries(FinalProject PRIVATE pthread)
endif()
