cmake_minimum_required(VERSION 3.10)

project(ECE4122_FinalProject LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(FinalProject
    main.cpp
    ECE_UAV.cpp
    OBJLoader.cpp
)

target_include_directories(FinalProject PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

if(APPLE)
    target_link_libraries(FinalProject PRIVATE
        "-framework OpenGL"
        "-framework GLUT"
    )
elseif(UNIX)
    find_package(OpenGL REQUIRED)
    find_package(GLUT   REQUIRED)

    target_include_directories(FinalProject PRIVATE
        ${OPENGL_INCLUDE_DIRS}
        ${GLUT_INCLUDE_DIRS}
    )

    target_link_libraries(FinalProject PRIVATE
        ${OPENGL_LIBRARIES}
        ${GLUT_LIBRARIES}
        pthread
    )
else()
    find_package(OpenGL REQUIRED)
    find_package(GLUT   REQUIRED)

    target_include_directories(FinalProject PRIVATE
        ${OPENGL_INCLUDE_DIRS}
        ${GLUT_INCLUDE_DIRS}
    )

    target_link_libraries(FinalProject PRIVATE
        ${OPENGL_LIBRARIES}
        ${GLUT_LIBRARIES}
    )
endif()
