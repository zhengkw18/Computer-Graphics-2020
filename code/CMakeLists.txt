CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
PROJECT(Render CXX)

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF()

ADD_SUBDIRECTORY(vecmath)

SET(Render_SOURCES
        src/camera.cpp
        src/hit.cpp
        src/hitpointmap.cpp
        src/image.cpp
        src/kdtree.cpp
        src/light.cpp
        src/main.cpp
        src/material.cpp
        src/obj_parser.cpp
        src/object3d.cpp
        src/photontracer.cpp
        src/raytracer.cpp
        src/scene.cpp
        src/scene_parser.cpp
        src/pathtracer.cpp
        src/photonmap.cpp
        src/volumetricphotontracer.cpp
        )

SET(Render_INCLUDES
        include/camera.hpp
        include/hit.hpp
        include/hitpointmap.hpp
        include/image.hpp
        include/kdtree.hpp
        include/light.hpp
        include/material.hpp
        include/obj_parser.hpp
        include/object3d.hpp
        include/photon.hpp
        include/photontracer.hpp
        include/ray.hpp
        include/raytracer.hpp
        include/scene.hpp
        include/scene_parser.hpp
        include/utils.h
        include/pathtracer.hpp
        include/photonmap.hpp
        include/volumetricphotontracer.hpp
        )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

FIND_PACKAGE( OpenMP REQUIRED)
if(OPENMP_FOUND)
message("OPENMP FOUND")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS"${CMAKE_EXE_LINKER_FLAGS}${OpenMP_EXE_LINKER_FLAGS}")
endif()

ADD_EXECUTABLE(${PROJECT_NAME} ${Render_SOURCES} ${Render_INCLUDES})

SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES
      CXX_STANDARD 11
      CXX_STANDARD_REQUIRED YES
      CXX_EXTENSIONS YES)

find_package(Threads)
target_link_libraries (${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})

TARGET_LINK_LIBRARIES(${PROJECT_NAME} vecmath)
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} PRIVATE include)
