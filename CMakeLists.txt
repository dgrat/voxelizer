cmake_minimum_required(VERSION 3.5)

project(VoxelMagick LANGUAGES CXX)

find_package(OpenMP)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


#set ( CMAKE_CXX_FLAGS "-msse -mavx")


include_directories(
    glm
    pugixml/src
)

set(voxel_lib
    glm_ext/glm_extensions.h
    mesh/polyhedron.h
    mesh/polyhedron.cpp
    stl/stl_import.h
    stl/stl_import.cpp
    voxelizer.h
    rules.h
    checks.h
    checks.cpp
    xml_config.h
)

find_package(Git QUIET)
if(GIT_FOUND AND EXISTS "${PROJECT_SOURCE_DIR}/.git")
# Update submodules as needed
    option(GIT_SUBMODULE "Check submodules during build" ON)
    if(GIT_SUBMODULE)
        message(STATUS "Submodule update")
        execute_process(COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive
                        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                        RESULT_VARIABLE GIT_SUBMOD_RESULT)
        if(NOT GIT_SUBMOD_RESULT EQUAL "0")
            message(FATAL_ERROR "git submodule update --init failed with ${GIT_SUBMOD_RESULT}, please checkout submodules")
        endif()
    endif()
endif()

add_subdirectory(pugixml)
link_directories(pugixml)

add_executable(VoxelMagick main.cpp ${voxel_lib})
if(OpenMP_CXX_FOUND)
    target_link_libraries(VoxelMagick pugixml OpenMP::OpenMP_CXX)
else()
    target_link_libraries(VoxelMagick pugixml)
endif()
