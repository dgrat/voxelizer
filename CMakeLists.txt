cmake_minimum_required(VERSION 3.5)

project(VoxelMagick LANGUAGES CXX)

find_package(OpenMP)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


include_directories(${incl_dirs})

set(voxel_lib
    glm/glm_extensions.h
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

add_executable(VoxelMagick main.cpp ${voxel_lib})
if(OpenMP_CXX_FOUND)
    target_link_libraries(VoxelMagick stdc++fs pugixml OpenMP::OpenMP_CXX)
else()
    target_link_libraries(VoxelMagick stdc++fs pugixml)
endif()
