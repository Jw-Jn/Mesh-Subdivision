cmake_minimum_required(VERSION 3.5)
project(nanogui_test)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

add_subdirectory(lib/nanogui)
include_directories(lib/nanogui/include)
include_directories(${NANOGUI_EXTRA_INCS})
add_definitions(${NANOGUI_EXTRA_DEFS})

include_directories(lib/embed-resource)
add_subdirectory(lib/embed-resource)

SET(TARGET_H WingEdge.h)
SET(TARGET_H vec3.h)

SET(TARGET_SRC WingEdge.cpp)

embed_resources(MyResources shader/vert.glsl shader/frag.glsl)

add_executable(MeshSubdivision obj_view.cpp ${MyResources} ${TARGET_H} ${TARGET_SRC})
target_link_libraries(MeshSubdivision nanogui ${NANOGUI_EXTRA_LIBS})
