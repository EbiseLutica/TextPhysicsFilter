cmake_minimum_required(VERSION 3.22)
project(TextPhysicsFilter LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# Link the CRT statically so the plugin does not need the VC++ redistributable.
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")

set(AVIUTL2_SDK_DIR "" CACHE PATH "Directory containing AviUtl2 SDK headers (plugin2.h, filter2.h)")
if(NOT EXISTS "${AVIUTL2_SDK_DIR}/filter2.h")
    message(FATAL_ERROR "Set AVIUTL2_SDK_DIR to the folder that contains filter2.h and plugin2.h")
endif()

include(FetchContent)
set(BOX2D_SAMPLES OFF CACHE BOOL "" FORCE)
set(BOX2D_UNIT_TESTS OFF CACHE BOOL "" FORCE)
set(BOX2D_BENCHMARKS OFF CACHE BOOL "" FORCE)
set(BOX2D_DOCS OFF CACHE BOOL "" FORCE)
set(BOX2D_VALIDATE OFF CACHE BOOL "" FORCE)
FetchContent_Declare(box2d
    GIT_REPOSITORY https://github.com/erincatto/box2d.git
    GIT_TAG v3.2.0
    GIT_SHALLOW TRUE)
FetchContent_MakeAvailable(box2d)

add_library(TextPhysicsFilter MODULE TextPhysicsFilter.cpp AlphaComponentSegmentation.h)
target_include_directories(TextPhysicsFilter PRIVATE "${AVIUTL2_SDK_DIR}")
target_link_libraries(TextPhysicsFilter PRIVATE box2d)
# The sources are UTF-8 without BOM; without this MSVC reads them in the system
# code page and the plugin name (and every label) turns into mojibake.
if(MSVC)
    target_compile_options(TextPhysicsFilter PRIVATE /utf-8)
endif()
# AviUtl2 loads filter plugins named *.auf2, which are plain DLLs.
set_target_properties(TextPhysicsFilter PROPERTIES PREFIX "" SUFFIX ".auf2")
