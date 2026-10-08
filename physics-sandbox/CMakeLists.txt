cmake_minimum_required(VERSION 3.20)

project(ClothElasticPlayground LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

include(FetchContent)

if (EMSCRIPTEN)
    # Raylib must be configured for Web before its GLFW dependency is added.
    set(PLATFORM Web CACHE STRING "raylib target platform" FORCE)
endif()

option(SPRING_MASS_FETCH_RAYLIB "Download raylib when it is not installed" ON)

find_package(raylib 5.0 QUIET)

if (NOT raylib_FOUND AND SPRING_MASS_FETCH_RAYLIB)
    message(STATUS "raylib was not found locally; fetching raylib 5.5")
    FetchContent_Declare(
        raylib
        GIT_REPOSITORY https://github.com/raysan5/raylib.git
        GIT_TAG 5.5
        GIT_SHALLOW TRUE
    )
    FetchContent_MakeAvailable(raylib)
endif()

if (NOT raylib_FOUND AND NOT TARGET raylib)
    message(FATAL_ERROR "raylib is required. Install raylib or configure with -DSPRING_MASS_FETCH_RAYLIB=ON")
endif()

add_executable(spring_mass_sandbox
    src/main.cpp
    src/vibrating_string.h
    src/sph_fluid.h
    src/web_mouse.h
    src/flip_fluid.cpp
    src/powder_scene.cpp
    src/buoyancy_scene.cpp
    src/cloth_elastic_solids_scene.cpp
    src/beam_bending_scene.cpp
    src/bridge_builder_scene.cpp
)
target_link_libraries(spring_mass_sandbox PRIVATE raylib)

if (EMSCRIPTEN)
    set_target_properties(spring_mass_sandbox PROPERTIES SUFFIX ".html")
    target_link_options(spring_mass_sandbox PRIVATE
        "-sUSE_GLFW=3"
        "-sALLOW_MEMORY_GROWTH=1"
        "-sASYNCIFY"
    )
endif()

if (MSVC)
    target_compile_options(spring_mass_sandbox PRIVATE /W4)
else()
    target_compile_options(spring_mass_sandbox PRIVATE -Wall -Wextra -Wpedantic)
endif()
