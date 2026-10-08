cmake_minimum_required(VERSION 3.20)

project(SpringMassSandbox LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

include(FetchContent)

if (EMSCRIPTEN)
    # Raylib 5.5 needs the platform selected explicitly when it is brought in
    # through FetchContent; otherwise its GLFW dependency can select X11.
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
    src/vibrating_string.cpp
    src/flip_fluid.cpp
    src/sph_fluid.cpp
    src/powder_scene.cpp
    src/buoyancy_scene.cpp
)
target_link_libraries(spring_mass_sandbox PRIVATE raylib)

if (EMSCRIPTEN)
    # Emscripten emits the HTML loader alongside the JavaScript/WebAssembly
    # payloads so the target can be published as a static GitHub Pages site.
    set_target_properties(spring_mass_sandbox PROPERTIES SUFFIX ".html")
    target_link_options(spring_mass_sandbox PRIVATE
        "-sUSE_GLFW=3"
        "-sALLOW_MEMORY_GROWTH=1"
    )
endif()

if (MSVC)
    target_compile_options(spring_mass_sandbox PRIVATE /W4)
else()
    target_compile_options(spring_mass_sandbox PRIVATE -Wall -Wextra -Wpedantic)
endif()
