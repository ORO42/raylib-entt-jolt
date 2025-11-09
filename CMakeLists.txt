cmake_minimum_required(VERSION 3.25)
project(emergent-command CXX) # Declare the project as C++

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF) # Ensure strict C++17 standard compliance

message(STATUS "CMAKE_CXX_COMPILER: ${CMAKE_CXX_COMPILER}")
message(STATUS "CMAKE_CXX_COMPILER_VERSION: ${CMAKE_CXX_COMPILER_VERSION}")

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()

# Raylib
add_subdirectory(vendor/raylib)

# ReactPhysics3D
add_subdirectory(vendor/reactphysics3d)

# Game sources
# set(GAME_SOURCES
#     src/main.cpp
# )

# This includes all .cpp files in src
file(GLOB_RECURSE GAME_SOURCES
    CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)

add_executable(emergent-command
    ${GAME_SOURCES}
)

# *** IMPORTANT FIX: Ensure C++17 features are explicitly linked to the target ***
target_compile_features(emergent-command PRIVATE cxx_std_17) 

target_include_directories(emergent-command PRIVATE
    src/include
    vendor/entt/single_include
    vendor/json
)

target_link_libraries(emergent-command PRIVATE
    raylib
    reactphysics3d
)

if(UNIX AND NOT APPLE)
    target_link_libraries(emergent-command PRIVATE m dl pthread rt X11)
endif()
