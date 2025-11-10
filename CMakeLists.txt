cmake_minimum_required(VERSION 3.25)
project(raylib-entt-jolt CXX) # Declare the project as C++

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF) # Ensure strict C++17 standard compliance

message(STATUS "CMAKE_CXX_COMPILER: ${CMAKE_CXX_COMPILER}")
message(STATUS "CMAKE_CXX_COMPILER_VERSION: ${CMAKE_CXX_COMPILER_VERSION}")

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()

include(FetchContent)

# ============================================================================
# Jolt Physics Configuration
# ============================================================================

# When turning this option on, the library will be compiled using doubles for positions. This allows for much bigger worlds.
set(DOUBLE_PRECISION OFF)

# When turning this option on, the library will be compiled with debug symbols
set(GENERATE_DEBUG_SYMBOLS ON)

# When turning this option on, the library will override the default CMAKE_CXX_FLAGS_DEBUG/RELEASE values, otherwise they will use the platform defaults
set(OVERRIDE_CXX_FLAGS OFF)

# When turning this option on, the library will be compiled in such a way to attempt to keep the simulation deterministic across platforms
set(CROSS_PLATFORM_DETERMINISTIC OFF)

# When turning this option on, the library will be compiled with interprocedural optimizations enabled, also known as link-time optimizations or link-time code generation.
set(INTERPROCEDURAL_OPTIMIZATION ON)

# When turning this on, in Debug and Release mode, the library will emit extra code to ensure that the 4th component of a 3-vector is kept the same as the 3rd component 
# and will enable floating point exceptions during simulation to detect divisions by zero. 
set(FLOATING_POINT_EXCEPTIONS_ENABLED OFF)

# When turning this on, the library will be compiled with C++ exceptions enabled.
# This adds some overhead and Jolt doesn't use exceptions so by default it is off.
set(CPP_EXCEPTIONS_ENABLED OFF)

# When turning this on, the library will be compiled with C++ RTTI enabled.
# This adds some overhead and Jolt doesn't use RTTI so by default it is off.
set(CPP_RTTI_ENABLED OFF)

# Number of bits to use in ObjectLayer. Can be 16 or 32.
set(OBJECT_LAYER_BITS 16)

# Select X86 processor features to use, by default the library compiles with AVX2, if everything is off it will be SSE2 compatible.
set(USE_SSE4_1 ON)
set(USE_SSE4_2 ON)
set(USE_AVX ON)
set(USE_AVX2 ON)
set(USE_AVX512 OFF)
set(USE_LZCNT ON)
set(USE_TZCNT ON)
set(USE_F16C ON)
set(USE_FMADD ON)

# ============================================================================
# Fetch All Dependencies
# ============================================================================

# Jolt Physics
FetchContent_Declare(
    JoltPhysics
    GIT_REPOSITORY "https://github.com/jrouwe/JoltPhysics"
    GIT_TAG "v5.4.0"
    SOURCE_SUBDIR "Build"
)

# Raylib
FetchContent_Declare(
    raylib
    GIT_REPOSITORY "https://github.com/raysan5/raylib.git"
    GIT_TAG "5.5"
)

# EnTT (header-only library)
FetchContent_Declare(
    EnTT
    GIT_REPOSITORY "https://github.com/skypjack/entt.git"
    GIT_TAG "v3.15.0"
)

# Make all dependencies available
FetchContent_MakeAvailable(JoltPhysics raylib EnTT)

# ============================================================================
# Game Executable
# ============================================================================

# This includes all .cpp files in src
file(GLOB_RECURSE GAME_SOURCES
    CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)

add_executable(raylib-entt-jolt
    ${GAME_SOURCES}
)

# *** IMPORTANT FIX: Ensure C++17 features are explicitly linked to the target ***
target_compile_features(raylib-entt-jolt PRIVATE cxx_std_17) 

target_include_directories(raylib-entt-jolt PRIVATE
    src/include
    ${entt_SOURCE_DIR}/single_include
    ${JoltPhysics_SOURCE_DIR}/..
)

target_link_libraries(raylib-entt-jolt PRIVATE
    raylib
    Jolt
)

# Platform-specific libraries
if(UNIX AND NOT APPLE)
    target_link_libraries(raylib-entt-jolt PRIVATE m dl pthread rt X11)
endif()