# About

Example integration of Jolt Physics, EnTT, and Raylib for a 3D game.

# Features

- **Jolt Physics:** Integrated physics world for physics bodies, collisions, character control, and raycasting
- **EnTT:** ECS storage linking entities to physics bodies
- **Raylib:** Rendering library that provides other common gamedev-related functions
- **Data Driven:** Data driven/functional architecture
- **3D Freecam:** Fly through 3D space

# Prerequisites

Before you begin, ensure you have the following installed:

- A C++ compiler
- CMake

## Install/Build/Run

1. Clone the repo.
2. Navigate to project folder

```
mkdir build
cd build
cmake ..
make
./raylib-entt-jolt
```

### VSCode

Ensure correct C++ standard is set in `.vscode/c_cpp_properties.json` (e.g. `"cppStandard": "c++17",`)

# TODO

- [ x ] Draw position debug markers
- [ ] Draw facing angle debug markers
- [ ] Fix character controller move directions
- [ ] Fix character controller jump
- [ ] Add functions for creating sensor bodies
- [ ] FPS rig with local Camera
- [ ] Ray-pick a character to assume control
- [ ] Minimal server/client with Enet
