#include "raylib.h"
#include "raymath.h"
#include <entt/entt.hpp>
#include "physics.hpp"

#include <vector>

struct FreeCam
{
    Camera3D camera;
    float yaw;
    float pitch;
    float moveSpeed;
    float mouseSensitivity;
    bool isCurrent = true;
};

struct GameState
{
    FreeCam freeCam;

    GameState()
    {
        freeCam.camera = {
            .position = {10.0f, 10.0f, 10.0f},
            .target = {0.0f, 0.0f, 0.0f},
            .up = {0.0f, 1.0f, 0.0f},
            .fovy = 45.0f,
            .projection = CAMERA_PERSPECTIVE};
        freeCam.yaw = 180.0f;
        freeCam.pitch = 0.0f;
        freeCam.moveSpeed = 20.0f;
        freeCam.mouseSensitivity = 0.08f;
    }
};

struct ECSState
{
    entt::registry registry;
};

// Body type enum
enum class BodyType
{
    Box,
    Sphere,
};

// Simple struct to track physics bodies for rendering
struct PhysicsBody
{
    JPH::BodyID id;
    BodyType type;
};

void sControlFreecam(GameState &gameState)
{
    FreeCam &cam = gameState.freeCam;
    float dt = GetFrameTime();

    // --- Mouse look ---
    if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT))
    {
        Vector2 mouseDelta = GetMouseDelta();
        cam.yaw -= mouseDelta.x * cam.mouseSensitivity;
        cam.pitch -= mouseDelta.y * cam.mouseSensitivity;

        // Clamp pitch to avoid flipping
        if (cam.pitch > 89.0f)
            cam.pitch = 89.0f;
        if (cam.pitch < -89.0f)
            cam.pitch = -89.0f;
    }

    // --- Compute planar movement basis (ignore pitch) ---
    float yawRad = DEG2RAD * cam.yaw;

    Vector3 forward = {sinf(yawRad), 0.0f, cosf(yawRad)}; // flat forward
    Vector3 right = {-cosf(yawRad), 0.0f, sinf(yawRad)};  // fixed A/D direction
    Vector3 up = {0.0f, 1.0f, 0.0f};

    // --- Movement input ---
    Vector3 moveDir = {0.0f, 0.0f, 0.0f};
    if (IsKeyDown(KEY_W))
        moveDir = Vector3Add(moveDir, forward);
    if (IsKeyDown(KEY_S))
        moveDir = Vector3Subtract(moveDir, forward);
    if (IsKeyDown(KEY_A))
        moveDir = Vector3Subtract(moveDir, right);
    if (IsKeyDown(KEY_D))
        moveDir = Vector3Add(moveDir, right);
    if (IsKeyDown(KEY_SPACE))
        moveDir = Vector3Add(moveDir, up);
    if (IsKeyDown(KEY_LEFT_SHIFT))
        moveDir = Vector3Subtract(moveDir, up);

    if (Vector3Length(moveDir) > 0.0f)
        moveDir = Vector3Normalize(moveDir);

    // --- Apply movement ---
    cam.camera.position = Vector3Add(
        cam.camera.position,
        Vector3Scale(moveDir, cam.moveSpeed * dt));

    // --- Update camera target ---
    float pitchRad = DEG2RAD * cam.pitch;
    Vector3 lookDir = {
        cosf(pitchRad) * sinf(yawRad),
        sinf(pitchRad),
        cosf(pitchRad) * cosf(yawRad)};
    cam.camera.target = Vector3Add(cam.camera.position, lookDir);
}

// Example character controller system
void sControlCharacter(PhysicsCharacter *character, float deltaTime)
{
    Vector3 velocity = physicsCharacterGetVelocity(character);

    // Movement input (horizontal only, preserve vertical velocity)
    Vector3 moveDir = {0.0f, 0.0f, 0.0f};
    float moveSpeed = 5.0f;

    if (IsKeyDown(KEY_W))
        moveDir.z += 1.0f;
    if (IsKeyDown(KEY_S))
        moveDir.z -= 1.0f;
    if (IsKeyDown(KEY_A))
        moveDir.x -= 1.0f;
    if (IsKeyDown(KEY_D))
        moveDir.x += 1.0f;

    if (Vector3Length(moveDir) > 0.0f)
        moveDir = Vector3Normalize(moveDir);

    // Apply horizontal movement
    velocity.x = moveDir.x * moveSpeed;
    velocity.z = moveDir.z * moveSpeed;

    // Jump
    if (IsKeyPressed(KEY_SPACE) && physicsCharacterIsOnGround(character))
    {
        velocity.y = 5.0f; // Jump impulse
    }

    physicsCharacterSetVelocity(character, velocity);
}

// Draw character
void sDrawCharacter(PhysicsCharacter *character, bool solid)
{
    Vector3 pos = physicsCharacterGetPosition(character);

    // Get actual dimensions from the character
    float radius = physicsCharacterGetRadius(character);
    float halfHeight = physicsCharacterGetHalfHeight(character);

    // Calculate actual top and bottom positions
    float bottomY = pos.y - (halfHeight + radius);
    float topY = pos.y + (halfHeight + radius);

    if (solid)
    {
        // Draw cylinder body
        DrawCylinderEx(
            Vector3Add(pos, {0.0f, -halfHeight, 0.0f}),
            Vector3Add(pos, {0.0f, halfHeight, 0.0f}),
            radius, radius, 16, YELLOW);

        // Draw top hemisphere
        DrawSphereEx(Vector3Add(pos, {0.0f, halfHeight, 0.0f}),
                     radius, 16, 8, YELLOW);

        // Draw bottom hemisphere
        DrawSphereEx(Vector3Add(pos, {0.0f, -halfHeight, 0.0f}),
                     radius, 16, 8, YELLOW);
    }

    // Draw wireframe
    DrawCylinderWiresEx(
        Vector3Add(pos, {0.0f, -halfHeight, 0.0f}),
        Vector3Add(pos, {0.0f, halfHeight, 0.0f}),
        radius, radius, 16, DARKGREEN);

    DrawSphereWires(Vector3Add(pos, {0.0f, halfHeight, 0.0f}),
                    radius, 8, 8, DARKGREEN);

    DrawSphereWires(Vector3Add(pos, {0.0f, -halfHeight, 0.0f}),
                    radius, 8, 8, DARKGREEN);

    DrawSphereWires(pos, 0.1f, 4, 4, ORANGE); // Draw position marker

    // Draw small crosses at top and bottom of the capsule
    DrawSphereWires({pos.x, bottomY, pos.z}, 0.05f, 4, 4, RED); // bottom
    DrawSphereWires({pos.x, topY, pos.z}, 0.05f, 4, 4, RED);    // top
}

void sDrawPhysicsCharacters(PhysicsWorld *world, entt::registry &registry, bool solid)
{
    auto view = registry.view<PhysicsCharacter>();
    for (auto entity : view)
    {
        PhysicsCharacter &character = view.get<PhysicsCharacter>(entity);
        sDrawCharacter(&character, solid);
    }
}

void sUpdatePhysicsCharacters(PhysicsWorld *world, entt::registry &registry, float deltaTime)
{
    auto view = registry.view<PhysicsCharacter>();
    for (auto entity : view)
    {
        PhysicsCharacter &character = view.get<PhysicsCharacter>(entity);
        physicsUpdateCharacter(world, &character, deltaTime);
    }
}

void destroyPhysicsCharacters(entt::registry &registry)
{
    auto view = registry.view<PhysicsCharacter>();
    for (auto entity : view)
    {
        PhysicsCharacter &character = view.get<PhysicsCharacter>(entity);
        physicsDestroyCharacter(&character);
        registry.remove<PhysicsCharacter>(entity);
    }
}

void sDrawFreeCamReticle()
{
    int windowWidth = GetScreenWidth();
    int windowHeight = GetScreenHeight();
    Vector2 center = {windowWidth * 0.5f, windowHeight * 0.5f};
    float lineLength = 10.0f;
    float halfLength = lineLength * 0.5f;
    DrawLineEx({center.x - halfLength, center.y}, {center.x + halfLength, center.y}, 1.0f, BLACK);
    DrawLineEx({center.x, center.y - halfLength}, {center.x, center.y + halfLength}, 1.0f, BLACK);
}

void sDrawPhysicsBodies(PhysicsWorld *world, entt::registry &registry, bool solid)
{
    auto view = registry.view<PhysicsBody>();
    for (auto entity : view)
    {
        const PhysicsBody &body = view.get<PhysicsBody>(entity);
        Vector3 pos = physicsGetPosition(world, body.id);
        Quaternion rot = physicsGetRotation(world, body.id);

        // Convert quaternion to axis-angle for Raylib
        Vector3 axis;
        float angle;
        QuaternionToAxisAngle(rot, &axis, &angle);
        angle = angle * RAD2DEG;

        switch (body.type)
        {
        case BodyType::Sphere:
            if (solid)
            {
                // DrawSphereEx(pos, 0.5f, 16, 16, BLUE);
                DrawSphereEx(pos, physicsGetRadius(world, body.id), 16, 16, BLUE);
            }
            DrawSphereWires(pos, physicsGetRadius(world, body.id), 8, 8, DARKBLUE);
            break;

        case BodyType::Box:
            if (solid)
            {
                DrawCubeV(pos, Vector3Scale(physicsGetHalfExtents(world, body.id), 2.0f), RED);
            }
            DrawCubeWiresV(pos, Vector3Scale(physicsGetHalfExtents(world, body.id), 2.0f), MAROON);
            break;
        }
        DrawSphereWires(pos, 0.1f, 4, 4, ORANGE); // Draw position marker
    }
}

entt::entity getEntityFromBodyId(PhysicsWorld *world, JPH::BodyID bodyId)
{
    entt::entity entity = static_cast<entt::entity>(
        world->bodyInterface->GetUserData(bodyId));
    return entity;
}

entt::entity getEntityFromCharacter(PhysicsCharacter *character)
{
    entt::entity entity = static_cast<entt::entity>(
        character->character->GetUserData());
    return entity;
}

void createStaticBoxEntity(entt::registry &registry, PhysicsWorld *world, Vector3 position, Vector3 halfExtents)
{
    JPH::BodyID bodyId = physicsCreateStaticBox(world, position, halfExtents);

    auto entity = registry.create();
    registry.emplace<PhysicsBody>(entity, bodyId, BodyType::Box);

    // Store entity in physics body user data
    world->bodyInterface->SetUserData(bodyId, static_cast<JPH::uint64>(entity));
}

void createDynamicBoxEntity(entt::registry &registry, PhysicsWorld *world, Vector3 position, Vector3 halfExtents)
{
    JPH::BodyID bodyId = physicsCreateDynamicBox(world, position, halfExtents);

    auto entity = registry.create();
    registry.emplace<PhysicsBody>(entity, bodyId, BodyType::Box);

    // Store entity in physics body user data
    world->bodyInterface->SetUserData(bodyId, static_cast<JPH::uint64>(entity));
}

void createDynamicSphereEntity(entt::registry &registry, PhysicsWorld *world, Vector3 position, float radius)
{
    JPH::BodyID bodyId = physicsCreateDynamicSphere(world, position, radius);

    auto entity = registry.create();
    registry.emplace<PhysicsBody>(entity, bodyId, BodyType::Sphere);

    // Store entity in physics body user data
    world->bodyInterface->SetUserData(bodyId, static_cast<JPH::uint64>(entity));
}

void createCharacterEntity(entt::registry &registry, PhysicsWorld *world, Vector3 position, float radius, float halfHeight)
{
    PhysicsCharacter *character = physicsCreateCharacter(world, position, radius, halfHeight);

    auto entity = registry.create();
    registry.emplace<PhysicsCharacter>(entity, character->character);

    // Store entity in physics character user data
    character->character->SetUserData(static_cast<JPH::uint64>(entity));

    character->character = nullptr; // Prevent double deletion
}

int main()
{
    SetConfigFlags(FLAG_VSYNC_HINT | FLAG_WINDOW_HIGHDPI | FLAG_WINDOW_RESIZABLE);
    InitWindow(800, 450, "raylib-entt-jolt");
    // SetTargetFPS(60);
    SetMousePosition(GetScreenWidth() / 2.0f, GetScreenHeight() / 2.0f);
    DisableCursor();

    GameState gameState;
    ECSState ecsState;

    // Initialize physics
    PhysicsWorld *physicsWorld = physicsInit();

    // Create floor
    createStaticBoxEntity(ecsState.registry, physicsWorld, {0.0f, -1.0f, 0.0f}, {50.0f, 1.0f, 50.0f});

    // Create character
    createCharacterEntity(ecsState.registry, physicsWorld, {0.0f, 0.5f, 0.0f}, 0.5f, 1.0f);

    while (!WindowShouldClose())
    {
        float dt = GetFrameTime();

        // Update
        physicsUpdate(physicsWorld, dt);

        // sControlCharacter(player, dt);
        sUpdatePhysicsCharacters(physicsWorld, ecsState.registry, dt);

        sControlFreecam(gameState);

        // Spawn new sphere on key press
        if (IsKeyPressed(KEY_B))
        {
            createDynamicSphereEntity(ecsState.registry, physicsWorld, gameState.freeCam.camera.position, 0.5f);
        }

        // Draw
        BeginDrawing();
        ClearBackground(RAYWHITE);

        BeginMode3D(gameState.freeCam.camera);
        DrawGrid(20, 1.0f);
        sDrawPhysicsBodies(physicsWorld, ecsState.registry, false);
        sDrawPhysicsCharacters(physicsWorld, ecsState.registry, false);
        EndMode3D();

        DrawFPS(10, 10);
        DrawText("Press B to spawn sphere", 10, 30, 20, DARKGRAY);
        DrawText("WASD to move freecam", 10, 50, 20, DARKGRAY);
        DrawText("Shift/Space to raise/lower freecam", 10, 70, 20, DARKGRAY);
        DrawText("Right-Mouse to look around", 10, 90, 20, DARKGRAY);
        sDrawFreeCamReticle();
        EndDrawing();
    }

    // Cleanup
    destroyPhysicsCharacters(ecsState.registry);
    physicsShutdown(physicsWorld);
    CloseWindow();
    return 0;
}