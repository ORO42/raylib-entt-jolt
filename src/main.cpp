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
    Capsule
};

// Simple struct to track physics bodies for rendering
struct PhysicsBody
{
    JPH::BodyID id;
    BodyType type;
    Vector3 halfExtents; // For boxes
    float radius;        // For spheres and capsules
    float height;        // For capsules
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
void sDrawCharacter(PhysicsCharacter *character)
{
    Vector3 pos = physicsCharacterGetPosition(character);

    // Draw capsule (approximate, adjust sizes to match your character)
    float radius = 0.5f;
    float halfHeight = 0.5f;

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

    // Draw wireframe
    DrawCylinderWiresEx(
        Vector3Add(pos, {0.0f, -halfHeight, 0.0f}),
        Vector3Add(pos, {0.0f, halfHeight, 0.0f}),
        radius, radius, 16, ORANGE);
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

void sDrawPhysicsBodies(PhysicsWorld *world, const std::vector<PhysicsBody> &bodies)
{
    for (const auto &body : bodies)
    {
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
            // DrawSphereEx(pos, body.radius, 16, 16, BLUE);
            DrawSphereWires(pos, body.radius, 16, 16, DARKBLUE);
            break;

        case BodyType::Box:
            // DrawCubeV(pos, Vector3Scale(body.halfExtents, 2.0f), RED);
            DrawCubeWiresV(pos, Vector3Scale(body.halfExtents, 2.0f), MAROON);
            break;

        case BodyType::Capsule:
        {
            // Jolt's capsule: height parameter is HALF the cylinder height
            // Total capsule height = 2 * halfHeight + 2 * radius
            float halfHeight = body.height; // This is what Jolt stores

            // Draw cylinder body (the cylindrical portion between the hemispheres)
            // DrawCylinderEx(
            //     Vector3Add(pos, {0.0f, -halfHeight, 0.0f}),
            //     Vector3Add(pos, {0.0f, halfHeight, 0.0f}),
            //     body.radius, body.radius, 16, GREEN);

            // // Draw top hemisphere
            // DrawSphereEx(Vector3Add(pos, {0.0f, halfHeight, 0.0f}),
            //              body.radius, 16, 8, GREEN);

            // // Draw bottom hemisphere
            // DrawSphereEx(Vector3Add(pos, {0.0f, -halfHeight, 0.0f}),
            //              body.radius, 16, 8, GREEN);

            // Draw wireframe
            DrawCylinderWiresEx(
                Vector3Add(pos, {0.0f, -halfHeight, 0.0f}),
                Vector3Add(pos, {0.0f, halfHeight, 0.0f}),
                body.radius, body.radius, 16, DARKGREEN);
            DrawSphereWires(Vector3Add(pos, {0.0f, halfHeight, 0.0f}),
                            body.radius, 8, 8, DARKGREEN);
            DrawSphereWires(Vector3Add(pos, {0.0f, -halfHeight, 0.0f}),
                            body.radius, 8, 8, DARKGREEN);
            break;
        }
        }
    }
}

int main()
{
    SetConfigFlags(FLAG_VSYNC_HINT | FLAG_WINDOW_HIGHDPI | FLAG_WINDOW_RESIZABLE);
    InitWindow(800, 450, "emergent-command");
    // SetTargetFPS(60);
    SetMousePosition(GetScreenWidth() / 2.0f, GetScreenHeight() / 2.0f);
    DisableCursor();

    GameState gameState;
    ECSState ecsState;

    // Initialize physics
    PhysicsWorld *physicsWorld = physicsInit();
    std::vector<PhysicsBody> bodies;

    // Create floor
    JPH::BodyID floorId = physicsCreateStaticBox(physicsWorld, {0.0f, -1.0f, 0.0f}, {50.0f, 1.0f, 50.0f});
    bodies.push_back({floorId, BodyType::Box, {50.0f, 1.0f, 50.0f}, 0.0f, 0.0f});

    // Create some test objects
    JPH::BodyID sphere1 = physicsCreateDynamicSphere(physicsWorld, {0.0f, 10.0f, 0.0f}, 0.5f);
    bodies.push_back({sphere1, BodyType::Sphere, {0.0f, 0.0f, 0.0f}, 0.5f, 0.0f});

    JPH::BodyID box1 = physicsCreateDynamicBox(physicsWorld, {2.0f, 15.0f, 0.0f}, {0.5f, 0.5f, 0.5f});
    bodies.push_back({box1, BodyType::Box, {0.5f, 0.5f, 0.5f}, 0.0f, 0.0f});

    JPH::BodyID sphere2 = physicsCreateDynamicSphere(physicsWorld, {-2.0f, 20.0f, 0.0f}, 0.7f);
    bodies.push_back({sphere2, BodyType::Sphere, {0.0f, 0.0f, 0.0f}, 0.7f, 0.0f});

    // Create a test capsule
    JPH::BodyID capsule1 = physicsCreateDynamicCapsule(physicsWorld, {-5.0f, 15.0f, 0.0f}, 0.5f, 1.0f);
    bodies.push_back({capsule1, BodyType::Capsule, {0.0f, 0.0f, 0.0f}, 0.5f, 1.0f});

    // Create a character controller
    PhysicsCharacter *player = physicsCreateCharacter(physicsWorld, {0.0f, 5.0f, 0.0f}, 0.5f, 2.0f);

    while (!WindowShouldClose())
    {
        float dt = GetFrameTime();

        // Update
        physicsUpdate(physicsWorld, dt);

        sControlCharacter(player, dt);
        physicsUpdateCharacter(physicsWorld, player, dt);

        // sControlFreecam(gameState);

        // Spawn new sphere on key press
        if (IsKeyPressed(KEY_B))
        {
            JPH::BodyID newSphere = physicsCreateDynamicSphere(
                physicsWorld,
                Vector3Add(gameState.freeCam.camera.position, {0.0f, 2.0f, 0.0f}),
                0.5f);
            bodies.push_back({newSphere, BodyType::Sphere, {0.0f, 0.0f, 0.0f}, 0.5f, 0.0f});
        }

        // Spawn new capsule on key press
        if (IsKeyPressed(KEY_C))
        {
            JPH::BodyID newCapsule = physicsCreateDynamicCapsule(
                physicsWorld,
                Vector3Add(gameState.freeCam.camera.position, {0.0f, 2.0f, 0.0f}),
                0.5f, 1.5f);
            bodies.push_back({newCapsule, BodyType::Capsule, {0.0f, 0.0f, 0.0f}, 0.5f, 1.5f});
        }

        // Draw
        BeginDrawing();
        ClearBackground(RAYWHITE);

        BeginMode3D(gameState.freeCam.camera);
        DrawGrid(20, 1.0f);
        sDrawPhysicsBodies(physicsWorld, bodies);
        sDrawCharacter(player);
        EndMode3D();

        DrawFPS(10, 10);
        DrawText("Press B to spawn sphere", 10, 30, 20, DARKGRAY);
        DrawText("Press C to spawn capsule", 10, 50, 20, DARKGRAY);
        DrawText(TextFormat("Spheres spawned: %d", bodies.size() - 1), 10, 70, 20, DARKGRAY);
        sDrawFreeCamReticle();
        EndDrawing();
    }

    // Cleanup
    physicsDestroyCharacter(player);
    physicsShutdown(physicsWorld);
    CloseWindow();
    return 0;
}