#include <reactphysics3d/reactphysics3d.h> // Must come before Raylib to avoid type collisions
#include "raylib.h"
#include "raymath.h"
#include <entt/entt.hpp>

struct FreeCam
{
    Camera3D camera;
    float yaw;
    float pitch;
    float moveSpeed;
    float mouseSensitivity;
    bool isCurrent = true;
};

struct FPSController
{
    // Camera3D camera;
    const float moveSpeedMax = 10.0f;
    float moveSpeedCurr = 0.0f;
    const float panSpeedMax = 10.0f;
    float panSpeedCurr = 0.0f;
    const float rotationSpeedMax = 0.07f;
    float rotationSpeedCurr = 0.0f;
};

struct RigidBodyPointer
{
    reactphysics3d::RigidBody *body = nullptr;
};

struct TagUnitRifleman
{
};

struct PhysicsState
{
    reactphysics3d::PhysicsCommon physicsCommon;
    reactphysics3d::PhysicsWorld *physicsWorld = nullptr;
    const float physicsTimeStep = 1.0f / 60.0f;
    double physicsAccumulator = 0.0;
    float physicsAlpha = 0.0f;
    bool physicsUpdated = false;

    PhysicsState()
    {
        physicsWorld = physicsCommon.createPhysicsWorld();

        physicsWorld->setIsDebugRenderingEnabled(true);
        reactphysics3d::DebugRenderer &debugRenderer = physicsWorld->getDebugRenderer();
        debugRenderer.setIsDebugItemDisplayed(reactphysics3d::DebugRenderer::DebugItem::COLLISION_SHAPE, true);
        debugRenderer.setIsDebugItemDisplayed(reactphysics3d::DebugRenderer::DebugItem::COLLIDER_BROADPHASE_AABB, true);
        debugRenderer.setIsDebugItemDisplayed(reactphysics3d::DebugRenderer::DebugItem::COLLIDER_AABB, true);
    }

    void sUpdatePhysics()
    {
        physicsAccumulator += GetFrameTime();
        physicsUpdated = false;
        // Step physics in fixed increments
        while (physicsAccumulator >= physicsTimeStep)
        {
            physicsWorld->update(physicsTimeStep);
            physicsAccumulator -= physicsTimeStep;
            physicsUpdated = true;
        }
        // Since physics is updated at discrete intervals (e.g. 60Hz) but rendering happens at any frame rate, can interpolate object transforms for smoother motion
        // https://www.reactphysics3d.com/documentation/index.html#rigidbody
        physicsAlpha = physicsAccumulator / physicsTimeStep;
        // E.g. interpolate between previous and current physics states
        // Vector3 renderPos = Lerp(previousPos, currentPos, alpha);

        // Get the updated transform of the body
        // Transform currTransform = body->getTransform();
        // Compute the interpolated transform of the rigid body
        // Transform interpolatedTransform = Transform::interpolateTransforms(prevTransform, currTransform, factor);
        // Now you can render your body using the interpolated transform here
        // Update the previous transform
        // prevTransform = currTranform;
    }

    void sDrawPhysicsDebugShapes()
    {
        reactphysics3d::DebugRenderer &debugRenderer = physicsWorld->getDebugRenderer();

        if (physicsUpdated)
        {
            const auto nbLines = debugRenderer.getNbLines();
            const auto *lines = debugRenderer.getLinesArray();

            for (reactphysics3d::uint32 i = 0; i < nbLines; i++)
            {
                const auto &p1 = lines[i].point1;
                const auto &p2 = lines[i].point2;

                unsigned int c = lines[i].color1;
                unsigned char r = (c >> 24) & 0xFF;
                unsigned char g = (c >> 16) & 0xFF;
                unsigned char b = (c >> 8) & 0xFF;
                unsigned char a = (c) & 0xFF;

                DrawLine3D({p1.x, p1.y, p1.z}, {p2.x, p2.y, p2.z}, {r, g, b, a});
            }
        }
    }

    ~PhysicsState()
    {
        physicsCommon.destroyPhysicsWorld(physicsWorld);
    }
};

struct GameState
{
    entt::registry registry;
    FreeCam freeCam;
    PhysicsState physicsState;

    entt::entity controlledUnit = entt::null;

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

    ~GameState()
    {
    }
};

class MyRaycastCallbackClass : public reactphysics3d::RaycastCallback
{
public:
    // User-supplied callback function
    std::function<void(const reactphysics3d::RaycastInfo &)> onHit;

    // Constructor that takes a callback
    MyRaycastCallbackClass(std::function<void(const reactphysics3d::RaycastInfo &)> callback)
        : onHit(callback) {}

    // Called by ReactPhysics3D for each hit
    virtual reactphysics3d::decimal notifyRaycastHit(const reactphysics3d::RaycastInfo &info) override
    {
        if (onHit)
        {
            onHit(info);
        }

        // SEE REACTPHYSICS3D DOCS - can return 1.0, 0.0, or hit fraction
        return reactphysics3d::decimal(0.0);
    }
};

void PrintSystemTime()
{
    // Get current time as a time_point
    auto now = std::chrono::system_clock::now();

    // Convert to time_t to extract hours, minutes, seconds
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm localTime = *std::localtime(&t);

    // Extract milliseconds
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                  now.time_since_epoch()) %
              1000;

    // Print in hh::mm::ss::ms format
    std::cout << std::setfill('0')
              << std::setw(2) << localTime.tm_hour << "::"
              << std::setw(2) << localTime.tm_min << "::"
              << std::setw(2) << localTime.tm_sec << "::"
              << std::setw(3) << ms.count()
              << std::endl;
}

void PrintRaycastHitInfo(const reactphysics3d::RaycastInfo &info)
{
    std::cout << "=== Raycast Hit ===" << std::endl;

    PrintSystemTime();

    // World-space hit point
    std::cout << "Hit point     : ("
              << info.worldPoint.x << ", "
              << info.worldPoint.y << ", "
              << info.worldPoint.z << ")" << std::endl;

    // World-space surface normal at hit
    std::cout << "World normal  : ("
              << info.worldNormal.x << ", "
              << info.worldNormal.y << ", "
              << info.worldNormal.z << ")" << std::endl;

    // Fraction along the ray
    std::cout << "Hit fraction  : " << info.hitFraction << std::endl;

    // Pointer to rigid body hit
    if (info.body)
        std::cout << "Rigid body    : " << info.body << std::endl;
    else
        std::cout << "Rigid body    : null" << std::endl;

    // Pointer to collider hit
    if (info.collider)
        std::cout << "Collider      : " << info.collider << std::endl;
    else
        std::cout << "Collider      : null" << std::endl;

    std::cout << "==================" << std::endl;
}

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

void sDrawFreeCamReticle()
{
    int windowWidth = GetScreenWidth();
    int windowHeight = GetScreenHeight();
    Vector2 center = {windowWidth * 0.5f, windowHeight * 0.5f};
    float lineLength = 10.0f;
    float halfLength = lineLength * 0.5f;
    // Draw horizontal line
    DrawLineEx({center.x - halfLength, center.y}, {center.x + halfLength, center.y}, 1.0f, BLACK);
    // Draw vertical line
    DrawLineEx({center.x, center.y - halfLength}, {center.x, center.y + halfLength}, 1.0f, BLACK);
}

void sFreeCamCenterRay(GameState &gameState,
                       std::function<void(const reactphysics3d::RaycastInfo &)> onHit)
{
    if (gameState.freeCam.isCurrent && IsMouseButtonPressed(MOUSE_BUTTON_LEFT))
    {
        // Get the center of the screen
        Vector2 screenCenter = {GetScreenWidth() / 2.0f, GetScreenHeight() / 2.0f};

        // Convert Raylib ray to ReactPhysics3D
        Ray screenRay = GetScreenToWorldRay(screenCenter, gameState.freeCam.camera);

        reactphysics3d::Vector3 rayStart(screenRay.position.x, screenRay.position.y, screenRay.position.z);
        reactphysics3d::Vector3 rayEnd(
            screenRay.position.x + screenRay.direction.x * 1000.0f,
            screenRay.position.y + screenRay.direction.y * 1000.0f,
            screenRay.position.z + screenRay.direction.z * 1000.0f);

        reactphysics3d::Ray ray(rayStart, rayEnd);

        // Construct callback using the function passed in
        MyRaycastCallbackClass callback(onHit);

        // Perform the raycast
        gameState.physicsState.physicsWorld->raycast(ray, &callback);
    }
}

entt::entity CreateUnitRifleman(GameState &gameState, reactphysics3d::Vector3 position)
{
    // --- Create a transform for the rigid body ---
    reactphysics3d::Quaternion orientation = reactphysics3d::Quaternion::identity();
    reactphysics3d::Transform transform(position, orientation);

    // --- Create rigid body ---
    reactphysics3d::RigidBody *body = gameState.physicsState.physicsWorld->createRigidBody(transform);
    body->setType(reactphysics3d::BodyType::DYNAMIC);
    body->setIsDebugEnabled(true);

    // --- Add collider ---
    reactphysics3d::CapsuleShape *capsuleShape = gameState.physicsState.physicsCommon.createCapsuleShape(0.25f, 0.5f);
    reactphysics3d::Transform colliderTransform = reactphysics3d::Transform::identity();
    reactphysics3d::Collider *collider = body->addCollider(capsuleShape, colliderTransform);

    // --- Create entt entity ---
    entt::entity entity = gameState.registry.create();
    // TODO REMOVE (DEBUG ONLY)
    gameState.controlledUnit = entity;

    // --- Attach rigid body pointer to entity ---
    gameState.registry.emplace<RigidBodyPointer>(entity, body);

    FPSController fpsController;
    gameState.registry.emplace<FPSController>(entity, fpsController);

    // --- Store entity pointer in rigid body user data ---
    // Note: userData is a void*, so we cast the entity to a pointer-sized integer
    body->setUserData(reinterpret_cast<void *>(static_cast<uintptr_t>(entity)));

    return entity;
}

void sFPSController(GameState &gameState)
{
    if (gameState.controlledUnit == entt::null)
    {
        return;
    }

    RigidBodyPointer &rigidBodyComp = gameState.registry.get<RigidBodyPointer>(gameState.controlledUnit);
    FPSController &fpsControllerComp = gameState.registry.get<FPSController>(gameState.controlledUnit);

    reactphysics3d::Vector3 moveForce(0.0f, 0.0f, 0.0f);

    if (IsKeyDown(KEY_W))
        moveForce.z -= 10.0f;
    if (IsKeyDown(KEY_S))
        moveForce.z += 10.0f;
    if (IsKeyDown(KEY_A))
        moveForce.x -= 10.0f;
    if (IsKeyDown(KEY_D))
        moveForce.x += 10.0f;

    // Apply in local space so it respects player rotation
    rigidBodyComp.body->applyLocalForceAtCenterOfMass(moveForce);

    if (IsKeyPressed(KEY_J))
    {
        reactphysics3d::Vector3 jumpForce(0.0f, 50.0f, 0.0f);
        rigidBodyComp.body->applyLocalForceAtCenterOfMass(jumpForce);
        std::cout << "Jump: " << static_cast<std::uint32_t>(gameState.controlledUnit) << std::endl;
    }

    // reactphysics3d::Quaternion orientation = reactphysics3d::Quaternion::fromEulerAngles(0.0f, fpsControllerComp.yaw, 0.0f);
    // rigidBodyComp.body->setOrientation(orientation);
}

int main()
{
    SetConfigFlags(FLAG_VSYNC_HINT | FLAG_WINDOW_HIGHDPI | FLAG_WINDOW_RESIZABLE);
    InitWindow(800, 450, "emergent-command");
    SetTargetFPS(60); // TODO changing this to anything other than physics timestep causes visual jittering. Need to employ physicsAlpha
    SetMousePosition(GetScreenWidth() / 2.0f, GetScreenHeight() / 2.0f);
    DisableCursor();

    GameState gameState;

    {
        // Create a static ground box

        // Define position and orientation
        reactphysics3d::Vector3 position(0.0f, 0.0f, 0.0f);
        reactphysics3d::Quaternion orientation = reactphysics3d::Quaternion::identity();
        reactphysics3d::Transform transform(position, orientation);

        // Create rigid body (static by setting its type)
        reactphysics3d::RigidBody *groundBody = gameState.physicsState.physicsWorld->createRigidBody(transform);
        groundBody->setType(reactphysics3d::BodyType::STATIC);
        groundBody->setIsDebugEnabled(true);

        // Create box shape (ReactPhysics3D uses half-extents)
        reactphysics3d::Vector3 halfExtents(5.0f, 0.25f, 5.0f); // 10x0.5x10 total size
        reactphysics3d::BoxShape *boxShape = gameState.physicsState.physicsCommon.createBoxShape(halfExtents);

        // Collider transform relative to body (none in this case)
        reactphysics3d::Transform colliderTransform = reactphysics3d::Transform::identity();

        // Attach collider
        reactphysics3d::Collider *collider = groundBody->addCollider(boxShape, colliderTransform);
    }

    {
        // Create a dynamic sphere

        // Define position and orientation
        reactphysics3d::Vector3 position(0.0f, 10.0f, 0.0f); // 10 meters above origin
        reactphysics3d::Quaternion orientation = reactphysics3d::Quaternion::identity();
        reactphysics3d::Transform transform(position, orientation);

        // Create rigid body (dynamic by setting its type)
        reactphysics3d::RigidBody *sphereBody = gameState.physicsState.physicsWorld->createRigidBody(transform);
        sphereBody->setType(reactphysics3d::BodyType::DYNAMIC);
        sphereBody->setIsDebugEnabled(true);

        // Create sphere shape
        float radius = 1.0f; // 1 meter radius
        reactphysics3d::SphereShape *sphereShape = gameState.physicsState.physicsCommon.createSphereShape(radius);

        // Collider transform relative to body (identity)
        reactphysics3d::Transform colliderTransform = reactphysics3d::Transform::identity();

        // Attach collider
        reactphysics3d::Collider *collider = sphereBody->addCollider(sphereShape, colliderTransform);

        // Need to make it not bouncy by accessing Material (https://www.reactphysics3d.com/documentation/index.html#rigidbody)
    }

    CreateUnitRifleman(gameState, {2.5f, 10.0f, 2.5f});

    while (!WindowShouldClose())
    {
        // Update
        sFPSController(gameState);
        gameState.physicsState.sUpdatePhysics();
        sControlFreecam(gameState);
        sFreeCamCenterRay(gameState, PrintRaycastHitInfo);

        // Draw
        BeginDrawing();
        ClearBackground(RAYWHITE);

        BeginMode3D(gameState.freeCam.camera);
        DrawGrid(10, 1.0f);
        gameState.physicsState.sDrawPhysicsDebugShapes();
        EndMode3D();

        DrawFPS(10, 10);
        sDrawFreeCamReticle();
        EndDrawing();
    }

    CloseWindow();
    return 0;
}