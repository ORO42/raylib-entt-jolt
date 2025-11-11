#pragma once

#include <Jolt/Jolt.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Body/BodyID.h>
#include <Jolt/Physics/Character/CharacterVirtual.h>
#include "raylib.h"
#include "raymath.h"

// Physics world state
struct PhysicsWorld
{
    JPH::PhysicsSystem *physicsSystem;
    JPH::BodyInterface *bodyInterface;
    JPH::TempAllocator *tempAllocator;
    JPH::JobSystem *jobSystem;
    void *broadPhaseLayerInterface;
    void *objectVsBroadphaseLayerFilter;
    void *objectVsObjectLayerFilter;
    float fixedTimestep;
    float accumulator;
};

// Character controller handle
struct PhysicsCharacter
{
    JPH::CharacterVirtual *character;
};

// Initialize the physics world
PhysicsWorld *physicsInit();

// Clean up physics world
void physicsShutdown(PhysicsWorld *world);

// Update physics (call this once per frame)
void physicsUpdate(PhysicsWorld *world, float deltaTime);

// Create a static box (floor, walls, etc.)
JPH::BodyID physicsCreateStaticBox(PhysicsWorld *world, Vector3 position, Vector3 halfExtents);

// Create a dynamic box
JPH::BodyID physicsCreateDynamicBox(PhysicsWorld *world, Vector3 position, Vector3 halfExtents);

// Create a dynamic sphere
JPH::BodyID physicsCreateDynamicSphere(PhysicsWorld *world, Vector3 position, float radius);

// Get position of a body
Vector3 physicsGetPosition(PhysicsWorld *world, JPH::BodyID bodyId);

// Get rotation of a body (as quaternion)
Quaternion physicsGetRotation(PhysicsWorld *world, JPH::BodyID bodyId);

float physicsGetRadius(PhysicsWorld *world, JPH::BodyID bodyId);

Vector3 physicsGetHalfExtents(PhysicsWorld *world, JPH::BodyID bodyId);

// Set linear velocity
void physicsSetVelocity(PhysicsWorld *world, JPH::BodyID bodyId, Vector3 velocity);

// Apply impulse to a body
void physicsApplyImpulse(PhysicsWorld *world, JPH::BodyID bodyId, Vector3 impulse);

// Check if body is active
bool physicsIsActive(PhysicsWorld *world, JPH::BodyID bodyId);

// Remove and destroy a body
void physicsDestroyBody(PhysicsWorld *world, JPH::BodyID bodyId);

// --- Character Controller Functions ---

// Create a character controller (capsule shape)
PhysicsCharacter *physicsCreateCharacter(PhysicsWorld *world, Vector3 position, float radius, float halfHeight);

// Destroy character controller
void physicsDestroyCharacter(PhysicsCharacter *character);

// Update character controller (call after physicsUpdate)
void physicsUpdateCharacter(PhysicsWorld *world, PhysicsCharacter *character, float deltaTime);

// Set character linear velocity
void physicsCharacterSetVelocity(PhysicsCharacter *character, Vector3 velocity);

// Get character linear velocity
Vector3 physicsCharacterGetVelocity(PhysicsCharacter *character);

// Get character position
Vector3 physicsCharacterGetPosition(PhysicsCharacter *character);

// Set character position
void physicsCharacterSetPosition(PhysicsCharacter *character, Vector3 position);

// Check if character is on ground
bool physicsCharacterIsOnGround(PhysicsCharacter *character);

// Get character up direction
Vector3 physicsCharacterGetUp(PhysicsCharacter *character);

// Set character up direction (for gravity)
void physicsCharacterSetUp(PhysicsCharacter *character, Vector3 up);

// Get character shape radius
float physicsCharacterGetRadius(PhysicsCharacter *character);

// Get character shape half height (cylinder portion only, not including spherical caps)
float physicsCharacterGetHalfHeight(PhysicsCharacter *character);

// Raycast hit result
struct PhysicsRaycastHit
{
    bool hit;           // Did the ray hit something?
    JPH::BodyID bodyId; // ID of the body that was hit
    Vector3 position;   // World position of hit point
    Vector3 normal;     // Surface normal at hit point
    float fraction;     // Fraction along ray where hit occurred (0.0 to 1.0)
    float distance;     // Distance from ray origin to hit point
};

// Cast a ray and return the closest hit
// origin: Start point of the ray
// direction: Direction of the ray (will be normalized)
// maxDistance: Maximum distance to check
PhysicsRaycastHit physicsRaycast(PhysicsWorld *world, Vector3 origin, Vector3 direction, float maxDistance);