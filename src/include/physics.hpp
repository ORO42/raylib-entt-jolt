#pragma once

#include <Jolt/Jolt.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Body/BodyID.h>
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

// Create a dynamic capsule
JPH::BodyID physicsCreateDynamicCapsule(PhysicsWorld *world, Vector3 position, float radius, float height);

// Get position of a body
Vector3 physicsGetPosition(PhysicsWorld *world, JPH::BodyID bodyId);

// Get rotation of a body (as quaternion)
Quaternion physicsGetRotation(PhysicsWorld *world, JPH::BodyID bodyId);

// Set linear velocity
void physicsSetVelocity(PhysicsWorld *world, JPH::BodyID bodyId, Vector3 velocity);

// Apply impulse to a body
void physicsApplyImpulse(PhysicsWorld *world, JPH::BodyID bodyId, Vector3 impulse);

// Check if body is active
bool physicsIsActive(PhysicsWorld *world, JPH::BodyID bodyId);

// Remove and destroy a body
void physicsDestroyBody(PhysicsWorld *world, JPH::BodyID bodyId);