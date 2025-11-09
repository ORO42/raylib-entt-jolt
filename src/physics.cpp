#include "physics.hpp"

#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>

#include <iostream>
#include <thread>

using namespace JPH;

// Layer definitions
namespace Layers
{
    static constexpr ObjectLayer NON_MOVING = 0;
    static constexpr ObjectLayer MOVING = 1;
    static constexpr ObjectLayer NUM_LAYERS = 2;
};

// Broadphase layers
namespace BroadPhaseLayers
{
    static constexpr BroadPhaseLayer NON_MOVING(0);
    static constexpr BroadPhaseLayer MOVING(1);
    static constexpr uint NUM_LAYERS(2);
};

// Simple collision filter implementations
class ObjectLayerPairFilterImpl : public ObjectLayerPairFilter
{
public:
    virtual bool ShouldCollide(ObjectLayer inObject1, ObjectLayer inObject2) const override
    {
        switch (inObject1)
        {
        case Layers::NON_MOVING:
            return inObject2 == Layers::MOVING;
        case Layers::MOVING:
            return true;
        default:
            return false;
        }
    }
};

class BPLayerInterfaceImpl final : public BroadPhaseLayerInterface
{
public:
    BPLayerInterfaceImpl()
    {
        mObjectToBroadPhase[Layers::NON_MOVING] = BroadPhaseLayers::NON_MOVING;
        mObjectToBroadPhase[Layers::MOVING] = BroadPhaseLayers::MOVING;
    }

    virtual uint GetNumBroadPhaseLayers() const override
    {
        return BroadPhaseLayers::NUM_LAYERS;
    }

    virtual BroadPhaseLayer GetBroadPhaseLayer(ObjectLayer inLayer) const override
    {
        return mObjectToBroadPhase[inLayer];
    }

#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
    virtual const char *GetBroadPhaseLayerName(BroadPhaseLayer inLayer) const override
    {
        switch ((BroadPhaseLayer::Type)inLayer)
        {
        case (BroadPhaseLayer::Type)BroadPhaseLayers::NON_MOVING:
            return "NON_MOVING";
        case (BroadPhaseLayer::Type)BroadPhaseLayers::MOVING:
            return "MOVING";
        default:
            return "INVALID";
        }
    }
#endif

private:
    BroadPhaseLayer mObjectToBroadPhase[Layers::NUM_LAYERS];
};

class ObjectVsBroadPhaseLayerFilterImpl : public ObjectVsBroadPhaseLayerFilter
{
public:
    virtual bool ShouldCollide(ObjectLayer inLayer1, BroadPhaseLayer inLayer2) const override
    {
        switch (inLayer1)
        {
        case Layers::NON_MOVING:
            return inLayer2 == BroadPhaseLayers::MOVING;
        case Layers::MOVING:
            return true;
        default:
            return false;
        }
    }
};

// Trace callback
static void TraceImpl(const char *inFMT, ...)
{
    va_list list;
    va_start(list, inFMT);
    char buffer[1024];
    vsnprintf(buffer, sizeof(buffer), inFMT, list);
    va_end(list);
    std::cout << buffer << std::endl;
}

PhysicsWorld *physicsInit()
{
    // Register Jolt allocator and callbacks
    RegisterDefaultAllocator();
    Trace = TraceImpl;

    // Create factory and register types
    Factory::sInstance = new Factory();
    RegisterTypes();

    // Allocate world struct
    PhysicsWorld *world = new PhysicsWorld();

    // Create temp allocator (10 MB)
    world->tempAllocator = new TempAllocatorImpl(10 * 1024 * 1024);

    // Create job system
    world->jobSystem = new JobSystemThreadPool(cMaxPhysicsJobs, cMaxPhysicsBarriers,
                                               std::thread::hardware_concurrency() - 1);

    // Physics system limits
    const uint cMaxBodies = 10240;
    const uint cNumBodyMutexes = 0;
    const uint cMaxBodyPairs = 65536;
    const uint cMaxContactConstraints = 10240;

    // Create collision filters (must persist!)
    world->broadPhaseLayerInterface = new BPLayerInterfaceImpl();
    world->objectVsBroadphaseLayerFilter = new ObjectVsBroadPhaseLayerFilterImpl();
    world->objectVsObjectLayerFilter = new ObjectLayerPairFilterImpl();

    // Create physics system
    world->physicsSystem = new PhysicsSystem();
    world->physicsSystem->Init(
        cMaxBodies, cNumBodyMutexes, cMaxBodyPairs, cMaxContactConstraints,
        *static_cast<BPLayerInterfaceImpl *>(world->broadPhaseLayerInterface),
        *static_cast<ObjectVsBroadPhaseLayerFilterImpl *>(world->objectVsBroadphaseLayerFilter),
        *static_cast<ObjectLayerPairFilterImpl *>(world->objectVsObjectLayerFilter));

    world->bodyInterface = &world->physicsSystem->GetBodyInterface();
    world->fixedTimestep = 1.0f / 60.0f;
    world->accumulator = 0.0f;

    return world;
}

void physicsShutdown(PhysicsWorld *world)
{
    if (!world)
        return;

    delete world->physicsSystem;
    delete static_cast<ObjectLayerPairFilterImpl *>(world->objectVsObjectLayerFilter);
    delete static_cast<ObjectVsBroadPhaseLayerFilterImpl *>(world->objectVsBroadphaseLayerFilter);
    delete static_cast<BPLayerInterfaceImpl *>(world->broadPhaseLayerInterface);
    delete static_cast<JobSystemThreadPool *>(world->jobSystem);
    delete static_cast<TempAllocatorImpl *>(world->tempAllocator);

    UnregisterTypes();
    delete Factory::sInstance;
    Factory::sInstance = nullptr;

    delete world;
}

void physicsUpdate(PhysicsWorld *world, float deltaTime)
{
    // Fixed timestep accumulator
    world->accumulator += deltaTime;

    while (world->accumulator >= world->fixedTimestep)
    {
        world->physicsSystem->Update(world->fixedTimestep, 1,
                                     static_cast<TempAllocator *>(world->tempAllocator),
                                     static_cast<JobSystem *>(world->jobSystem));
        world->accumulator -= world->fixedTimestep;
    }
}

BodyID physicsCreateStaticBox(PhysicsWorld *world, Vector3 position, Vector3 halfExtents)
{
    BoxShapeSettings shapeSettings(Vec3(halfExtents.x, halfExtents.y, halfExtents.z));
    ShapeSettings::ShapeResult shapeResult = shapeSettings.Create();
    ShapeRefC shape = shapeResult.Get();

    BodyCreationSettings bodySettings(shape, RVec3(position.x, position.y, position.z),
                                      Quat::sIdentity(), EMotionType::Static, Layers::NON_MOVING);

    Body *body = world->bodyInterface->CreateBody(bodySettings);
    world->bodyInterface->AddBody(body->GetID(), EActivation::DontActivate);

    return body->GetID();
}

BodyID physicsCreateDynamicBox(PhysicsWorld *world, Vector3 position, Vector3 halfExtents)
{
    BoxShapeSettings shapeSettings(Vec3(halfExtents.x, halfExtents.y, halfExtents.z));
    ShapeSettings::ShapeResult shapeResult = shapeSettings.Create();
    ShapeRefC shape = shapeResult.Get();

    BodyCreationSettings bodySettings(shape, RVec3(position.x, position.y, position.z),
                                      Quat::sIdentity(), EMotionType::Dynamic, Layers::MOVING);

    BodyID bodyId = world->bodyInterface->CreateAndAddBody(bodySettings, EActivation::Activate);
    return bodyId;
}

BodyID physicsCreateDynamicSphere(PhysicsWorld *world, Vector3 position, float radius)
{
    SphereShapeSettings shapeSettings(radius);
    ShapeSettings::ShapeResult shapeResult = shapeSettings.Create();
    ShapeRefC shape = shapeResult.Get();

    BodyCreationSettings bodySettings(shape, RVec3(position.x, position.y, position.z),
                                      Quat::sIdentity(), EMotionType::Dynamic, Layers::MOVING);

    BodyID bodyId = world->bodyInterface->CreateAndAddBody(bodySettings, EActivation::Activate);
    return bodyId;
}

Vector3 physicsGetPosition(PhysicsWorld *world, BodyID bodyId)
{
    RVec3 pos = world->bodyInterface->GetCenterOfMassPosition(bodyId);
    return {(float)pos.GetX(), (float)pos.GetY(), (float)pos.GetZ()};
}

Quaternion physicsGetRotation(PhysicsWorld *world, BodyID bodyId)
{
    Quat rot = world->bodyInterface->GetRotation(bodyId);
    return {rot.GetX(), rot.GetY(), rot.GetZ(), rot.GetW()};
}

void physicsSetVelocity(PhysicsWorld *world, BodyID bodyId, Vector3 velocity)
{
    world->bodyInterface->SetLinearVelocity(bodyId, Vec3(velocity.x, velocity.y, velocity.z));
}

void physicsApplyImpulse(PhysicsWorld *world, BodyID bodyId, Vector3 impulse)
{
    world->bodyInterface->AddImpulse(bodyId, Vec3(impulse.x, impulse.y, impulse.z));
}

bool physicsIsActive(PhysicsWorld *world, BodyID bodyId)
{
    return world->bodyInterface->IsActive(bodyId);
}

void physicsDestroyBody(PhysicsWorld *world, BodyID bodyId)
{
    world->bodyInterface->RemoveBody(bodyId);
    world->bodyInterface->DestroyBody(bodyId);
}