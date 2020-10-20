#pragma once

#include "PxPhysicsAPI.h"
#include "MathInterface.h"

/**
 * @brief An abstract actor object. Everything is an actor. 
 *  e.g. planes, articulation links ...
 * @note Do not construct an Actor directly, use derived classes
 * 
 */
class Actor
{
// API BEGIN
// API END
public:
    physx::PxActor* pxActor;
};

/**
 * @brief An abstract class for rigid actors. Every rigid body is a rigid actor.
 *  Rigid actors can be assign a collision group and a collision mask for 
 *  collision detection and reporting.
 * 
 */
class RigidActor :public Actor
{
// API BEGIN
public:
    /**
     * @brief Set up collision group and collision mask of a rigid body. 
     *  Relevant collisions involving this object will then be reported 
     *  via Scene::GetAllContactPairs function call.
     * 
     * @param collisionGroup An integer with only one bit set to 1. All
     *  other bits must be zero. The object will be added to the i^th
     *  collision group if the i^th bit is set to 1.
     * @param collisionMask An integer which specifies the set of collision
     *  groups with which collisions will be reported. If the i^th bit if
     *  the mask is set to 1, then collisions between self group and the i^th
     *  group will be reported via Scene::GetAllContactPairs function call.
     * 
     * @remark By using this collision system, there're only 32 available
     *  collision groups.
     * @remark See [Collision System](wiki/CollisionSystem.md) page for details.
     */
    void SetupCollisionFiltering(int collisionGroup, int collisionMask);
// API END
};

/**
 * @brief An abstract class for static rigid actors. A static rigid actor
 *  is a rigid actor which is also fixed in the scene.
 * 
 */
class RigidStatic :public RigidActor
{
// API BEGIN
// API END
};

/**
 * @brief An abstract class for rigid bodies. Rigid bodies can be applied forces
 * 
 */
class RigidBody :public RigidActor
{
// API BEGIN
public:
    /**
     * @brief Apply external force to the rigid body
     * 
     * @param force Cartesian force in global frame
     */
    void ApplyExternalForce(vec3 force) const;
// API END
};
