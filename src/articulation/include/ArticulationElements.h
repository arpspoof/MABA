#pragma once

#include "PxPhysicsAPI.h"
#include "LinkBody.h"
#include "Actor.h"

#include <vector>
#include <string>

class Link;

/**
 * @brief Articulation joint
 * 
 */
class Joint {
public:
// API BEGIN
    /**
     * @brief Pointer to the parent Link
     * @note A joint always has a parent Link
     * 
     */
    Link *parentLink;
    /**
     * @brief Pointer to the child Link
     * @note A joint always has a child Link
     * 
     */
    Link *childLink;
    /**
     * @brief Name of the joint
     * 
     */
    std::string name;
    /**
     * @brief id of the joint in articulation
     * 
     */
    int id;
    /**
     * @brief Number of degrees of freedom
     * 
     */
    int nDof;
    /**
     * @brief Start index of this joint in any array containing all
     * 	joint information. This is suitable for any type of joint information
     * 	including generalized joint positions, velocities, forces, or 
     * 	kp, kd and force limit parameters, etc.
     * @remark Suppose we have 4 joints in total, say ABCD. A and C have 1 
     * 	degree of freedom each while B and D have 3 degrees of freedom each.
     * 	Suppose cacheIndex for A,B,C,D is 0,1,4,5 respectively, and suppose
     * 	we have an array of size 8 (total degrees of freedom) containing joint
     * 	velocities, then entry [0], [1-3], [4], [5-7] will correspond to the
     * 	four joints respectively. 
     * 
     */
    int cacheIndex;
    Joint() {} // For API only
// API END
    physx::PxArticulationJointReducedCoordinate *joint;
    physx::PxVec3 globalPositionOffset;
    physx::PxVec3 posOffsetJointToParentJoint;
protected:
    Joint(Link *link, physx::PxTransform parentPose, physx::PxTransform childPose);
};

/**
 * @brief Articulation link
 * 
 */
class Link :public RigidBody {
public:
// API BEGIN
    /**
     * @brief Pointer to parent Link. 
     * @note This is set to nullptr if there is no parent.
     * 
     */
    Link *parentLink;
    /**
     * @brief Pointer to inbound Joint.
     * @note This is set to nullptr if there is no inbound Joint (base).
     * 
     */
    Joint *inboundJoint;
    /**
     * @brief Name of the link
     * 
     */
    std::string name;
    /**
     * @brief Set of child links. Each element is a pointer to a child Link.
     * 
     */
    std::vector<Link*> childLinks;
    Link() {} // For API only
// API END
    LinkBody *body;
    physx::PxArticulationLink *link;
    physx::PxVec3 globalPositionOffset;
    physx::PxVec3 posOffsetLinkToInboundJoint;
    Link(physx::PxArticulationReducedCoordinate* pxArticulation, Link *parent, 
        physx::PxTransform transform, LinkBody *body);
    ~Link();
};

class FixedJoint : public Joint {
public:
    FixedJoint(Link *link, physx::PxTransform parentPose, physx::PxTransform childPose);
};

class SphericalJoint : public Joint {
public:
    SphericalJoint(Link *link, physx::PxTransform parentPose, physx::PxTransform childPose);
};

class RevoluteJoint : public Joint {
public:
    physx::PxArticulationAxis::Enum axis;
    RevoluteJoint(Link *link, physx::PxArticulationAxis::Enum axis,
        physx::PxTransform parentPose, physx::PxTransform childPose);
};
