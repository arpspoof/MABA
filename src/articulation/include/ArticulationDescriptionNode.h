#pragma once

#include "PxPhysicsAPI.h"
#include "LinkBody.h"
#include "Articulation.h"

#include <string>
#include <vector>
#include <cassert>

/**
 * @internal
 * @brief Wrapper class for physx::PxArticulationAxis. This specifies the
 * 	rotational axis for a revolute joint
 * 
 */
class ArticulationAxis
{
public:
    enum Enum { X, Y, Z };
    Enum axis;
    ArticulationAxis(Enum axis) :axis(axis) {}
    operator physx::PxArticulationAxis::Enum() const {
        switch (axis) {
            case X: return physx::PxArticulationAxis::eTWIST;
            case Y: return physx::PxArticulationAxis::eSWING1;
            case Z: return physx::PxArticulationAxis::eSWING2;
            default:
                assert(false);
                return physx::PxArticulationAxis::eTWIST;
        }
    }
};

class ArticulationDescriptionNode {
public:
    std::string linkName;
    std::string jointName;
    LinkBody *body;
    physx::PxVec3 posOffsetLinkToInboundJoint;
    physx::PxVec3 posOffsetJointToParentJoint;
    ArticulationDescriptionNode *parent;
    std::vector<ArticulationDescriptionNode*> children;
protected:
    ArticulationDescriptionNode(std::string linkName, std::string jointName, LinkBody *body,
        physx::PxVec3 posOffsetLinkToInboundJoint, physx::PxVec3 posOffsetJointToParentJoint);
    virtual Joint* CreateJoint(Articulation& ar, Link *link,
        physx::PxTransform parentPose,
        physx::PxTransform childPose) const = 0;
public:
    Link* CreateLink(Articulation& ar, Link *parentLink, physx::PxQuat frameTransform,
        physx::PxVec3 parentJointPos, physx::PxVec3 parentLinkPos) const;
    virtual ~ArticulationDescriptionNode() {}
};

class NULLDescriptionNode : public ArticulationDescriptionNode {
public:
    NULLDescriptionNode(std::string linkName, LinkBody *body);
    Joint* CreateJoint(Articulation&, Link *,
        physx::PxTransform,
        physx::PxTransform) const override;
};

class FixedDescriptionNode : public ArticulationDescriptionNode {
public:
    FixedDescriptionNode(std::string linkName, std::string jointName, LinkBody *body,
        physx::PxVec3 posOffsetLinkToInboundJoint, physx::PxVec3 posOffsetJointToParentJoint);
    Joint* CreateJoint(Articulation& ar, Link *link,
        physx::PxTransform parentPose,
        physx::PxTransform childPose) const override;
};

class SpericalDescriptionNode : public ArticulationDescriptionNode {
public:
    SpericalDescriptionNode(std::string linkName, std::string jointName, LinkBody *body,
        physx::PxVec3 posOffsetLinkToInboundJoint, physx::PxVec3 posOffsetJointToParentJoint);
    Joint* CreateJoint(Articulation& ar, Link *link,
        physx::PxTransform parentPose,
        physx::PxTransform childPose) const override;
};

class RevoluteDescriptionNode : public ArticulationDescriptionNode {
    physx::PxArticulationAxis::Enum axis;
public:
    RevoluteDescriptionNode(std::string linkName, std::string jointName, LinkBody *body,
        physx::PxArticulationAxis::Enum axis,
        physx::PxVec3 posOffsetLinkToInboundJoint, physx::PxVec3 posOffsetJointToParentJoint);
    Joint* CreateJoint(Articulation& ar, Link *link,
        physx::PxTransform parentPose,
        physx::PxTransform childPose) const override;
};
