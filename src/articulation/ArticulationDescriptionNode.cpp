#include "ArticulationDescriptionNode.h"

using namespace physx;
using namespace std;

ArticulationDescriptionNode::ArticulationDescriptionNode(string linkName, string jointName, LinkBody *body,
    PxVec3 posOffsetLinkToInboundJoint, PxVec3 posOffsetJointToParentJoint)
    :linkName(linkName), jointName(jointName), body(body),
    posOffsetLinkToInboundJoint(posOffsetLinkToInboundJoint),
    posOffsetJointToParentJoint(posOffsetJointToParentJoint)
{
}

Link* ArticulationDescriptionNode::CreateLink(Articulation& ar, Link *parentLink, PxQuat frameTransform,
    PxVec3 parentJointPos, PxVec3 parentLinkPos) const 
{
    PxVec3 jointPos = parentJointPos + posOffsetJointToParentJoint;
    PxVec3 linkPos = jointPos + posOffsetLinkToInboundJoint;
    Link *link = ar.AddLink(linkName, parentLink, PxTransform(linkPos, frameTransform), body);
    link->globalPositionOffset = linkPos;
    link->posOffsetLinkToInboundJoint = posOffsetLinkToInboundJoint;
    if (parentLink) {
        Joint *joint = CreateJoint(ar, link,
            PxTransform(frameTransform.rotateInv(jointPos - parentLinkPos)),
            PxTransform(frameTransform.rotateInv(jointPos - linkPos))
        );
        link->inboundJoint = joint;
        joint->globalPositionOffset = jointPos;
        joint->posOffsetJointToParentJoint = posOffsetJointToParentJoint;
        joint->childLink = link;
        joint->parentLink = parentLink;
    }
    return link;
}

NULLDescriptionNode::NULLDescriptionNode(string linkName, LinkBody *body)
    : ArticulationDescriptionNode(linkName, "", body,
        PxVec3(0, 0, 0), PxVec3(0, 0, 0)) 
{
}

Joint* NULLDescriptionNode::CreateJoint(Articulation&, Link *,
    PxTransform,
    PxTransform) const 
{
    return NULL;
}

FixedDescriptionNode::FixedDescriptionNode(string linkName, string jointName, LinkBody *body,
    PxVec3 posOffsetLinkToInboundJoint, PxVec3 posOffsetJointToParentJoint)
    : ArticulationDescriptionNode(linkName, jointName, body,
        posOffsetLinkToInboundJoint, posOffsetJointToParentJoint) 
{
}

Joint* FixedDescriptionNode::CreateJoint(Articulation& ar, Link *link,
    PxTransform parentPose,
    PxTransform childPose) const 
{
    return ar.AddFixedJoint(jointName, link, parentPose, childPose);
}

SpericalDescriptionNode::SpericalDescriptionNode(string linkName, string jointName, LinkBody *body,
    PxVec3 posOffsetLinkToInboundJoint, PxVec3 posOffsetJointToParentJoint)
    : ArticulationDescriptionNode(linkName, jointName, body,
        posOffsetLinkToInboundJoint, posOffsetJointToParentJoint) 
{
}

Joint* SpericalDescriptionNode::CreateJoint(Articulation& ar, Link *link,
    PxTransform parentPose,
    PxTransform childPose) const 
{
    return ar.AddSpericalJoint(jointName, link, parentPose, childPose);
}

RevoluteDescriptionNode::RevoluteDescriptionNode(string linkName, string jointName, LinkBody *body,
    PxArticulationAxis::Enum axis,
    PxVec3 posOffsetLinkToInboundJoint, PxVec3 posOffsetJointToParentJoint)
    : ArticulationDescriptionNode(linkName, jointName, body,
        posOffsetLinkToInboundJoint, posOffsetJointToParentJoint), axis(axis) 
{
}

Joint* RevoluteDescriptionNode::CreateJoint(Articulation& ar, Link *link,
    PxTransform parentPose,
    PxTransform childPose) const 
{
    return ar.AddRevoluteJoint(jointName, link, axis, parentPose, childPose);
}
