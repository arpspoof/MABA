#include "ArticulationElements.h"

using namespace physx;

Joint::Joint(Link *link, PxTransform parentPose, PxTransform childPose)
    : id(-1), nDof(-1), cacheIndex(-1) {
    joint = static_cast<PxArticulationJointReducedCoordinate*>(link->link->getInboundJoint());
    joint->setParentPose(parentPose);
    joint->setChildPose(childPose);
}

FixedJoint::FixedJoint(Link *link, PxTransform parentPose, PxTransform childPose)
    : Joint(link, parentPose, childPose) {
    joint->setJointType(PxArticulationJointType::eFIX);
}

SphericalJoint::SphericalJoint(Link *link, PxTransform parentPose, PxTransform childPose)
    : Joint(link, parentPose, childPose) {
    joint->setJointType(PxArticulationJointType::eSPHERICAL);
    joint->setMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eFREE);
    joint->setMotion(PxArticulationAxis::eSWING1, PxArticulationMotion::eFREE);
    joint->setMotion(PxArticulationAxis::eSWING2, PxArticulationMotion::eFREE);
}

RevoluteJoint::RevoluteJoint(Link *link, PxArticulationAxis::Enum axis,
    PxTransform parentPose, PxTransform childPose)
    : Joint(link, parentPose, childPose), axis(axis) {
    joint->setJointType(PxArticulationJointType::eREVOLUTE);
    joint->setMotion(axis, PxArticulationMotion::eFREE);
}

Link::Link(PxArticulationReducedCoordinate* pxArticulation, Link *parent, PxTransform transform, LinkBody *body)
    :parentLink(parent), inboundJoint(nullptr), body(body) {
    link = pxArticulation->createLink(parent ? parent->link : NULL, transform);
    if (body->hasGeometry) {
        PxRigidActorExt::createExclusiveShape(*link, body->getGeometry(), *body->material);
    }
    if (parent) {
        parent->childLinks.push_back(this);
    }
    PxRigidBodyExt::updateMassAndInertia(*link, body->getDensity());
    pxActor = link;
}

Link::~Link()
{
    delete body;
}
