#include "Actor.h"

using namespace physx;

void RigidActor::SetupCollisionFiltering(int collisionGroup, int collisionMask)
{
    PxRigidActor* actor = (PxRigidActor*)pxActor;
    PxFilterData filterData;
    filterData.word0 = (PxU32)collisionGroup; // word0 = own ID
    filterData.word1 = (PxU32)collisionMask;  // word1 = ID mask to filter pairs that trigger a contact callback;
    const PxU32 numShapes = actor->getNbShapes();
    PxShape** shapes = (PxShape**)malloc(sizeof(PxShape*)*numShapes);
    actor->getShapes(shapes, numShapes);
    for (PxU32 i = 0; i < numShapes; i++)
    {
        PxShape* shape = shapes[i];
        shape->setSimulationFilterData(filterData);
    }
    free(shapes);
}

void RigidBody::ApplyExternalForce(vec3 force) const
{
    PxRigidBody* rigidBody = (PxRigidBody*)pxActor;
    rigidBody->addForce(force);
}
