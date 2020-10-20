#pragma once

#include "Actor.h"

/**
 * @brief A material specifies the surface property of an actor.
 * @note Do not construct this object directly, use 
 *  Scene::CreateMaterial instead. 
 * 
 */
class Material
{
// API BEGIN
// API END
public:
    physx::PxMaterial* pxMaterial;
};

/**
 * @brief An object representing a static plane.
 * @note Do not construct this object directly, use
 *  Scene::CreatePlane instead.
 * 
 */
class Plane :public RigidStatic
{
// API BEGIN
// API END
};
