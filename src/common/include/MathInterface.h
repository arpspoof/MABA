#pragma once

#include "PxPhysicsAPI.h"

/**
 * @brief 3d floating point vector wrapper. No calculation function provided.
 * 
 */
class vec3
{
public:
// API BEGIN
    /**
     * @brief x components of the vector
     * 
     */
    float x;
    /**
     * @brief y components of the vector
     * 
     */
    float y;
    /**
     * @brief z components of the vector
     * 
     */
    float z;
    /**
     * @brief Construct a new 3d vector. All components are set t0 zero
     * 
     */
    vec3() :x(0), y(0), z(0) {}
    /**
     * @brief Construct a new 3d vector by specifying all 3 components
     * 
     * @param x 
     * @param y 
     * @param z 
     */
    vec3(float x, float y, float z) :x(x), y(y), z(z) {}
// API END
    operator physx::PxVec3() const { return physx::PxVec3(x, y, z); }
};

/**
 * @brief Quaternion wrapper. No calculation function provided.
 * 
 */
class quat
{
public:
// API BEGIN
    /**
     * @brief x components of the quaternion
     * 
     */
    float x;
    /**
     * @brief y components of the quaternion
     * 
     */
    float y;
    /**
     * @brief z components of the quaternion
     * 
     */
    float z;
    /**
     * @brief w components of the quaternion
     * 
     */
    float w;
    /**
     * @brief Construct a quaternion representing the zero rotation (0, 0, 0, 1)
     * 
     */
    quat() :x(0), y(0), z(0), w(1) {}
    /**
     * @brief Construct a new quaternion object by specifying all 4 components
     * 
     * @param x 
     * @param y 
     * @param z 
     * @param w 
     */
    quat(float x, float y, float z, float w) :x(x), y(y), z(z), w(w) {}
// API END
    operator physx::PxQuat() const { return physx::PxQuat(w, y, z ,w); }
};

/**
 * @brief 3d transform wrapper including a 3d translation represented
 *  using a 3d vector and a 3d rotation represented by a quaternion. 
 *  No calculation function provided.
 * 
 */
class Transform
{
public:
// API BEGIN
    /**
     * @brief Translation component
     * 
     */
    vec3 p;
    /**
     * @brief Rotational component
     * 
     */
    quat q;
    /**
     * @brief Construct an identity transform
     * 
     */
    Transform() {}
    /**
     * @brief Construct a transform by specifying translation and rotation
     * 
     * @param p Translation component
     * @param q Rotational component
     */
    Transform(vec3 p, quat q) :p(p), q(q) {}
// API END
    operator physx::PxTransform() const { return physx::PxTransform(p, q); }
};
