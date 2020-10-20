#pragma once

#include "PxPhysicsAPI.h"

inline void UniformQuaternion(physx::PxQuat& q)
{
    q.normalize();
    if (q.w < 0) q = -q;
}

inline physx::PxQuat ConvertTwistSwingToQuaternion(float t, float s1, float s2)
{
    physx::PxVec3 s(0, s1, s2);
    physx::PxQuat swingQuat = s.isZero() ? physx::PxQuat(0, 0, 0, 1) : physx::PxQuat(s.magnitude(), s.getNormalized());
    physx::PxQuat result = swingQuat * physx::PxQuat(t, physx::PxVec3(1, 0, 0));
    UniformQuaternion(result);
    return result;
}

inline void SeparateTwistSwing(const physx::PxQuat& quat, physx::PxQuat& swing, physx::PxQuat& twist)
{
    physx::PxQuat q(quat);
    UniformQuaternion(q);
	twist = q.x != 0.0f ? physx::PxQuat(q.x, 0, 0, q.w).getNormalized() : physx::PxQuat(physx::PxIdentity);
	swing = q * twist.getConjugate();
    UniformQuaternion(twist);
    UniformQuaternion(swing);
}

inline physx::PxVec3 QuatToExpMap(physx::PxQuat q)
{
    physx::PxVec3 axis;
    physx::PxReal angle;
    q.toRadiansAndUnitAxis(angle, axis);
    return axis *= angle;
}
