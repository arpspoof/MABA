#include "Articulation.h"
#include "CommonMath.h"

#include <cassert>

using namespace std;
using namespace physx;

extern PxQuat g_JointQuat[256];

void Articulation::AddSPDForcesABA(const std::vector<float>& targetPositions, float timeStep, bool applyRootExternalForce)
{
    int nDof = GetNDof();

    PxReal *positions = mainCache->jointPosition;
    PxReal *velocities = mainCache->jointVelocity;
    PxReal *forces = mainCache->jointForce;

    vector<float> proportionalTorquePlusQDotDeltaT(nDof);
    vector<float> derivativeTorque(nDof);

    int nJoints = GetNJoints();
    int targetPositionsIndex = 7;

    for (int i = 0; i < nJoints; i++) {
        const Joint* joint = jointList[i];

        const int jointDof = joint->nDof;
        const int cacheIndex = joint->cacheIndex;

        if (jointDof == 3) {
            PxQuat targetPosition(
                targetPositions[targetPositionsIndex + 1],
                targetPositions[targetPositionsIndex + 2],
                targetPositions[targetPositionsIndex + 3],
                targetPositions[targetPositionsIndex]
            );
            UniformQuaternion(targetPosition); // Never trust user input

            PxVec3 kp(
                kps[cacheIndex],
                kps[cacheIndex + 1],
                kps[cacheIndex + 2]
            );

            targetPosition = frameTransform.getConjugate() * targetPosition * frameTransform;
            UniformQuaternion(targetPosition);

            PxVec3 localRotationExpMap(
                positions[cacheIndex],
                positions[cacheIndex + 1],
                positions[cacheIndex + 2]
            );
            PxQuat localRotation(localRotationExpMap.magnitude(), localRotationExpMap.getNormalized());
            UniformQuaternion(localRotation);

            PxQuat posDifference = targetPosition * localRotation.getConjugate();
            UniformQuaternion(posDifference);
            
            PxVec3 axis;
            PxReal angle;
            posDifference.toRadiansAndUnitAxis(angle, axis);
            axis *= angle;

            PxVec3 proportionalForceInParentFrame(
                kps[cacheIndex] * axis.x,
                kps[cacheIndex + 1] * axis.y,
                kps[cacheIndex + 2] * axis.z
            );
            PxVec3 proportionalForceInChildFrame = localRotation.getConjugate().rotate(proportionalForceInParentFrame);

            proportionalTorquePlusQDotDeltaT[cacheIndex] = proportionalForceInChildFrame[0] - 
                timeStep * velocities[cacheIndex] * kps[cacheIndex];
            proportionalTorquePlusQDotDeltaT[cacheIndex + 1] = proportionalForceInChildFrame[1] -
                timeStep * velocities[cacheIndex + 1] * kps[cacheIndex + 1];
            proportionalTorquePlusQDotDeltaT[cacheIndex + 2] = proportionalForceInChildFrame[2] -
                timeStep * velocities[cacheIndex + 2] * kps[cacheIndex + 2];

            derivativeTorque[cacheIndex] = -kds[cacheIndex] * velocities[cacheIndex];
            derivativeTorque[cacheIndex + 1] = -kds[cacheIndex + 1] * velocities[cacheIndex + 1];
            derivativeTorque[cacheIndex + 2] = -kds[cacheIndex + 2] * velocities[cacheIndex + 2];

            targetPositionsIndex += 4;
        }
        else if (jointDof == 1) {
            proportionalTorquePlusQDotDeltaT[cacheIndex] = kps[cacheIndex] * (
                targetPositions[targetPositionsIndex] - positions[cacheIndex] - 
                timeStep * velocities[cacheIndex]
            );
            derivativeTorque[cacheIndex] = -kds[cacheIndex] * velocities[cacheIndex];

            targetPositionsIndex += 1;
        }
        else if (jointDof == 0) {
            continue;
        }
        else {
            printf("no controller defined for Dof %d\n", jointDof);
            assert(false);
        }
    }

    for (int i = 0; i < nDof; i++) {
        forces[i] = proportionalTorquePlusQDotDeltaT[i] + derivativeTorque[i];
    }

    pxArticulation->applyCache(*mainCache, PxArticulationCache::eFORCE);
#ifdef ENABLE_SPD_ABA
    extern float  			g_SPD_Dt;
    extern const float* 	g_SPD_Kd;
    extern const int*		g_SPD_LinkIdCacheIndexMap;

    g_SPD_Dt = timeStep;
    g_SPD_Kd = kds.data();
    g_SPD_LinkIdCacheIndexMap = linkIdCacheIndexMap.data();
#endif

    if (applyRootExternalForce) {
        vector<float> rootForcePD(6, 0);

        PxVec3 rootGlobalPosition = rootLink->link->getGlobalPose().p;
        PxQuat rootGlobalRotation = rootLink->link->getGlobalPose().q;
        UniformQuaternion(rootGlobalRotation);

        PxVec3 rootGlobalLinearVelocity = rootLink->link->getLinearVelocity();
        PxVec3 rootGlobalAngularVelocity = rootLink->link->getAngularVelocity();
        
        PxVec3 rootGlobalProportionalLinearForcePlusQDotDeltaT(
            root_kps[0] * (targetPositions[0] - rootGlobalPosition[0] - timeStep * rootGlobalLinearVelocity[0]),
            root_kps[1] * (targetPositions[1] - rootGlobalPosition[1] - timeStep * rootGlobalLinearVelocity[1]),
            root_kps[2] * (targetPositions[2] - rootGlobalPosition[2] - timeStep * rootGlobalLinearVelocity[2])
        );
        PxVec3 rootGlobalDerivativeLinearForce(
            -root_kds[0] * rootGlobalLinearVelocity[0],
            -root_kds[1] * rootGlobalLinearVelocity[1],
            -root_kds[2] * rootGlobalLinearVelocity[2]
        );

        PxQuat rootGlobalTargetRotationUser(targetPositions[4], targetPositions[5], targetPositions[6], targetPositions[3]);
        rootGlobalTargetRotationUser.normalize();
        
        // transform from { x:front, y:up, z:right } to { x:up, y:back, z:right }
        PxQuat rootGlobalTargetRotation = rootGlobalTargetRotationUser * frameTransform;

        PxQuat diffQuat = rootGlobalTargetRotation * rootGlobalRotation.getConjugate();
        if (PxAbs(diffQuat.w) < 0.70710678118f) {
            diffQuat = (-rootGlobalTargetRotation) * rootGlobalRotation.getConjugate();
        }
        UniformQuaternion(diffQuat);

        PxVec3 diffRotExpMapGlobal = QuatToExpMap(diffQuat);
        PxVec3 rootGlobalProportionalTorquePlusQDotDeltaT(
            root_kps[3] * (diffRotExpMapGlobal[0] - timeStep * rootGlobalAngularVelocity[0]),
            root_kps[4] * (diffRotExpMapGlobal[1] - timeStep * rootGlobalAngularVelocity[1]),
            root_kps[5] * (diffRotExpMapGlobal[2] - timeStep * rootGlobalAngularVelocity[2])
        );
        PxVec3 rootGlobalDerivativeTorque(
            -root_kds[3] * rootGlobalAngularVelocity[0],
            -root_kds[4] * rootGlobalAngularVelocity[1],
            -root_kds[5] * rootGlobalAngularVelocity[2]
        );

        for (int i = 0; i < 3; i++) {
            rootForcePD[i] += rootGlobalProportionalLinearForcePlusQDotDeltaT[i] + rootGlobalDerivativeLinearForce[i];
        }
        for (int i = 0; i < 3; i++) {
            rootForcePD[i + 3] += rootGlobalProportionalTorquePlusQDotDeltaT[i] + rootGlobalDerivativeTorque[i];
        }

#ifdef ENABLE_SPD_ABA
        extern bool g_ApplyABARootForce;
        extern float g_RootExternalSpatialForce[100][6];
        extern const float* g_ABA_Root_Kd;

        g_ApplyABARootForce = true;
        g_ABA_Root_Kd = root_kds.data();
        memcpy(g_RootExternalSpatialForce[_id], rootForcePD.data(), 6 * sizeof(float));
#endif
    }
}
