#include "Articulation.h"
#include "CommonMath.h"
#include "sparseLTL.h"

#include <cassert>

using namespace std;
using namespace physx;

extern PxQuat g_JointQuat[256];
static vector<float> pred(36);

void Articulation::AddSPDForcesSparse(const std::vector<float>& targetPositions, float timeStep, bool applyRootExternalForce)
{
    extern const float* g_InvD_Root_Kd;
    g_InvD_Root_Kd = nullptr;

    int nDof = GetNDof();

    pxArticulation->commonInit();
    pxArticulation->computeGeneralizedMassMatrix(*massMatrixCache, false);

    pxArticulation->copyInternalStateToCache(*coriolisCache, PxArticulationCache::eVELOCITY);
    pxArticulation->computeCoriolisAndCentrifugalForce(*coriolisCache, true);

    pxArticulation->computeGeneralizedGravityForce(*gravityCache, true);
    pxArticulation->computeGeneralizedExternalForce(*externalForceCache, true);

    vector<float> centrifugalCoriolisGravityExternal(nDof + 6);
    for (int i = 0; i < nDof + 6; i++) {
        centrifugalCoriolisGravityExternal[(i + 6) % (nDof + 6)] = -(
            coriolisCache->jointForce[i] + 
            gravityCache->jointForce[i] + 
            externalForceCache->jointForce[i]
        );
    }

    PxReal *positions = mainCache->jointPosition;
    PxReal *velocities = mainCache->jointVelocity;
    PxReal *forces = mainCache->jointForce;

    vector<float> proportionalTorquePlusQDotDeltaT(nDof + 6);
    vector<float> derivativeTorque(nDof + 6);

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

            if (targetPosition.w < 0) {
                targetPosition = -targetPosition;
            }

            PxVec3 localRotationExpMap(
                positions[cacheIndex],
                positions[cacheIndex + 1],
                positions[cacheIndex + 2]
            );
            PxQuat localRotation(localRotationExpMap.magnitude(), localRotationExpMap.getNormalized());

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

            proportionalTorquePlusQDotDeltaT[cacheIndex + 6] = proportionalForceInChildFrame[0] - 
                timeStep * velocities[cacheIndex] * kps[cacheIndex];
            proportionalTorquePlusQDotDeltaT[cacheIndex + 7] = proportionalForceInChildFrame[1] -
                timeStep * velocities[cacheIndex + 1] * kps[cacheIndex + 1];
            proportionalTorquePlusQDotDeltaT[cacheIndex + 8] = proportionalForceInChildFrame[2] -
                timeStep * velocities[cacheIndex + 2] * kps[cacheIndex + 2];

            derivativeTorque[cacheIndex + 6] = -kds[cacheIndex] * velocities[cacheIndex];
            derivativeTorque[cacheIndex + 7] = -kds[cacheIndex + 1] * velocities[cacheIndex + 1];
            derivativeTorque[cacheIndex + 8] = -kds[cacheIndex + 2] * velocities[cacheIndex + 2];

            for (int k = 0; k < 3; k++)
            {
                massMatrixCache->massMatrix[(cacheIndex + 6 + k) * (nDof + 7)] += kds[cacheIndex + k] * timeStep;
            }

            targetPositionsIndex += 4;
        }
        else if (jointDof == 1) {
            proportionalTorquePlusQDotDeltaT[cacheIndex + 6] = kps[cacheIndex] * (
                targetPositions[targetPositionsIndex] - positions[cacheIndex] - 
                timeStep * velocities[cacheIndex]
            );
            derivativeTorque[cacheIndex + 6] = -kds[cacheIndex] * velocities[cacheIndex];
            massMatrixCache->massMatrix[(cacheIndex + 6) * (nDof + 7)] += kds[cacheIndex] * timeStep;

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

    float* rhs = new float[nDof + 6];
    for (int i = 0; i < nDof + 6; i++) {
        rhs[i] = centrifugalCoriolisGravityExternal[i] + proportionalTorquePlusQDotDeltaT[i] + derivativeTorque[i];
    }

    if (applyRootExternalForce) {
        for (int i = 0; i < 6; i++) {
            massMatrixCache->massMatrix[i * (nDof + 7)] += root_kds[i] * timeStep;
        }
        
        PxVec3 rootGlobalPosition = rootLink->link->getGlobalPose().p;
        PxQuat rootGlobalRotation = rootLink->link->getGlobalPose().q;

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
            proportionalTorquePlusQDotDeltaT[i] = rootGlobalProportionalLinearForcePlusQDotDeltaT[i];
            derivativeTorque[i] = rootGlobalDerivativeLinearForce[i];
            rhs[i] += proportionalTorquePlusQDotDeltaT[i] + derivativeTorque[i];
        }
        for (int i = 3; i < 6; i++) {
            proportionalTorquePlusQDotDeltaT[i] = rootGlobalProportionalTorquePlusQDotDeltaT[i - 3];
            derivativeTorque[i] = rootGlobalDerivativeTorque[i - 3];
            rhs[i] += proportionalTorquePlusQDotDeltaT[i] + derivativeTorque[i];
        }
    }

    LTLInPlace(massMatrixCache->massMatrix, parentIndexMapForSparseLTL.data(), nDof + 6);
    backSubstitutionInPlace(massMatrixCache->massMatrix, rhs, parentIndexMapForSparseLTL.data(), nDof + 6);
    forwardSubstitutionInPlace(massMatrixCache->massMatrix, rhs, parentIndexMapForSparseLTL.data(), nDof + 6);

    for (int i = 0; i < nDof; i++) {
        forces[i] = (PxReal)(proportionalTorquePlusQDotDeltaT[i + 6] + derivativeTorque[i + 6]
            - timeStep * kds[i] * rhs[i + 6]);
        if (forceLimits[i] > 0) {
            forces[i] = PxClamp(forces[i], -forceLimits[i], forceLimits[i]);
        }
    }

    extern float g_RootExternalSpatialForce[100][6];
    if (applyRootExternalForce) {
        for (int i = 0; i < 6; i++) {
            g_RootExternalSpatialForce[_id][i] = proportionalTorquePlusQDotDeltaT[i] + derivativeTorque[i]
                - timeStep * root_kds[i] * rhs[i];
        }
    }
    else {
        memset(g_RootExternalSpatialForce[_id], 0, sizeof(float) * 6);
    }

    pxArticulation->applyCache(*mainCache, PxArticulationCache::eFORCE);
    delete rhs;

    static int parity = 0;
    if (parity == 1)
        AddSPDForcesABA(targetPositions, timeStep, applyRootExternalForce);
    else {
        extern const int*		g_SPD_LinkIdCacheIndexMap;
        extern bool g_ApplyABARootForce;
        g_SPD_LinkIdCacheIndexMap = nullptr;
        g_ApplyABARootForce = false;
    }
  //  parity = (parity + 1) % 10;
}
