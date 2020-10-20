#include "Articulation.h"
#include "CommonMath.h"

#include <cassert>
#include <Eigen/Dense>

using namespace std;
using namespace physx;
using namespace Eigen;

extern PxQuat g_JointQuat[256];
static vector<float> pred(36);

void Articulation::AddSPDForces(const std::vector<float>& targetPositions, float timeStep, bool applyRootExternalForce)
{
    extern const float* g_InvD_Root_Kd;
    extern float g_InvD_Dt;

    if (applyRootExternalForce) {
        g_InvD_Root_Kd = root_kds.data();
        g_InvD_Dt = timeStep;
    }
    else {
        g_InvD_Root_Kd = nullptr;
    }

    int nDof = GetNDof();

    pxArticulation->commonInit();
    pxArticulation->computeGeneralizedMassMatrix(*massMatrixCache, true);

    pxArticulation->copyInternalStateToCache(*coriolisCache, PxArticulationCache::eVELOCITY);
    pxArticulation->computeCoriolisAndCentrifugalForce(*coriolisCache);

    pxArticulation->computeGeneralizedGravityForce(*gravityCache);
    pxArticulation->computeGeneralizedExternalForce(*externalForceCache);

    VectorXf centrifugalCoriolisGravityExternal(nDof);
    for (int i = 0; i < nDof; i++) {
        centrifugalCoriolisGravityExternal(i) = -(
            coriolisCache->jointForce[i] + 
            gravityCache->jointForce[i] + 
            externalForceCache->jointForce[i]
        );
    }
    MatrixXf H(nDof, nDof);
    for (int i = 0; i < nDof; i++) {
        for (int j = i; j < nDof; j++) {
            H(i, j) = H(j, i) = massMatrixCache->massMatrix[(i + 6) * (nDof + 6) + j + 6];
        }
    }

    PxReal *positions = mainCache->jointPosition;
    PxReal *velocities = mainCache->jointVelocity;
    PxReal *forces = mainCache->jointForce;

    VectorXf proportionalTorquePlusQDotDeltaT(nDof);
    VectorXf derivativeTorque(nDof);

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

            proportionalTorquePlusQDotDeltaT(cacheIndex) = proportionalForceInChildFrame[0] - 
                timeStep * velocities[cacheIndex] * kps[cacheIndex];
            proportionalTorquePlusQDotDeltaT(cacheIndex + 1) = proportionalForceInChildFrame[1] -
                timeStep * velocities[cacheIndex + 1] * kps[cacheIndex + 1];
            proportionalTorquePlusQDotDeltaT(cacheIndex + 2) = proportionalForceInChildFrame[2] -
                timeStep * velocities[cacheIndex + 2] * kps[cacheIndex + 2];

            derivativeTorque(cacheIndex) = -kds[cacheIndex] * velocities[cacheIndex];
            derivativeTorque(cacheIndex + 1) = -kds[cacheIndex + 1] * velocities[cacheIndex + 1];
            derivativeTorque(cacheIndex + 2) = -kds[cacheIndex + 2] * velocities[cacheIndex + 2];

            H(cacheIndex, cacheIndex) += kds[cacheIndex] * timeStep;
            H(cacheIndex + 1, cacheIndex + 1) += kds[cacheIndex + 1] * timeStep;
            H(cacheIndex + 2, cacheIndex + 2) += kds[cacheIndex + 2] * timeStep;

            targetPositionsIndex += 4;
        }
        else if (jointDof == 1) {
            proportionalTorquePlusQDotDeltaT(cacheIndex) = kps[cacheIndex] * (
                targetPositions[targetPositionsIndex] - positions[cacheIndex] - 
                timeStep * velocities[cacheIndex]
            );
            derivativeTorque(cacheIndex) = -kds[cacheIndex] * velocities[cacheIndex];
            H(cacheIndex, cacheIndex) += kds[cacheIndex] * timeStep;

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

    VectorXf rootForcePD;
    MatrixXf I0cPlusKdDeltaT;
    MatrixXf F;

    if (applyRootExternalForce) {
        rootForcePD = VectorXf::Zero(6);

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
            rootForcePD(i) += rootGlobalProportionalLinearForcePlusQDotDeltaT[i] + rootGlobalDerivativeLinearForce[i];
        }
        for (int i = 0; i < 3; i++) {
            rootForcePD(i + 3) += rootGlobalProportionalTorquePlusQDotDeltaT[i] + rootGlobalDerivativeTorque[i];
        }

        I0cPlusKdDeltaT = MatrixXf(6, 6);
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 6; j++) {
                I0cPlusKdDeltaT(i, j) = massMatrixCache->massMatrix[i * (nDof + 6) + j];
            }
        }
        for (int i = 0; i < 6; i++) {
            I0cPlusKdDeltaT(i, i) += root_kds[i] * timeStep;
        }

        F = MatrixXf(6, nDof);
        for (int i = 0; i < 6; i++) {
            for (int j = 6; j < nDof + 6; j++) {
                F(i, j - 6) = massMatrixCache->massMatrix[i * (nDof + 6) + j];
            }
        }

        centrifugalCoriolisGravityExternal -= F.transpose() * I0cPlusKdDeltaT.inverse() * rootForcePD;
    }

    VectorXf qDotDot = H.llt().solve(centrifugalCoriolisGravityExternal + proportionalTorquePlusQDotDeltaT + derivativeTorque);
    for (int i = 0; i < nDof; i++) {
        forces[i] = (PxReal)(proportionalTorquePlusQDotDeltaT(i) + derivativeTorque(i) - timeStep * kds[i] * qDotDot(i));
        if (forceLimits[i] > 0) {
            forces[i] = PxClamp(forces[i], -forceLimits[i], forceLimits[i]);
        }
    }

    pxArticulation->applyCache(*mainCache, PxArticulationCache::eFORCE);

    extern float g_RootExternalSpatialForce[100][6];
    if (applyRootExternalForce) {
        VectorXf p0c(6);
        for (int i = 0; i < 6; i++) {
            p0c(i) = coriolisCache->jointForce[nDof + i] + 
                gravityCache->jointForce[nDof + i] +
                externalForceCache->jointForce[nDof + i];
        }
        VectorXf q0DotDot = I0cPlusKdDeltaT.llt().solve(rootForcePD - p0c - F * qDotDot);
        
        for (int i = 0; i < 6; i++) {
            g_RootExternalSpatialForce[_id][i] = rootForcePD(i) - timeStep * root_kds[i] * q0DotDot(i);
        }
    }
    else {
        memset(g_RootExternalSpatialForce[_id], 0, sizeof(float) * 6);
    }
}
