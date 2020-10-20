#pragma once

#include "PxPhysicsAPI.h"
#include "ArticulationElements.h"
#include "IDisposable.h"

#include <unordered_map>
#include <string>

/**
 * @brief Featherstone generalized coordinate articulation
 * 
 */
class Articulation :public IDisposable
{
// API BEGIN
public: // components
    /**
     * @brief Get the number of total degrees of freedom in this articulation
     * 
     * @return The number of degrees of freedom
     */
    int GetNDof() const;
    /**
     * @brief Set whether or not to fix the articulation base
     * 
     * @param shouldFixBase (true / false)
     */
    void SetFixBaseFlag(bool shouldFixBase);
    /**
     * @brief Get the total number of joints in articulation.
     *  Joints with 0 dof are also included.
     * 
     * @return Number of joints. 
     */
    int GetNJoints() const;
    /**
     * @brief Get the all Joint degrees of freedom. Result is packed in an
     *  array of Articulation::GetNDof elements and ordered by Joint::id.
     * 
     * @return array of joint defrees of freedom.
     */
    const std::vector<int>& GetJointDofsInIdOrder() const;
    /**
     * @brief Get all Joint objects ordered by Joint::id.
     * 
     * @return Vector of pointers to Joint
     */
    const std::vector<Joint*>& GetAllJointsInIdOrder() const;
    /**
     * @brief Get the Link name specified in the Urdf description.
     * 
     * @param name Link name (string).
     * @return Pointer to Link object.
     */
    const Link* GetLinkByName(std::string name) const;
    /**
     * @brief Get the Joint name specified in the Urdf description.
     * 
     * @param name Joint name (string).
     * @return Pointer to Joint object.
     */
    const Joint* GetJointByName(std::string name) const;
    /**
     * @brief Get the root Link of the Articulation
     * 
     * @return Pointer to root Link object
     */
    const Link* GetRootLink() const;
public: // kinematic
    /**
     * @brief Get root position, orientation, and all joints' orientation
     *  in quaternion representation. Root global translational position
     *  is in result[0-2], root global rotational position (quaternion)
     *  is in result[3-6]. All other joints' position info are placed into
     *  the result by joint id. A one-dof joint takes up one entry in result
     *  array representing the joint angle (radians). A three-dof joint takes
     *  up four entries in result array representing the joint local frame
     *  rotaion in quaternion representaion in WXYZ order.
     * 
     * @return All joint position information packed into one single array.
     */
    std::vector<float> GetJointPositionsQuaternion() const;
    /**
     * @brief Set root position, orientation, and all joints' orientation
     *  in quaternion representation from a packed array. See
     *  Articulation::GetJointPositionsQuaternion for details about the
     *  structure of this array.
     * 
     * @param positions 
     */
    void SetJointPositionsQuaternion(const std::vector<float>& positions) const;
    /**
     * @brief Get root linear velocity, root angular velocity and all joints'
     *  angular velocities. Root linear velocity is in result[0-2], root
     *  angular velocity in global frame is in result[3-5]. Result[6] is
     *  always zero. All other joints' velocity info are placed into the
     *  result by joint id. A one-dof joint takes up one entry in result
     *  array representing the joint velocity. A three-dof joint takes
     *  up four entries in result array. First three of these four entries
     *  represent the joint angular velocity (cartesian) in joint frame.
     *  The forth entry is always zero.
     * 
     * @return All joint velocity information packed into one single array.
     */
    std::vector<float> GetJointVelocitiesPack4() const;
    /**
     * @brief Set root linear velocity, root angular velocity and all joints'
     *  angular velocities from a packed array. See
     *  Articulation::GetJointVelocitiesPack4 for details about the
     *  structure of this array.
     * 
     * @param velocities 
     */
    void SetJointVelocitiesPack4(const std::vector<float>& velocities) const;
public:
    std::vector<physx::PxVec3> jointPositions;
    std::vector<physx::PxQuat> jointLocalRotations;
    std::vector<physx::PxQuat> jointGlobalRotations;
    std::vector<physx::PxVec3> linkPositions;
    std::vector<physx::PxQuat> linkGlobalRotations;
    void CalculateFK(std::vector<float>& jData);
public: // control
    int _id; // for multiple articulation
    /**
     * @brief Set joint proportional gains through a single array
     * 
     * @param kps A single array which packs all joint proportional gains.
     *  ordered by Joint::id.
     * @remark Number of parameters needed for each joint is the same
     *  as the degree of freedom of that joint. Parameters needed for
     *  each joint is equal to Joint::nDof.
     */
    void SetKPs(const std::vector<float>& kps);
    /**
     * @brief Set joint derivative gains through a single array
     * 
     * @param kds A single array which packs all joint derivative gains.
     *  ordered by Joint::id.
     * @remark Number of parameters needed for each joint is the same
     *  as the degree of freedom of that joint. Parameters needed for
     *  each joint is equal to Joint::nDof.
     */
    void SetKDs(const std::vector<float>& kds);
    /**
     * @brief Set joint force limits through a single array
     * 
     * @param forceLimits A single array which packs all joint force limits.
     *  ordered by Joint::id.
     * @remark Number of parameters needed for each joint is the same
     *  as the degree of freedom of that joint. Parameters needed for
     *  each joint is equal to Joint::nDof.
     */
    void SetForceLimits(const std::vector<float>& forceLimits);
    /**
     * @brief Compute and apply stable PD control to all joints in articulation.
     * @note Joint force is cleared before applying control forces.
     * 
     * @param targetPositions A packed array specifying target positions for all
     *  joints ordered by Joint::id. Note that this array does not have to have
     *  Articulation::GetNDof entries. Target position for a 1-dof joint is 
     *  represented by a single angle (in radians) while target position for a
     *  3-dof joint is represented by a quaternion in **WXYZ** order.
     * 
     * @param timeStep Simulation time step is required. Get this via Scene::timeStep.
     */
    void AddSPDForces(const std::vector<float>& targetPositions,
        float timeStep, bool applyRootExternalForce = false); // WXYZ or angle
    /**
     * @brief Linear time implementation of stable PD control.
     *  See Articulation::AddSPDForces for usage instructions.
     * 
     * @param targetPositions 
     * @param timeStep 
     * 
     * @warning Experimental
     */
    void AddSPDForcesSparse(const std::vector<float>& targetPositions,
        float timeStep, bool applyRootExternalForce = false); // WXYZ or angle
    /**
     * @brief Sparse matrix factorization implementation of stable PD control.
     *  See Articulation::AddSPDForces for usage instructions.
     * 
     * @param targetPositions 
     * @param timeStep 
     * 
     * @warning Experimental
     */
    void AddSPDForcesABA(const std::vector<float>& targetPositions,
        float timeStep, bool applyRootExternalForce = false); // WXYZ or angle
// API END
public:
    std::vector<float> kps, kds, forceLimits;
    std::vector<float> root_kps, root_kds;
    std::unordered_map<std::string, Link*> linkMap;
    std::unordered_map<std::string, Joint*> jointMap;
    physx::PxQuat frameTransform;
private:
    Link* rootLink;
    std::vector<Joint*> jointList;
    std::vector<int> jointDofs;
    std::vector<int> linkIdCacheIndexMap;
    std::vector<int> parentIndexMapForSparseLTL;
    int nSphericalJoint, nRevoluteJoint;
public:
    physx::PxArticulationReducedCoordinate* pxArticulation;
private:
    physx::PxArticulationCache* mainCache;
    physx::PxArticulationCache* massMatrixCache;
    physx::PxArticulationCache* coriolisCache;
    physx::PxArticulationCache* gravityCache;
    physx::PxArticulationCache* externalForceCache;
public:
    Link* AddLink(std::string name, Link *parent, physx::PxTransform transform, LinkBody *body);
    Joint* AddSpericalJoint(std::string name, Link *link,
        physx::PxTransform parentPose, physx::PxTransform childPose);
    Joint* AddRevoluteJoint(std::string name, Link *link, physx::PxArticulationAxis::Enum axis,
        physx::PxTransform parentPose, physx::PxTransform childPose);
    Joint* AddFixedJoint(std::string name, Link *link, 
        physx::PxTransform parentPose, physx::PxTransform childPose);
public:
    void InitControl(std::unordered_map<std::string, int>& jointIdMap);
    void FetchKinematicData() const;
    void Dispose() override;
private:
    void SetJointParams(std::vector<float>& target, const std::vector<float>& params);
    void AssignIndices();
};
