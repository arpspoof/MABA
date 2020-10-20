%{
    #include "Articulation.h"
%}

%include "std_unordered_map.i"
%include "std_vector.i"
%include "std_string.i"

namespace std {
   %template(vectori) vector<int>;
   %template(vectorf) vector<float>;
   %template(vectorj) vector<Joint*>;
};

class Articulation :public IDisposable
{
// API BEGIN
public:
    int GetNDof() const;
    void SetFixBaseFlag(bool shouldFixBase);
    int GetNJoints() const;
    const std::vector<int>& GetJointDofsInIdOrder() const;
    const Link* GetLinkByName(std::string name) const;
    const Joint* GetJointByName(std::string name) const;
    const std::vector<Joint*>& GetAllJointsInIdOrder() const;
    const Link* GetRootLink() const;
public:
    std::vector<float> GetJointPositionsQuaternion() const;
    void SetJointPositionsQuaternion(const std::vector<float>& positions) const;
    std::vector<float> GetJointVelocitiesPack4() const;
    void SetJointVelocitiesPack4(const std::vector<float>& velocities) const;
public:
    void SetKPs(const std::vector<float>& kps);
    void SetKDs(const std::vector<float>& kds);
    void SetForceLimits(const std::vector<float>& forceLimits);
    void AddSPDForces(const std::vector<float>& targetPositions, float timeStep);
    void AddSPDForcesSparse(const std::vector<float>& targetPositions, float timeStep);
    void AddSPDForcesABA(const std::vector<float>& targetPositions, float timeStep);
// API END
};
