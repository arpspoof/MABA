%{
    #include "Actor.h"
%}

class Actor
{
// API BEGIN
// API END
};

class RigidActor :public Actor
{
// API BEGIN
public:
    void SetupCollisionFiltering(int collisionGroup, int collisionMask);
// API END
};

class RigidStatic :public RigidActor
{
// API BEGIN
// API END
};

class RigidBody :public RigidActor
{
// API BEGIN
public:
    void ApplyExternalForce(vec3 force) const;
// API END
};
