%{
    #include "ArticulationElements.h"
%}

%include "std_vector.i"
%include "std_string.i"
namespace std {
   %template() vector<Link*>;
};

class Link;

class Joint {
public:
// API BEGIN
    Link* parentLink;
    Link* childLink;
    std::string name;
    int id;
    int nDof;
    int cacheIndex;
    Joint() {} // For API only
// API END
};

class Link :public RigidBody {
public:
// API BEGIN
    Link* parentLink;
    Joint* inboundJoint;
    std::string name;
    std::vector<Link*> childLinks;
    Link() {} // For API only
// API END
};
