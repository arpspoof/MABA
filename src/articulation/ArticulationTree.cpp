#include "ArticulationTree.h"
#include <cassert>

using namespace std;
using namespace physx;

LinkBody* ArticulationTree::CreateNullLinkBody()
{
    LinkBody* body = new NULLLinkBody();
    linkBodies.insert(body);
    return body;
}

LinkBody* ArticulationTree::CreateSphereLinkBody(float mass, float radius, Material *material)
{
    LinkBody* body = new SphereLinkBody(mass, radius, material);
    linkBodies.insert(body);
    return body;
}

LinkBody* ArticulationTree::CreateCapsuleLinkBody(float mass, float radius, float length, Material *material)
{
    LinkBody* body = new CapsuleLinkBody(mass, radius, length, material);
    linkBodies.insert(body);
    return body;
}

LinkBody* ArticulationTree::CreateBoxLinkBody(float mass, float lenX, float lenY, float lenZ, Material *material)
{
    LinkBody* body = new BoxLinkBody(mass, lenX, lenY, lenZ, material);
    linkBodies.insert(body);
    return body;
}

ArticulationDescriptionNode* ArticulationTree::CreateNULLDescriptionNode(
    std::string linkName, LinkBody *body)
{
    NULLDescriptionNode *node = new NULLDescriptionNode(linkName, body);
    nodeMap[node->linkName] = node;
    return node;
}

ArticulationDescriptionNode* ArticulationTree::CreateFixedDescriptionNode(
    std::string linkName, std::string jointName, LinkBody *body,
    physx::PxVec3 posOffsetLinkToInboundJoint, physx::PxVec3 posOffsetJointToParentJoint)
{
    FixedDescriptionNode *node = new FixedDescriptionNode(
        linkName, jointName, body, posOffsetLinkToInboundJoint, posOffsetJointToParentJoint);
    nodeMap[node->linkName] = node;
    return node;
}

ArticulationDescriptionNode* ArticulationTree::CreateSpericalDescriptionNode(
    std::string linkName, std::string jointName, LinkBody *body,
    physx::PxVec3 posOffsetLinkToInboundJoint, physx::PxVec3 posOffsetJointToParentJoint)
{
    SpericalDescriptionNode *node = new SpericalDescriptionNode(
        linkName, jointName, body, posOffsetLinkToInboundJoint, posOffsetJointToParentJoint);
    nodeMap[node->linkName] = node;
    return node;
}
ArticulationDescriptionNode* ArticulationTree::CreateRevoluteDescriptionNode(
    std::string linkName, std::string jointName, LinkBody *body,
    physx::PxArticulationAxis::Enum axis,
    physx::PxVec3 posOffsetLinkToInboundJoint, physx::PxVec3 posOffsetJointToParentJoint)
{
    RevoluteDescriptionNode *node = new RevoluteDescriptionNode(
        linkName, jointName, body, axis, posOffsetLinkToInboundJoint, posOffsetJointToParentJoint);
    nodeMap[node->linkName] = node;
    return node;
}

void ArticulationTree::SetRoot(std::string linkName) {
    assert(nodeMap.find(linkName) != nodeMap.end());
    root = nodeMap[linkName];
    root->parent = nullptr;
}

void ArticulationTree::Connect(std::string parentLinkName, std::string childLinkName) {
    assert(nodeMap.find(parentLinkName) != nodeMap.end());
    assert(nodeMap.find(childLinkName) != nodeMap.end());
    ArticulationDescriptionNode *parent = nodeMap[parentLinkName];
    ArticulationDescriptionNode *child = nodeMap[childLinkName];
    child->parent = parent;
    parent->children.push_back(child);
}

ArticulationDescriptionNode* ArticulationTree::GetRootNode() const
{
    return root;
}

ArticulationTree::~ArticulationTree() {
    for (auto& it : nodeMap) {
        delete it.second;
    }
}

unordered_set<LinkBody*> ArticulationTree::GetLinkBodies()
{
    return linkBodies;
}

unordered_map<string, ArticulationDescriptionNode*> ArticulationTree::GetNodeMap()
{
    return nodeMap;
}
