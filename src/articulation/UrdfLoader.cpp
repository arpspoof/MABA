#include "UrdfLoader.h"
#include "xml/rapidxml.hpp"
#include "PxPhysicsAPI.h"

#include <fstream>
#include <streambuf>
#include <cstring>
#include <cassert>
#include <sstream>
#include <algorithm>

using namespace rapidxml;
using namespace std;

UrdfLoader::UrdfLoader(float scalingFactor) :buffer(nullptr)
{
    Loader::scalingFactor = scalingFactor;
}

void UrdfLoader::LoadDescriptionFromFile(std::string path)
{
    Dispose();

    ifstream t(path);
    string str((istreambuf_iterator<char>(t)), istreambuf_iterator<char>());

    buffer = new char[str.length() + 1];
    strcpy(buffer, str.c_str());

    doc.parse<0>(buffer);
}

void UrdfLoader::Dispose()
{
    doc.clear();
    delete buffer;
}

static xml_node<char>* FindFirstChildWithName(xml_node<char>* node, const char* name)
{
    for (auto child = node->first_node(); child != nullptr; child = child->next_sibling()) {
        if (strcmp(child->name(), name) == 0) {
            return child;
        }
    }
    return nullptr;
}

static xml_attribute<char>* FindFirstAttributeWithName(xml_node<char>* node, const char* name)
{
    for (auto attr = node->first_attribute(); attr != nullptr; attr = attr->next_attribute()) {
        if (strcmp(attr->name(), name) == 0) {
            return attr;
        }
    }
    return nullptr;
}

static float ExtractMassValueFromLinkNode(xml_node<char>* node)
{
    auto inertialNode = FindFirstChildWithName(node, "inertial");
    assert(inertialNode != nullptr);
    auto massNode = FindFirstChildWithName(inertialNode, "mass");
    assert(massNode != nullptr);
    auto massAttr = FindFirstAttributeWithName(massNode, "value");
    assert(massAttr != nullptr);
    return atof(massAttr->value());
}

static xml_node<char>* GetGeometryNode(xml_node<char>* node)
{
    auto collisionNode = FindFirstChildWithName(node, "collision");
    if (!collisionNode) return nullptr;
    auto geometryNode = FindFirstChildWithName(collisionNode, "geometry");
    assert(geometryNode != nullptr);
    return geometryNode;
}

void UrdfLoader::ParseLinkBodies(ArticulationTree& tree, Material* material)
{
    auto docRoot = doc.first_node();
    auto docRootFirstChild = docRoot->first_node();

    for (auto node = docRootFirstChild; node != nullptr; node = node->next_sibling()) {
        if (strcmp(node->name(), "link") == 0) {
            auto attrName = FindFirstAttributeWithName(node, "name");
            assert(attrName);
            string linkName(attrName->value());

            auto geometryNode = GetGeometryNode(node);
            if (geometryNode == nullptr) {
                LinkBody* body = tree.CreateNullLinkBody();
                linkBodyMap[linkName] = body;
            }
            else {
                float mass = ExtractMassValueFromLinkNode(node);
                
                auto sphereNode = FindFirstChildWithName(geometryNode, "sphere");
                auto capsuleNode = FindFirstChildWithName(geometryNode, "capsule");
                auto boxNode = FindFirstChildWithName(geometryNode, "box");

                assert(sphereNode || capsuleNode || boxNode);
                if (sphereNode) {
                    auto attr = FindFirstAttributeWithName(sphereNode, "radius");
                    assert(attr);
                    float radius = atof(attr->value()) * scalingFactor;
                    LinkBody* body = tree.CreateSphereLinkBody(mass, radius, material);
                    linkBodyMap[linkName] = body;
                    continue;
                }
                if (capsuleNode) {
                    auto attrLength = FindFirstAttributeWithName(capsuleNode, "length");
                    auto attrRadius = FindFirstAttributeWithName(capsuleNode, "radius");
                    assert(attrLength && attrRadius);
                    float length = atof(attrLength->value()) * scalingFactor;
                    float radius = atof(attrRadius->value()) * scalingFactor;
                    LinkBody* body = tree.CreateCapsuleLinkBody(mass, radius, length, material);
                    linkBodyMap[linkName] = body;
                    continue;
                }
                if (boxNode) {
                    auto attr = FindFirstAttributeWithName(boxNode, "size");
                    assert(attr);
                    stringstream ss(attr->value());
                    float y, x, z; // our axis: x up, y back, z right
                    ss >> y >> x >> z;
                    x *= scalingFactor;
                    y *= scalingFactor;
                    z *= scalingFactor;
                    LinkBody* body = tree.CreateBoxLinkBody(mass, x, y, z, material);
                    linkBodyMap[linkName] = body;
                    continue;
                }
            }
        }
    }
}

void UrdfLoader::ParseLinkOffsets()
{
    auto docRoot = doc.first_node();
    auto docRootFirstChild = docRoot->first_node();

    for (auto node = docRootFirstChild; node != nullptr; node = node->next_sibling()) {
        if (strcmp(node->name(), "link") == 0) {
            auto attrName = FindFirstAttributeWithName(node, "name");
            assert(attrName);
            string linkName(attrName->value());
            auto inertialNode = FindFirstChildWithName(node, "inertial");
            assert(inertialNode);
            auto originNode = FindFirstChildWithName(inertialNode, "origin");
            assert(originNode);
            auto xyzAttr = FindFirstAttributeWithName(originNode, "xyz");
            assert(xyzAttr);
            stringstream ss(xyzAttr->value());
            float x,y,z;
            ss >> x >> y >> z;
            x *= scalingFactor;
            y *= scalingFactor;
            z *= scalingFactor;
            linkOffsetMap[linkName] = vec3(x, y, z);
        }
    }   
}

void UrdfLoader::ParseJoints(ArticulationTree& tree)
{
    auto docRoot = doc.first_node();
    auto docRootFirstChild = docRoot->first_node();

    int jointId = 0;

    for (auto node = docRootFirstChild; node != nullptr; node = node->next_sibling()) {
        if (strcmp(node->name(), "joint") == 0) {
            auto attrJointName = FindFirstAttributeWithName(node, "name");
            auto attrJointType = FindFirstAttributeWithName(node, "type");
            assert(attrJointName && attrJointType);
            string jointName(attrJointName->value());
            string jointType(attrJointType->value());

            auto parentNode = FindFirstChildWithName(node, "parent");
            auto childNode = FindFirstChildWithName(node, "child");
            assert(parentNode && childNode);
            auto attrParentLink = FindFirstAttributeWithName(parentNode, "link");
            auto attrChildLink = FindFirstAttributeWithName(childNode, "link");
            assert(attrParentLink && attrChildLink);
            string parentLinkName(attrParentLink->value());
            string childLinkName(attrChildLink->value());
            parentLinkMap[childLinkName] = parentLinkName;

            auto originNode = FindFirstChildWithName(node, "origin");
            assert(originNode);
            auto xyzAttr = FindFirstAttributeWithName(originNode, "xyz");
            assert(xyzAttr);
            stringstream ss(xyzAttr->value());
            float x,y,z;
            ss >> x >> y >> z;
            x *= scalingFactor;
            y *= scalingFactor;
            z *= scalingFactor;

            if (jointType == "fixed") {
                tree.CreateFixedDescriptionNode(childLinkName, jointName, linkBodyMap[childLinkName],
                    linkOffsetMap[childLinkName], vec3(x, y, z));
            }
            else if (jointType == "spherical") {
                tree.CreateSpericalDescriptionNode(childLinkName, jointName, linkBodyMap[childLinkName],
                    linkOffsetMap[childLinkName], vec3(x, y, z));
            }
            else if (jointType == "revolute") {
                auto axisNode = FindFirstChildWithName(node, "axis");
                assert(axisNode);
                auto axisAttrXYZ = FindFirstAttributeWithName(axisNode, "xyz");
                assert(axisAttrXYZ);
                stringstream axisss(axisAttrXYZ->value());
                float axisX, axisY, axisZ;
                axisss >> axisX >> axisY >> axisZ;
                if (axisX == 0.f && axisY == 0.f && axisZ == 1.f) {
                    tree.CreateRevoluteDescriptionNode(childLinkName, jointName, linkBodyMap[childLinkName],
                        ArticulationAxis(ArticulationAxis::Z), linkOffsetMap[childLinkName], vec3(x, y, z));
                }
                else {
                    printf("In current stage we only support revolute joint to rotate around z axis\n");
                    assert(false);
                }
            }
            else {
                printf("unsupported joint type %s\n", jointType.c_str());
                assert(false);
                continue;
            }

            jointIdMap[jointName] = jointId++;
        }
    }

    for (auto &kvp : linkBodyMap) {
        if (parentLinkMap.find(kvp.first) == parentLinkMap.end()) {
            tree.CreateNULLDescriptionNode(kvp.first, kvp.second);
            tree.SetRoot(kvp.first);
        }
    }

    for (auto &kvp : parentLinkMap) {
        tree.Connect(kvp.second, kvp.first);
    }
}

void UrdfLoader::BuildArticulationTree(ArticulationTree& tree, Material* material)
{
    // This is hard coded here. Blame PhysX, do not blame me...
    tree.frameTransform = physx::PxQuat(physx::PxPi / 2, physx::PxVec3(0, 0, 1));
    ParseLinkBodies(tree, material);
    ParseLinkOffsets();
    ParseJoints(tree);
}
