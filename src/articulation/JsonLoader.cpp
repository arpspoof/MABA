#include "JsonLoader.h"
#include "PxPhysicsAPI.h"

#include <fstream>
#include <iostream>

using namespace std;
using namespace nlohmann;

JsonLoader::JsonLoader(float scalingFactor)
{
    Loader::scalingFactor = scalingFactor;
}

void JsonLoader::Dispose()
{
}

void JsonLoader::LoadDescriptionFromFile(string path)
{
    Dispose();

    ifstream t(path);
    string str((istreambuf_iterator<char>(t)), istreambuf_iterator<char>());

    j = json::parse(str);
}

void JsonLoader::ParseLinkBodies(ArticulationTree& tree, Material* material)
{
    float scalingFactorcube = scalingFactor*scalingFactor*scalingFactor;

    auto def = j["BodyDefs"];
    for (auto& body : def) {
        string linkName = body["Name"];
        float mass = body["Mass"];
        string shape = body["Shape"];
        if (shape == "sphere") {
            float diameter = body["Param0"];
            LinkBody* body = tree.CreateSphereLinkBody(mass*scalingFactorcube, diameter * scalingFactor / 2, material);
            linkBodyMap[linkName] = body;
        }
        else if (shape == "capsule") {
            float diameter = body["Param0"];
            float length = body["Param1"];
            LinkBody* body = tree.CreateCapsuleLinkBody(mass*scalingFactorcube, 
                diameter * scalingFactor / 2, length * scalingFactor, material);
            linkBodyMap[linkName] = body;
        }
        else if (shape == "box") {
            float x = body["Param0"];
            float y = body["Param1"];
            float z = body["Param2"];
            LinkBody* body = tree.CreateBoxLinkBody(mass*scalingFactorcube, 
                x * scalingFactor, y * scalingFactor, z * scalingFactor, material);
            linkBodyMap[linkName] = body;
        }
        else {
            printf("No link shaoe %s defined.\n", shape.c_str());
            assert(false);
        }
        float offsetX = body["AttachX"];
        float offsetY = body["AttachY"];
        float offsetZ = body["AttachZ"];
        linkOffsetMap[linkName] = vec3(offsetX * scalingFactor, offsetY * scalingFactor, offsetZ * scalingFactor);
    }
}

void JsonLoader::ParseJoints(ArticulationTree& tree)
{
    unordered_map<int, string> jointNames;
    unordered_map<int, int> parentIds;

    auto joints = j["Skeleton"]["Joints"];
    for (auto& joint : joints) {
        int id = joint["ID"];
        int parentJointId = joint["Parent"];
        string jointName = joint["Name"];
        jointIdMap[jointName] = id;
        parentIds[id] = parentJointId;
        jointNames[id] = jointName;
    }

    for (auto& joint : joints) {
        string jointName = joint["Name"];
        LinkBody* body = linkBodyMap[jointName];
        vec3 linkOffset = linkOffsetMap[jointName];
        float x = joint["AttachX"];
        float y = joint["AttachY"];
        float z = joint["AttachZ"];
        vec3 jointOffset(x * scalingFactor, y * scalingFactor, z * scalingFactor);
        string type = joint["Type"];
        if (type == "none") {
            tree.CreateNULLDescriptionNode(jointName, body);
        }
        else if (type == "spherical") {
            tree.CreateSpericalDescriptionNode(jointName, jointName, body, linkOffset, jointOffset);
        }
        else if (type == "revolute") {
            tree.CreateRevoluteDescriptionNode(jointName, jointName, body, 
                physx::PxArticulationAxis::eSWING2, linkOffset, jointOffset);
        }
        else if (type == "fixed") {
            tree.CreateFixedDescriptionNode(jointName, jointName, body, linkOffset, jointOffset);
        }
        else {
            printf("No joint type %s defined.\n", type.c_str());
            assert(false);
        }
    }

    tree.SetRoot(jointNames[0]);
    for (auto& kvp : parentIds) {
        int id = kvp.first;
        int pid = kvp.second;
        if (pid >= 0) tree.Connect(jointNames[pid], jointNames[id]);
    }
}

void JsonLoader::BuildArticulationTree(ArticulationTree& tree, Material* material)
{
    // This is hard coded here. Blame PhysX, do not blame me...
    tree.frameTransform = physx::PxQuat(physx::PxIdentity);
    ParseLinkBodies(tree, material);
    ParseJoints(tree);
}
