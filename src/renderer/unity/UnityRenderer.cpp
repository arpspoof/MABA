#include "UnityRenderer.h"
#include "PxPhysicsAPI.h"

#include <KinematicsClient.hpp>
#include <string>
#include <vector>

using namespace std;
using namespace physx;

static vector<Articulation*> articulations;

class DataProvider : public AbstractDataProvider
{
    virtual FrameState GetCurrentState() const override
    {
        FrameState frame;
        for (int i = 0; i < (int)articulations.size(); i++) {
            auto& ar = articulations[i];
            frame.groups.push_back(GroupState("articulation" + to_string(i)));
            GroupState& group = frame.groups[i];
            for (auto& kvp : ar->linkMap) {
                const string& name = kvp.first;
                Link* link = kvp.second;
                LinkBody* body = link->body;
                if (body->geometry) {
                    PxVec3 p = link->link->getGlobalPose().p;
                    PxQuat q = link->link->getGlobalPose().q;
                    if (body->type == "capsule") {
                        q = q * PxQuat(-PxPi / 2, PxVec3(0, 0, 1));
                    }
                    ObjectState obj(name, p.x, p.y, p.z, q.w, q.x, q.y, q.z);
                    group.objectStates.push_back(obj);
                }
            }
        }
        return frame;
    }  
};

class CommandHandler : public AbstractCommandHandler
{ 
    virtual void HandleCommand(const Command& cmd) const override
    {
        printf("cls recv cmd: %s\n", cmd.name.c_str());
    }
};

static DataProvider dataProvider;
static CommandHandler commandHandler;

static float timeStep;

void UR_Init(float timeStep, 
    const std::string remoteIP, 
    unsigned short remotePort,
    const std::string localIP,
    unsigned short localPort)
{
    ::timeStep = timeStep;
    InitRenderController(remoteIP, remotePort, localIP, localPort, 
        (int)round(1.0 / timeStep), &dataProvider, &commandHandler, 10000);
}

void UR_Tick(double duration)
{
    Tick(timeStep, duration);
}

void UR_Stop()
{
    DisposeRenderController();
}

void UR_AddArticulation(Articulation* ar)
{
    articulations.push_back(ar);
}

void UR_InitPrimitives()
{
    for (int i = 0; i < (int)articulations.size(); i++) {
        auto& ar = articulations[i];
        for (auto& kvp : ar->linkMap) {
            const string& name = kvp.first;
            Link* link = kvp.second;
            LinkBody* body = link->body;
            if (body->hasGeometry) {
                BodyGeometryData data;
                body->FillBodyGeometryData(data);
                CreatePrimitive(data.type, "articulation" + to_string(i), name, data.param0, data.param1, data.param2);
                printf("track name %s : %s, %f, %f, %f\n", name.c_str(), 
                    data.type.c_str(), data.param0, data.param1, data.param2);
            }
        }
    }
}

void UR_SetPhysicalFPS(int fps)
{
    SetPhysicalFPS(fps);
}
