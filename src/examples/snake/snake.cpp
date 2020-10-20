#include "PxPhysicsAPI.h"
#ifdef ENABLE_UNITY_KINEMATICS
#include "UnityRenderer.h"
#endif
#include "GlutRenderer.h"
#include "JsonLoader.h"
#include "Foundation.h"
#include "Scene.h"
#include "cxxopts.hpp"
#include "json.hpp"

#include <string>
#include <vector>
#include <chrono>
#include <fstream>
#include <iomanip>

using namespace nlohmann;
using namespace physx;
using namespace std;
using namespace std::chrono;

static Foundation* foundation;
static Scene* scene;
static Material* material;

static float height = 0.251f;

static int N = 1;
static vector<Articulation*> articulations;

void InitControl() {
    vector<float> kps, kds, fls;

    for (auto& articulation : articulations) {

        for (auto &j : articulation->GetAllJointsInIdOrder()) {
            int nDof = j->nDof;

            for (int i = 0; i < nDof; i++) {
                kps.push_back(2000);
                kds.push_back(200);
            }
        }

        articulation->SetKPs(kps);
        articulation->SetKDs(kds);

        float rootKpL = 20000, rootKdL = 2000;
        float rootKpA = 20000, rootKdA = 2000;
        // 2000, 1600 is a good combo for walking (t = 0.033, kp = 75000, kd = 6600)
        articulation->root_kps = { rootKpL, rootKpL, rootKpL, rootKpA, rootKpA, rootKpA };
        articulation->root_kds = { rootKdL, rootKdL, rootKdL, rootKdA, rootKdA, rootKdA };

    }
}

void initPhysics(float dt)
{
    foundation = new Foundation();
    scene = foundation->CreateScene(SceneDescription(), dt);

    material = scene->CreateMaterial(.5f, .5f, 0.f);

    scene->CreatePlane(material, vec3(0, 1, 0), 0);

    JsonLoader jsonLoader(0.5f);
    string filepath = "resources/snake.txt";

    jsonLoader.LoadDescriptionFromFile(filepath);
    for (int i = 0; i < N; i++) {
        auto art = scene->CreateArticulation(&jsonLoader, material, vec3(0, height, 0));
        art->_id = i;
        articulations.push_back(art);
    }

    InitControl();
}

void cleanupPhysics()
{
    foundation->Dispose();
    delete foundation;
}

static long spdTime = 0;
static int controlMethod = 0;

static float frequencyHz = 30;
static float omega = 0.8;
static float amplitude = 1.3;
static float segdist = 0.65f;
static float pos = 0;

float getsin(float x)
{
    return (float)(amplitude * sin(omega * x));
}

float getcos(float x)
{
    return (float)(omega * amplitude * cos(omega * x));
}

void getFrame(vector<float>& motionFrame, int i)
{
    motionFrame = vector<float>(articulations[i]->GetNDof() / 3 * 4 + 7);
    motionFrame[0] = getsin(pos) + i*2;
    motionFrame[1] = articulations[i]->linkMap["s0"]->link->getGlobalPose().p.y;
    motionFrame[2] = pos;

    PxQuat prevQuat(PxIdentity);
    for (int i = 3; i < (int)motionFrame.size(); i += 4) {
        PxQuat rotG(atan(getcos(-(i - 3) / 4 * segdist + pos)), PxVec3(0, 1, 0));
        PxQuat rotL = prevQuat.getConjugate() * rotG;
        motionFrame[i] = rotL.w;
        motionFrame[i + 1] = rotL.x;
        motionFrame[i + 2] = rotL.y;
        motionFrame[i + 3] = rotL.z;
        prevQuat = rotG;
    }
}

static float cumulative_t = 0;
void control(PxReal dt) 
{
    if (cumulative_t > 1.0f / frequencyHz) {
        pos += PxPi * 2 / frequencyHz;
        cumulative_t -= 1.0f / frequencyHz;
    }
    cumulative_t += dt;
    auto starttime = high_resolution_clock::now();

    for (int i = 0; i < (int)articulations.size(); i++) {
        
        auto& articulation = articulations[i];

        vector<float> motionFrame;
        getFrame(motionFrame, i);

        switch (controlMethod)
        {
        case 0:
            articulation->AddSPDForcesABA(motionFrame, dt, true);
            break;
        case 1:
            articulation->AddSPDForcesSparse(motionFrame, dt, true);
            break;
        case 2:
            articulation->AddSPDForces(motionFrame, dt, true);
            break;
        case 3:
            // Kinematic controller still has artifacts for cartwheel.
            // Maybe still due to backend bug.
            articulation->SetJointPositionsQuaternion(motionFrame);
            articulation->SetJointVelocitiesPack4(vector<float>(motionFrame.size(), 0));
            break;
        default:
            break;
        }
    }

    auto endtime = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(endtime - starttime).count();
    spdTime += duration;
}

json getJointNode(int id) {
    json j;
    j["ID"] = id;
    j["Name"] = string("s") + to_string(id);
    j["Type"] = id == 0 ? string("none") : string("spherical");
    j["Parent"] = id - 1;
    j["AttachX"] = 0;
    j["AttachY"] = 0;
    j["AttachZ"] = id == 0 ? 0 : -1.3;
    return j;
}

json getBodyNode(int id) {
    json j;
    j["ID"] = id;
    j["Name"] = string("s") + to_string(id);
    j["Shape"] = string("box");
    j["Mass"] = 1.0;
    j["AttachX"] = 0;
    j["AttachY"] = 0;
    j["AttachZ"] = 0;
    j["Param0"] = 1;
    j["Param1"] = 1;
    j["Param2"] = 1;
    return j;
}

class GlutHandler :public glutRenderer::GlutRendererCallback
{
    void keyboardHandler(unsigned char /*key*/) override
    {
    }
    void beforeSimulationHandler() override
    {
        control(scene->timeStep);
    }
} glutHandler;

int main(int argc, char** argv)
{
    cxxopts::Options opts("Example", "Pose tracking");
    opts.add_options()
        ("p,performance", "Run performance test")
        ("c,control", "Controller", cxxopts::value<int>()->default_value("0"))
        ("L,length", "Length", cxxopts::value<int>()->default_value("0"))
        ("N,num", "how many characters in the scene", cxxopts::value<int>()->default_value("1"))
        ("t,dt", "Simulation time step", cxxopts::value<float>()->default_value("0.033"))
        ("hz", "Control frequency", cxxopts::value<float>()->default_value("30"));

    auto result = opts.parse(argc, argv);

    N = result["num"].as<int>();

    extern int g_nArticulations;
    g_nArticulations = N;

    int snakelength = result["length"].as<int>();
    if (snakelength > 0) {
        string filepath = "resources/snake.txt";
        json joints;
        json bodies;
        for (int i = 0; i < snakelength; i++) {
            joints.push_back(getJointNode(i));
            bodies.push_back(getBodyNode(i));
        }
        json skeleton;
        skeleton["Joints"] = joints;
        json j;
        j["Skeleton"] = skeleton;
        j["BodyDefs"] = bodies;
        ofstream out(filepath);
        out << setw(4) << j << endl;
        out.close();
    }
    
    controlMethod = result["control"].as<int>();
    frequencyHz = result["hz"].as<float>();

    float dt = result["dt"].as<float>();
    initPhysics(dt);

    for (int i = 0; i < (int)articulations.size(); i++) {
        auto& articulation = articulations[i];
        vector<float> motionFrame;
        getFrame(motionFrame, i);
        articulation->SetJointPositionsQuaternion(motionFrame);
    }

    if (result["performance"].as<bool>()) {
        static const PxU32 frameCount = 10000;
        auto starttime = high_resolution_clock::now();
        for(PxU32 i = 0; i < frameCount; i++) {
            control(scene->timeStep);
            scene->Step();
        }
        auto endtime = high_resolution_clock::now();
        auto duration = duration_cast<microseconds>(endtime - starttime).count();
        printf("total time is %ld\n", duration);
        extern long g_ABA_Timer;
        printf("aba time is %ld\n", g_ABA_Timer);
        printf("spd time is %ld\n", spdTime);
    }
    else {
#ifdef ENABLE_UNITY_KINEMATICS
        UR_Init(scene->timeStep, "localhost", 8080, "localhost", 8081);
        for (auto& articulation : articulations)
            UR_AddArticulation(articulation);
        UR_InitPrimitives();
        for (;;) {
            auto starttime = high_resolution_clock::now();
            control(scene->timeStep);
            scene->Step();
            auto endtime = high_resolution_clock::now();
            auto duration = duration_cast<microseconds>(endtime - starttime).count();
            UR_Tick((double)duration / 1e6);
        }
        cleanupPhysics();
#else
        auto renderer = glutRenderer::GlutRenderer::GetInstance();
        renderer->AttachScene(scene, &glutHandler);
        renderer->StartRenderLoop();
#endif
    }
}
