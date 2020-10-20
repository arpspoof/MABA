
#include <ctype.h>
#include <vector>
#include <stdio.h>
#include <chrono>

#include "PxPhysicsAPI.h"
#include "ArticulationTree.h"

#include <iostream>
#include <vector>
#include <fstream>
#include <algorithm>
#ifdef ENABLE_UNITY_KINEMATICS
#include "UnityRenderer.h"
#endif
#include "GlutRenderer.h"
#include "MathInterface.h"
#include "PrimitiveObjects.h"
#include "Foundation.h"
#include "UrdfLoader.h"
#include "JsonLoader.h"
#include "Scene.h"
#include "Actor.h"
#include "cxxopts.hpp"
#include "json.hpp"

using namespace physx;
using namespace std;
using namespace std::chrono;
using namespace nlohmann;

static Foundation* foundation;
static Scene* scene;
static Material* material;

static int columns = 1;
static float spacingZ = 6; //4
static float spacingX = 2; //2

int N = 1;
static vector<Articulation*> articulations;

static float kp, kd, rkpA, rkdA, rkpL, rkdL;
static vector<vector<float>> motions;
static int xFrame = 0;

static bool useDog;

static int dim;

static long spdTime = 0;

void reset()
{
    for (auto& articulation : articulations) {
        dim = 7;
        auto dofs = articulation->GetJointDofsInIdOrder();

        vector<float> p { 0, 3.55f, 0, 1, 0, 0, 0 };
        for (int d : dofs) {
            if (d == 1) {
                dim++;
                p.push_back(0);
            }
            else if (d == 3) {
                dim += 4;
                p.push_back(1);
                p.push_back(0);
                p.push_back(0);
                p.push_back(0);
            }
        }

        if (dim == 95) {
            // dog
            p[1] = 2.781f;
            float r2 = 0.70710678118f;
            int legs[4] = { 23, 39, 55, 71 };
            for (int i = 0; i < 4; i++) {
                p[legs[i]] = r2;
                p[legs[i] + 3] = -r2;
            }
        }

        vector<float> v(dim, 0);
        articulation->SetJointVelocitiesPack4(v);
        articulation->SetJointPositionsQuaternion(p);
    }
}

void InitControl() {
    for (auto& articulation : articulations) {
        vector<float> kps, kds, fls;

        for (auto &j : articulation->GetAllJointsInIdOrder()) {
            int nDof = j->nDof;

            for (int i = 0; i < nDof; i++) {
                kps.push_back(kp);
                kds.push_back(kd);
            }
        }

        articulation->SetKPs(kps);
        articulation->SetKDs(kds);

        float rootKpL = rkpL, rootKdL = rkdL;
        float rootKpA = rkpA, rootKdA = rkdA;
        // 2000, 1600 is a good combo for walking (t = 0.033, kp = 75000, kd = 6600)
        articulation->root_kps = { rootKpL, rootKpL, rootKpL, rootKpA, rootKpA, rootKpA };
        articulation->root_kds = { rootKdL, rootKdL, rootKdL, rootKdA, rootKdA, rootKdA };
    }
    reset();
}

void initPhysics(float dt)
{
    foundation = new Foundation();
    scene = foundation->CreateScene(SceneDescription(), dt);

    material = scene->CreateMaterial(1.f, 1.f, 0.f);

    auto plane = scene->CreatePlane(material, vec3(0, 1, 0), 0);
    plane->SetupCollisionFiltering(1, 2 | 4);

    if (useDog) {
        JsonLoader jsonLoader(4);
        jsonLoader.LoadDescriptionFromFile("resources/dog3d.txt");
        for (int i = 0; i < N; i++) {
            auto articulation = scene->CreateArticulation(&jsonLoader, material, vec3(0, 3.25f, 0));
            articulation->_id = i;
            articulations.push_back(articulation);
        }
    }
    else {
        for (int i = 0; i < N; i++) {
            auto articulation = scene->CreateArticulation("resources/humanoid.urdf", material, vec3(0, 2.781f, 0));
            articulation->_id = i;
            articulations.push_back(articulation);
        }
    }

    extern int g_nArticulations;
    printf("test1\n");
    g_nArticulations = N;
    printf("test2\n");

    InitControl();
}
    
void cleanupPhysics()
{
    foundation->Dispose();
    delete foundation;
}

static int controller = 0; // 0-ABA 1-Sparse 2-Dense
static float trackingFrequency = 0.033f;

void control(PxReal dt) {
    static int currentRound = 0;
    static float cumulateTime = 0;
    
    auto starttime = high_resolution_clock::now();

    for (int i = 0; i < N; i++) {
        auto& articulation = articulations[i];

        auto motionFrame = motions[xFrame];
        motionFrame[0] += (motions[motions.size()-1][0] - motions[0][0]) * currentRound;
        motionFrame[2] += (motions[motions.size()-1][2] - motions[0][2]) * currentRound;

        motionFrame[0] += spacingX*(i / columns);
        motionFrame[2] += spacingZ*(i % columns);

        switch (controller)
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
            articulation->SetJointPositionsQuaternion(motionFrame);
            articulation->SetJointVelocitiesPack4(vector<float>(dim, 0));
            break;
        default:
            break;
        }
    }
    
    auto endtime = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(endtime - starttime).count();
    spdTime += duration;

    int cycleLen = motions.size();

    cumulateTime += dt;
    if (cumulateTime >= (currentRound * cycleLen + xFrame + 1) * trackingFrequency) {
        xFrame = (xFrame + 1) % cycleLen;
        if (xFrame == 0) {
            currentRound++;
        }
    }
}

#include <iomanip>

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
        ("d,dog", "Use dog model")
        ("p,performance", "Run performance test")
        ("c,control", "SPD controller: 0=MABA, 1=Sparse, 2=Dense, 3=Kinematic", cxxopts::value<int>()->default_value("0"))
        ("column", "choose number of columns if there are many characters", cxxopts::value<int>()->default_value("1"))
        ("N,num", "how many characters in the scene", cxxopts::value<int>()->default_value("1"))
        ("m,mocap", "Mocap data name, like `run.txt`, see resources folder", cxxopts::value<string>()->default_value("run.txt"))
        ("f,frequency", "Control frequency, default 30Hz", cxxopts::value<float>()->default_value("30"))
        ("t,dt", "Simulation time step", cxxopts::value<float>()->default_value("0.016667"))
        ("kp", "Internal joint kp", cxxopts::value<float>()->default_value("750000"))
        ("kd", "Internal joint kd", cxxopts::value<float>()->default_value("40000"))
        ("rkpa", "Root kp angular", cxxopts::value<float>()->default_value("200000"))
        ("rkda", "Root kd angular", cxxopts::value<float>()->default_value("20000"))
        ("rkpl", "Root kp linear", cxxopts::value<float>()->default_value("200000"))
        ("rkdl", "Root kd linear", cxxopts::value<float>()->default_value("20000"));
    
    auto result = opts.parse(argc, argv);
    N = result["num"].as<int>();
    columns = result["column"].as<int>();

    useDog = result["dog"].as<bool>();

    kp = result["kp"].as<float>();
    kd = result["kd"].as<float>();
    rkpA = result["rkpa"].as<float>();
    rkdA = result["rkda"].as<float>();
    rkpL = result["rkpl"].as<float>();
    rkdL = result["rkdl"].as<float>();
    controller = result["control"].as<int>();
    trackingFrequency = 1.0f / result["frequency"].as<float>();

    initPhysics(result["dt"].as<float>());

    string mocap = result["mocap"].as<string>();
    printf("mocap is %s\n", mocap.c_str());

    ifstream motioninput("./resources/motions/" + mocap);
    string motionStr((istreambuf_iterator<char>(motioninput)), istreambuf_iterator<char>());
    
    auto motionJson = json::parse(motionStr);
    auto frames = motionJson["Frames"];

    for (auto frame : frames) {
        motions.push_back(vector<float>(dim));
        for (int i = 0; i < dim; i++) {
            float value = frame[i + 1]; 
            if (i <= 2) value *= 4;
            motions.back()[i] = value;
        }
    }

    motioninput.close();
    printf("%ld lines\n", motions.size());

    for (int i = 0; i < (int)articulations.size(); i++) {
        auto motion = motions[0];
        motion[0] += spacingX*(i / columns);
        motion[2] += spacingZ*(i % columns);
        articulations[i]->SetJointPositionsQuaternion(motion);
    }

    if (!useDog) {
        spacingX = spacingZ;
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
        UR_Init(scene->timeStep, "localhost, 8080, "localhost", 8081);
        for (auto& articulation : articulations)
            UR_AddArticulation(articulation);
        UR_InitPrimitives();
        for(;;) {
            auto starttime = high_resolution_clock::now();
            control(scene->timeStep);
            scene->Step();
            auto endtime = high_resolution_clock::now();
            auto duration = duration_cast<microseconds>(endtime - starttime).count();
            UR_Tick((double)duration / 1e6);
        }
#else
        auto renderer = glutRenderer::GlutRenderer::GetInstance();
        renderer->AttachScene(scene, &glutHandler);
        renderer->StartRenderLoop();
#endif
        cleanupPhysics();
    }

    return 0;
}
