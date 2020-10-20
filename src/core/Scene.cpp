#include "Scene.h"
#include "Foundation.h"
#include "UrdfLoader.h"

#include <cassert>

using namespace std;
using namespace physx;

SceneDescription::SceneDescription()
{
    gravity = -9.81f;
    nWorkerThreads = 0;
    enableGPUDynamics = false;
    enableGPUBroadPhase = false;
}

Scene::Scene()
{
    // For API binding only. 
    assert(false);
}

Scene::Scene(Foundation* foundation, SceneDescription description, float timeStep)
    :foundation(foundation)
{
    PxSceneDesc pxSceneDesc(foundation->GetPxPhysics()->getTolerancesScale());

    pxSceneDesc.gravity = PxVec3(0.f, description.gravity, 0.f);
    pxSceneDesc.solverType = PxSolverType::eTGS;
    pxSceneDesc.filterShader = CollisionShader;
    pxSceneDesc.simulationEventCallback = this;

    auto cudaContextManager = foundation->GetPxCudaContextManager();
    if (cudaContextManager) {
        pxSceneDesc.cudaContextManager = cudaContextManager;
    }

    pxCpuDispatcher = PxDefaultCpuDispatcherCreate(description.nWorkerThreads);
    pxSceneDesc.cpuDispatcher = pxCpuDispatcher;

    if (description.enableGPUDynamics)
        pxSceneDesc.flags |= PxSceneFlag::eENABLE_GPU_DYNAMICS;
    if (description.enableGPUBroadPhase)
        pxSceneDesc.broadPhaseType = PxBroadPhaseType::eGPU;
    
    pxScene = foundation->GetPxPhysics()->createScene(pxSceneDesc);
    this->timeStep = timeStep;
}

void Scene::Dispose()
{
    printf("disposing materials, actors, articulations ...\n");
    for (auto &p : materials) {
        delete p;
    }
    for (auto &p : actors) {
        if (p->isReleasable()) {
            p->release();
        }
    }
    for (auto &p : articulations) {
        p->Dispose();
        delete p;
    }
    pxScene->release();
    pxCpuDispatcher->release();
}

Material* Scene::CreateMaterial(float staticFriction, float dynamicFriction, float restitution)
{
    Material* material = new Material();
    material->pxMaterial = foundation->GetPxPhysics()->createMaterial(staticFriction, dynamicFriction, restitution);
    materials.insert(material);
    return material;
}

Plane* Scene::CreatePlane(Material* material, vec3 planeNormal, float distance)
{
    Plane* plane = new Plane();
    plane->pxActor = PxCreatePlane(
        *foundation->GetPxPhysics(), 
        PxPlane(planeNormal.x, planeNormal.y, planeNormal.z, distance), 
        *material->pxMaterial
    );
    actors.insert(plane->pxActor);
    pxScene->addActor(*plane->pxActor);
    return plane;
}

Articulation* Scene::CreateArticulation(Loader* loader, Material* material, vec3 basePosition)
{
    ArticulationTree tree;
    loader->BuildArticulationTree(tree, material);

    Articulation* articulation = new Articulation();
    articulation->pxArticulation = foundation->GetPxPhysics()->createArticulationReducedCoordinate();
    articulation->frameTransform = tree.frameTransform;

    assert(tree.GetRootNode() != nullptr);
    BuildArticulation(*articulation, tree.GetRootNode(), tree.frameTransform, nullptr, basePosition, basePosition);

    pxScene->addArticulation(*articulation->pxArticulation);

    articulation->InitControl(loader->jointIdMap);
    articulations.insert(articulation);

    return articulation;
}

Articulation* Scene::CreateArticulation(string urdfFilePath, Material* material, 
    vec3 basePosition, float scalingFactor)
{
    UrdfLoader urdfLoader(scalingFactor);
    urdfLoader.LoadDescriptionFromFile(urdfFilePath);
    Articulation* articulation = CreateArticulation(&urdfLoader, material, basePosition);
    urdfLoader.Dispose();
    return articulation;
}

void Scene::BuildArticulation(Articulation &ar, ArticulationDescriptionNode* startNode, physx::PxQuat frameTransform,
    Link* parentLink, physx::PxVec3 parentJointPos, physx::PxVec3 parentLinkPos) const
{
    Link *link = startNode->CreateLink(ar, parentLink, frameTransform, parentJointPos, parentLinkPos);
    for (auto it : startNode->children) {
        BuildArticulation(ar, it, frameTransform, link,
            parentJointPos + startNode->posOffsetJointToParentJoint,
            link->globalPositionOffset);
    }
}

void Scene::SetJointVelocitiesPack4AndStep(Articulation* ar, const std::vector<float>& velocities)
{
    ar->SetJointVelocitiesPack4(velocities);
    float tmp = timeStep;
    timeStep = 0.00001f;
    Step();
    timeStep = tmp;
}

void Scene::Step()
{
    contacts.clear();
    pxScene->simulate(timeStep);
    pxScene->fetchResults(true);
    for (auto &ar : articulations) {
        ar->FetchKinematicData();
    }
}

const vector<pair<int, int>>& Scene::GetAllContactPairs() const
{
    return contacts;
}

PxScene* Scene::GetPxScene() const
{
    return pxScene;
}

PxFilterFlags Scene::CollisionShader(
    PxFilterObjectAttributes attributes0, PxFilterData filterData0,
    PxFilterObjectAttributes attributes1, PxFilterData filterData1,
    PxPairFlags& pairFlags, const void* /*constantBlock*/, PxU32 /*constantBlockSize*/)
{
    // let triggers through
    if (PxFilterObjectIsTrigger(attributes0) || PxFilterObjectIsTrigger(attributes1))
    {
        pairFlags = PxPairFlag::eTRIGGER_DEFAULT;
        return PxFilterFlag::eDEFAULT;
    }

    // special collision group 31 is used for collision killing
    if (filterData0.word0 & filterData1.word0 & (1 << 31))
        return PxFilterFlag::eKILL;

    // generate contacts for all that were not filtered above
    pairFlags = PxPairFlag::eCONTACT_DEFAULT;

    // trigger the contact callback for pairs (A,B) where 
    // the filtermask of A contains the ID of B and vice versa.
    if ((filterData0.word0 & filterData1.word1) && (filterData1.word0 & filterData0.word1))
        pairFlags |= PxPairFlag::eNOTIFY_TOUCH_FOUND;

    return PxFilterFlag::eDEFAULT;
}

void Scene::onContact(const PxContactPairHeader &/*pairHeader*/, 
    const PxContactPair *pairs, PxU32 nbPairs) 
{
    for (PxU32 i = 0; i < nbPairs; i++)
    {
        const PxContactPair& cp = pairs[i];

        if (cp.events & PxPairFlag::eNOTIFY_TOUCH_FOUND)
        {
            int collisionGroup0 = cp.shapes[0]->getSimulationFilterData().word0;
            int collisionGroup1 = cp.shapes[1]->getSimulationFilterData().word0;
            contacts.push_back(make_pair(collisionGroup0, collisionGroup1));
        }
    }
}
