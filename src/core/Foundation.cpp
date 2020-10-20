#include "Foundation.h"

using namespace physx;

Foundation::Foundation()
{
    pxFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, pxAllocator, pxErrorCallback);
    pxPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *pxFoundation, PxTolerancesScale(), true, nullptr);
    
    PxInitExtensions(*pxPhysics, nullptr);
#if 0
    PxCudaContextManagerDesc cudaContextManagerDesc;
    pxCudaContextManager = PxCreateCudaContextManager(*pxFoundation, cudaContextManagerDesc, PxGetProfilerCallback());
#else
    pxCudaContextManager = nullptr;
#endif
}

void Foundation::Dispose()
{
    printf("disposing scenes ...\n");
    for (auto &p : scenes) {
        p->Dispose();
        delete p;
    }
    printf("disposing foundation ...\n");
    pxPhysics->release();
    PxCloseExtensions();
    pxFoundation->release();
}

Scene* Foundation::CreateScene(SceneDescription description, float timeStep)
{
    Scene* scene = new Scene(this, description, timeStep);
    scenes.insert(scene);
    return scene;
}

PxPhysics* Foundation::GetPxPhysics() const 
{
    return pxPhysics;
}

PxCudaContextManager* Foundation::GetPxCudaContextManager() const
{
    return pxCudaContextManager;
}
