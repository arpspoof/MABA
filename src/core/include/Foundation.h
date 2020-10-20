#pragma once

#include "PxPhysicsAPI.h"
#include "IDisposable.h"
#include "Scene.h"

#include <unordered_set>

/**
 * @brief pyPhysX foundation.
 * 
 */
class Foundation :public IDisposable
{
// API BEGIN
public:
    /**
     * @brief Construct a new Foundation object.
     * @note You must destroy it by calling Foundation::Dispose
     *  before exiting your program.
     * 
     */
    Foundation();
    /**
     * @brief Dispose the physics engine and all scenes created.
     *  Call this when releasing resources before exiting the program.
     * 
     */
    void Dispose() override;
    /**
     * @brief Create a new Scene.
     * 
     * @param description Description about how to create the scene.
     * @param timeStep Simulation time step.
     * @return Pointer to the created Scene object. Use this to create
     *  objects in the scene.
     */
    Scene* CreateScene(SceneDescription description, float timeStep);
// API END
public:
    physx::PxPhysics* GetPxPhysics() const;
    physx::PxCudaContextManager* GetPxCudaContextManager() const;
private:
    physx::PxFoundation*            pxFoundation;
    physx::PxPhysics*               pxPhysics;
    physx::PxDefaultAllocator		pxAllocator;
    physx::PxDefaultErrorCallback	pxErrorCallback;
    physx::PxCudaContextManager*    pxCudaContextManager;
    std::unordered_set<Scene*> scenes;
};
