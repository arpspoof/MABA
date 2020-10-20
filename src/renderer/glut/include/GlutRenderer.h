#pragma once

#include "Scene.h"
#include "GlutCamera.h"
#include <functional>

namespace glutRenderer
{

/**
 * @brief Callback functions for GlutRenderer. Please inherit
 *  this class and override all abstract methods.
 * 
 */
class GlutRendererCallback
{
// API BEGIN
public:
    /**
     * @brief Handler key press event.
     * 
     * @param key Character code of the key.
     */
    virtual void keyboardHandler(unsigned char key) = 0;
    /**
     * @brief Callback before each simulation step starts.
     * 
     */
    virtual void beforeSimulationHandler() = 0;
    virtual ~GlutRendererCallback() {}
// API END
};

/**
 * @brief A Scene renderer using Glut
 * @warning This will be deprecated sooner or later for
 *  its ugly user interface.
 * @note GlutRenderer has to be a singleton. Do not try
 *  to construct an object directly. Call GlutRenderer::GetInstance
 *  to get the singleton instance instead.
 * 
 */
class GlutRenderer
{
// API BEGIN
public:
    /**
     * @brief Attach the renderer to a scene.
     * 
     * @param scene Pointer to the scene.
     * @param callback Pointer to a callback function class object. This
     *  callback function class has to be derived from GlutRendererCallback.
     */
    void AttachScene(::Scene* scene, GlutRendererCallback* callback = nullptr);
    /**
     * @brief Start the render loop. Press space to start / pause.
     * 
     */
    void StartRenderLoop();
    /**
     * @brief Get the global singleton instance object.
     * 
     * @return GlutRenderer* Pointer to the global singleton renderer.
     */
    static GlutRenderer* GetInstance();
    GlutRenderer(); // For API only
public:
    int renderFrequenceHz;
// API END
private:
    static GlutRenderer globalGlutRenderer;
private:
    ::Scene* scene;
    GlutCamera* glutCamera;
    GlutRendererCallback* callback;
    bool simulating;
private:
    void renderScene();
    void motionCallback(int x, int y);
    void keyboardCallback(unsigned char key, int x, int y);
    void mouseCallback(int button, int state, int x, int y);
    void idleCallback();
    void renderCallback();
    void exitCallback();
};

}
