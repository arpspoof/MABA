//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2019 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include <vector>
#include <chrono>

#include "PxPhysicsAPI.h"

#include "GlutRenderer.h"
#include "GlutRendererLowLevel.h"
#include "GlutCamera.h"

using namespace physx;
using namespace std::chrono;

namespace glutRenderer
{

GlutRenderer GlutRenderer::globalGlutRenderer;

GlutRenderer* GlutRenderer::GetInstance()
{
    return &globalGlutRenderer;
}

GlutRenderer::GlutRenderer()
{
    this->renderFrequenceHz = 60;
    this->scene = nullptr;
    this->callback = nullptr;
    this->glutCamera = nullptr;
    this->simulating = false;
}

void GlutRenderer::AttachScene(::Scene* scene, GlutRendererCallback* callback)
{
    this->scene = scene;
    this->callback = callback;
}

void GlutRenderer::renderScene() {
    startRender(glutCamera->getEye(), glutCamera->getDir());
    
    PxU32 nbActors = scene->GetPxScene()->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC);
    if (nbActors)
    {
        std::vector<PxRigidActor*> actors(nbActors);
        scene->GetPxScene()->getActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC, reinterpret_cast<PxActor**>(&actors[0]), nbActors);
        renderActors(&actors[0], static_cast<PxU32>(actors.size()), true);
    }

    PxU32 nbArticulations = scene->GetPxScene()->getNbArticulations();
    for (PxU32 i = 0; i<nbArticulations; i++)
    {
        PxArticulationBase* articulation;
        scene->GetPxScene()->getArticulations(&articulation, 1, i);

        const PxU32 nbLinks = articulation->getNbLinks();
        std::vector<PxArticulationLink*> links(nbLinks);
        articulation->getLinks(&links[0], nbLinks);

        renderActors(reinterpret_cast<PxRigidActor**>(&links[0]), static_cast<PxU32>(links.size()), true);
    }

    finishRender();
}

void GlutRenderer::motionCallback(int x, int y)
{
    static int test = 0;
    glutCamera->handleMotion(x, y);
    if (test == 0) {
        test = 1;
        return;
    }
    renderScene();
}

void GlutRenderer::keyboardCallback(unsigned char key, int x, int y)
{
    switch (key) {
    case 27:
        exit(0); return;
    case ' ':
        simulating = !simulating; return;
    }

    if(!glutCamera->handleKey(key, x, y) && callback != nullptr)
        callback->keyboardHandler(key);

    renderScene();
}

void GlutRenderer::mouseCallback(int button, int state, int x, int y)
{
    glutCamera->handleMouse(button, state, x, y);
}

void GlutRenderer::idleCallback()
{
    glutPostRedisplay();
}

void GlutRenderer::renderCallback()
{
    static int phyFrameCount = 10000;
    static int rendercount = 59;
    static high_resolution_clock::time_point starttime;

    if (simulating) {
        if (callback)
            callback->beforeSimulationHandler();
        scene->Step();
        phyFrameCount++;
    }

    int framesPerRender = round(1.0 / scene->timeStep / renderFrequenceHz);
    if (phyFrameCount >= framesPerRender) {
        phyFrameCount = 0;
    } 
    else {
        return;
    }

    renderScene();

    rendercount++;
    if (rendercount == 60) {
        rendercount = 0;
        auto endtime = high_resolution_clock::now();
        auto duration = endtime - starttime;
        printf("fps: %.2lf\n", 1e9 * 60.0 / (double)duration.count());
        starttime = high_resolution_clock::now();
    }
}

void GlutRenderer::exitCallback()
{
    delete glutCamera;
}

void GlutRenderer::StartRenderLoop()
{
    const PxVec3 camEyeLift(8.0f, 4.050591f, 0);
    const PxVec3 camDirLift(-8.0, -0, -0);
    glutCamera = new GlutCamera(camEyeLift, camDirLift);

    setupDefaultWindow("PhysX Application");
    setupDefaultRenderState();

    glutIdleFunc([]() { globalGlutRenderer.idleCallback(); });
    glutDisplayFunc([]() { globalGlutRenderer.renderCallback(); });
    glutKeyboardFunc([](unsigned char key, int x, int y) { globalGlutRenderer.keyboardCallback(key, x, y); });
    glutMouseFunc([](int button, int state, int x, int y) { globalGlutRenderer.mouseCallback(button, state, x, y); });
    glutMotionFunc([](int x, int y) { globalGlutRenderer.motionCallback(x, y); });
    motionCallback(0,0);
    atexit([]() { globalGlutRenderer.exitCallback(); });

    glutMainLoop();
}

}
