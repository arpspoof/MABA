%{
    #include "GlutRenderer.h"
%}

%feature("director") glutRenderer::GlutRendererCallback;

namespace glutRenderer
{

class GlutRendererCallback
{
// API BEGIN
public:
    virtual void keyboardHandler(unsigned char key) = 0;
    virtual void beforeSimulationHandler() = 0;
    virtual ~GlutRendererCallback() {}
// API END
};

class GlutRenderer
{
// API BEGIN
public:
    void AttachScene(::Scene* scene, GlutRendererCallback* callback = nullptr);
    void StartRenderLoop();
    static GlutRenderer* GetInstance();
    GlutRenderer(); // For API only
public:
    int renderFrequenceHz;
// API END
};

}
