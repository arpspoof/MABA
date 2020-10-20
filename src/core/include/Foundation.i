%{
    #include "Foundation.h"
%}

class Foundation :public IDisposable
{
// API BEGIN
public:
    Foundation();
    void Dispose() override;
    Scene* CreateScene(SceneDescription description, float timeStep);
// API END
};
