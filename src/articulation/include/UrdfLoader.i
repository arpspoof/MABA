%{
    #include "UrdfLoader.h"
%}

%include "std_string.i"

class UrdfLoader :public Loader
{
// API BEGIN
public:
    UrdfLoader(float scalingFactor = 1.0f);
    void Dispose() override;
    void LoadDescriptionFromFile(std::string path);
// API END
};
