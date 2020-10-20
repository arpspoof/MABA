%{
    #include "JsonLoader.h"
%}

class JsonLoader :public Loader
{
// API BEGIN
public:
    JsonLoader(float scalingFactor = 1.0f);
    void Dispose() override;
    void LoadDescriptionFromFile(std::string path);
// API END
};
