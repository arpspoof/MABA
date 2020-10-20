#pragma once

#include "Loader.h"
#include "MathInterface.h"
#include "LinkBody.h"
#include "json.hpp"

/**
 * @brief Loader for JSON skeleton files
 * @warning Do not attempt to load files other than those with all 
 *  links aligned with global X axis in zero pose.
 *  Blame PhysX if this brings any inconvenience...
 * 
 */
class JsonLoader :public Loader
{
// API BEGIN
public:
    /**
     * @brief Construct an empty loader. Call LoadDescriptionFromFile
     *  to load a new URDF file.
     * @param scalingFactor Specify this parameter to scale the model. 
     * 
     */
    JsonLoader(float scalingFactor = 1.0f);
    /**
     * @brief Dispose all resources allocated by the loader. If a Json loader
     *  is created in your code logic, you must call this when it is not needed.
     * 
     */
    void Dispose() override;
    /**
     * @brief Load a new skeleton description.
     * @note Old skeleton description will be discarded.
     * @param path File path to JSON skeleton file
     */
    void LoadDescriptionFromFile(std::string path);
// API END
public:
    void BuildArticulationTree(ArticulationTree& tree, Material* material) override;
private:
    nlohmann::json j;
private:
    std::unordered_map<std::string, LinkBody*> linkBodyMap;
    std::unordered_map<std::string, vec3> linkOffsetMap;
    void ParseLinkBodies(ArticulationTree& tree, Material* material);
    void ParseJoints(ArticulationTree& tree);
};
