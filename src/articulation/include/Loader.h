#pragma once

#include "ArticulationTree.h"
#include "PrimitiveObjects.h"
#include "IDisposable.h"

#include <string>
#include <unordered_map>

/**
 * @brief Generic skeleton loader
 * 
 */
class Loader :public IDisposable
{
public:
    float scalingFactor;
    std::unordered_map<std::string, int> jointIdMap;
    virtual void BuildArticulationTree(ArticulationTree& tree, Material* material) = 0;
    virtual ~Loader() {}
};
