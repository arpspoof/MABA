#pragma once

// Interface only
// API BEGIN

/**
 * @brief Interface for classes holding dynamic resources that requires 
 *  an explicit dispose action
 * 
 */
class IDisposable
{
public:
    /**
     * @brief Dispose all resources held by this object
     * 
     */
    virtual void Dispose() = 0;
    virtual ~IDisposable() {}
};

// API END
