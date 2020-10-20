%{
    #include "IDisposable.h"
%}

// Interface only
// API BEGIN
class IDisposable
{
public:
    virtual void Dispose() = 0;
    virtual ~IDisposable() {}
};

// API END
