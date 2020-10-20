#pragma once

#include "Articulation.h"

void UR_Init(float timeStep, 
    const std::string remoteIP, 
    unsigned short remotePort,
    const std::string localIP,
    unsigned short localPort);

void UR_Tick(double duration);
void UR_Stop();

void UR_AddArticulation(Articulation* ar);

void UR_InitPrimitives();
void UR_SetPhysicalFPS(int fps);
