#include "LinkBody.h"
#include <cassert>

using namespace physx;
using namespace std;

// LinkBody

PxGeometry& LinkBody::getGeometry() const
{
    return *geometry;
}

LinkBody::~LinkBody()
{
    delete geometry;
}

LinkBody::LinkBody(float mass, PxGeometry *geometry, Material *material) 
    :hasGeometry(true), mass(mass), geometry(geometry)
{
    this->material = material ? material->pxMaterial : nullptr;
}


// NULLLinkBody

float NULLLinkBody::getDensity() const 
{
    return 1;
}

NULLLinkBody::NULLLinkBody() :LinkBody(1, nullptr, nullptr) 
{
    hasGeometry = false;
    type = "null";
}

PxGeometry& NULLLinkBody::getGeometry() const 
{
    assert(false);
    return *geometry;
}

void NULLLinkBody::FillBodyGeometryData(BodyGeometryData&) const
{
    assert(false);
}


// BoxLinkBody

float BoxLinkBody::getDensity() const 
{
    return mass / (lenX * lenY * lenZ);
}

BoxLinkBody::BoxLinkBody(float mass, float lenX, float lenY, float lenZ, Material *material)
    :LinkBody(mass, new PxBoxGeometry(lenX / 2, lenY / 2, lenZ / 2), material),
    lenX(lenX), lenY(lenY), lenZ(lenZ)
{
    assert(material != nullptr);
    type = "box";
}

void BoxLinkBody::FillBodyGeometryData(BodyGeometryData& data) const
{
    data.type = "box";
    data.param0 = lenX;
    data.param1 = lenY;
    data.param2 = lenZ;
}


// SphereLinkBody

float SphereLinkBody::getDensity() const 
{
    return mass / (4.0f / 3.0f*physx::PxPi*radius*radius*radius);
}

SphereLinkBody::SphereLinkBody(float mass, float radius, Material *material)
    :LinkBody(mass, new physx::PxSphereGeometry(radius), material), radius(radius) 
{
    assert(material != nullptr);
    type = "sphere";
}

void SphereLinkBody::FillBodyGeometryData(BodyGeometryData& data) const
{
    data.type = "sphere";
    data.param0 = radius;
    data.param1 = 0;
    data.param2 = 0;
}


// CapsuleLinkBody

float CapsuleLinkBody::getDensity() const 
{
    return mass / (
        4.0f / 3.0f*physx::PxPi*radius*radius*radius +
        4 * physx::PxPi*radius*radius*length
    );
}

CapsuleLinkBody::CapsuleLinkBody(float mass, float radius, float length, Material *material)
    :LinkBody(mass, new physx::PxCapsuleGeometry(radius, length / 2), material), 
    radius(radius), length(length) 
{
    assert(material != nullptr);
    type = "capsule";
}

void CapsuleLinkBody::FillBodyGeometryData(BodyGeometryData& data) const
{
    data.type = "capsule";
    data.param0 = radius;
    data.param1 = length;
    data.param2 = 0;
}
