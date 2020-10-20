#include "Articulation.h"
#include "Foundation.h"

#include <algorithm>

using namespace physx;
using namespace std;

Link* Articulation::AddLink(std::string name, Link *parent, physx::PxTransform transform, LinkBody *body) 
{
    Link *link = new Link(pxArticulation, parent, transform, body);
    linkMap[name] = link;
    return link;
}

Joint* Articulation::AddSpericalJoint(std::string name, Link *link,
    physx::PxTransform parentPose, physx::PxTransform childPose) 
{
    Joint *joint = new SphericalJoint(link, parentPose, childPose);
    jointMap[name] = joint;
    return joint;
}

Joint* Articulation::AddRevoluteJoint(std::string name, Link *link, physx::PxArticulationAxis::Enum axis,
    physx::PxTransform parentPose, physx::PxTransform childPose) 
{
    Joint *joint = new RevoluteJoint(link, axis, parentPose, childPose);
    jointMap[name] = joint;
    return joint;
}

Joint* Articulation::AddFixedJoint(std::string name, Link *link, 
    physx::PxTransform parentPose, physx::PxTransform childPose) 
{
    Joint *joint = new FixedJoint(link, parentPose, childPose);
    jointMap[name] = joint;
    return joint;
}

void Articulation::Dispose() 
{
    pxArticulation->release();
    for (auto& it : linkMap) {
        delete it.second;
    }
    for (auto& it : jointMap) {
        delete it.second;
    }
}

void Articulation::InitControl(unordered_map<string, int>& jointIdMap)
{
    AssignIndices();

    for (auto &kvp : linkMap) {
        kvp.second->name = kvp.first;
    }
    for (auto &kvp : jointMap) {
        kvp.second->name = kvp.first;
    }

    mainCache = pxArticulation->createCache();
    massMatrixCache = pxArticulation->createCache();
    coriolisCache = pxArticulation->createCache();
    gravityCache = pxArticulation->createCache();
    externalForceCache = pxArticulation->createCache();

    int nDof = GetNDof();
    kps = vector<float>(nDof, 0);
    kds = vector<float>(nDof, 0);
    forceLimits = vector<float>(nDof, -1);

    typedef tuple<int, Joint*> PIJ;
    vector<PIJ> joints;
    
    for (auto &kvp : jointMap) {
        joints.push_back(make_tuple(jointIdMap[kvp.first], kvp.second));
    }

    sort(joints.begin(), joints.end(), [](PIJ &a, PIJ &b) { return get<0>(a) < get<0>(b); });

    for (PIJ &pijn : joints) {
        auto j = get<1>(pijn);
        j->id = get<0>(pijn);
        jointList.push_back(j);
        jointDofs.push_back(j->nDof);
    }

    parentIndexMapForSparseLTL = vector<int>(nDof + 6 + 1, -1);
    for (int k = 1; k <= 6; k++) parentIndexMapForSparseLTL[k] =  k - 1;

    for (auto &kvp : jointMap) {
        Joint* j = kvp.second;
        if (j->cacheIndex != -1) {
            Joint* p = j->parentLink->inboundJoint;
            while (p && p->cacheIndex == -1) p = p->parentLink->inboundJoint;
            
            if (p) parentIndexMapForSparseLTL[j->cacheIndex + 7] = p->cacheIndex + 6 + p->nDof;
            else parentIndexMapForSparseLTL[j->cacheIndex + 7] = 6;
            
            for (int k = 1; k < j->nDof; k++) {
                parentIndexMapForSparseLTL[j->cacheIndex + 7 + k] = j->cacheIndex + 7 + k - 1;
            }
        }
    }

    printf("parent index\n");
    for (int i = 0; i < (int)parentIndexMapForSparseLTL.size(); i++) {
        printf("%d ", parentIndexMapForSparseLTL[i]);
    }
    printf("\n");
}

void Articulation::AssignIndices() {
    typedef pair<PxU32, Link*> PIDL;
    vector<PIDL> linkIndices;
    for (auto &kvp : linkMap) {
        linkIndices.push_back(make_pair(kvp.second->link->getLinkIndex(), kvp.second));
    }
    sort(linkIndices.begin(), linkIndices.end(), [=](PIDL a, PIDL b) { return a.first < b.first; });

    nSphericalJoint = 0;
    nRevoluteJoint = 0;

    rootLink = nullptr;
    linkIdCacheIndexMap = vector<int>(pxArticulation->getNbLinks(), -1);

    int currentIndex = 0;

    for (PIDL &p : linkIndices) {
        int nDof = (int)p.second->link->getInboundJointDof();
        if (!p.second->inboundJoint) {
            rootLink = p.second;
            continue;
        }
        p.second->inboundJoint->nDof = nDof;
        p.second->inboundJoint->cacheIndex = nDof > 0 ? currentIndex : -1;
        linkIdCacheIndexMap[p.first] = p.second->inboundJoint->cacheIndex;
        currentIndex += nDof;
        if (nDof == 3) nSphericalJoint++;
        if (nDof == 1) nRevoluteJoint++;
    }

    assert(rootLink != nullptr);
}

int Articulation::GetNDof() const
{
    return (int)pxArticulation->getDofs();
}

const std::vector<int>& Articulation::GetJointDofsInIdOrder() const
{
    return jointDofs;
}

void Articulation::SetFixBaseFlag(bool shouldFixBase)
{
    pxArticulation->setArticulationFlag(PxArticulationFlag::eFIX_BASE, shouldFixBase);
}

int Articulation::GetNJoints() const
{
    return (int)jointList.size();
}

void Articulation::SetJointParams(std::vector<float>& target, const std::vector<float>& params)
{
    int index = 0;
    int nJoints = GetNJoints();
    for (int i = 0; i < nJoints; i++) {
        int jointDof = jointList[i]->nDof;
        for (int k = 0; k < jointDof; k++) {
            target[jointList[i]->cacheIndex + k] = params[index + k];
        }
        index += jointDof;
    }
}

void Articulation::FetchKinematicData() const
{
    pxArticulation->copyInternalStateToCache(*mainCache, PxArticulationCache::eALL);
}

const Link* Articulation::GetLinkByName(std::string name) const
{
    const auto& it = linkMap.find(name);
    if (it == linkMap.end()) {
        printf("Error: Link with name %s not found.\n", name.c_str());
        assert(false);
        return nullptr;
    }
    return it->second;
}

const Joint* Articulation::GetJointByName(std::string name) const
{
    const auto& it = jointMap.find(name);
    if (it == jointMap.end()) {
        printf("Error: Joint with name %s not found.\n", name.c_str());
        assert(false);
        return nullptr;
    }
    return it->second;
}

const vector<Joint*>& Articulation::GetAllJointsInIdOrder() const
{
    return jointList;
}

const Link* Articulation::GetRootLink() const 
{
    return rootLink;
}

void Articulation::SetKPs(const std::vector<float>& kps)
{
    SetJointParams(this->kps, kps);
}

void Articulation::SetKDs(const std::vector<float>& kds)
{
    SetJointParams(this->kds, kds);
}

void Articulation::SetForceLimits(const std::vector<float>& forceLimits)
{
    SetJointParams(this->forceLimits, forceLimits);
}
