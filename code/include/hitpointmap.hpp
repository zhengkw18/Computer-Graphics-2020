#ifndef HITPOINTMAP_H
#define HITPOINTMAP_H

#include <algorithm>
#include <mutex>
#include <vecmath.h>

class Photon;
class Object3D;
class HitpointKDTreeNode;

class Hitpoint {
public:
    Vector3f pos, dir, N;
    Object3D* object;
    int cr;
    Vector3f weight;
    double R2;
    double num, deltanum;
    Vector3f color;

    Hitpoint();
    void includePhoton(const Photon& photon);
};

class HitpointMap {
    static int D;
    int size, num;
    Hitpoint* hitpoints;
    HitpointKDTreeNode* kdtree;
    static bool Hitpointcmp(const Hitpoint& h1, const Hitpoint& h2);

    void build(int now, int l, int r);
    void locatePhoton(int i, const Photon& photon);
    void MaintainKDTreeNodeR2(int i);

public:
    HitpointMap(int _size);
    ~HitpointMap();
    void MaintainHitpoints();
    void Store(const Hitpoint& hitpoint);
    void SetupKDTree();
    void includePhoton(const Photon& photon);
    int getNum() const;
    Hitpoint* getHitpoints() const;
};

#endif