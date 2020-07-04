#include "hitpointmap.hpp"
#include "kdtree.hpp"
#include "object3d.hpp"
#include "photon.hpp"
#include "utils.h"

std::mutex HitpointMapMtx;

int HitpointMap::D;

Hitpoint::Hitpoint()
{
    object = nullptr;
    weight = Vector3f(1, 1, 1);
    cr = -1;
    R2 = num = deltanum = 0;
}
void Hitpoint::includePhoton(const Photon& photon)
{
    deltanum += 1;
    color += photon.power * object->getMaterial()->BRDF(-dir, N, -photon.dir);
}

bool HitpointMap::Hitpointcmp(const Hitpoint& h1, const Hitpoint& h2)
{
    return h1.pos[D] < h2.pos[D];
}

void HitpointMap::build(int now, int l, int r)
{
    if (l + 1 == r) {
        kdtree[now].isleaf = true;
        kdtree[now].id = l;
        kdtree[now].boundingbox.box_max = kdtree[now].boundingbox.box_min = hitpoints[l].pos;
        kdtree[now].maxR2 = hitpoints[l].R2;
        return;
    } else
        kdtree[now].isleaf = false;
    int x = (l + r + 1) >> 1;
    Vector3f avg = Vector3f::ZERO;
    for (int i = l; i < r; i++) {
        avg += hitpoints[i].pos;
    }
    avg = avg / (r - l);
    Vector3f variance = Vector3f::ZERO;
    Vector3f delta;
    for (int i = l; i < r; i++) {
        delta = hitpoints[i].pos - avg;
        variance += delta * delta;
    }
    D = kdtree[now].d = variance.maxD();
    std::nth_element(hitpoints + l, hitpoints + x, hitpoints + r, Hitpointcmp);
    int ls = (now << 1) + 1, rs = (now << 1) + 2;
    build(ls, l, x);
    build(rs, x, r);
    kdtree[now].boundingbox.box_max = Vector3f::max(kdtree[ls].boundingbox.box_max, kdtree[rs].boundingbox.box_max);
    kdtree[now].boundingbox.box_min = Vector3f::min(kdtree[ls].boundingbox.box_min, kdtree[rs].boundingbox.box_min);
    kdtree[now].maxR2 = std::max(kdtree[ls].maxR2, kdtree[rs].maxR2);
}
void HitpointMap::locatePhoton(int i, const Photon& photon)
{
    if (kdtree[i].isleaf) {
        if (Vector3f::distance2(photon.pos, hitpoints[kdtree[i].id].pos) < hitpoints[kdtree[i].id].R2)
            hitpoints[kdtree[i].id].includePhoton(photon);
        return;
    }
    int ls = (i << 1) + 1, rs = (i << 1) + 2;
    if ((kdtree[ls].boundingbox.box_min - photon.pos).max() < sqrt(kdtree[ls].maxR2) && (photon.pos - kdtree[ls].boundingbox.box_max).max() < sqrt(kdtree[ls].maxR2)) {
        locatePhoton(ls, photon);
    }
    if ((kdtree[rs].boundingbox.box_min - photon.pos).max() < sqrt(kdtree[rs].maxR2) && (photon.pos - kdtree[rs].boundingbox.box_max).max() < sqrt(kdtree[rs].maxR2)) {
        locatePhoton(rs, photon);
    }
}
void HitpointMap::MaintainKDTreeNodeR2(int i)
{
    if (kdtree[i].isleaf)
        kdtree[i].maxR2 = hitpoints[kdtree[i].id].R2;
    else {
        int ls = (i << 1) + 1, rs = (i << 1) + 2;
        MaintainKDTreeNodeR2(ls);
        MaintainKDTreeNodeR2(rs);
        kdtree[i].maxR2 = std::max(kdtree[ls].maxR2, kdtree[rs].maxR2);
    }
}

HitpointMap::HitpointMap(int _size)
{
    size = _size;
    num = 0;
    hitpoints = new Hitpoint[size];
    kdtree = new HitpointKDTreeNode[size << 2];
}
HitpointMap::~HitpointMap()
{
    delete[] hitpoints;
    delete[] kdtree;
}
void HitpointMap::MaintainHitpoints()
{
    for (int i = 0; i < num; i++) {
        Hitpoint* hp = &hitpoints[i];
        if (hp->deltanum < EPS)
            continue;
        double k = (hp->num + REDUCTION * hp->deltanum) / (hp->num + hp->deltanum);
        hp->R2 *= k;
        hp->color *= k;
        hp->num += REDUCTION * hp->deltanum;
        hp->deltanum = 0;
    }
    MaintainKDTreeNodeR2(0);
}
void HitpointMap::Store(const Hitpoint& hitpoint)
{
    HitpointMapMtx.lock();
    if (num < size) {
        hitpoints[num] = hitpoint;
        num++;
    }
    HitpointMapMtx.unlock();
}
void HitpointMap::SetupKDTree()
{
    printf("Building Hitpoint KDTree...\n");
    build(0, 0, num);
    printf("Hitpoint KDTree Built\n");
}
void HitpointMap::includePhoton(const Photon& photon)
{
    locatePhoton(0, photon);
}
int HitpointMap::getNum() const { return num; }
Hitpoint* HitpointMap::getHitpoints() const { return hitpoints; }