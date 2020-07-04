#include "photonmap.hpp"
#include "kdtree.hpp"
#include "photon.hpp"
#include "ray.hpp"
#include "utils.h"
#include <algorithm>
#include <mutex>

std::mutex PhotonMapMtx;

int PhotonMap::D;

bool PhotonMap::Photoncmp(const Photon& p1, const Photon& p2)
{
    return p1.pos[D] < p2.pos[D];
}

void PhotonMap::build(int now, int l, int r)
{
    if (l + 1 == r) {
        kdtree[now].isleaf = true;
        kdtree[now].id = l;
        kdtree[now].boundingbox.box_max = kdtree[now].boundingbox.box_min = photons[l].pos;
        return;
    } else
        kdtree[now].isleaf = false;
    int x = (l + r + 1) >> 1;
    Vector3f avg = Vector3f::ZERO;
    for (int i = l; i < r; i++) {
        avg += photons[i].pos;
    }
    avg = avg / (r - l);
    Vector3f variance = Vector3f::ZERO;
    Vector3f delta;
    for (int i = l; i < r; i++) {
        delta = photons[i].pos - avg;
        variance += delta * delta;
    }
    D = kdtree[now].d = variance.maxD();
    std::nth_element(photons + l, photons + x, photons + r, Photoncmp);
    int ls = (now << 1) + 1, rs = (now << 1) + 2;
    build(ls, l, x);
    build(rs, x, r);
    kdtree[now].boundingbox.box_max = Vector3f::max(kdtree[ls].boundingbox.box_max, kdtree[rs].boundingbox.box_max);
    kdtree[now].boundingbox.box_min = Vector3f::min(kdtree[ls].boundingbox.box_min, kdtree[rs].boundingbox.box_min);
}
Vector3f PhotonMap::include(int i, const Ray& ray, double maxdist)
{
    if (kdtree[i].isleaf) {
        Photon p = photons[kdtree[i].id];
        Vector3f V = p.pos - ray.getOrigin();
        if (Vector3f::dot(V, ray.getDirection()) > maxdist)
            return Vector3f();
        if (!VOLUMETRIC_DECAY)
            return p.power;
        double dist = p.step * VOLUMETRIC_STEP;
        if (dist < 1)
            dist = 1;
        return p.power / (dist * dist);
    } else {
        int ls = (i << 1) + 1, rs = (i << 1) + 2;
        return include(ls, ray, maxdist) + include(rs, ray, maxdist);
    }
}
Vector3f PhotonMap::locateRay(int i, const Ray& ray, double maxdist)
{
    Vector3f ret;
    if (kdtree[i].boundingbox.insideRay(ray))
        return include(i, ray, maxdist);
    int ls = (i << 1) + 1, rs = (i << 1) + 2;
    if (kdtree[ls].boundingbox.nearRay(ray))
        ret += locateRay(ls, ray, maxdist);
    if (kdtree[rs].boundingbox.nearRay(ray))
        ret += locateRay(rs, ray, maxdist);
    return ret;
}

PhotonMap::PhotonMap(int size)
{
    max_photons = size;
    stored_photons = 0;
    photons = new Photon[size];
    kdtree = new KDTreeNode[size << 2];
}
PhotonMap::~PhotonMap()
{
    delete[] photons;
    delete[] kdtree;
}

int PhotonMap::getStoredPhotons() const { return stored_photons; }

Vector3f PhotonMap::getIrradiance(const Ray& ray, double maxdist)
{
    return locateRay(0, ray, maxdist) * VOLUMETRIC_COEFFICIENT;
}
void PhotonMap::Store(const Photon& photon)
{
    PhotonMapMtx.lock();
    if (stored_photons < max_photons) {
        photons[stored_photons] = photon;
        stored_photons++;
    }
    PhotonMapMtx.unlock();
}
void PhotonMap::SetupKDTree()
{
    printf("Building Photon KDTree...\n");
    build(0, 0, stored_photons);
    printf("Photon KDTree Built...\n");
}