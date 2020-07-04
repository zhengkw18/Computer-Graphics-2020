#ifndef PHOTONMAP_H
#define PHOTONMAP_H

#include <vecmath.h>

class Photon;
class ObjectHit;
class Ray;
class KDTreeNode;

class PhotonMap {
    static int D;
    int max_photons, stored_photons;

    Photon* photons;
    KDTreeNode* kdtree;
    static bool Photoncmp(const Photon& p1, const Photon& p2);

    void build(int now, int l, int r);
    Vector3f locateRay(int i, const Ray& ray, double maxdist);
    Vector3f include(int i, const Ray& ray, double maxdist);

public:
    PhotonMap(int size);
    ~PhotonMap();

    int getStoredPhotons() const;

    Vector3f getIrradiance(const Ray& ray, double maxdist);
    void Store(const Photon& photon);
    void SetupKDTree();
};

#endif