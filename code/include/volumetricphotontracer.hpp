#ifndef VOLUMETRICPHOTONTRACER_H
#define VOLUMETRICPHOTONTRACER_H

class Photon;
class ObjectHit;
class Scene;
class PhotonMap;

class VolumetricPhotonTracer {
    PhotonMap* photonmap;
    Scene* scene;
    bool* completeThread;
    void PhotonReflection(const ObjectHit& hit, const Photon& photon, int dep, bool refracted, unsigned short* X);
    void PhotonRefraction(const ObjectHit& hit, const Photon& photon, int dep, bool refracted, unsigned short* X);
    void PhotonTracing(const Photon& photon, int dep, bool refracted, unsigned short* X);

public:
    VolumetricPhotonTracer(Scene* _scene, PhotonMap* _photonmap);
    ~VolumetricPhotonTracer() = default;
    void Run(int randIDBase = 0);
};
#endif