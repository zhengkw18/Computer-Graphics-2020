#ifndef PHOTONTRACER_H
#define PHOTONTRACER_H

class Photon;
class ObjectHit;
class HitpointMap;
class Scene;

class PhotonTracer {
    HitpointMap* hitpointmap;
    Scene* scene;
    bool* completeThread;
    void PhotonDiffusion(const ObjectHit& hit, const Photon& photon, int dep, bool refracted, unsigned short* X);
    void PhotonReflection(const ObjectHit& hit, const Photon& photon, int dep, bool refracted, unsigned short* X);
    void PhotonRefraction(const ObjectHit& hit, const Photon& photon, int dep, bool refracted, unsigned short* X);
    void PhotonTracing(const Photon& photon, int dep, bool refracted, unsigned short* X);

public:
    PhotonTracer(Scene* _scene, HitpointMap* _hitpointmap);
    ~PhotonTracer() = default;
    void Run(int iteration, int randIDBase = 0);
};
#endif