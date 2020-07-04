#ifndef RAYTRACER_H
#define RAYTRACER_H

#include <mutex>
#include <vecmath.h>

class Scene;
class Camera;
class HitpointMap;
class ObjectHit;
class Ray;
class PhotonMap;
class Image;

class RayTracer {
    Scene* scene;
    Camera* camera;
    HitpointMap* hitpointmap;
    PhotonMap* photonmap;
    Image* image;
    int W, H;
    int** hash;
    Vector3f RayDiffusion(const ObjectHit& hit, int dep, int* hash, int cr, Vector3f weight, unsigned short* X);
    Vector3f RayReflection(const ObjectHit& hit, int dep, bool refracted, int* hash, int cr, Vector3f weight, unsigned short* X);
    Vector3f RayRefraction(const ObjectHit& hit, int dep, bool refracted, int* hash, int cr, Vector3f weight, unsigned short* X);
    Vector3f RayTracing(const Ray& ray, int dep, bool refracted, int* hash, int cr, Vector3f weight, unsigned short* X);
    void Sampling(int row);
    void Resampling(int row);
    void MultiThreadSampling();
    void MultiThreadResampling();

public:
    RayTracer(Scene* _scene, Image* _image, HitpointMap* _hitpointmap = nullptr, PhotonMap* _photonmap = nullptr);
    ~RayTracer() = default;
    void Run();
};

#endif