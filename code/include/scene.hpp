#ifndef SCENE_H
#define SCENE_H

#include "vecmath.h"

class Image;
class Camera;
class SceneParser;
class Light;
class Object3D;
class Material;
class Ray;
class LightHit;
class ObjectHit;

class Scene {
    Image* image;
    int width, height;
    Camera* camera;
    SceneParser* sceneparser;
    void RayTracing();
    void StochasticProgressivePhotonMapping();
    void output(int sppmiter);

public:
    Scene() = delete;
    Scene(const char* filename);
    ~Scene();
    int getWidth() const;
    int getHeight() const;
    Vector3f getBackgroundColor() const;
    Camera* getCamera() const;
    int getNumLights() const;
    Light* getLight(int i) const;
    int getNumMaterials() const;
    Material* getMaterial(int i) const;
    int getNumObjects() const;
    Object3D* getObject(int i) const;
    Image* getImage() const;
    bool intersectLights(const Ray& ray, LightHit& h);
    void Run();
    bool intersectObjects(const Ray& r, ObjectHit& h);
};

#endif