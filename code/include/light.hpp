#ifndef LIGHT_H
#define LIGHT_H

#include "photon.hpp"
#include "utils.h"
#include <mutex>
#include <vecmath.h>

class Object3D;
class Scene;
class ObjectHit;
class Ray;
class LightHit;
class Image;

class Light {
public:
    double t;
    double renderscale, emitscale;
    Light();
    const Vector3f& getColor() const;
    virtual Vector3f getColor(const LightHit& hit) const = 0;
    int getHash() const;

    virtual ~Light() = default;

    virtual Vector3f getIrradiance(const ObjectHit& hit, Scene* scene, int* hash, unsigned short* X) const = 0;
    virtual bool intersect(const Ray& ray, LightHit& hit) = 0;
    virtual Photon EmitPhoton() = 0;
    virtual void Input(std::stringstream& fin) = 0;
    virtual void Initialize() = 0;
    virtual bool hasTexture() = 0;

protected:
    int hash;
    int cnt;
    Vector3f color;
    std::mutex mute;
};

class PointLight : public Light {
public:
    PointLight();

    ~PointLight() override = default;

    Vector3f getIrradiance(const ObjectHit& hit, Scene* scene, int* hash, unsigned short* X) const;
    bool intersect(const Ray& ray, LightHit& hit);
    Photon EmitPhoton();
    void Input(std::stringstream& fin);
    void Initialize();
    Vector3f getColor(const LightHit& hit) const;
    bool hasTexture();

private:
    Vector3f origin;
    double radius;
};

class AreaLight : public Light {
public:
    AreaLight();
    ~AreaLight() override;
    Vector3f getIrradiance(const ObjectHit& hit, Scene* scene, int* hash, unsigned short* X) const;
    bool intersect(const Ray& ray, LightHit& hit);
    Photon EmitPhoton();
    void Input(std::stringstream& fin);
    Vector3f getColor(const LightHit& hit) const;
    bool hasTexture();
    void Initialize();

private:
    Vector3f _origin;
    Vector3f origin;
    Vector3f dx, dy;
    Vector3f dz;
    double angle;
    double cos1, sin2;
};

class TextureLight : public Light {
public:
    TextureLight();
    ~TextureLight() override;
    Vector3f getIrradiance(const ObjectHit& hit, Scene* scene, int* hash, unsigned short* X) const;
    bool intersect(const Ray& ray, LightHit& hit);
    Photon EmitPhoton();
    void Input(std::stringstream& fin);
    Vector3f getColor(const LightHit& hit) const;

    void Initialize();
    bool hasTexture();

private:
    Vector3f _origin;
    Vector3f origin;
    Vector3f dx, dy;
    Vector3f dz;
    Image* texture;
};
#endif // LIGHT_H