#ifndef MATERIAL_H
#define MATERIAL_H

#include <cassert>
#include <iostream>
#include <vecmath.h>

class ObjectHit;
class Image;

class Material {
public:
    Vector3f Color, absorbColor;
    double reflection_refraction;
    int transparent;
    double diffusion, specularity;
    double refractionIndex;
    double dreflRadius;
    double shininess;
    Image* texture;
    Image* bump;

    Material();
    ~Material();
    double BRDF(const Vector3f& View, const Vector3f& N, const Vector3f& dirToLight);
    Vector3f getColor(const ObjectHit& hit) const;
    void Input(std::stringstream& fin);
};

static Material* defaultMaterial = new Material();
#endif // MATERIAL_H
