#ifndef HIT_H
#define HIT_H

#include <vecmath.h>

class Light;
class Object3D;

class ObjectHit {
    Object3D* object;
    double t;
    Vector3f N, C, I;
    double u, v;
    bool front;

public:
    // constructors
    ObjectHit();
    ObjectHit(double _t, Object3D* _object, const Vector3f& _N, const Vector3f& _C, const Vector3f& _I, double _u, double _v, bool _front);
    // destructor
    ~ObjectHit() = default;
    double getT() const;
    double getU() const;
    double getV() const;
    const Vector3f& getN() const;
    const Vector3f& getC() const;
    const Vector3f& getI() const;
    bool isFront() const;
    Object3D* getObject() const;
};

class LightHit {
    Light* light;
    double t;
    Vector3f C;

public:
    // constructors
    LightHit();
    LightHit(double _t, Light* _light, const Vector3f& _C);
    // destructor
    ~LightHit() = default;
    double getT() const;
    Light* getLight() const;
    const Vector3f& getC() const;
};

#endif // HIT_H
