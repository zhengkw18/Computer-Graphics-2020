#include "hit.hpp"

ObjectHit::ObjectHit()
{
    object = nullptr;
    t = 1e38;
}
ObjectHit::ObjectHit(double _t, Object3D* _object, const Vector3f& _N, const Vector3f& _C, const Vector3f& _I, double _u, double _v, bool _front)
{
    t = _t;
    object = _object;
    N = _N.normalized();
    C = _C;
    I = _I.normalized();
    u = _u;
    v = _v;
    front = _front;
}
double ObjectHit::getT() const { return t; }
double ObjectHit::getU() const { return u; }
double ObjectHit::getV() const { return v; }
const Vector3f& ObjectHit::getN() const { return N; }
const Vector3f& ObjectHit::getC() const { return C; }
const Vector3f& ObjectHit::getI() const { return I; }
bool ObjectHit::isFront() const { return front; }
Object3D* ObjectHit::getObject() const { return object; }

LightHit::LightHit()
{
    light = nullptr;
    t = 1e38;
}
LightHit::LightHit(double _t, Light* _light, const Vector3f& _C)
{
    t = _t;
    light = _light;
    C = _C;
}
double LightHit::getT() const { return t; }
Light* LightHit::getLight() const { return light; }
const Vector3f& LightHit::getC() const { return C; }