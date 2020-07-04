#include "light.hpp"
#include "hit.hpp"
#include "image.hpp"
#include "object3d.hpp"
#include "ray.hpp"
#include "scene.hpp"

Light::Light()
{
    hash = rand();
    cnt = 0;
}
const Vector3f& Light::getColor() const
{
    return color;
}
int Light::getHash() const
{
    return hash;
}

PointLight::PointLight() {}

Vector3f PointLight::getIrradiance(const ObjectHit& hit, Scene* scene, int* hash, unsigned short* X) const
{
    Object3D* object = hit.getObject();
    Vector3f V = origin - hit.getC();
    double dist = V.length();
    ObjectHit h;
    bool crash = scene->intersectObjects(Ray(hit.getC(), V), h);
    if (crash && h.getT() < dist)
        return Vector3f();
    Vector3f ret = color * object->getMaterial()->BRDF(-hit.getI(), hit.getN(), V);
    if (LIGHT_DECAY)
        ret = ret * renderscale / V.length2();
    else
        ret = ret / color.power();
    if (!ret.isZero() && hash != nullptr) {
        *hash = *hash + getHash();
    }
    return ret;
}

bool PointLight::intersect(const Ray& ray, LightHit& hit)
{
    Vector3f ocvec = origin - ray.getOrigin();
    double oh = Vector3f::dot(ray.getDirection(), ocvec);
    double ch = ocvec.length2() - oh * oh;
    if (ch > radius * radius - EPS)
        return false;
    double ph = radius * radius - ch;
    double t = oh - sqrt(ph);
    if (t < EPS)
        return false;
    hit = LightHit(t, this, ray.pointAtParameter(t));
    return true;
}
Photon PointLight::EmitPhoton()
{
    mute.lock();
    Photon photon;
    photon.pos = origin;
    double x, y, z;
    do {
        cnt++;
        x = 2 * Halton2(cnt) - 1;
        y = 2 * Halton3(cnt) - 1;
        z = 2 * Halton5(cnt) - 1;
    } while (x * x + y * y + z * z > 1 || x * x + y * y + z * z < EPS);
    photon.dir = Vector3f(x, y, z).normalized();
    photon.power = color / color.power();
    mute.unlock();
    return photon;
}
void PointLight::Input(std::stringstream& fin)
{
    std::string var;
    fin >> var;
    if (var == "color") {
        color.Input(fin);
        if (GAMMA) {
            color[0] = std::pow(color[0], 2.2);
            color[1] = std::pow(color[1], 2.2);
            color[2] = std::pow(color[2], 2.2);
        }
    } else if (var == "renderscale")
        fin >> renderscale;
    else if (var == "emitscale")
        fin >> emitscale;
    else if (var == "origin")
        origin.Input(fin);
    else if (var == "radius")
        fin >> radius;
}

void PointLight::Initialize() {}

bool PointLight::hasTexture()
{
    return false;
}

Vector3f PointLight::getColor(const LightHit& hit) const
{
    double dist2 = Vector3f::distance2(origin, hit.getC());
    return color * (1 - dist2 / (radius * radius));
}

AreaLight::AreaLight()
{
    angle = PI / 2;
}
AreaLight::~AreaLight() {}
Vector3f AreaLight::getColor(const LightHit& hit) const
{
    return color;
}
Vector3f AreaLight::getIrradiance(const ObjectHit& hit, Scene* scene, int* hash, unsigned short* X) const
{
    Object3D* object = hit.getObject();
    Vector3f ret;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            for (int k = 0; k < SHADE_QUALITY; k++) {
                double u = (erand48(X) + i) / 4, v = (erand48(X) + j) / 4;
                Vector3f V = (origin + dx * (2 * u - 1) + dy * (2 * v - 1)) - hit.getC();
                double dist = V.length();
                if (Vector3f::dot(-V.normalized(), dz) > cos(angle) - EPS) {
                    ObjectHit h;
                    bool crash = scene->intersectObjects(Ray(hit.getC(), V), h);
                    if (crash && h.getT() < dist)
                        continue;
                    Vector3f _ret = color * object->getMaterial()->BRDF(-hit.getI(), hit.getN(), V);
                    if (LIGHT_DECAY)
                        _ret = _ret * renderscale / V.length2();
                    else
                        _ret = _ret / color.power();
                    ret = ret + _ret;
                }
            }
        }
    }
    ret = ret / (16.0 * SHADE_QUALITY);
    if (!ret.isZero() && hash != nullptr) {
        *hash = *hash + getHash();
    }
    return ret;
}
bool AreaLight::intersect(const Ray& ray, LightHit& hit)
{
    Vector3f V = ray.getDirection();
    double d = Vector3f::dot(dz, V);
    if (fabs(d) < EPS)
        return false;
    double l = (Vector3f::dot(dz, origin) - Vector3f::dot(dz, ray.getOrigin())) / d;
    if (l < EPS)
        return false;

    Vector3f C = ray.pointAtParameter(l) - origin;
    if (fabs(Vector3f::dot(dx, C)) > Vector3f::dot(dx, dx))
        return false;
    if (fabs(Vector3f::dot(dy, C)) > Vector3f::dot(dy, dy))
        return false;
    hit = LightHit(l, this, ray.pointAtParameter(l));
    return true;
}
Photon AreaLight::EmitPhoton()
{
    mute.lock();
    Photon photon;
    cnt++;
    if (fabs(angle - PI / 2) > EPS) {
        double u = Halton2(cnt), v = Halton3(cnt);
        photon.pos = origin + dx * (u * 2 - 1) + dy * (v * 2 - 1);
        photon.dir = (photon.pos - _origin).normalized();
        photon.power = color * (1 - cos1) / color.power();
    } else {
        double u = Halton2(cnt), v = Halton3(cnt);
        photon.pos = origin + dx * (u * 2 - 1) + dy * (v * 2 - 1);
        Vector3f vert = dz.getVertical();
        double theta = acos(sqrt(cos1 * cos1 + sin2 * Halton5(cnt)));
        double phi = 2 * PI * Halton7(cnt);
        photon.dir = dz.rotate(vert, theta).rotate(dz, phi).normalized();
        photon.power = color * sin2 / color.power();
    }
    mute.unlock();
    return photon;
}
void AreaLight::Input(std::stringstream& fin)
{
    std::string var;
    fin >> var;
    if (var == "color") {
        color.Input(fin);
        if (GAMMA) {
            color[0] = std::pow(color[0], 2.2);
            color[1] = std::pow(color[1], 2.2);
            color[2] = std::pow(color[2], 2.2);
        }
    } else if (var == "renderscale")
        fin >> renderscale;
    else if (var == "emitscale")
        fin >> emitscale;
    else if (var == "origin")
        origin.Input(fin);
    else if (var == "dx")
        dx.Input(fin);
    else if (var == "dy")
        dy.Input(fin);
    else if (var == "angle") {
        fin >> angle;
        angle = DegreesToRadians(angle);
    }
}

bool AreaLight::hasTexture()
{
    return false;
}

void AreaLight::Initialize()
{
    dz = Vector3f::cross(dx, dy).normalized();
    if (fabs(angle - PI / 2) > EPS)
        _origin = origin - dz * dx.length() / tan(angle);
    cos1 = cos(angle);
    sin2 = sin(angle) * sin(angle);
}

TextureLight::TextureLight()
{
    texture = nullptr;
}
TextureLight::~TextureLight()
{
    if (texture != nullptr)
        delete texture;
}
Vector3f TextureLight::getColor(const LightHit& hit) const
{
    if (texture == nullptr)
        return color;
    Vector3f X = hit.getC() - origin;
    double u = (Vector3f::dot(X, dx) / dx.length2() + 1) / 2;
    double v = (Vector3f::dot(X, dy) / dy.length2() + 1) / 2;
    return color * texture->getSmoothPixel(u, v);
}
Vector3f TextureLight::getIrradiance(const ObjectHit& hit, Scene* scene, int* hash, unsigned short* X) const
{
    Object3D* object = hit.getObject();
    Vector3f V = _origin - hit.getC();
    V.normalize();
    double d = Vector3f::dot(dz, V);
    if (fabs(d) < EPS)
        return Vector3f();
    double dist = (Vector3f::dot(dz, origin) - Vector3f::dot(dz, hit.getC())) / d;
    if (dist < EPS)
        return Vector3f();
    Vector3f C = hit.getC() + dist * V - origin;
    double u = (Vector3f::dot(C, dx) / dx.length2() + 1) / 2;
    double v = (Vector3f::dot(C, dy) / dy.length2() + 1) / 2;
    if (u < 0 || u > 1 || v < 0 || v > 1)
        return Vector3f();
    ObjectHit h;
    bool crash = scene->intersectObjects(Ray(hit.getC(), V), h);
    if (crash && h.getT() < dist)
        return Vector3f();

    Vector3f ret = color * texture->getSmoothPixel(u, v) * object->getMaterial()->BRDF(-hit.getI(), hit.getN(), V);
    if (LIGHT_DECAY)
        ret = ret * renderscale / V.length2();
    else
        ret = ret / color.power();
    if (!ret.isZero() && hash != nullptr) {
        *hash = *hash + getHash();
    }
    return ret;
}
bool TextureLight::intersect(const Ray& ray, LightHit& hit)
{
    Vector3f V = ray.getDirection();
    double d = Vector3f::dot(dz, V);
    if (fabs(d) < EPS)
        return false;
    double l = (Vector3f::dot(dz, origin) - Vector3f::dot(dz, ray.getOrigin())) / d;
    if (l < EPS)
        return false;
    Vector3f C = ray.pointAtParameter(l) - origin;
    if (fabs(Vector3f::dot(dx, C)) > Vector3f::dot(dx, dx))
        return false;
    if (fabs(Vector3f::dot(dy, C)) > Vector3f::dot(dy, dy))
        return false;
    hit = LightHit(l, this, ray.pointAtParameter(l));
    return true;
}
Photon TextureLight::EmitPhoton()
{
    mute.lock();
    cnt++;
    Photon photon;
    double u = Halton2(cnt), v = Halton3(cnt);
    photon.pos = origin + dx * (u * 2 - 1) + dy * (v * 2 - 1);
    photon.dir = (photon.pos - _origin).normalized();
    photon.power = color * texture->getSmoothPixel(u, v) / color.power();
    mute.unlock();
    return photon;
}
void TextureLight::Input(std::stringstream& fin)
{
    std::string var;
    fin >> var;
    if (var == "color") {
        color.Input(fin);
        if (GAMMA) {
            color[0] = std::pow(color[0], 2.2);
            color[1] = std::pow(color[1], 2.2);
            color[2] = std::pow(color[2], 2.2);
        }
    } else if (var == "renderscale")
        fin >> renderscale;
    else if (var == "emitscale")
        fin >> emitscale;
    else if (var == "origin")
        origin.Input(fin);
    else if (var == "_origin")
        _origin.Input(fin);
    else if (var == "dx")
        dx.Input(fin);
    else if (var == "dy")
        dy.Input(fin);
    else if (var == "texture") {
        std::string texturefile;
        fin >> texturefile;
        texture = Image::LoadBMP(texturefile.c_str());
    }
}

bool TextureLight::hasTexture()
{
    return (texture != nullptr);
}

void TextureLight::Initialize()
{
    dz = Vector3f::cross(dx, dy).normalized();
}