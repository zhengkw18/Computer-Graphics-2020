#include "photontracer.hpp"
#include "hit.hpp"
#include "hitpointmap.hpp"
#include "light.hpp"
#include "material.hpp"
#include "object3d.hpp"
#include "photon.hpp"
#include "ray.hpp"
#include "scene.hpp"
#include "utils.h"
#include <omp.h>

void PhotonTracer::PhotonDiffusion(const ObjectHit& hit, const Photon& photon, int dep, bool refracted, unsigned short* X)
{
    if (!PPM_RENDER_DIFFUSION && dep > 0)
        return;
    Object3D* object = hit.getObject();
    Material* material = object->getMaterial();
    if (!PPM_RENDER_DIFFUSION && material->reflection_refraction > EPS)
        return;
    Vector3f color = material->getColor(hit);
    Photon newphoton = photon;

    Vector3f vert = hit.getN().getVertical();
    double theta = acos(sqrt(erand48(X)));
    double phi = erand48(X) * 2 * PI;
    newphoton.dir = hit.getN().rotate(vert, theta).rotate(hit.getN(), phi);

    newphoton.power = newphoton.power * color / color.power();
    PhotonTracing(newphoton, dep + 1, refracted, X);
}
void PhotonTracer::PhotonReflection(const ObjectHit& hit, const Photon& photon, int dep, bool refracted, unsigned short* X)
{
    Object3D* object = hit.getObject();
    Material* material = object->getMaterial();
    Vector3f color = material->getColor(hit);

    double reflection = object->getMaterial()->reflection_refraction;
    if (object->getMaterial()->transparent) {
        return;
        double cosI = -Vector3f::dot(hit.getN(), photon.dir);
        double n = object->getMaterial()->refractionIndex;
        if (!refracted)
            n = 1 / n;
        double cosT2 = 1 - (n * n) * (1 - cosI * cosI);
        if (cosT2 > EPS) {
            return;
        }
    }

    Photon newphoton = photon;
    newphoton.dir = photon.dir.reflect(hit.getN());
    double dreflRadius = material->dreflRadius;
    if (dreflRadius > EPS) {
        Vector3f Dx = newphoton.dir.getVertical();
        Vector3f Dy = Vector3f::cross(newphoton.dir, Dx);
        Dx = Dx.normalized() * dreflRadius;
        Dy = Dy.normalized() * dreflRadius;
        double x, y;
        do {
            x = erand48(X) * 2 - 1;
            y = erand48(X) * 2 - 1;
        } while (x * x + y * y > 1);
        x *= dreflRadius;
        y *= dreflRadius;
        newphoton.dir = newphoton.dir + Dx * x + Dy * y;
    }
    newphoton.power = newphoton.power * color / color.power();
    PhotonTracing(newphoton, dep + 1, refracted, X);
}
void PhotonTracer::PhotonRefraction(const ObjectHit& hit, const Photon& photon, int dep, bool refracted, unsigned short* X)
{
    Object3D* object = hit.getObject();
    Material* material = object->getMaterial();
    double n = material->refractionIndex;
    if (!refracted)
        n = 1 / n;
    double cosI = -Vector3f::dot(hit.getN(), photon.dir);
    double cosT2 = 1 - (n * n) * (1 - cosI * cosI);
    double refraction = object->getMaterial()->reflection_refraction;
    if (cosT2 > EPS) {
        double cosT = sqrt(cosT2);
        Photon newphoton = photon;
        //if (refracted) {
        //Vector3f trans = Vector3f::exp(material->absorbColor * -hit.getT());
        //newphoton.power = newphoton.power * trans / trans.power();
        //}
        Vector3f trans = object->getMaterial()->Color;
        newphoton.power = newphoton.power * trans / trans.power();
        newphoton.dir = newphoton.dir * n + hit.getN() * (n * cosI - cosT);
        PhotonTracing(newphoton, dep + 1, !refracted, X);
    }
}
void PhotonTracer::PhotonTracing(const Photon& photon, int dep, bool refracted, unsigned short* X)
{
    if (dep > MAX_PHOTONTRACING_DEP)
        return;
    if (photon.power.isNan())
        return;
    ObjectHit objecthit;
    bool objectcrash = scene->intersectObjects(Ray(photon.pos, photon.dir), objecthit);

    if (objectcrash) {
        Object3D* object = objecthit.getObject();
        Photon newphoton = photon;
        newphoton.pos = objecthit.getC();
        if (object->getMaterial()->diffusion > EPS && dep > 1) {
            hitpointmap->includePhoton(newphoton);
        }
        double prob = erand48(X);
        if (prob < object->getMaterial()->diffusion) {
            PhotonDiffusion(objecthit, newphoton, dep, refracted, X);
        } else if (prob > object->getMaterial()->diffusion && prob < object->getMaterial()->diffusion + object->getMaterial()->reflection_refraction) {
            PhotonReflection(objecthit, newphoton, dep, refracted, X);
            if (object->getMaterial()->transparent)
                PhotonRefraction(objecthit, newphoton, dep, refracted, X);
        }
    }
}

PhotonTracer::PhotonTracer(Scene* _scene, HitpointMap* _hitpointmap)
{
    scene = _scene;
    hitpointmap = _hitpointmap;
}
void PhotonTracer::Run(int iteration, int randIDBase)
{
    double totalPower = 0;
    int lightnum = scene->getNumLights();
    for (int i = 0; i < lightnum; i++) {
        totalPower += (scene->getLight(i)->getColor() * scene->getLight(i)->emitscale).power();
    }
    double photonPower = totalPower / NUM_EMIT_PHOTONS;
    for (int i = 0; i < lightnum; i++) {
        Light* light = scene->getLight(i);
        int lightPhotons = (int)((scene->getLight(i)->getColor() * scene->getLight(i)->emitscale).power() / photonPower);
#pragma omp parallel for schedule(dynamic, 1)
        for (int j = 0; j < lightPhotons; j++) {
            unsigned short X[3] = { i, j, time(0) };
            Photon photon = light->EmitPhoton();
            photon.power *= totalPower;
            PhotonTracing(photon, 1, false, X);
        }
    }
}