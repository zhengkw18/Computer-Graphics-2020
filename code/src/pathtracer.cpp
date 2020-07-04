#include "pathtracer.hpp"
#include "camera.hpp"
#include "hit.hpp"
#include "image.hpp"
#include "light.hpp"
#include "object3d.hpp"
#include "ray.hpp"
#include "scene.hpp"
#include "utils.h"
#include <omp.h>
#include <stack>

Vector3f PathTracer::radiance(Ray ray, int dep, bool refracted, unsigned short* X)
{
    std::stack<Vector3f> vecs;
    ObjectHit objecthit;
    LightHit lighthit;
    bool objectcrash, lightcrash;
    for (;;) {
        objectcrash = scene->intersectObjects(ray, objecthit);
        lightcrash = scene->intersectLights(ray, lighthit);
        if (lightcrash) {
            Light* nearest_light = lighthit.getLight();
            if (!objectcrash || lighthit.getT() < objecthit.getT() + EPS) {
                vecs.push(nearest_light->getColor(lighthit) * nearest_light->renderscale);
                break;
            }
        }
        if (objectcrash) {
            Object3D* object = objecthit.getObject();
            Vector3f f = object->getMaterial()->getColor(objecthit);
            double p = f.max();
            if (++dep > 5) {
                if (erand48(X) < p)
                    f = f / p;
                else {
                    return Vector3f();
                }
            }
            double pp = erand48(X);

            if (pp < object->getMaterial()->diffusion) {
                double r1 = 2 * PI * erand48(X), r2 = erand48(X), r2s = sqrt(r2);
                Vector3f w = objecthit.getN();
                Vector3f u = w.getVertical();
                Vector3f v = Vector3f::cross(w, u);
                Vector3f dir = u * cos(r1) * r2s + v * sin(r1) * r2s + w * sqrt(1 - r2);
                vecs.push(f);
                ray = Ray(objecthit.getC(), dir);
                continue;
            } else if (object->getMaterial()->diffusion < pp && pp < object->getMaterial()->diffusion + object->getMaterial()->reflection_refraction) {
                Vector3f reflv = objecthit.getI().reflect(objecthit.getN());
                Ray reflray = Ray(objecthit.getC(), reflv);
                if (!object->getMaterial()->transparent) {
                    double dreflRadius = object->getMaterial()->dreflRadius;
                    if (dreflRadius < EPS || dep > MAX_DREFL_DEP) {
                        vecs.push(f);
                        ray = reflray;
                        continue;
                    }
                    Vector3f ret;
                    Vector3f Dx = reflv.getVertical();
                    Vector3f Dy = Vector3f::cross(reflv, Dx);
                    Dx = Dx.normalized() * dreflRadius;
                    Dy = Dy.normalized() * dreflRadius;
                    for (int k = 0; k < DREFL_QUALITY; k++) {
                        double x, y;
                        do {
                            x = erand48(X) * 2 - 1;
                            y = erand48(X) * 2 - 1;
                        } while (x * x + y * y > 1);
                        x *= dreflRadius;
                        y *= dreflRadius;
                        ret += radiance(Ray(objecthit.getC(), reflv + Dx * x + Dy * y), dep + MAX_DREFL_DEP, refracted, X);
                    }
                    ret = ret / DREFL_QUALITY;
                    vecs.push(f);
                    vecs.push(ret);
                    break;
                } else {
                    double n = object->getMaterial()->refractionIndex;
                    if (!refracted)
                        n = 1 / n;
                    double cosI = -Vector3f::dot(objecthit.getN(), objecthit.getI());
                    double cosT2 = 1 - (n * n) * (1 - cosI * cosI);
                    if (cosT2 > EPS) {
                        double cosT = sqrt(cosT2);
                        Vector3f ray_V = objecthit.getI() * n + objecthit.getN() * (n * cosI - cosT);
                        double r1 = (n * cosI - cosT) / (n * cosI + cosT);
                        double r2 = (cosI - n * cosT) / (cosI + n * cosT);
                        double Re = (r1 * r1 + r2 * r2) / 2, Tr = 1 - Re;
                        double P = .25 + .5 * Re, RP = Re / P, TP = Tr / (1 - P);
                        if (dep > 2) {
                            if (erand48(X) < P) {
                                vecs.push(f);
                                vecs.push(Vector3f(RP));
                                ray = reflray;
                                continue;
                            } else {
                                vecs.push(f);
                                vecs.push(Vector3f(TP));
                                ray = Ray(objecthit.getC(), ray_V);
                                refracted = !refracted;
                                continue;
                            }
                        } else {
                            vecs.push(f);
                            vecs.push(radiance(reflray, dep, refracted, X) * Re + radiance(Ray(objecthit.getC(), ray_V), dep, !refracted, X) * Tr);
                            break;
                        }
                    } else {
                        vecs.push(f);
                        ray = reflray;
                        continue;
                    }
                }
            }
        } else {
            return Vector3f();
        }
    }
    Vector3f ret = Vector3f(1.);
    while (!vecs.empty()) {
        ret = ret * vecs.top();
        vecs.pop();
    }
    return ret;
}

PathTracer::PathTracer(Scene* _scene, Image* _image)
{
    scene = _scene;
    camera = scene->getCamera();
    image = _image;
    W = scene->getWidth();
    H = scene->getHeight();
}
void PathTracer::Run()
{
    printf("Begin Path Tracing...\n");
#pragma omp parallel for schedule(dynamic, 1)
    for (int y = 0; y < camera->getHeight(); y++) {
        fprintf(stderr, "\rRendering (%d spp) %5.2f%%", SPP, 100. * y / camera->getHeight());
        for (int x = 0; x < camera->getWidth(); x++) {
            Vector3f r2;
            for (int sy = 0; sy < 2; ++sy) {
                for (int sx = 0; sx < 2; ++sx) {
                    unsigned short X[3] = { y + sx, y * x + sy, y * x * y + sx * sy + time(0) };
                    Vector3f r;
                    for (int s = 0; s < SPP / 4; ++s) {
                        double r1 = 2 * erand48(X), dx = r1 < 1 ? sqrt(r1) : 1 - sqrt(2 - r1);
                        double r2 = 2 * erand48(X), dy = r2 < 1 ? sqrt(r2) : 1 - sqrt(2 - r2);
                        r += radiance(camera->generateRay(x + (dx + sx + .5) / 2, y + (dy + sy + .5) / 2, X), 0, false, X);
                    }
                    r2 += r / SPP;
                }
            }
            image->SetPixel(x, y, r2.confined());
        }
    }
    printf("\n");
}