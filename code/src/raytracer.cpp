#include "raytracer.hpp"
#include "camera.hpp"
#include "hit.hpp"
#include "hitpointmap.hpp"
#include "image.hpp"
#include "light.hpp"
#include "object3d.hpp"
#include "photonmap.hpp"
#include "ray.hpp"
#include "scene.hpp"
#include "utils.h"
#include <omp.h>

Vector3f RayTracer::RayDiffusion(const ObjectHit& hit, int dep, int* hash, int cr, Vector3f weight, unsigned short* X)
{
    Vector3f ret;
    Object3D* object = hit.getObject();
    Vector3f color = object->getMaterial()->getColor(hit);
    if (ALGORITHM == RT) {
        ret += color * scene->getBackgroundColor() * object->getMaterial()->diffusion;
        for (int i = 0; i < scene->getNumLights(); i++) {
            Light* light = scene->getLight(i);
            ret += color * light->getIrradiance(hit, scene, hash, X);
        }
    }
    if (hitpointmap != nullptr && dep <= MAX_HITPOINT_DEP) {
        Hitpoint hitpoint;
        hitpoint.pos = hit.getC();
        hitpoint.dir = hit.getI();
        hitpoint.N = hit.getN();
        hitpoint.object = object;
        hitpoint.cr = cr;
        hitpoint.weight = weight * color;
        hitpoint.R2 = SAMPLE_DIST * SAMPLE_DIST;
        hitpointmap->Store(hitpoint);
    }
    return ret;
}
Vector3f RayTracer::RayReflection(const ObjectHit& hit, int dep, bool refracted, int* hash, int cr, Vector3f weight, unsigned short* X)
{
    Object3D* object = hit.getObject();
    double reflection = object->getMaterial()->reflection_refraction;
    if (object->getMaterial()->transparent) {
        double cosI = -Vector3f::dot(hit.getN(), hit.getI());
        double n = object->getMaterial()->refractionIndex;
        if (!refracted)
            n = 1 / n;
        double cosT2 = 1 - (n * n) * (1 - cosI * cosI);
        if (cosT2 > EPS) {
            double cosT = sqrt(cosT2);
            double r1 = (n * cosI - cosT) / (n * cosI + cosT);
            double r2 = (cosI - n * cosT) / (cosI + n * cosT);
            reflection = 0.5 * (r1 * r1 + r2 * r2) * reflection;
        }
    }
    Vector3f ray_V = hit.getI().reflect(hit.getN());

    double dreflRadius = object->getMaterial()->dreflRadius;

    if (dreflRadius < EPS || dep > MAX_DREFL_DEP) {
        Vector3f alpha = object->getMaterial()->getColor(hit) * reflection;
        if (ALGORITHM == RT) {
            if (refracted) {
                alpha = alpha * Vector3f::exp(object->getMaterial()->absorbColor * -hit.getT());
            }
        }
        return RayTracing(Ray(hit.getC(), ray_V), dep + 1, refracted, hash, cr, weight * alpha, X) * alpha;
    }

    Vector3f Dx = ray_V.getVertical();
    Vector3f Dy = Vector3f::cross(ray_V, Dx);
    Dx = Dx.normalized() * dreflRadius;
    Dy = Dy.normalized() * dreflRadius;

    Vector3f rcol, alpha = object->getMaterial()->getColor(hit) * reflection / DREFL_QUALITY;
    if (ALGORITHM == RT) {
        if (refracted) {
            alpha = alpha * Vector3f::exp(object->getMaterial()->absorbColor * -hit.getT());
        }
    }
    for (int k = 0; k < DREFL_QUALITY; k++) {
        double x, y;
        do {
            x = erand48(X) * 2 - 1;
            y = erand48(X) * 2 - 1;
        } while (x * x + y * y > 1);
        x *= dreflRadius;
        y *= dreflRadius;

        rcol += RayTracing(Ray(hit.getC(), ray_V + Dx * x + Dy * y), dep + MAX_DREFL_DEP, refracted, nullptr, cr, weight * alpha, X);
    }
    return rcol * alpha;
}
Vector3f RayTracer::RayRefraction(const ObjectHit& hit, int dep, bool refracted, int* hash, int cr, Vector3f weight, unsigned short* X)
{
    Object3D* object = hit.getObject();
    double n = object->getMaterial()->refractionIndex;
    Vector3f ray_V;
    if (!refracted)
        n = 1 / n;

    double cosI = -Vector3f::dot(hit.getN(), hit.getI());
    double cosT2 = 1 - (n * n) * (1 - cosI * cosI);
    double reflection_refraction = object->getMaterial()->reflection_refraction;
    if (cosT2 > EPS) {
        double cosT = sqrt(cosT2);
        double r1 = (n * cosI - cosT) / (n * cosI + cosT);
        double r2 = (cosI - n * cosT) / (cosI + n * cosT);
        double refraction = (1 - 0.5 * (r1 * r1 + r2 * r2)) * reflection_refraction;
        ray_V = hit.getI() * n + hit.getN() * (n * cosI - cosT);
        Vector3f alpha = Vector3f(1, 1, 1) * refraction;
        if (ALGORITHM == RT) {
            if (refracted) {
                alpha = alpha * Vector3f::exp(object->getMaterial()->absorbColor * -hit.getT());
            }
        } else {
            alpha = alpha * object->getMaterial()->Color;
        }

        Vector3f rcol = RayTracing(Ray(hit.getC(), ray_V), dep + 1, !refracted, hash, cr, weight * alpha, X);
        return rcol * alpha;
    } else {
        return Vector3f();
    }
}
Vector3f RayTracer::RayTracing(const Ray& ray, int dep, bool refracted, int* hash, int cr, Vector3f weight, unsigned short* X)
{
    if (ALGORITHM == RT) {
        if (dep > MAX_RAYTRACING_DEP)
            return Vector3f();
    } else {
        if (hitpointmap != nullptr && photonmap != nullptr) {
            if (dep > std::max(MAX_HITPOINT_DEP, MAX_VOLUMETRIC_DEP))
                return Vector3f();
        } else {
            if (hitpointmap != nullptr && dep >= MAX_HITPOINT_DEP)
                return Vector3f();
            if (photonmap != nullptr && dep >= MAX_VOLUMETRIC_DEP)
                return Vector3f();
        }
    }
    if (hash != nullptr)
        *hash = *hash * HASH_FAC;
    Vector3f ret;
    ObjectHit objecthit;
    LightHit lighthit;
    bool objectcrash = scene->intersectObjects(ray, objecthit);
    bool lightcrash = scene->intersectLights(ray, lighthit);
    if (lightcrash) {
        Light* nearest_light = lighthit.getLight();
        if (!objectcrash || lighthit.getT() < objecthit.getT()) {
            if (hash != nullptr) {
                *hash = *hash + nearest_light->getHash();
                if (nearest_light->hasTexture()) {
                    *hash = *hash + rand();
                }
            }
            if (ALGORITHM == RT)
                ret += nearest_light->getColor(lighthit) * nearest_light->renderscale;
            if (photonmap != nullptr && dep < MAX_VOLUMETRIC_DEP) {
                ret += photonmap->getIrradiance(ray, lighthit.getT()) / NUM_EMIT_VOLUMETRIC_PHOTONS;
            }
        }
    }

    if (objectcrash) {
        Object3D* nearest_object = objecthit.getObject();
        if (hash != nullptr) {
            *hash = *hash + nearest_object->getHash();
            if (nearest_object->getMaterial()->texture != nullptr) {
                *hash = *hash + rand();
            }
        }
        if (nearest_object->getMaterial()->diffusion > EPS)
            ret += RayDiffusion(objecthit, dep, hash, cr, weight, X);
        if (nearest_object->getMaterial()->reflection_refraction > EPS)
            ret += RayReflection(objecthit, dep, refracted, hash, cr, weight, X);
        if (nearest_object->getMaterial()->transparent)
            ret += RayRefraction(objecthit, dep, refracted, hash, cr, weight, X);
        if (photonmap != nullptr && dep < MAX_VOLUMETRIC_DEP) {
            ret += photonmap->getIrradiance(ray, objecthit.getT()) / NUM_EMIT_VOLUMETRIC_PHOTONS;
        }
    }
    if (!objectcrash && !lightcrash) {
        if (photonmap != nullptr && dep < MAX_VOLUMETRIC_DEP) {
            ret += photonmap->getIrradiance(ray, INF) / NUM_EMIT_VOLUMETRIC_PHOTONS;
        }
    }
    if (dep == 1)
        ret = ret.confined();
    return ret;
}

void RayTracer::Sampling(int row)
{
    int j = row;
    for (int i = 0; i < W; i++) {
        unsigned short X[3] = { i, j, time(0) };
        if (camera->enableDOF()) {
            int iteration = (ALGORITHM == SPPM) ? 1 : NUM_DOF_SAMPLE;
            Vector3f color;
            for (int k = 0; k < iteration; k++) {
                Ray ray = camera->generateRay(i, j, X);
                color += RayTracing(ray, 1, false, nullptr, j * W + i, Vector3f(1, 1, 1) / NUM_DOF_SAMPLE, X) / NUM_DOF_SAMPLE;
            }
            image->SetPixel(i, j, color.confined());
        } else {
            Ray ray = camera->generateRay(i, j, X);
            hash[i][j] = 0;
            Vector3f color;
            color += RayTracing(ray, 1, false, &hash[i][j], j * W + i, Vector3f(1, 1, 1), X);
            image->SetPixel(i, j, color.confined());
        }
    }
}
void RayTracer::Resampling(int row)
{
    int j = row;
    for (int i = 0; i < W; i++) {
        unsigned short X[3] = { i, j, time(0) };
        if (!((j == 0 || hash[i][j] == hash[i][j - 1]) && (j == H - 1 || hash[i][j] == hash[i][j + 1]) && (i == 0 || hash[i][j] == hash[i - 1][j]) && (i == W - 1 || hash[i][j] == hash[i + 1][j]))) {
            Vector3f color = image->GetPixel(i, j) / 9;
            for (int r = -1; r <= 1; r++) {
                for (int c = -1; c <= 1; c++) {
                    if (r == 0 && c == 0)
                        continue;
                    Ray ray = camera->generateRay(i + (double)c / 3, j + (double)r / 3, X);
                    color += RayTracing(ray, 1, false, nullptr, j * W + i, Vector3f(1, 1, 1) / 9, X) / 9;
                }
            }
            image->SetPixel(i, j, color.confined());
        }
    }
}
void RayTracer::MultiThreadSampling()
{
#pragma omp parallel for schedule(dynamic, 1)
    for (int i = 0; i < H; i++) {
        fprintf(stderr, "\rSampling %5.2f%%", 100. * i / camera->getHeight());
        Sampling(i);
    }
    printf("\n");
}
void RayTracer::MultiThreadResampling()
{
#pragma omp parallel for schedule(dynamic, 1)
    for (int i = 0; i < H; i++) {
        fprintf(stderr, "\rResampling %5.2f%%", 100. * i / camera->getHeight());
        Resampling(i);
    }
    printf("\n");
}

RayTracer::RayTracer(Scene* _scene, Image* _image, HitpointMap* _hitpointmap, PhotonMap* _photonmap)
{
    scene = _scene;
    camera = scene->getCamera();
    hitpointmap = _hitpointmap;
    photonmap = _photonmap;
    image = _image;
    W = scene->getWidth();
    H = scene->getHeight();
}
void RayTracer::Run()
{
    printf("Begin Ray Tracing...\n");
    hash = new int*[W];
    for (int i = 0; i < W; i++)
        hash[i] = new int[H];
    printf("Sampling...\n");
    MultiThreadSampling();
    if (!camera->enableDOF()) {
        if (hitpointmap != nullptr) {
            int storedHitpoints = hitpointmap->getNum();
            Hitpoint* hitpoints = hitpointmap->getHitpoints();
            for (int i = 0; i < storedHitpoints; i++) {
                int c = hitpoints[i].cr % W;
                int r = hitpoints[i].cr / W;
                if (!((r == 0 || hash[c][r] == hash[c][r - 1]) && (r == H - 1 || hash[c][r] == hash[c][r + 1]) && (c == 0 || hash[c][r] == hash[c - 1][r]) && (c == W - 1 || hash[c][r] == hash[c + 1][r])))
                    hitpoints[i].weight = hitpoints[i].weight / 9;
            }
        }
        printf("Resampling...\n");
        MultiThreadResampling();
    }
    for (int i = 0; i < W; i++)
        delete[] hash[i];
    delete[] hash;
}