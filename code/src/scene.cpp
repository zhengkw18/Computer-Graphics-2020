#include "scene.hpp"
#include "camera.hpp"
#include "hit.hpp"
#include "hitpointmap.hpp"
#include "image.hpp"
#include "light.hpp"
#include "object3d.hpp"
#include "pathtracer.hpp"
#include "photonmap.hpp"
#include "photontracer.hpp"
#include "raytracer.hpp"
#include "scene_parser.hpp"
#include "utils.h"
#include "volumetricphotontracer.hpp"
#include <stdio.h>
#include <string>

void Scene::RayTracing()
{
    if (ALGORITHM == RT) {
        if (VOLUMETRIC) {
            PhotonMap* photonmap = new PhotonMap(MAX_VOLUMETRIC_PHOTONS);
            VolumetricPhotonTracer* photontracer = new VolumetricPhotonTracer(this, photonmap);
            photontracer->Run();
            delete photontracer;
            printf("PhotonMap Balancing...\n");
            photonmap->SetupKDTree();
            Image* _image = new Image(width, height);
            RayTracer* raytracer = new RayTracer(this, _image, nullptr, photonmap);
            raytracer->Run();
            delete raytracer;
            delete photonmap;
            image->absorb(_image);
            delete _image;
        } else {
            Image* _image = new Image(width, height);
            RayTracer* raytracer = new RayTracer(this, _image);
            raytracer->Run();
            delete raytracer;
            image->absorb(_image);
            delete _image;
        }
    } else {
        Image* _image = new Image(width, height);
        PathTracer* pathtracer = new PathTracer(this, _image);
        pathtracer->Run();
        delete pathtracer;
        image->absorb(_image);
        delete _image;
        if (VOLUMETRIC) {
            PhotonMap* photonmap = new PhotonMap(MAX_VOLUMETRIC_PHOTONS);
            VolumetricPhotonTracer* photontracer = new VolumetricPhotonTracer(this, photonmap);
            photontracer->Run();
            delete photontracer;
            printf("PhotonMap Balancing...\n");
            photonmap->SetupKDTree();
            Image* _image = new Image(width, height);
            RayTracer* raytracer = new RayTracer(this, _image, nullptr, photonmap);
            raytracer->Run();
            delete raytracer;
            delete photonmap;
            image->absorb(_image);
            delete _image;
        }
    }
}
void Scene::StochasticProgressivePhotonMapping()
{
    int sppmiters = camera->enableDOF() ? NUM_DOF_SAMPLE : 1;
    if (ALGORITHM == PT) {
        Image* _image = new Image(width, height);
        PathTracer* pathtracer = new PathTracer(this, _image);
        pathtracer->Run();
        delete pathtracer;
        image->absorb(_image);
        delete _image;
    }
    for (int sppmiter = 0; sppmiter < sppmiters; sppmiter++) {
        printf("SPPM Iteration= %d\n", sppmiter);
        output(sppmiter);
        HitpointMap* hitpointmap = new HitpointMap(MAX_HITPOINTS);
        if (VOLUMETRIC) {
            PhotonMap* photonmap = new PhotonMap(MAX_VOLUMETRIC_PHOTONS);
            VolumetricPhotonTracer* photontracer = new VolumetricPhotonTracer(this, photonmap);
            photontracer->Run();
            delete photontracer;
            printf("PhotonMap Balancing...\n");
            photonmap->SetupKDTree();
            Image* _image = new Image(width, height);
            RayTracer* raytracer = new RayTracer(this, _image, hitpointmap, photonmap);
            raytracer->Run();
            delete raytracer;
            delete photonmap;
            image->absorb(_image);
            delete _image;
        } else {
            Image* _image = new Image(width, height);
            RayTracer* raytracer = new RayTracer(this, _image, hitpointmap);
            raytracer->Run();
            delete raytracer;
            image->absorb(_image);
            delete _image;
        }
        int storedHitpoints = hitpointmap->getNum();
        printf("Stored Hitpoints: %d\n", storedHitpoints);
        printf("HitpointMap Balancing...\n");
        hitpointmap->SetupKDTree();
        PhotonTracer* photontracer = new PhotonTracer(this, hitpointmap);
        printf("Photon Tracing...\n");
        for (int iter = 1; iter <= ITERATIONS; iter++) {
            photontracer->Run(iter, sppmiter * ITERATIONS);
            hitpointmap->MaintainHitpoints();
            if (iter % 100 == 0)
                printf("Iter=%d/%d\n", iter, ITERATIONS);
        }
        printf("Rendering Image...\n");
        Hitpoint* hitpoints = hitpointmap->getHitpoints();
        for (int i = 0; i < storedHitpoints; i++) {
            int c = hitpoints[i].cr % width;
            int r = hitpoints[i].cr / width;
            Vector3f color = image->GetPixel(c, r) + hitpoints[i].color * hitpoints[i].weight * (4.0 / (hitpoints[i].R2 * NUM_EMIT_PHOTONS * ITERATIONS));
            image->SetPixel(c, r, color.confined());
        }
        delete photontracer;
        delete hitpointmap;
    }
}
void Scene::output(int sppmiter)
{
    std::string outputname = std::to_string(sppmiter) + ".bmp";
    image->SaveBMP(outputname.c_str());
}

Scene::Scene(const char* filename)
{
    sceneparser = new SceneParser(filename);
    camera = sceneparser->getCamera();
    width = camera->getWidth();
    height = camera->getHeight();
    image = new Image(width, height);
    //image = Image::LoadBMP("output/load.bmp");
}
Scene::~Scene()
{
    delete sceneparser;
    delete image;
}
int Scene::getWidth() const { return width; }
int Scene::getHeight() const { return height; }
Vector3f Scene::getBackgroundColor() const { return sceneparser->getBackgroundColor(); }
Camera* Scene::getCamera() const { return camera; }
int Scene::getNumLights() const { return sceneparser->getNumLights(); }
Light* Scene::getLight(int i) const { return sceneparser->getLight(i); }
int Scene::getNumMaterials() const { return sceneparser->getNumMaterials(); }
Material* Scene::getMaterial(int i) const { return sceneparser->getMaterial(i); }
int Scene::getNumObjects() const { return sceneparser->getNumObjects(); }
Object3D* Scene::getObject(int i) const { return sceneparser->getObject(i); }
Image* Scene::getImage() const { return image; }
bool Scene::intersectLights(const Ray& ray, LightHit& h)
{
    int num_lights = getNumLights();
    if (num_lights) {
        bool result = false;
        for (int i = 0; i < num_lights; ++i) {
            Light* light = getLight(i);
            LightHit hit;
            bool isIntersect = light->intersect(ray, hit);
            if (isIntersect) {
                if (hit.getT() < h.getT()) {
                    h = hit;
                }
                result = true;
            }
        }
        return result;
    }
    return false;
}
void Scene::Run()
{
    if (!SPPM) {
        RayTracing();
    } else {
        StochasticProgressivePhotonMapping();
    }
}
bool Scene::intersectObjects(const Ray& r, ObjectHit& h)
{
    int num_objects = getNumObjects();
    if (num_objects) {
        bool result = false;
        ObjectHit hhit;
        for (int i = 0; i < num_objects; ++i) {
            Object3D* object = getObject(i);
            ObjectHit hit;
            bool isIntersect = object->intersect(r, hit);
            if (isIntersect) {
                if (hit.getT() < hhit.getT()) {
                    hhit = hit;
                    result = true;
                }
            }
        }
        if (result)
            h = hhit;
        return result;
    }
    return false;
}