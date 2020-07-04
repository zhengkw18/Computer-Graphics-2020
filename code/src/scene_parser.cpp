#include "scene_parser.hpp"
#include "camera.hpp"
#include "light.hpp"
#include "material.hpp"
#include "object3d.hpp"
#include "utils.h"
#include <iostream>
#include <sstream>
#include <string.h>

SceneParser::SceneParser(const char* filename)
{
    // initialize some reasonable default values
    objects = nullptr;
    camera = nullptr;
    background_color = Vector3f(0.5, 0.5, 0.5);
    num_lights = 0;
    lights = nullptr;
    num_materials = 0;
    materials = nullptr;
    current_material = nullptr;

    // parse the file
    assert(filename != nullptr);
    const char* ext = &filename[strlen(filename) - 4];

    if (strcmp(ext, ".txt") != 0) {
        printf("wrong file name extension\n");
        exit(0);
    }
    std::ifstream fin(filename);

    if (!fin.is_open()) {
        printf("cannot open scene file\n");
        exit(0);
    }

    lighttypes.insert("PointLight");
    lighttypes.insert("AreaLight");
    lighttypes.insert("TextureLight");

    objecttypes.insert("Mesh");
    objecttypes.insert("Plane");
    objecttypes.insert("Sphere");
    objecttypes.insert("Triangle");
    objecttypes.insert("Transform");
    objecttypes.insert("Box");
    objecttypes.insert("Bezier");

    std::string line;

    while (getline(fin, line)) {
        trim(line);
        if (line == "Camera") {
            camera = new Camera;
            while (getline(fin, line)) {
                trim(line);
                if (line == "end")
                    break;
                std::stringstream ss(line);
                camera->Input(ss);
            }
            camera->Initialize();
        } else if (line == "Lights") {
            getline(fin, line);
            std::string var;
            std::stringstream ss(line);
            ss >> var >> num_lights;
            assert(var == "numLights");
            lights = new Light*[num_lights];
            int i = 0;
            while (getline(fin, line)) {
                trim(line);
                if (line == "end")
                    break;
                if (lighttypes.find(line) != lighttypes.end())
                    lights[i++] = parseLight(fin, line);
            }
            assert(i == num_lights);
        } else if (line == "Background") {
            getline(fin, line);
            std::string var;
            std::stringstream ss(line);
            ss >> var;
            assert(var == "color");
            background_color.Input(ss);
            getline(fin, line);
            trim(line);
            assert(line == "end");
        } else if (line == "Materials") {
            getline(fin, line);
            std::string var;
            std::stringstream ss(line);
            ss >> var >> num_materials;
            assert(var == "numMaterials");
            materials = new Material*[num_materials];
            int i = 0;
            while (getline(fin, line)) {
                trim(line);
                if (line == "end")
                    break;
                if (line == "Material")
                    materials[i++] = parseMaterial(fin, line);
            }
            assert(i == num_materials);
        } else if (line == "Objects") {
            getline(fin, line);
            std::string var;
            std::stringstream ss(line);
            ss >> var >> num_objects;
            assert(var == "numObjects");
            objects = new Object3D*[num_objects];
            int i = 0;
            while (getline(fin, line)) {
                trim(line);
                if (line == "end")
                    break;
                if (objecttypes.find(line) != objecttypes.end())
                    objects[i++] = parseObject(fin, line);
                else {
                    std::stringstream ss(line);
                    std::string var;
                    int id;
                    ss >> var >> id;
                    assert(var == "MaterialIndex");
                    current_material = materials[id];
                }
            }
            assert(i == num_objects);
        }
    }

    fin.close();
    if (num_lights == 0) {
        printf("WARNING:    No lights specified\n");
    }
}

SceneParser::~SceneParser()
{
    delete camera;

    int i;
    for (i = 0; i < num_materials; i++) {
        delete materials[i];
    }
    delete[] materials;
    for (i = 0; i < num_lights; i++) {
        delete lights[i];
    }
    delete[] lights;
    for (i = 0; i < num_objects; i++) {
        delete objects[i];
    }
    delete[] objects;
}
Camera* SceneParser::getCamera() const
{
    return camera;
}

Vector3f SceneParser::getBackgroundColor() const
{
    return background_color;
}

int SceneParser::getNumLights() const
{
    return num_lights;
}

Light* SceneParser::getLight(int i) const
{
    assert(i >= 0 && i < num_lights);
    return lights[i];
}

int SceneParser::getNumMaterials() const
{
    return num_materials;
}

Material* SceneParser::getMaterial(int i) const
{
    assert(i >= 0 && i < num_materials);
    return materials[i];
}
int SceneParser::getNumObjects() const
{
    return num_objects;
}

Object3D* SceneParser::getObject(int i) const
{
    assert(i >= 0 && i < num_objects);
    return objects[i];
}

Material* SceneParser::parseMaterial(std::ifstream& fin, std::string type)
{
    Material* material;
    std::string line;
    if (type == "Material")
        material = new Material;
    while (getline(fin, line)) {
        trim(line);
        if (line == "end")
            break;
        std::stringstream ss(line);
        material->Input(ss);
    }
    return material;
}
Light* SceneParser::parseLight(std::ifstream& fin, std::string type)
{
    Light* light;
    std::string line;
    if (type == "PointLight")
        light = new PointLight;
    else if (type == "AreaLight")
        light = new AreaLight;
    else if (type == "TextureLight")
        light = new TextureLight;
    while (getline(fin, line)) {
        trim(line);
        if (line == "end")
            break;
        std::stringstream ss(line);
        light->Input(ss);
    }
    light->Initialize();
    return light;
}
Object3D* SceneParser::parseObject(std::ifstream& fin, std::string type)
{
    assert(current_material != nullptr);
    Object3D* object;
    std::string line;
    if (type == "Mesh")
        object = new Mesh;
    else if (type == "Plane")
        object = new Plane;
    else if (type == "Sphere")
        object = new Sphere;
    else if (type == "Triangle")
        object = new Triangle;
    else if (type == "Transform")
        object = new Transform;
    else if (type == "Box")
        object = new Box;
    else if (type == "Bezier")
        object = new Bezier;
    object->setMaterial(current_material);
    if (type == "Transform") {
        while (getline(fin, line)) {
            trim(line);
            if (line == "end")
                break;
            if (objecttypes.find(line) == objecttypes.end()) {
                std::stringstream ss(line);
                object->Input(ss);
            } else {
                Object3D* subobject = parseObject(fin, line);
                Transform* transform = dynamic_cast<Transform*>(object);
                transform->setObject(subobject);
            }
        }
    } else {
        while (getline(fin, line)) {
            trim(line);
            if (line == "end")
                break;
            std::stringstream ss(line);
            object->Input(ss);
        }
    }
    object->Initialize();
    return object;
}