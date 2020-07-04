#ifndef SCENE_PARSER_H
#define SCENE_PARSER_H

#include <cassert>
#include <fstream>
#include <set>
#include <vecmath.h>

class Camera;
class Light;
class Object3D;
class Material;

class SceneParser {
public:
    SceneParser() = delete;
    SceneParser(const char* filename);

    ~SceneParser();
    Camera* getCamera() const;

    Vector3f getBackgroundColor() const;

    int getNumLights() const;

    Light* getLight(int i) const;

    int getNumMaterials() const;

    Material* getMaterial(int i) const;
    int getNumObjects() const;

    Object3D* getObject(int i) const;

private:
    Camera* camera;
    Vector3f background_color;
    int num_lights;
    Light** lights;
    int num_materials;
    Material** materials;
    int num_objects;
    Object3D** objects;
    Material* current_material;
    std::set<std::string> lighttypes;
    std::set<std::string> objecttypes;
    Material* parseMaterial(std::ifstream& fin, std::string type);
    Light* parseLight(std::ifstream& fin, std::string type);
    Object3D* parseObject(std::ifstream& fin, std::string type);
};

#endif // SCENE_PARSER_H