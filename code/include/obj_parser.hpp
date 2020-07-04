#ifndef OBJ_PARSER_H
#define OBJ_PARSER_H

#include <map>
#include <vecmath.h>

class Mesh;
class Triangle;
class Material;

class ObjParser {
    int vSize, vtSize, vnSize, fSize, matSize;
    Vector3f* v;
    Vector2f* vt;
    Vector3f* vn;
    Triangle** tris;
    Material** mat;
    std::map<std::string, int> matMap;
    Mesh* mesh;
    void ReadMtlSize(std::string file);

    void ReadObjSize(std::string file);
    void ReadMtl(std::string file);

public:
    ObjParser(Mesh* _mesh);
    ~ObjParser() = default;
    void ReadObj(std::string file);
};

#endif