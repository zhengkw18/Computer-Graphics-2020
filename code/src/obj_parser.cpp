#include "obj_parser.hpp"
#include "image.hpp"
#include "material.hpp"
#include "object3d.hpp"
#include <algorithm>
#include <fstream>

void ObjParser::ReadMtlSize(std::string file)
{
    std::ifstream fin(file.c_str());
    std::string order;

    while (getline(fin, order, '\n')) {
        std::stringstream fin2(order);
        std::string var;
        if (!(fin2 >> var))
            continue;
        if (var == "newmtl")
            matSize++;
    }
    fin.close();

    if (matSize > 0)
        mat = new Material*[matSize];
}

void ObjParser::ReadObjSize(std::string file)
{
    std::ifstream fin(file.c_str());
    std::string order;

    while (getline(fin, order, '\n')) {
        std::stringstream fin2(order);
        std::string var;
        if (!(fin2 >> var))
            continue;
        if (var == "mtllib") {
            std::string mtlFile;
            fin2 >> mtlFile;
            ReadMtlSize(mtlFile);
        }
        if (var == "v")
            vSize++;
        if (var == "vt")
            vtSize++;
        if (var == "vn")
            vnSize++;
        if (var == "f") {
            int vertexCnt = 0;
            std::string var;
            while (fin2 >> var)
                vertexCnt++;
            fSize += std::max(0, vertexCnt - 2);
        }
    }
    fin.close();

    v = new Vector3f[vSize];
    if (vtSize > 0)
        vt = new Vector2f[vtSize];
    if (vnSize > 0)
        vn = new Vector3f[vnSize];
    tris = new Triangle*[fSize];
}
void ObjParser::ReadMtl(std::string file)
{
    std::ifstream fin(file.c_str());
    std::string order;

    int matCnt = 0;
    while (getline(fin, order, '\n')) {
        std::stringstream fin2(order);
        std::string var;
        if (!(fin2 >> var))
            continue;

        if (var == "newmtl") {
            std::string matName;
            fin2 >> matName;
            matMap[matName] = matCnt;
            mat[matCnt] = new Material;
            matCnt++;
        }
        if (var == "Ka") {
        }
        if (var == "Kd") {
            mat[matCnt - 1]->Color.Input(fin2);
            if (GAMMA) {
                mat[matCnt - 1]->Color[0] = std::pow(mat[matCnt - 1]->Color[0], 2.2);
                mat[matCnt - 1]->Color[1] = std::pow(mat[matCnt - 1]->Color[1], 2.2);
                mat[matCnt - 1]->Color[2] = std::pow(mat[matCnt - 1]->Color[2], 2.2);
            }
            if (mat[matCnt - 1]->reflection_refraction < EPS)
                mat[matCnt - 1]->diffusion = 1;
        }
        if (var == "Ks") {
            fin2 >> mat[matCnt - 1]->specularity;
        }
        if (var == "Tf") {
            Vector3f color;
            color.Input(fin2);
            color = Vector3f(1.) - color;
            if (color.power() < 1 - EPS) {
                mat[matCnt - 1]->transparent = 1;
                mat[matCnt - 1]->diffusion = color.max();
                mat[matCnt - 1]->reflection_refraction = 1 - mat[matCnt - 1]->diffusion;
                if (mat[matCnt - 1]->diffusion > EPS)
                    color = color / mat[matCnt - 1]->diffusion;
            } else {
                mat[matCnt - 1]->diffusion = 1;
            }
        }
        if (var == "Ni") {
            fin2 >> mat[matCnt - 1]->refractionIndex;
            mat[matCnt - 1]->transparent = 1;
        }
        if (var == "Ns") {
            fin2 >> mat[matCnt - 1]->shininess;
        }
        if (var == "map_Kd") {
            std::string bmpFile;
            fin2 >> bmpFile;
            mat[matCnt - 1]->texture = Image::LoadBMP(bmpFile.c_str());
        }
        if (var == "map_bump") {
            std::string bmpFile;
            fin2 >> bmpFile;
            mat[matCnt - 1]->bump = Image::LoadBMP(bmpFile.c_str(), true);
        }
    }
    fin.close();
}

ObjParser::ObjParser(Mesh* _mesh)
{
    mesh = _mesh;
    vSize = 0;
    vtSize = 0;
    vnSize = 0;
    fSize = 0;
    matSize = 0;
    vt = nullptr;
    vn = nullptr;
    mat = nullptr;
}
void ObjParser::ReadObj(std::string file)
{
    ReadObjSize(file);
    std::ifstream fin(file.c_str());
    std::string order;

    int matID = -1;
    int vCnt = 0, vtCnt = 0, vnCnt = 0, fCnt = 0;
    while (getline(fin, order, '\n')) {
        std::stringstream fin2(order);
        std::string var;
        if (!(fin2 >> var))
            continue;

        if (var == "mtllib") {
            std::string mtlFile;
            fin2 >> mtlFile;
            ReadMtl(mtlFile);
        }
        if (var == "usemtl") {
            std::string matName;
            fin2 >> matName;
            matID = matMap[matName];
        }
        if (var == "v") {
            v[vCnt].Input(fin2);
            vCnt++;
        }
        if (var == "vt") {
            vt[vtCnt].Input(fin2);
            vtCnt++;
        }
        if (var == "vn") {
            vn[vnCnt].Input(fin2);
            vnCnt++;
        }
        if (var == "f") {
            Triangle* tri = tris[fCnt] = new Triangle;
            tri->setParent(mesh);
            if (matID != -1)
                tri->setMaterial(mat[matID]);
            else
                tri->setMaterial(mesh->getMaterial());
            std::string str;
            for (int i = 0; fin2 >> str; i++) {
                int bufferLen = 0, buffer[3];
                buffer[0] = buffer[1] = buffer[2] = -1;
                for (int s = 0, t = 0; t < (int)str.length(); t++)
                    if (t + 1 >= (int)str.length() || str[t + 1] == '/') {
                        buffer[bufferLen++] = atoi(str.substr(s, t - s + 1).c_str());
                        s = t + 2;
                    }
                int vertexID = i;
                if (i >= 3) {
                    vertexID = 2;
                    tri = tris[fCnt] = new Triangle;
                    *tri = *tris[fCnt - 1];
                    tri->vertices[1] = tri->vertices[2];
                    tri->VertixID[1] = tri->VertixID[2];
                    tri->textureVertixID[1] = tri->textureVertixID[2];
                    tri->normalVertixID[1] = tri->normalVertixID[2];
                }
                if (buffer[0] > 0) {
                    tri->VertixID[vertexID] = buffer[0] - 1;
                    tri->vertices[vertexID] = v[buffer[0] - 1];
                }
                if (buffer[1] > 0) {
                    tri->textureVertixID[vertexID] = buffer[1] - 1;
                }
                if (buffer[2] > 0) {
                    tri->normalVertixID[vertexID] = buffer[2] - 1;
                } else {
                    tri->normalVertixID[vertexID] = tri->VertixID[vertexID];
                }
                if (i >= 2) {
                    tri->Initialize();
                    fCnt++;
                }
            }
        }
    }
    fin.close();
    mesh->setData(v, vn, vt, tris, mat, vSize, vtSize, vnSize, fSize, matSize);
}