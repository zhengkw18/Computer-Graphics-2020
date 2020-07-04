#ifndef OBJECT3D_H
#define OBJECT3D_H

#include "utils.h"
#include <sstream>
#include <vecmath.h>

class Material;
class ObjectHit;
class Ray;
class TriangleKDTreeNode;

// Base class for all 3d entities.
class Object3D {
public:
    virtual ~Object3D() = default;
    Object3D();
    // Intersect Ray with this object. If hit, store information in hit structure.
    virtual bool intersect(const Ray& r, ObjectHit& h) = 0;
    void setMaterial(Material* _material);
    Material* getMaterial() const;
    int getHash() const;
    virtual void Input(std::stringstream& fin) = 0;
    virtual void Initialize() = 0;

protected:
    Material* material;
    int hash;
};

class Plane : public Object3D {
public:
    bool intersect(const Ray& r, ObjectHit& h) override;
    void Input(std::stringstream& fin);
    void Initialize();

private:
    Vector3f normal;
    double d;
    Vector3f dx, dy;
    Vector3f center;
};

class Sphere : public Object3D {
public:
    bool intersect(const Ray& r, ObjectHit& h) override;
    void Input(std::stringstream& fin);
    void Initialize();

private:
    Vector3f center;
    double radius;
    Vector3f dz, dx, dy;
};

class Mesh;

class Triangle : public Object3D {

public:
    Vector3f vertices[3];
    int VertixID[3], normalVertixID[3], textureVertixID[3];
    Triangle();
    bool intersect(const Ray& ray, ObjectHit& hit) override;
    void Input(std::stringstream& fin);
    void Initialize();
    Vector3f getNormal() const;
    void setParent(Mesh* mesh);
    double GetMinCoord(int coord) const;
    double GetMaxCoord(int coord) const;

private:
    Vector3f normal;
    int mainCoord;
    double nu, nv, nd, bnu, bnv, cnu, cnv;
    Mesh* parent;
};

class Mesh : public Object3D {
    static int coord_sort;
    static bool minCoord_sort;
    Vector3f* vn;
    Vector2f* vt;
    Vector3f* v;
    Triangle** tris;
    Material** mat;
    int vSize, vtSize, vnSize, fSize, matSize;
    int enableInterpolate;
    std::string objfile;
    TriangleKDTreeNode* root;

    static bool Trianglecmp(const Triangle* t1, const Triangle* t2);

    void normalInterpolate();
    void SplitNode(TriangleKDTreeNode* node);
    void SetupKDTree();
    bool TravelTree(TriangleKDTreeNode* node, const Ray& ray, ObjectHit& hit);

public:
    Mesh();
    ~Mesh();
    void setData(Vector3f* _v, Vector3f* _vn, Vector2f* _vt, Triangle** _tris, Material** _mat, int _vSize, int _vtSize, int _vnSize, int _fSize, int _matSize);
    bool intersect(const Ray& r, ObjectHit& h) override;
    void Input(std::stringstream& fin);
    void Initialize();
    Vector3f getVN(int i);
    Vector2f getVT(int i);
    int isenableInterpolate() const;
    bool hasVT() const;
    bool hasVN() const;
};

class Transform : public Object3D {
    // transforms a 3D point using a matrix, returning a 3D point
    static Vector3f transformPoint(const Matrix4f& mat, const Vector3f& point);
    // transform a 3D direction using a matrix, returning a direction
    static Vector3f transformDirection(const Matrix4f& mat, const Vector3f& dir);

public:
    Transform();
    ~Transform();
    bool intersect(const Ray& r, ObjectHit& h);
    void Input(std::stringstream& fin);
    void setObject(Object3D* object);
    void Initialize();

protected:
    Object3D* o; //un-transformed object
    Matrix4f transform;
    Matrix4f recover;
    Matrix4f matrix;
};

class Box : public Object3D {
public:
    bool intersect(const Ray& r, ObjectHit& h) override;
    void Input(std::stringstream& fin);
    void Initialize();

private:
    bool contain(const Vector3f& pos) const;
    Vector3f box_min;
    Vector3f box_max;
    Vector3f dx;
};

class Bezier : public Object3D {
public:
    bool intersect(const Ray& r, ObjectHit& h) override;
    void Input(std::stringstream& fin);
    void Initialize();
    ~Bezier();

private:
    double width, height, width2;
    int num;
    double *x_coefficient, *y_coefficient;
    double r = .365;
    struct Segment {
        double t0, t1, y0, y1, width, width2;
    } * segments;
    Vector3f center;
    std::string bezierfile;
    Vector2f getPos(double t);
    Vector2f getDir(double t);
    double sphere_intersect(const Ray& ray, const Vector3f& o, double radius);
    //solve y(t)=z0
    double solve_1(double z0);
    //sqrt(a(y(t)-b)^2+c)=x(t)
    //f(t)=x(t)-sqrt(a(y(t)-b)^2+c)
    bool solve_2(double low, double high, double init, const Ray& ray, double a, double b, double c, double& final_dis);
    double getft(double t, double a, double b, double c);
    double newton(double t, double a, double b, double c, double low = EPS, double high = 1 - EPS);
};

#endif
