#ifndef KDTREE_H
#define KDTREE_H

#include <vecmath.h>

class Ray;
class Triangle;

class AABB {
public:
    Vector3f box_min, box_max;
    AABB();
    ~AABB();
    double getArea();
    bool contain(const Vector3f& pos);
    //Woo
    double intersect(const Ray& ray);
    bool nearRay(const Ray& ray);
    bool insideRay(const Ray& ray);
    void update(Triangle* tri);
};

class KDTreeNode {
public:
    int d;
    AABB boundingbox;
    int id;
    bool isleaf;
};

class TriangleKDTreeNode : public KDTreeNode {
public:
    Triangle** tris;
    int size;
    double split;
    TriangleKDTreeNode* left;
    TriangleKDTreeNode* right;
    TriangleKDTreeNode();
    ~TriangleKDTreeNode();
};

class HitpointKDTreeNode : public KDTreeNode {
public:
    double maxR2;
};
#endif