#include "kdtree.hpp"
#include "object3d.hpp"
#include "ray.hpp"
#include "utils.h"

AABB::AABB()
{
    box_min = Vector3f(INF, INF, INF);
    box_max = Vector3f(-INF, -INF, -INF);
}
AABB::~AABB() {}
double AABB::getArea()
{
    double a = box_max[0] - box_min[0];
    double b = box_max[1] - box_min[1];
    double c = box_max[2] - box_min[2];
    return 2 * (a * b + b * c + c * a);
}
bool AABB::contain(const Vector3f& pos)
{
    for (int coord = 0; coord < 3; coord++)
        if (pos[coord] <= box_min[coord] - EPS || pos[coord] >= box_max[coord] + EPS)
            return false;
    return true;
}
double AABB::intersect(const Ray& ray)
{
    if (contain(ray.getOrigin())) {
        double minDist = INF;
        for (int coord = 0; coord < 3; coord++) {
            double t = -1;
            if (ray.getDirection()[coord] >= EPS)
                t = (box_max[coord] - ray.getOrigin()[coord]) / ray.getDirection()[coord];
            else if (ray.getDirection()[coord] <= -EPS)
                t = (box_min[coord] - ray.getOrigin()[coord]) / ray.getDirection()[coord];
            if (t > EPS && t < minDist) {
                minDist = t;
            }
        }
        if (minDist > INF / 2)
            return -1;
        return minDist;
    }
    double maxDist = -1;
    Vector3f candidate;
    for (int coord = 0; coord < 3; coord++) {
        double t = -1;
        if (ray.getDirection()[coord] >= EPS)
            t = (box_min[coord] - ray.getOrigin()[coord]) / ray.getDirection()[coord];
        else if (ray.getDirection()[coord] <= -EPS)
            t = (box_max[coord] - ray.getOrigin()[coord]) / ray.getDirection()[coord];
        if (t > EPS && t > maxDist) {
            Vector3f C = ray.pointAtParameter(t);
            maxDist = t;
            candidate = C;
        }
    }
    if (maxDist > EPS) {
        if (contain(candidate))
            return maxDist;
    }
    return -1;
}
bool AABB::nearRay(const Ray& ray)
{
    Vector3f V;
    V[0] = (ray.getDirection()[0] > 0 ? box_max[0] : box_min[0]) - ray.getOrigin()[0];
    V[1] = (ray.getDirection()[1] > 0 ? box_max[1] : box_min[1]) - ray.getOrigin()[1];
    V[2] = (ray.getDirection()[2] > 0 ? box_max[2] : box_min[2]) - ray.getOrigin()[2];
    if (Vector3f::dot(V, ray.getDirection()) < EPS)
        return false;
    if (intersect(ray) > 0)
        return true;
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            for (int k = 0; k < 2; k++) {
                V[0] = (i ? box_max[0] : box_min[0]) - ray.getOrigin()[0];
                V[1] = (j ? box_max[1] : box_min[1]) - ray.getOrigin()[1];
                V[2] = (k ? box_max[2] : box_min[2]) - ray.getOrigin()[2];
                double a = V.length2(), b = Vector3f::dot(V, ray.getDirection());
                if (a - b * b < VOLUMETRIC_SAMPLE_DIST * VOLUMETRIC_SAMPLE_DIST)
                    return true;
            }
        }
    }
    for (int coord = 0; coord < 3; coord++) {
        int i = (coord + 1) % 3, j = (coord + 2) % 3;
        Vector3f D = ray.getDirection(), O = ray.getOrigin();
        double a = D[i] * D[i] + D[j] * D[j];
        if (a < EPS)
            continue;
        for (int s = 0; s < 2; s++) {
            for (int t = 0; t < 2; t++) {
                double ii = s ? box_max[i] : box_min[i];
                double jj = t ? box_max[j] : box_min[j];
                double b = D[i] * (O[i] - ii) + D[j] * (O[j] - jj);
                double c = (O[i] - ii) * (O[i] - ii) + (O[j] - jj) * (O[j] - jj);
                double tt = -b / a;
                double zz = O[coord] + tt * D[coord];
                if (box_min[coord] < zz && zz < box_max[coord] && c - b * b / a < VOLUMETRIC_SAMPLE_DIST * VOLUMETRIC_SAMPLE_DIST) {
                    return true;
                }
            }
        }
    }
    return false;
}
bool AABB::insideRay(const Ray& ray)
{
    Vector3f V;
    V[0] = (ray.getDirection()[0] < 0 ? box_max[0] : box_min[0]) - ray.getOrigin()[0];
    V[1] = (ray.getDirection()[1] < 0 ? box_max[1] : box_min[1]) - ray.getOrigin()[1];
    V[2] = (ray.getDirection()[2] < 0 ? box_max[2] : box_min[2]) - ray.getOrigin()[2];
    if (Vector3f::dot(V, ray.getDirection()) < EPS)
        return false;
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            for (int k = 0; k < 2; k++) {
                V[0] = (i ? box_max[0] : box_min[0]) - ray.getOrigin()[0];
                V[1] = (j ? box_max[1] : box_min[1]) - ray.getOrigin()[1];
                V[2] = (k ? box_max[2] : box_min[2]) - ray.getOrigin()[2];
                double a = V.length2(), b = Vector3f::dot(V, ray.getDirection());
                if (a - b * b > VOLUMETRIC_SAMPLE_DIST * VOLUMETRIC_SAMPLE_DIST)
                    return false;
            }
        }
    }
    return true;
}
void AABB::update(Triangle* tri)
{
    for (int coord = 0; coord < 3; coord++) {
        if (tri->GetMinCoord(coord) < box_min[coord])
            box_min[coord] = tri->GetMinCoord(coord);
        if (tri->GetMaxCoord(coord) > box_max[coord])
            box_max[coord] = tri->GetMaxCoord(coord);
    }
}

TriangleKDTreeNode::TriangleKDTreeNode()
{
    size = 0;
    split = 0;
    d = -1;
    left = right = nullptr;
}
TriangleKDTreeNode::~TriangleKDTreeNode()
{
    delete left;
    delete right;
}