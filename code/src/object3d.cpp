#include "object3d.hpp"
#include "hit.hpp"
#include "image.hpp"
#include "kdtree.hpp"
#include "material.hpp"
#include "obj_parser.hpp"
#include "ray.hpp"
#include "utils.h"
#include <algorithm>
#include <fstream>

Object3D::Object3D()
{
    material = defaultMaterial;
    hash = rand();
}
void Object3D::setMaterial(Material* _material)
{
    material = _material;
}
Material* Object3D::getMaterial() const { return material; }
int Object3D::getHash() const { return hash; }

bool Plane::intersect(const Ray& r, ObjectHit& h)
{
    double dot = Vector3f::dot(normal, r.getDirection());
    if (fabs(dot) < EPS)
        return false;
    double t = (d - Vector3f::dot(normal, r.getOrigin())) / dot;
    if (t < EPS)
        return false;
    else {
        Vector3f C = r.pointAtParameter(t);
        double u = Vector3f::dot(C - center, dx) / dx.length2() + 0.5;
        double v = Vector3f::dot(C - center, dy) / dy.length2() + 0.5;
        Vector3f N = normal;
        if (material->bump != nullptr) {
            Vector3f addN = material->bump->getSmoothPixel(u, v);
            addN = 2 * addN - Vector3f(1.);
            N = N * addN[2] + dx.normalized() * addN[0] + dy.normalized() * addN[1];
        }
        if (Vector3f::dot(normal, r.getDirection()) < 0)
            h = ObjectHit(t, this, N, C, r.getDirection(), u, v, true);
        else
            h = ObjectHit(t, this, -N, C, r.getDirection(), u, v, false);
        return true;
    }
}
void Plane::Input(std::stringstream& fin)
{
    std::string var;
    fin >> var;
    if (var == "normal") {
        normal.Input(fin);
        normal.normalize();
    } else if (var == "d")
        fin >> d;
    else if (var == "dx")
        dx.Input(fin);
    else if (var == "dy")
        dy.Input(fin);
    else if (var == "center")
        center.Input(fin);
}
void Plane::Initialize() {}

bool Sphere::intersect(const Ray& r, ObjectHit& h)
{
    Vector3f ro = center - r.getOrigin();
    double bb = Vector3f::dot(ro, r.getDirection());
    double d = bb * bb - ro.length2() + radius * radius;
    if (d < 0)
        return false;
    else
        d = sqrt(d);
    double t = bb - d > EPS ? bb - d : bb + d > EPS ? bb + d : -1;
    if (t < EPS)
        return false;
    Vector3f C = r.pointAtParameter(t);
    Vector3f N = C - center;
    N.normalize();
    Vector3f N_old = N;
    double a = acos(Vector3f::dot(N, dz));
    double b = acos(std::min(std::max(Vector3f::dot(N, dx) / (double)sin(a), -1.0), 1.0));
    double v = a / PI, u = b / 2 / PI;
    if (Vector3f::dot(N, dy) < 0)
        u = 1 - u;
    if (material->bump != nullptr) {
        Vector3f U = Vector3f::cross(dz, N).normalized();
        Vector3f V = Vector3f::cross(U, N).normalized();
        Vector3f addN = material->bump->getSmoothPixel(u, v);
        addN = 2 * addN - Vector3f(1.);
        N = N * addN[2] + U.normalized() * addN[0] + V.normalized() * addN[1];
    }
    if (Vector3f::dot(r.getDirection(), N_old) < 0) {
        h = ObjectHit(t, this, N, C, r.getDirection(), u, v, true);
    } else {
        h = ObjectHit(t, this, -N, C, r.getDirection(), u, v, false);
    }
    return true;
}
void Sphere::Input(std::stringstream& fin)
{
    std::string var;
    fin >> var;
    if (var == "center")
        center.Input(fin);
    else if (var == "radius")
        fin >> radius;
    else if (var == "dz")
        dz.Input(fin);
    else if (var == "dx")
        dx.Input(fin);
}
void Sphere::Initialize()
{
    dz.normalize();
    dx.normalize();
    dy = Vector3f::cross(dz, dx).normalized();
}

Triangle::Triangle()
{
    parent = nullptr;
}
bool Triangle::intersect(const Ray& ray, ObjectHit& hit)
{
    Vector3f _N;
    Vector3f _C;
    Vector3f ray_V = ray.getDirection();
    Vector3f ray_O = ray.getOrigin();
    int u = (mainCoord + 1) % 3, v = (mainCoord + 2) % 3;
    double lnd = 1.0 / (ray_V[mainCoord] + nu * ray_V[u] + nv * ray_V[v]);
    if (lnd > INF)
        return false;
    double l = (nd - ray_O[mainCoord] - nu * ray_O[u] - nv * ray_O[v]) * lnd;
    if (l < EPS)
        return false;
    double hu = ray_O[u] + l * ray_V[u] - vertices[0][u];
    double hv = ray_O[v] + l * ray_V[v] - vertices[0][v];
    double x = hv * bnu + hu * bnv;
    if (x < 0)
        return false;
    double y = hv * cnv + hu * cnu;
    if (y < 0)
        return false;
    if (x + y > 1)
        return false;
    if (parent != nullptr && parent->hasVN())
        _N = parent->getVN(normalVertixID[0]) * (1 - x - y) + parent->getVN(normalVertixID[1]) * x + parent->getVN(normalVertixID[2]) * y;
    else
        _N = normal;
    _C = ray.pointAtParameter(l);
    double uu = x, vv = y;
    if (parent != nullptr && parent->hasVT()) {
        Vector2f p0 = parent->getVT(textureVertixID[0]);
        Vector2f p1 = parent->getVT(textureVertixID[1]);
        Vector2f p2 = parent->getVT(textureVertixID[2]);
        double u1 = p1[0] - p0[0];
        double u2 = p2[0] - p0[0];
        double v1 = p1[1] - p0[1];
        double v2 = p2[1] - p0[1];
        uu = p0[0] + u1 * x + u2 * y;
        vv = p0[1] + v1 * x + v2 * y;
        if (material->bump != nullptr) {
            double det = u1 * v2 - u2 * v1;
            if (det != 0) {
                int W = material->bump->Width();
                int H = material->bump->Height();
                double deltaX = material->bump->getSmoothPixel(u - 0.5 / W, v).power() - material->bump->getSmoothPixel(u + 0.5 / W, v).power();
                double deltaY = material->bump->getSmoothPixel(u, v - 0.5 / H).power() - material->bump->getSmoothPixel(u, v + 0.5 / H).power();
                Vector3f tX = ((vertices[1] - vertices[0]) * u2 - (vertices[2] - vertices[0]) * u1) / (W * det);
                Vector3f tY = ((vertices[1] - vertices[0]) * v2 - (vertices[2] - vertices[0]) * v1) / (H * det);
                _N.normalize();
                _N = _N + tX.normalized() * deltaX * 10 + tY.normalized() * deltaY * 10;
            }
        }
    }
    double d = Vector3f::dot(_N, ray_V);
    if (d < 0) {
        hit = ObjectHit(l, this, _N, _C, ray_V, uu, vv, true);
    } else {
        hit = ObjectHit(l, this, -_N, _C, ray_V, uu, vv, false);
    }
    return true;
}
void Triangle::Input(std::stringstream& fin)
{
    std::string var;
    fin >> var;
    if (var == "v1")
        vertices[0].Input(fin);
    else if (var == "v2")
        vertices[1].Input(fin);
    else if (var == "v3")
        vertices[2].Input(fin);
}
void Triangle::Initialize()
{
    Vector3f B = vertices[2] - vertices[0], C = vertices[1] - vertices[0];
    normal = Vector3f::cross(C, B);
    if (normal.isZero()) {
        normal = Vector3f(0, 0, 1);
        return;
    }
    if (fabs(normal[0]) > fabs(normal[1]))
        mainCoord = (fabs(normal[0]) > fabs(normal[2])) ? 0 : 2;
    else
        mainCoord = (fabs(normal[1]) > fabs(normal[2])) ? 1 : 2;
    int u = (mainCoord + 1) % 3, v = (mainCoord + 2) % 3;
    double krec = 1.0 / normal[mainCoord];
    nu = normal[u] * krec;
    nv = normal[v] * krec;
    nd = Vector3f::dot(normal, vertices[0]) * krec;
    double reci = 1.0 / (B[u] * C[v] - B[v] * C[u]);
    bnu = B[u] * reci;
    bnv = -B[v] * reci;
    cnu = C[v] * reci;
    cnv = -C[u] * reci;
    normal.normalize();
}
Vector3f Triangle::getNormal() const { return normal; }
void Triangle::setParent(Mesh* mesh)
{
    parent = mesh;
}
double Triangle::GetMinCoord(int coord) const
{
    double x0 = vertices[0][coord];
    double x1 = vertices[1][coord];
    double x2 = vertices[2][coord];
    if (x0 < x1)
        return (x0 < x2) ? x0 : x2;
    return (x1 < x2) ? x1 : x2;
}

double Triangle::GetMaxCoord(int coord) const
{
    double x0 = vertices[0][coord];
    double x1 = vertices[1][coord];
    double x2 = vertices[2][coord];
    if (x0 > x1)
        return (x0 > x2) ? x0 : x2;
    return (x1 > x2) ? x1 : x2;
}

int Mesh::coord_sort;
bool Mesh::minCoord_sort;

bool Mesh::Trianglecmp(const Triangle* t1, const Triangle* t2)
{
    double (Triangle::*GetCoord)(int) const = (minCoord_sort ? &Triangle::GetMinCoord : &Triangle::GetMaxCoord);
    return (t1->*GetCoord)(coord_sort) < (t2->*GetCoord)(coord_sort);
}

void Mesh::normalInterpolate()
{
    vnSize = vSize;
    vn = new Vector3f[vSize];
    double* angleSum = new double[vSize];
    for (int i = 0; i < vSize; i++)
        angleSum[i] = 0;
    for (int i = 0; i < fSize; i++) {
        Triangle* tri = tris[i];
        Vector3f A = tri->vertices[1] - tri->vertices[0];
        Vector3f B = tri->vertices[2] - tri->vertices[0];
        Vector3f C = tri->vertices[2] - tri->vertices[1];
        A.normalize();
        B.normalize();
        C.normalize();
        double angle[3];
        if (enableInterpolate == 1) {
            angle[0] = 1;
            angle[1] = 1;
            angle[2] = 1;
        } else if (enableInterpolate == 2) {
            angle[0] = Vector3f::cross(A, B).length();
            angle[1] = Vector3f::cross(A, C).length();
            angle[2] = Vector3f::cross(C, B).length();
        } else if (enableInterpolate == 3) {
            angle[0] = acos(Vector3f::dot(A, B));
            angle[1] = acos(-Vector3f::dot(A, C));
            angle[2] = acos(Vector3f::dot(B, C));
        }
        for (int i = 0; i < 3; i++) {
            int id = tri->VertixID[i];
            vn[id] = vn[id] + tri->getNormal() * angle[i];
            angleSum[id] += angle[i];
        }
    }
    for (int i = 0; i < vSize; i++) {
        vn[i] = vn[i] / angleSum[i];
    }
    delete angleSum;
}
void Mesh::SplitNode(TriangleKDTreeNode* node)
{
    //iff area0 * size0 + area1 * size1 + totalArea <= totalArea * totalSize then divide
    Triangle** minNode = new Triangle*[node->size];
    Triangle** maxNode = new Triangle*[node->size];
    for (int i = 0; i < node->size; i++) {
        minNode[i] = node->tris[i];
        maxNode[i] = node->tris[i];
    }

    double thisCost = node->boundingbox.getArea() * (node->size - 1);
    double minCost = thisCost;
    int bestCoord = -1, leftSize = 0, rightSize = 0;
    double bestSplit = 0;
    for (int coord = 0; coord < 3; coord++) {
        coord_sort = coord;
        minCoord_sort = true;
        std::sort(minNode, minNode + node->size, Trianglecmp);
        minCoord_sort = false;
        std::sort(maxNode, maxNode + node->size, Trianglecmp);

        AABB leftBox = node->boundingbox;
        AABB rightBox = node->boundingbox;

        int j = 0;
        for (int i = 0; i < node->size; i++) {
            double split = minNode[i]->GetMinCoord(coord);
            leftBox.box_max[coord] = split;
            rightBox.box_min[coord] = split;
            for (; j < node->size && maxNode[j]->GetMaxCoord(coord) <= split + EPS; j++)
                ;
            double cost = leftBox.getArea() * i + rightBox.getArea() * (node->size - j);
            if (cost < minCost) {
                minCost = cost;
                bestCoord = coord;
                bestSplit = split;
                leftSize = i;
                rightSize = node->size - j;
            }
        }

        j = 0;
        for (int i = 0; i < node->size; i++) {
            double split = maxNode[i]->GetMaxCoord(coord);
            leftBox.box_max[coord] = split;
            rightBox.box_min[coord] = split;
            for (; j < node->size && minNode[j]->GetMinCoord(coord) <= split - EPS; j++)
                ;
            double cost = leftBox.getArea() * j + rightBox.getArea() * (node->size - i);
            if (cost < minCost) {
                minCost = cost;
                bestCoord = coord;
                bestSplit = split;
                leftSize = j;
                rightSize = node->size - i;
            }
        }
    }

    delete[] minNode;
    delete[] maxNode;

    if (bestCoord != -1) {
        leftSize = rightSize = 0;
        for (int i = 0; i < node->size; i++) {
            if (node->tris[i]->GetMinCoord(bestCoord) <= bestSplit - EPS || node->tris[i]->GetMaxCoord(bestCoord) <= bestSplit + EPS)
                leftSize++;
            if (node->tris[i]->GetMaxCoord(bestCoord) >= bestSplit + EPS || node->tris[i]->GetMinCoord(bestCoord) >= bestSplit - EPS)
                rightSize++;
        }
        AABB leftBox = node->boundingbox;
        AABB rightBox = node->boundingbox;
        leftBox.box_max[bestCoord] = bestSplit;
        rightBox.box_min[bestCoord] = bestSplit;
        double cost = leftBox.getArea() * leftSize + rightBox.getArea() * rightSize;

        if (cost < thisCost) {
            node->d = bestCoord;
            node->split = bestSplit;

            node->left = new TriangleKDTreeNode;
            node->left->boundingbox = node->boundingbox;
            node->left->boundingbox.box_max[node->d] = node->split;

            node->right = new TriangleKDTreeNode;
            node->right->boundingbox = node->boundingbox;
            node->right->boundingbox.box_min[node->d] = node->split;

            node->left->tris = new Triangle*[leftSize];
            node->right->tris = new Triangle*[rightSize];
            int leftCnt = 0, rightCnt = 0;
            for (int i = 0; i < node->size; i++) {
                if (node->tris[i]->GetMinCoord(node->d) <= node->split - EPS || node->tris[i]->GetMaxCoord(node->d) <= node->split + EPS)
                    node->left->tris[leftCnt++] = node->tris[i];
                if (node->tris[i]->GetMaxCoord(node->d) >= node->split + EPS || node->tris[i]->GetMinCoord(node->d) >= node->split - EPS)
                    node->right->tris[rightCnt++] = node->tris[i];
            }
            node->left->size = leftSize;
            node->right->size = rightSize;

            SplitNode(node->left);
            SplitNode(node->right);
        }
    }
}
void Mesh::SetupKDTree()
{
    printf("Building Mesh KDTree...\n");
    root = new TriangleKDTreeNode();
    root->tris = new Triangle*[fSize];
    root->size = fSize;
    for (int i = 0; i < fSize; i++) {
        root->tris[i] = tris[i];
        root->boundingbox.update(root->tris[i]);
    }
    SplitNode(root);
    printf("Mesh KDTree Built\n");
    printf("Box Min: %.2lf %.2lf %.2lf; Box Max: %.2lf %.2lf %.2lf\n", root->boundingbox.box_min[0], root->boundingbox.box_min[1], root->boundingbox.box_min[2], root->boundingbox.box_max[0], root->boundingbox.box_max[1], root->boundingbox.box_max[2]);
}
bool Mesh::TravelTree(TriangleKDTreeNode* node, const Ray& ray, ObjectHit& hit)
{
    if (node->boundingbox.intersect(ray) <= EPS)
        return false;

    if (node->left == nullptr && node->right == nullptr) {
        bool crash = false;
        ObjectHit hhit;
        for (int i = 0; i < node->size; i++) {
            ObjectHit h;
            if (node->tris[i]->intersect(ray, h)) {
                if (h.getT() < hhit.getT()) {
                    crash = true;
                    hhit = h;
                }
            }
        }
        if (crash)
            hit = hhit;
        return crash;
    }

    if (node->left->boundingbox.contain(ray.getOrigin())) {
        if (TravelTree(node->left, ray, hit)) {
            return true;
        } else {
            return TravelTree(node->right, ray, hit);
        }
    }
    if (node->right->boundingbox.contain(ray.getOrigin())) {
        if (TravelTree(node->right, ray, hit)) {
            return true;
        } else {
            return TravelTree(node->left, ray, hit);
        }
    }

    double leftDist = node->left->boundingbox.intersect(ray);
    double rightDist = node->right->boundingbox.intersect(ray);
    if (rightDist <= -EPS)
        return TravelTree(node->left, ray, hit);
    if (leftDist <= -EPS)
        return TravelTree(node->right, ray, hit);

    if (leftDist < rightDist) {
        if (TravelTree(node->left, ray, hit)) {
            return true;
        } else {
            return TravelTree(node->right, ray, hit);
        }
    }
    if (TravelTree(node->right, ray, hit)) {
        return true;
    } else {
        return TravelTree(node->left, ray, hit);
    }
}

Mesh::Mesh()
{
    enableInterpolate = 0;
}
Mesh::~Mesh()
{
    if (mat != nullptr) {
        for (int i = 0; i < matSize; i++) {
            delete mat[i];
        }
        delete[] mat;
        for (int i = 0; i < fSize; i++) {
            delete tris[i];
        }
    }
    delete[] tris;
    delete[] v;
    if (vn != nullptr)
        delete[] vn;
    if (vt != nullptr)
        delete[] vt;
    delete root;
}
void Mesh::setData(Vector3f* _v, Vector3f* _vn, Vector2f* _vt, Triangle** _tris, Material** _mat, int _vSize, int _vtSize, int _vnSize, int _fSize, int _matSize)
{
    v = _v;
    vn = _vn;
    vt = _vt;
    tris = _tris;
    mat = _mat;
    vSize = _vSize;
    vtSize = _vtSize;
    vnSize = _vnSize;
    fSize = _fSize;
    matSize = _matSize;
}
bool Mesh::intersect(const Ray& r, ObjectHit& h)
{
    if (TravelTree(root, r, h))
        return true;
    return false;
}
void Mesh::Input(std::stringstream& fin)
{
    std::string var;
    fin >> var;
    if (var == "objfile")
        fin >> objfile;
    else if (var == "enableInterpolate")
        fin >> enableInterpolate;
}
void Mesh::Initialize()
{
    ObjParser* objparser = new ObjParser(this);
    objparser->ReadObj(objfile);
    delete objparser;
    if (vn == nullptr) {
        if (enableInterpolate) {
            normalInterpolate();
        }
    }
    SetupKDTree();
}
Vector3f Mesh::getVN(int i)
{
    return vn[i];
}
Vector2f Mesh::getVT(int i)
{
    return vt[i];
}
int Mesh::isenableInterpolate() const { return enableInterpolate; }
bool Mesh::hasVT() const { return vt != nullptr; }
bool Mesh::hasVN() const { return vn != nullptr; }

Vector3f Transform::transformPoint(const Matrix4f& mat, const Vector3f& point)
{
    return (mat * Vector4f(point, 1)).xyz();
}
// transform a 3D direction using a matrix, returning a direction
Vector3f Transform::transformDirection(const Matrix4f& mat, const Vector3f& dir)
{
    return (mat * Vector4f(dir, 0)).xyz();
}

Transform::Transform()
{
    matrix = Matrix4f::identity();
}
Transform::~Transform()
{
    delete o;
}
bool Transform::intersect(const Ray& r, ObjectHit& h)
{
    Vector3f trSource = transformPoint(transform, r.getOrigin());
    Vector3f trDirection = transformDirection(transform, r.getDirection());
    double scale = trDirection.length();
    Ray tr(trSource, trDirection);
    ObjectHit hit;
    bool inter = o->intersect(tr, hit);
    if (inter) {
        h = ObjectHit(hit.getT() / scale, hit.getObject(), transformDirection(transform.transposed(), hit.getN()), transformPoint(recover, hit.getC()), r.getDirection(), hit.getU(), hit.getV(), hit.isFront());
    }
    return inter;
}
void Transform::Input(std::stringstream& fin)
{
    std::string var;
    fin >> var;
    if (var == "Scale") {
        Vector3f s;
        s.Input(fin);
        matrix = matrix * Matrix4f::scaling(s[0], s[1], s[2]);
    } else if (var == "UniformScale") {
        double s;
        fin >> s;
        matrix = matrix * Matrix4f::uniformScaling(s);
    } else if (var == "Translate") {
        Vector3f t;
        t.Input(fin);
        matrix = matrix * Matrix4f::translation(t);
    } else if (var == "XRotate") {
        double angle;
        fin >> angle;
        angle = DegreesToRadians(angle);
        matrix = matrix * Matrix4f::rotateX(angle);
    } else if (var == "YRotate") {
        double angle;
        fin >> angle;
        angle = DegreesToRadians(angle);
        matrix = matrix * Matrix4f::rotateY(angle);
    } else if (var == "ZRotate") {
        double angle;
        fin >> angle;
        angle = DegreesToRadians(angle);
        matrix = matrix * Matrix4f::rotateZ(angle);
    } else if (var == "Rotate") {
        Vector3f axis;
        axis.Input(fin);
        double angle;
        fin >> angle;
        angle = DegreesToRadians(angle);
        matrix = matrix * Matrix4f::rotation(axis, angle);
    }
}
void Transform::setObject(Object3D* object)
{
    o = object;
}
void Transform::Initialize()
{
    transform = matrix.inverse();
    recover = matrix;
}
bool Box::contain(const Vector3f& pos) const
{
    for (int coord = 0; coord < 3; coord++)
        if (pos[coord] <= box_min[coord] - EPS || pos[coord] >= box_max[coord] + EPS)
            return false;
    return true;
}
bool Box::intersect(const Ray& r, ObjectHit& h)
{
    int final_coord = -1;
    double Dist = -1;
    if (contain(r.getOrigin())) {
        double minDist = INF;
        int now_coord;
        for (int coord = 0; coord < 3; coord++) {
            double t = -1;
            if (r.getDirection()[coord] >= EPS) {
                t = (box_max[coord] - r.getOrigin()[coord]) / r.getDirection()[coord];
                now_coord = 3 + coord;
            } else if (r.getDirection()[coord] <= -EPS) {
                t = (box_min[coord] - r.getOrigin()[coord]) / r.getDirection()[coord];
                now_coord = coord;
            }
            if (t > EPS && t < minDist) {
                minDist = t;
                final_coord = now_coord;
            }
        }
        if (minDist > INF / 2)
            return false;
        Dist = minDist;
    } else {
        double maxDist = -1;
        int now_coord;
        for (int coord = 0; coord < 3; coord++) {
            double t = -1;
            if (r.getDirection()[coord] >= EPS) {
                t = (box_min[coord] - r.getOrigin()[coord]) / r.getDirection()[coord];
                now_coord = coord;
            } else if (r.getDirection()[coord] <= -EPS) {
                t = (box_max[coord] - r.getOrigin()[coord]) / r.getDirection()[coord];
                now_coord = 3 + coord;
            }
            if (t > EPS && t > maxDist) {
                maxDist = t;
                final_coord = now_coord;
            }
        }
        if (maxDist < EPS)
            return false;
        Dist = maxDist;
    }
    Vector3f C = r.pointAtParameter(Dist);
    if (!contain(C))
        return false;
    Vector3f N;
    double u, v;
    if (final_coord == 0) {
        N = Vector3f(-1, 0, 0);
        u = (C[1] - box_min[1]) / dx[1];
        v = (C[2] - box_min[2]) / dx[2];
    } else if (final_coord == 1) {
        N = Vector3f(0, -1, 0);
        u = (C[0] - box_min[0]) / dx[0];
        v = (C[2] - box_min[2]) / dx[2];
    } else if (final_coord == 2) {
        N = Vector3f(0, 0, -1);
        u = (C[0] - box_min[0]) / dx[0];
        v = (box_max[1] - C[1]) / dx[1];
    } else if (final_coord == 3) {
        N = Vector3f(1, 0, 0);
        u = (box_max[1] - C[1]) / dx[1];
        v = (box_max[2] - C[2]) / dx[2];
    } else if (final_coord == 4) {
        N = Vector3f(0, 1, 0);
        u = (C[0] - box_min[0]) / dx[0];
        v = (box_max[2] - C[2]) / dx[2];
    } else if (final_coord == 5) {
        N = Vector3f(0, 0, 1);
        u = (C[0] - box_min[0]) / dx[0];
        v = (C[1] - box_min[1]) / dx[1];
    }
    if (Vector3f::dot(r.getDirection(), N) < 0) {
        h = ObjectHit(Dist, this, N, C, r.getDirection(), u, v, true);
    } else {
        h = ObjectHit(Dist, this, -N, C, r.getDirection(), u, v, false);
    }
    return true;
}
void Box::Input(std::stringstream& fin)
{
    std::string var;
    fin >> var;
    if (var == "box_min")
        box_min.Input(fin);
    else if (var == "box_max")
        box_max.Input(fin);
}
void Box::Initialize()
{
    dx = box_max - box_min;
}
bool Bezier::intersect(const Ray& r, ObjectHit& h)
{
    double final_dis = INF;
    if (fabs(r.getDirection()[2]) < 5e-4) {
        double dis_to_axis = (Vector3f(0, 0, r.getOrigin()[2] - center[2]) + center - r.getOrigin()).length();
        double hit = r.pointAtParameter(dis_to_axis)[2];
        if (hit < center[2] + EPS || hit > center[2] + height - EPS)
            return false;
        double t = solve_1(hit - center[2]);
        if (t < 0 || t > 1)
            return false;
        Vector2f loc = getPos(t);
        double ft = center[2] + loc[1] - hit;
        if (fabs(ft) > EPS)
            return false;
        final_dis = sphere_intersect(r, center + Vector3f(0, 0, loc[1]), loc[0]);
        if (final_dis < 0)
            return false;
        Vector3f inter_p = r.pointAtParameter(final_dis);
        if (fabs((inter_p - Vector3f(center[0], center[1], inter_p[2])).length2() - loc[0] * loc[0]) > 1e-1)
            return false;
        // second iteration, more accuracy
        hit = inter_p[2];
        if (hit < center[2] + EPS || hit > center[2] + height - EPS)
            return false;
        t = solve_1(hit - center[2]);
        loc = getPos(t);
        ft = center[2] + loc[1] - hit;
        if (fabs(ft) > EPS)
            return false;
        final_dis = sphere_intersect(r, center + Vector3f(0, 0, loc[1]), loc[0]);
        if (final_dis < 0)
            return false;
        inter_p = r.pointAtParameter(final_dis);
        if (fabs((inter_p - Vector3f(center[0], center[1], inter_p[2])).length2() - loc[0] * loc[0]) > 1e-2)
            return false;
    }
    double a = 0, b = 0, c = 0, t1, t2;
    t1 = r.getOrigin()[0] - center[0] - r.getDirection()[0] / r.getDirection()[2] * r.getOrigin()[2];
    t2 = r.getDirection()[0] / r.getDirection()[2];
    a += t2 * t2;
    b += 2 * t1 * t2;
    c += t1 * t1;
    t1 = r.getOrigin()[1] - center[1] - r.getDirection()[1] / r.getDirection()[2] * r.getOrigin()[2];
    t2 = r.getDirection()[1] / r.getDirection()[2];
    a += t2 * t2;
    b += 2 * t1 * t2;
    c += t1 * t1;
    c = c - b * b / 4 / a;
    b = -b / 2 / a - center[2];
    if (0 <= b && b <= height && c > width2 || (b < 0 || b > height) && std::min(b * b, (height - b) * (height - b)) * a + c > width2) // no intersect
        return false;
    for (int ind = 0; ind < num; ind++) {
        double t0 = segments[ind].t0, t1 = segments[ind].t1;
        {
            solve_2(t0, t1, (t0 + t1 + t0) / 3, r, a, b, c, final_dis);
            solve_2(t0, t1, (t1 + t0 + t1) / 3, r, a, b, c, final_dis);
        }
    }
    if (final_dis > INF / 2)
        return false;
    Vector3f C = r.pointAtParameter(final_dis);
    double v = solve_1(C[2] - center[2]);
    double u = atan2(C[1] - center[1], C[0] - center[0]);
    if (u < 0)
        u += 2 * PI;
    Vector2f dir = getDir(v);
    Vector3f d_surface = Vector3f(cos(u), sin(u), dir[1] / dir[0]);
    Vector3f d_circ = Vector3f(-sin(u), cos(u), 0);
    Vector3f N = Vector3f::cross(d_circ, d_surface);
    if (Vector3f::dot(r.getDirection(), N) < 0) {
        h = ObjectHit(final_dis, this, N, C, r.getDirection(), u / 2 / PI, v, true);
    } else {
        h = ObjectHit(final_dis, this, -N, C, r.getDirection(), u / 2 / PI, v, false);
    }
    return true;
}
void Bezier::Input(std::stringstream& fin)
{
    std::string var;
    fin >> var;
    if (var == "center")
        center.Input(fin);
    else if (var == "bezierfile")
        fin >> bezierfile;
}
void Bezier::Initialize()
{
    std::ifstream fin(bezierfile);
    if (!fin.is_open()) {
        printf("cannot open bezier file\n");
        exit(0);
    }
    fin >> num;
    double* px = new double[num];
    double* py = new double[num];
    x_coefficient = new double[num];
    y_coefficient = new double[num];
    segments = new Segment[num];
    for (int i = 0; i < num; i++) {
        fin >> px[i] >> py[i];
    }
    assert(fabs(py[0]) < EPS);
    printf("Preprocessing Bezier...\n");
    for (int i = 0; i < num; i++) {
        x_coefficient[i] = px[0];
        y_coefficient[i] = py[0];
        for (int j = 0; j < num - i; j++) {
            px[j] = px[j + 1] - px[j];
            py[j] = py[j + 1] - py[j];
        }
    }
    double down = 1, up = 1, nxt = num - 1;
    for (int i = 0; i < num; i++, nxt--) {
        up = up * (i == 0 ? 1 : i);
        x_coefficient[i] = x_coefficient[i] * down / up;
        y_coefficient[i] = y_coefficient[i] * down / up;
        down *= nxt;
    }
    width = 0;
    double interval = 1. / (num - 1), c = 0;
    for (int i = 0; i < num; i++, c += interval) {
        segments[i].width = 0;
        segments[i].t0 = std::max(0., c - r);
        segments[i].t1 = std::min(1., c + r);
        segments[i].y0 = getPos(segments[i].t0)[1];
        segments[i].y1 = getPos(segments[i].t1)[1];
        for (double t = segments[i].t0; t <= segments[i].t1; t += 0.00001) {
            Vector2f pos = getPos(t);
            if (segments[i].width < pos[0])
                segments[i].width = pos[0];
        }
        if (width < segments[i].width)
            width = segments[i].width;
        segments[i].width += EPS;
        segments[i].width2 = segments[i].width * segments[i].width;
    }
    width += EPS;
    width2 = width * width;
    height = getPos(1)[1];
    delete[] px;
    delete[] py;
    fin.close();
}
Bezier::~Bezier()
{
    delete[] x_coefficient;
    delete[] y_coefficient;
    delete[] segments;
}
Vector2f Bezier::getPos(double t)
{
    double ans_x = 0, ans_y = 0, t_pow = 1;
    for (int i = 0; i < num; i++) {
        ans_x += x_coefficient[i] * t_pow;
        ans_y += y_coefficient[i] * t_pow;
        t_pow *= t;
    }
    return Vector2f(ans_x, ans_y);
}
Vector2f Bezier::getDir(double t)
{
    double ans_x = 0, ans_y = 0, t_pow = 1;
    for (int i = 1; i < num; i++) {
        ans_x += x_coefficient[i] * i * t_pow;
        ans_y += y_coefficient[i] * i * t_pow;
        t_pow *= t;
    }
    return Vector2f(ans_x, ans_y);
}
double Bezier::sphere_intersect(const Ray& ray, const Vector3f& o, double radius)
{
    Vector3f ro = o - ray.getOrigin();
    double b = Vector3f::dot(ro, ray.getDirection());
    double d = b * b - ro.length2() + radius * radius;
    if (d < 0)
        return -1;
    else
        d = sqrt(d);
    double t = b - d > EPS ? b - d : b + d > EPS ? b + d : -1;
    return t;
}
double Bezier::solve_1(double z0)
{
    double t = .5, ft, dft;
    for (int i = 10; i--;) {
        if (t < 0)
            t = 0;
        else if (t > 1)
            t = 1;
        ft = getPos(t)[1] - z0, dft = getDir(t)[1];
        if (fabs(ft) < EPS)
            return t;
        t -= ft / dft;
    }
    return -1;
}
bool Bezier::solve_2(double low, double high, double init, const Ray& ray, double a, double b, double c, double& final_dis)
{
    double t = newton(init, a, b, c, low, high);
    if (t <= 0 || t >= 1)
        return false;
    Vector2f loc = getPos(t);
    double x = loc[0], y = loc[1];
    double ft = x - sqrt(a * (y - b) * (y - b) + c);
    if (fabs(ft) > EPS)
        return false;
    double dis = (center[2] + y - ray.getOrigin()[2]) / ray.getDirection()[2];
    if (dis < EPS)
        return false;
    Vector3f inter_p = ray.pointAtParameter(dis);
    if (fabs((Vector3f(0, 0, y) + center - inter_p).length2() - x * x) > EPS)
        return false;
    if (dis < final_dis) {
        final_dis = dis;
        return true;
    }
    return false;
}
double Bezier::getft(double t, double a, double b, double c)
{
    if (t < 0)
        t = EPS;
    if (t > 1)
        t = 1 - EPS;
    Vector2f loc = getPos(t);
    double x = loc[0], y = loc[1];
    return x - sqrt(a * (y - b) * (y - b) + c);
}
double Bezier::newton(double t, double a, double b, double c, double low, double high)
{
    double ft, dft, x, y, dx, dy, sq;
    Vector2f loc, dir;
    for (int i = 10; i--;) {
        if (t < 0)
            t = low;
        if (t > 1)
            t = high;
        loc = getPos(t), dir = getDir(t);
        x = loc[0], dx = dir[0];
        y = loc[1], dy = dir[1];
        sq = sqrt(a * (y - b) * (y - b) + c);
        ft = x - sq;
        dft = dx - a * (y - b) * dy / sq;
        if (fabs(ft) < EPS)
            return t;
        t -= ft / dft;
    }
    return -1;
}