#ifndef PATHTRACER
#define PATHTRACER

#include <vecmath.h>

class Scene;
class Camera;
class ObjectHit;
class Ray;
class Image;
class PathTracer {
    Scene* scene;
    Camera* camera;
    Image* image;
    int W, H;
    Vector3f radiance(Ray ray, int dep, bool refracted, unsigned short* X);

public:
    PathTracer(Scene* _scene, Image* _image);
    ~PathTracer() = default;
    void Run();
};

#endif