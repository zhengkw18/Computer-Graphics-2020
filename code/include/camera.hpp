#ifndef CAMERA_H
#define CAMERA_H

#include "utils.h"
#include <cmath>
#include <vecmath.h>

#include "ray.hpp"

class Camera {
public:
    Camera();
    // Generate rays for each screen-space coordinate
    Ray generateRay(double i, double j, unsigned short* X);
    bool enableDOF() const;
    ~Camera() = default;

    int getWidth() const;
    int getHeight() const;

    void Input(std::stringstream& fin);
    void Initialize();

private:
    // Extrinsic parameters
    Vector3f center;
    Vector3f direction;
    Vector3f up;
    Vector3f horizontal;
    // Intrinsic parameters
    int width;
    int height;
    double f;
    double angle;
    double focalLen;
    double aperture;
    Matrix3f rotation;
};

#endif //CAMERA_H
