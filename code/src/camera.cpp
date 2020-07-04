#include "camera.hpp"
#include "ray.hpp"

Camera::Camera()
{
    aperture = 0;
    focalLen = 1;
}
Ray Camera::generateRay(double i, double j, unsigned short* X)
{
    if (enableDOF()) {
        Vector3f focalpoint = center + rotation * Vector3f((i - (width >> 1)) / f, (j - (height >> 1)) / f, 1) * focalLen;
        double x, y;
        do {
            x = erand48(X) * 2 - 1;
            y = erand48(X) * 2 - 1;
        } while (x * x + y * y > 1);
        Vector3f dof_O = center + horizontal * aperture * x + up * aperture * y;
        return Ray(dof_O, focalpoint - dof_O);
    } else {
        Vector3f direction = rotation * Vector3f((i - (width >> 1)) / f, (j - (height >> 1)) / f, 1);
        return Ray(this->center + focalLen * direction, direction);
    }
}
bool Camera::enableDOF() const { return aperture > EPS; }
int Camera::getWidth() const { return width; }
int Camera::getHeight() const { return height; }

void Camera::Input(std::stringstream& fin)
{
    std::string var;
    fin >> var;
    if (var == "center")
        center.Input(fin);
    else if (var == "direction")
        direction.Input(fin);
    else if (var == "up")
        up.Input(fin);
    else if (var == "width")
        fin >> width;
    else if (var == "height")
        fin >> height;
    else if (var == "angle") {
        fin >> angle;
        angle = DegreesToRadians(angle);
    } else if (var == "focalLen")
        fin >> focalLen;
    else if (var == "aperture")
        fin >> aperture;
}
void Camera::Initialize()
{
    direction = direction.normalized();
    horizontal = Vector3f::cross(direction, up).normalized();
    up = Vector3f::cross(horizontal, direction).normalized();
    double tg = tan(angle / 2);
    f = width / 2 / tg;
    rotation = Matrix3f(horizontal, up, direction);
}