#ifndef PHOTON_H
#define PHOTON_H

#include <vecmath.h>
class Photon {
public:
    Vector3f pos, dir;
    int d;
    Vector3f power;
    int step;
};
#endif