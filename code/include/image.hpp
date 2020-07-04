#ifndef IMAGE_H
#define IMAGE_H

#include "utils.h"
#include <vecmath.h>

// Simple image class
class Image {

public:
    Image(int w, int h);
    ~Image();
    int Width() const;
    int Height() const;
    const Vector3f& GetPixel(int x, int y) const;
    Vector3f getSmoothPixel(double u, double v) const;
    void SetAllPixels(const Vector3f& color);
    void SetPixel(int x, int y, const Vector3f& color);
    static Image* LoadPPM(const char* filename);
    void SavePPM(const char* filename) const;
    static Image* LoadBMP(const char* filename, bool bump = false);
    int SaveBMP(const char* filename);
    void absorb(Image* image);

private:
    int width;
    int height;
    Vector3f* data;
};

#endif // IMAGE_H
