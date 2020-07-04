#include <assert.h>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "image.hpp"
#include "utils.h"

Image::Image(int w, int h)
{
    width = w;
    height = h;
    data = new Vector3f[width * height];
}
Image::~Image()
{
    delete[] data;
}
int Image::Width() const { return width; }
int Image::Height() const { return height; }
const Vector3f& Image::GetPixel(int x, int y) const
{
    assert(x >= 0 && x < width);
    assert(y >= 0 && y < height);
    return data[y * width + x];
}
Vector3f Image::getSmoothPixel(double u, double v) const
{
    double U = getFractionalPart(u + EPS) * width;
    double V = getFractionalPart(v + EPS) * height;
    int U1 = (int)floor(U + EPS), U2 = U1 + 1;
    int V1 = (int)floor(V + EPS), V2 = V1 + 1;
    double rat_U = U2 - U;
    double rat_V = V2 - V;
    if (U1 < 0)
        U1 = width - 1;
    if (U2 == width)
        U2 = 0;
    if (V1 < 0)
        V1 = height - 1;
    if (V2 == height)
        V2 = 0;
    Vector3f ret;
    ret = ret + GetPixel(U1, V1) * rat_U * rat_V;
    ret = ret + GetPixel(U1, V2) * rat_U * (1 - rat_V);
    ret = ret + GetPixel(U2, V1) * (1 - rat_U) * rat_V;
    ret = ret + GetPixel(U2, V2) * (1 - rat_U) * (1 - rat_V);
    return ret;
}
void Image::SetAllPixels(const Vector3f& color)
{
    for (int i = 0; i < width * height; ++i) {
        data[i] = color;
    }
}
void Image::SetPixel(int x, int y, const Vector3f& color)
{
    assert(x >= 0 && x < width);
    assert(y >= 0 && y < height);
    data[y * width + x] = color;
}

// some helper functions for save & load

unsigned char ReadByte(FILE* file)
{
    unsigned char b;
    int success = fread((void*)(&b), sizeof(unsigned char), 1, file);
    assert(success == 1);
    return b;
}

void WriteByte(FILE* file, unsigned char b)
{
    int success = fwrite((void*)(&b), sizeof(unsigned char), 1, file);
    assert(success == 1);
}

unsigned char ClampColorComponent(double c)
{
    if (c > 1)
        c = 1;
    else if (c < 0)
        c = 0;
    //Gamma
    int tmp;
    if (GAMMA) {
        tmp = int(std::pow(c, 1 / 2.2) * 255 + .5);
    } else {
        tmp = int(c * 255);
    }
    if (tmp < 0) {
        tmp = 0;
    }

    if (tmp > 255) {
        tmp = 255;
    }

    return (unsigned char)tmp;
}
// Save and Load PPM image files using magic number 'P6'
// and having one comment line

void Image::SavePPM(const char* filename) const
{
    assert(filename != NULL);
    // must end in .ppm
    const char* ext = &filename[strlen(filename) - 4];
    assert(!strcmp(ext, ".ppm"));
    FILE* file = fopen(filename, "w");
    // misc header information
    assert(file != NULL);
    fprintf(file, "P6\n");
    fprintf(file, "# Creator: Image::SavePPM()\n");
    fprintf(file, "%d %d\n", width, height);
    fprintf(file, "255\n");
    // the data
    // flip y so that (0,0) is bottom left corner
    for (int y = height - 1; y >= 0; y--) {
        for (int x = 0; x < width; x++) {
            Vector3f v = GetPixel(x, y);
            fputc(ClampColorComponent(v[0]), file);
            fputc(ClampColorComponent(v[1]), file);
            fputc(ClampColorComponent(v[2]), file);
        }
    }
    fclose(file);
}

Image* Image::LoadPPM(const char* filename)
{
    assert(filename != NULL);
    // must end in .ppm
    const char* ext = &filename[strlen(filename) - 4];
    assert(!strcmp(ext, ".ppm"));
    FILE* file = fopen(filename, "rb");
    // misc header information
    int width = 0;
    int height = 0;
    char tmp[100];
    fgets(tmp, 100, file);
    assert(strstr(tmp, "P6"));
    fgets(tmp, 100, file);
    assert(tmp[0] == '#');
    fgets(tmp, 100, file);
    sscanf(tmp, "%d %d", &width, &height);
    fgets(tmp, 100, file);
    assert(strstr(tmp, "255"));
    // the data
    Image* answer = new Image(width, height);
    // flip y so that (0,0) is bottom left corner
    for (int y = height - 1; y >= 0; y--) {
        for (int x = 0; x < width; x++) {
            unsigned char r, g, b;
            r = fgetc(file);
            g = fgetc(file);
            b = fgetc(file);
            Vector3f color(r / 255.f, g / 255.f, b / 255.f);
            answer->SetPixel(x, y, color);
        }
    }
    fclose(file);
    return answer;
}

/****************************************************************************
    bmp.c - read and write bmp images.
    Distributed with Xplanet.  
    Copyright (C) 2002 Hari Nair <hari@alumni.caltech.edu>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
****************************************************************************/
struct BMPHeader {
    char bfType[3]; /* "BM" */
    int bfSize; /* Size of file in bytes */
    int bfReserved; /* set to 0 */
    int bfOffBits; /* Byte offset to actual bitmap data (= 54) */
    int biSize; /* Size of BITMAPINFOHEADER, in bytes (= 40) */
    int biWidth; /* Width of image, in pixels */
    int biHeight; /* Height of images, in pixels */
    short biPlanes; /* Number of planes in target device (set to 1) */
    short biBitCount; /* Bits per pixel (24 in this case) */
    int biCompression; /* Type of compression (0 if no compression) */
    int biSizeImage; /* Image size, in bytes (0 if no compression) */
    int biXPelsPerMeter; /* Resolution in pixels/meter of display device */
    int biYPelsPerMeter; /* Resolution in pixels/meter of display device */
    int biClrUsed; /* Number of colors in the color table (if 0, use 
                             maximum allowed by biBitCount) */
    int biClrImportant; /* Number of important colors.  If 0, all colors 
                             are important */
};

int Image::SaveBMP(const char* filename)
{
    int i, j, ipos;
    int bytesPerLine;
    unsigned char* line;
    Vector3f* rgb = data;
    FILE* file;
    struct BMPHeader bmph;

    /* The length of each line must be a multiple of 4 bytes */

    bytesPerLine = (3 * (width + 1) / 4) * 4;

    strcpy(bmph.bfType, "BM");
    bmph.bfOffBits = 54;
    bmph.bfSize = bmph.bfOffBits + bytesPerLine * height;
    bmph.bfReserved = 0;
    bmph.biSize = 40;
    bmph.biWidth = width;
    bmph.biHeight = height;
    bmph.biPlanes = 1;
    bmph.biBitCount = 24;
    bmph.biCompression = 0;
    bmph.biSizeImage = bytesPerLine * height;
    bmph.biXPelsPerMeter = 0;
    bmph.biYPelsPerMeter = 0;
    bmph.biClrUsed = 0;
    bmph.biClrImportant = 0;

    file = fopen(filename, "wb");
    if (file == NULL)
        return (0);

    fwrite(&bmph.bfType, 2, 1, file);
    fwrite(&bmph.bfSize, 4, 1, file);
    fwrite(&bmph.bfReserved, 4, 1, file);
    fwrite(&bmph.bfOffBits, 4, 1, file);
    fwrite(&bmph.biSize, 4, 1, file);
    fwrite(&bmph.biWidth, 4, 1, file);
    fwrite(&bmph.biHeight, 4, 1, file);
    fwrite(&bmph.biPlanes, 2, 1, file);
    fwrite(&bmph.biBitCount, 2, 1, file);
    fwrite(&bmph.biCompression, 4, 1, file);
    fwrite(&bmph.biSizeImage, 4, 1, file);
    fwrite(&bmph.biXPelsPerMeter, 4, 1, file);
    fwrite(&bmph.biYPelsPerMeter, 4, 1, file);
    fwrite(&bmph.biClrUsed, 4, 1, file);
    fwrite(&bmph.biClrImportant, 4, 1, file);

    line = (unsigned char*)malloc(bytesPerLine);
    if (line == NULL) {
        fprintf(stderr, "Can't allocate memory for BMP file.\n");
        return (0);
    }

    for (i = 0; i < height; i++) {
        for (j = 0; j < width; j++) {
            ipos = (width * i + j);
            line[3 * j] = ClampColorComponent(rgb[ipos][2]);
            line[3 * j + 1] = ClampColorComponent(rgb[ipos][1]);
            line[3 * j + 2] = ClampColorComponent(rgb[ipos][0]);
        }
        fwrite(line, bytesPerLine, 1, file);
    }

    free(line);
    fclose(file);

    return (1);
}

struct BITMAPFILEHEADER {
    unsigned int bfSize;
    unsigned short bfReserved1;
    unsigned short bfReserved2;
    unsigned int bfOffBits;
};

struct BITMAPINFOHEADER {
    unsigned int biSize;
    unsigned int biWidth;
    unsigned int biHeight;
    unsigned short biPlanes;
    unsigned short biBitCount;
    unsigned int biCompression;
    unsigned int biSizeImage;
    unsigned int biXPelsPerMeter;
    unsigned int biYPelsPerMeter;
    unsigned int biClrUsed;
    unsigned int biClrImportant;
};

struct RGBQUAD {
    unsigned char rgbBlue;
    unsigned char rgbGreen;
    unsigned char rgbRed;
    unsigned char rgbReserved;
};
Image* Image::LoadBMP(const char* filename, bool bump)
{
    assert(filename != NULL);
    FILE* file = fopen(filename, "rb");
    if (file == nullptr) {
        printf("BMP file open failed.\n");
        exit(1);
    }

    unsigned short bfType;
    BITMAPFILEHEADER strHead;
    BITMAPINFOHEADER strInfo;
    fread(&bfType, 1, sizeof(unsigned short), file);
    if (bfType != 0x4d42) {
        printf("Not BMP format.\n");
        exit(1);
    }
    fread(&strHead, 1, sizeof(BITMAPFILEHEADER), file);
    fread(&strInfo, 1, sizeof(BITMAPINFOHEADER), file);

    RGBQUAD Pla;
    for (int i = 0; i < (int)strInfo.biClrUsed; i++) {
        fread((char*)&(Pla.rgbBlue), 1, sizeof(unsigned char), file);
        fread((char*)&(Pla.rgbGreen), 1, sizeof(unsigned char), file);
        fread((char*)&(Pla.rgbRed), 1, sizeof(unsigned char), file);
    }
    // the data
    Image* answer = new Image(strInfo.biWidth, strInfo.biHeight);

    int pitch = strInfo.biWidth % 4;
    int ipos;
    unsigned char r, g, b;
    for (int i = 0; i < strInfo.biHeight; i++) {
        for (int j = 0; j < strInfo.biWidth; j++) {
            fread(&b, 1, sizeof(unsigned char), file);
            fread(&g, 1, sizeof(unsigned char), file);
            fread(&r, 1, sizeof(unsigned char), file);
            Vector3f color = Vector3f((double)r / 255, (double)g / 255, (double)b / 255);
            if (GAMMA && !bump) {
                color[0] = std::pow(color[0], 2.2);
                color[1] = std::pow(color[1], 2.2);
                color[2] = std::pow(color[2], 2.2);
            }
            answer->SetPixel(j, i, color);
        }
        unsigned char buffer = 0;
        for (int j = 0; j < pitch; j++)
            fread(&buffer, 1, sizeof(unsigned char), file);
    }
    fclose(file);
    return answer;
}

void Image::absorb(Image* image)
{
    for (int i = 0; i < width; i++) {
        for (int j = 0; j < height; j++) {
            SetPixel(i, j, (GetPixel(i, j) + image->GetPixel(i, j)).confined());
        }
    }
}