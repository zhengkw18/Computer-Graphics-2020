#include "material.hpp"
#include "hit.hpp"
#include "image.hpp"

Material::Material()
{
    reflection_refraction = diffusion = specularity = dreflRadius = 0;
    refractionIndex = 1;
    shininess = 50;
    texture = bump = nullptr;
    transparent = 0;
}
Material::~Material()
{
    if (texture != nullptr)
        delete texture;
    if (bump != nullptr)
        delete bump;
}
double Material::BRDF(const Vector3f& View, const Vector3f& N, const Vector3f& dirToLight)
{
    double ret = 0;
    Vector3f L = dirToLight.normalized();
    Vector3f V = View.normalized();
    double dot1 = Vector3f::dot(L, N);
    if (dot1 < 0)
        return ret;
    ret += dot1 * diffusion;
    Vector3f R = 2 * dot1 * N - L;
    double dot2 = Vector3f::dot(V, R);
    if (dot2 > 0)
        ret += specularity * pow(dot2, shininess);
    return ret;
}
Vector3f Material::getColor(const ObjectHit& hit) const
{
    Vector3f color = Color;
    if (texture != nullptr)
        color = color * texture->getSmoothPixel(hit.getU(), hit.getV());
    return color;
}
void Material::Input(std::stringstream& fin)
{
    std::string var;
    fin >> var;
    if (var == "Color") {
        Color.Input(fin);
        if (GAMMA) {
            Color[0] = std::pow(Color[0], 2.2);
            Color[1] = std::pow(Color[1], 2.2);
            Color[2] = std::pow(Color[2], 2.2);
        }
    } else if (var == "absorbColor") {
        absorbColor.Input(fin);
        if (GAMMA) {
            absorbColor[0] = std::pow(absorbColor[0], 2.2);
            absorbColor[1] = std::pow(absorbColor[1], 2.2);
            absorbColor[2] = std::pow(absorbColor[2], 2.2);
        }
    } else if (var == "reflection_refraction")
        fin >> reflection_refraction;
    else if (var == "transparent")
        fin >> transparent;
    else if (var == "diffusion")
        fin >> diffusion;
    else if (var == "specularity")
        fin >> specularity;
    else if (var == "refractionIndex")
        fin >> refractionIndex;
    else if (var == "dreflRadius")
        fin >> dreflRadius;
    else if (var == "shininess")
        fin >> shininess;
    else if (var == "texture") {
        std::string texturefile;
        fin >> texturefile;
        texture = Image::LoadBMP(texturefile.c_str());
    } else if (var == "bump") {
        std::string bumpfile;
        fin >> bumpfile;
        bump = Image::LoadBMP(bumpfile.c_str(), true);
    }
}