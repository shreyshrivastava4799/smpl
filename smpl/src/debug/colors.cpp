#include <smpl/debug/colors.h>

namespace smpl {
namespace visual {

Color MakeColorHSV(float h, float s, float v, float a)
{
    Color c;
    hsv_to_rgb(&c.r, &c.g, &c.b, h, s, v);
    c.a = a;
    return c;
}

template <class T>
void hsv_to_rgb(T* r, T* g, T* b, T h, T s, T v)
{
    if (s == T(0)) {
        // achromatic (grey)
        *r = *g = *b = v;
        return;
    }

    h /= 60.0;      // sector 0 to 5
    auto i = floor(h);
    T f = h - i;      // factorial part of h
    T p = v * (T(1.0) - s);
    T q = v * (T(1.0) - s * f);
    T t = v * (T(1.0) - s * (T(1.0) - f));
    switch ((int)i) {
    case 0:
        *r = v;
        *g = t;
        *b = p;
        break;
    case 1:
        *r = q;
        *g = v;
        *b = p;
        break;
    case 2:
        *r = p;
        *g = v;
        *b = t;
        break;
    case 3:
        *r = p;
        *g = q;
        *b = v;
        break;
    case 4:
        *r = t;
        *g = p;
        *b = v;
        break;
    default:
        *r = v;
        *g = p;
        *b = q;
        break;
    }
}

void hsv_to_rgb(float* r, float* g, float* b, float h, float s, float v)
{
    return hsv_to_rgb<float>(r, g, b, h, s, v);
}

void rgb_to_hsv(float* h, float* s, float* v, float r, float g, float b)
{
    // TODO:
}

void hsv_to_rgb(double* r, double* g, double* b, double h, double s, double v)
{
    return hsv_to_rgb<double>(r, g, b, h, s, v);
}

void rgb_to_hsv(double* h, double* s, double* v, double r, double g, double b)
{
}

} // namespace visual
} // namespace smpl
