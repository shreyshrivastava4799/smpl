#ifndef SMPL_COLORS_H
#define SMPL_COLORS_H

// standard includes
#include <stdint.h>

// project includes
#include <smpl/debug/marker.h>

namespace smpl {
namespace visual {

auto MakeColorHSV(float h, float s = 1.0f, float v = 1.0f, float a = 1.0f) -> Color;
auto MakeColorHexARGB(uint32_t value) -> Color;
auto MakeColorHexRGBA(uint32_t value) -> Color;

void hsv_to_rgb(float* r, float* g, float* b, float h, float s, float v);
void rgb_to_hsv(float* h, float* s, float* v, float r, float g, float b);

void hsv_to_rgb(double* r, double* g, double* b, double h, double s, double v);
void rgb_to_hsv(double* h, double* s, double* v, double r, double g, double b);

} // namespace visual
} // namespace smpl

#endif
