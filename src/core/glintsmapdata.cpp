#include "glintsmapdata.h"

GlintsMapData::GlintsMapData(const RGBSpectrum* texels, int w, int h) {
    width = w;
    height = h;
    data = new float[3 * width * height];
    memcpy(data, texels, 3 * width * height * sizeof(float));
}

GlintsMapData::~GlintsMapData() {
    if (data)
        delete[] data;
}

// evaluates D(s,t) given s,t, pixel footprint, and roughness
float GlintsMapData::D(float s, float t,
    const GlintsPixelFootprint& footprint,
    float roughness) const {

    // PLACEHOLDER: blinn distribution
    float exponent = 1.0f / roughness;
    float costhetah = sqrtf(max(1.0f - s*s - t*t, 0.0f));
    float D_w = (exponent + 2) * INV_TWOPI * powf(costhetah, exponent);
    return D_w / costhetah;    // convert to D_st
}

#include "rng.h"    // FOR THE BLINN PLACEHOLDER
#include "montecarlo.h"

void GlintsMapData::normalAt(float u, float v, float* s, float* t) const {
    // PLACEHOLDER: sample the blinn distribution
    float exponent = 10.0f;

    RNG rng;
    float u1 = rng.RandomFloat();
    float u2 = rng.RandomFloat();
    // Compute sampled half-angle vector $\wh$ for Blinn distribution
    float costheta = powf(u1, 1.f / (exponent + 1));
    float sintheta = sqrtf(max(0.f, 1.f - costheta*costheta));
    float phi = u2 * 2.f * M_PI;
    Vector wh = SphericalDirection(sintheta, costheta, phi);
    *s = wh.x;
    *t = wh.y;
}
