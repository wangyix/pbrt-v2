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
    float u, float v, float du0, float dv0, float du1, float dv1,
    float roughness) const {

    // PLACEHOLDER: blinn distribution
    float exponent = 10.0f;
    float costhetah = sqrtf(1.0f - s*s - t*t);
    return (exponent + 2) * INV_TWOPI * powf(costhetah, exponent);
}

