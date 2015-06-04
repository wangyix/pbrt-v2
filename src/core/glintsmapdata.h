#if defined(_MSC_VER)
#pragma once
#endif

#ifndef PBRT_TEXTURES_GLINTSMAPDATA_H
#define PBRT_TEXTURES_GLINTSMAPDATA_H

#include "pbrt.h"
#include "reflection.h"

class GlintsMapData {
public:
    GlintsMapData() : data(NULL) {}
    GlintsMapData(const RGBSpectrum* texels, int w, int h);
    ~GlintsMapData();

    // s,t represents normal vectors in the normal map
    // u,v represents texture coordinates here (instead of s,t)
    float D(float s, float t,
        const GlintsPixelFootprint& footprint,
        float roughness) const;

    void normalAt(float u, float v, float* s, float* t) const;

private:
    float* data;
    int width, height;
};

#endif // PBRT_TEXTURES_GLINTSMAPDATA_H