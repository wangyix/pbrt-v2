#if defined(_MSC_VER)
#pragma once
#endif

#ifndef PBRT_TEXTURES_GLINTSMAPDATA_H
#define PBRT_TEXTURES_GLINTSMAPDATA_H

#include "pbrt.h"


class GlintsMapData {
public:
    GlintsMapData() : data(NULL) {}
    GlintsMapData(const RGBSpectrum* texels, int w, int h);
    ~GlintsMapData();

    // s,t represents normal vectors in the normal map
    // u,v represents texture coordinates here (instead of s,t)
    float D(float s, float t,           // the normal for which we're evaluating D(s,t)
        float u, float v, float du0, float dv0, float du1, float dv1,  // pixelfootprint
        float roughness) const;

    void normalAt(float u, float v, Vector* n) const;

private:
    float* data;
    int width, height;
};

#endif // PBRT_TEXTURES_GLINTSMAPDATA_H