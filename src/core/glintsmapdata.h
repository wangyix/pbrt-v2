#if defined(_MSC_VER)
#pragma once
#endif

#ifndef PBRT_TEXTURES_GLINTSMAPDATA_H
#define PBRT_TEXTURES_GLINTSMAPDATA_H

#include "pbrt.h"


struct GlintsPixelFootprint;

struct ST {
    ST() : s(0.0f), t(0.0f) {}
    ST(float ss, float tt) : s(ss), t(tt) {}
    ST(const float* d) : s(d[0]), t(d[1]) {}
    ST operator+(const ST& b) const {
        return ST(s + b.s, t + b.t);
    }
    ST operator-(const ST& b) const {
        return ST(s - b.s, t - b.t);
    }
    ST operator*(float b) const {
        return ST(s * b, t * b);
    }
    ST operator/(float b) const {
        return ST(s / b, t / b);
    }
    ST& operator+=(const ST& b) {
        s += b.s;
        t += b.t;
        return *this;
    }
    ST& operator-=(const ST& b) {
        s -= b.s;
        t -= b.t;
        return *this;
    }
    ST& operator*=(const ST& b) {
        s *= b.s;
        t *= b.t;
        return *this;
    }
    ST& operator/=(const ST& b) {
        s /= b.s;
        t /= b.t;
        return *this;
    }
    float s, t;
};
ST operator*(float a, const ST& b);


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
    ST GlintsMapData::stAt(int x, int y) const;

    float* data;
    int width, height;
};

#endif // PBRT_TEXTURES_GLINTSMAPDATA_H