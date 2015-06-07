#if defined(_MSC_VER)
#pragma once
#endif

#ifndef PBRT_CORE_GLINTSMAPDATA_H
#define PBRT_CORE_GLINTSMAPDATA_H

#include "pbrt.h"


struct GlintsPixelFootprint;

struct Float2 {
    Float2() : x(0.0f), y(0.0f) {}
    Float2(float ss, float tt) : x(ss), y(tt) {}
    Float2(const float* d) : x(d[0]), y(d[1]) {}
    Float2 operator+(const Float2& b) const {
        return Float2(x + b.x, y + b.y);
    }
    Float2 operator-(const Float2& b) const {
        return Float2(x - b.x, y - b.y);
    }
    Float2 operator*(float b) const {
        return Float2(x * b, y * b);
    }
    Float2 operator/(float b) const {
        return Float2(x / b, y / b);
    }
    Float2& operator+=(const Float2& b) {
        x += b.x;
        y += b.y;
        return *this;
    }
    Float2& operator-=(const Float2& b) {
        x -= b.x;
        y -= b.y;
        return *this;
    }
    Float2& operator*=(const Float2& b) {
        x *= b.x;
        y *= b.y;
        return *this;
    }
    Float2& operator/=(const Float2& b) {
        x /= b.x;
        y /= b.y;
        return *this;
    }
    float x, y;
};
Float2 operator*(float a, const Float2& b);


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
    Float2 stAt(int x, int y) const;
    Float2 normalAt(const Float2& uv) const;

    float triangleIntegralG(float u0, float u1, float v0, float v1,
        const Float2& st_u0v0, const Float2& st_u1v0, const Float2& st_u0v1,
        float s, float t, const GlintsPixelFootprint& footprint,
        float roughness) const;

    float* data;
    int width, height;
};

#endif // PBRT_CORE_GLINTSMAPDATA_H