#if defined(_MSC_VER)
#pragma once
#endif

#ifndef PBRT_CORE_GLINTSMAPDATA_H
#define PBRT_CORE_GLINTSMAPDATA_H

#include "pbrt.h"
#include <Eigen/Dense>


struct GlintsPixelFootprint;

struct STMinMax {
    STMinMax() : smin(INFINITY), smax(-INFINITY), tmin(INFINITY), tmax(-INFINITY) {}
    void update(const Eigen::Vector2f& st) {
        float s = st(0), t = st(1);
        smin = min(smin, s);
        smax = max(smax, s);
        tmin = min(tmin, t);
        tmax = max(tmax, t);
    }
    void update(const STMinMax& st) {
        smin = min(smin, st.smin);
        smax = max(smax, st.smax);
        tmin = min(tmin, st.tmin);
        tmax = max(tmax, st.tmax);
    }
    float smin, smax;
    float tmin, tmax;
};

class GlintsMapData {
public:
    GlintsMapData() : data(NULL) {}
    GlintsMapData(const unsigned char* texels, int w, int h, int channels);
    ~GlintsMapData();

    // s,t represents normal vectors in the normal map
    // u,v represents texture coordinates here (instead of s,t)
    float D(float s, float t,
        const GlintsPixelFootprint& footprint,
        float roughness) const;

    void normalAt(float u, float v, float* s, float* t) const;


private:
    Eigen::Vector2f stAt(int x, int y) const;

    void buildStMinMaxTree();

    float triangleIntegralG(float p0, float p1, float q0, float q1,
        const Eigen::Vector2f& st_p0q0, const Eigen::Vector2f& st_p1q0, const Eigen::Vector2f& st_p0q1,
        float s, float t, float roughness, const Eigen::Vector2f& pqc, const Eigen::Matrix2f& dpq2dxy) const;


    float recursiveD(float s, float t, float stCullRadiusSq, float roughness,
        const Eigen::Vector2f& pqc, const Eigen::Matrix2f& dpq2dxy, float xyCullRadiusSq,
        int pfrom, int pto, int qfrom, int qto, int level) const;

    float* data;
    int width, height;

    vector<STMinMax*> stMinMaxTree;
};

#endif // PBRT_CORE_GLINTSMAPDATA_H