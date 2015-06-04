
#if defined(_MSC_VER)
#pragma once
#endif

#ifndef PBRT_SAMPLERS_PIXELCENTERS_H
#define PBRT_SAMPLERS_PIXELCENTERS_H

#include "sampler.h"
#include "paramset.h"
#include "film.h"
class PixelCentersSampler : public Sampler {
public:
    PixelCentersSampler(int xstart, int xend, int ystart,
        int yend, int ns, float sopen, float sclose);
    ~PixelCentersSampler() {
        FreeAligned(imageSamples);
    }
    int MaximumSampleCount() { return 1; }
    int GetMoreSamples(Sample *sample, RNG &rng);
    int RoundSize(int sz) const { return sz; }
    Sampler *GetSubSampler(int num, int count);
private:
    // PixelCentersSampler Private Data
    int xPos, yPos, nSamples;
    float *imageSamples, *lensSamples, *timeSamples;
    int samplePos;
};


Sampler *CreatePixelCentersSampler(const ParamSet &params, const Film *film,
    const Camera *camera);

#endif // PBRT_SAMPLERS_PIXELCENTERS_H
