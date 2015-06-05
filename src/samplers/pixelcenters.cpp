#include "stdafx.h"
#include "samplers/pixelcenters.h"
#include "montecarlo.h"
#include "camera.h"

PixelCentersSampler::PixelCentersSampler(int xstart, int xend,
    int ystart, int yend, float sopen, float sclose)
    : Sampler(xstart, xend, ystart, yend, 1, sopen, sclose) {
    xPos = xPixelStart;
    yPos = yPixelStart;
    // Get storage for a pixel's worth of stratified samples
    lensSamples = AllocAligned<float>(3);
    timeSamples = lensSamples + 2;

    RNG rng(xstart + ystart * (xend - xstart));
    for (int i = 0; i < 3; ++i)
        lensSamples[i] = rng.RandomFloat();

    samplePos = 0;
}



Sampler *PixelCentersSampler::GetSubSampler(int num, int count) {
    int x0, x1, y0, y1;
    ComputeSubWindow(num, count, &x0, &x1, &y0, &y1);
    if (x0 == x1 || y0 == y1) return NULL;
    return new PixelCentersSampler(x0, x1, y0, y1, shutterOpen, shutterClose);
}



int PixelCentersSampler::GetMoreSamples(Sample *sample, RNG &rng) {
    if (samplePos == 1) {
        if (xPixelStart == xPixelEnd || yPixelStart == yPixelEnd)
            return 0;
        if (++xPos == xPixelEnd) {
            xPos = xPixelStart;
            ++yPos;
        }
        if (yPos == yPixelEnd)
            return 0;

        for (int i = 0; i < 3; ++i)
            lensSamples[i] = rng.RandomFloat();
        
        samplePos = 0;
    }
    // Return next \mono{PixelCentersSampler} sample point
    sample->imageX = xPos + 0.5f;
    sample->imageY = yPos + 0.5f;
    sample->lensU = lensSamples[0];
    sample->lensV = lensSamples[1];
    sample->time = Lerp(timeSamples[samplePos], shutterOpen, shutterClose);
    // Generate stratified samples for integrators
    for (uint32_t i = 0; i < sample->n1D.size(); ++i)
        for (uint32_t j = 0; j < sample->n1D[i]; ++j)
            sample->oneD[i][j] = rng.RandomFloat();
    for (uint32_t i = 0; i < sample->n2D.size(); ++i)
        for (uint32_t j = 0; j < 2 * sample->n2D[i]; ++j)
            sample->twoD[i][j] = rng.RandomFloat();
    ++samplePos;
    return 1;
}



PixelCentersSampler *CreatePixelCentersSampler(const ParamSet &params,
    const Film *film, const Camera *camera) {
    int xstart, xend, ystart, yend;
    film->GetSampleExtent(&xstart, &xend, &ystart, &yend);
    return new PixelCentersSampler(xstart, xend, ystart, yend,
        camera->shutterOpen, camera->shutterClose);
}


