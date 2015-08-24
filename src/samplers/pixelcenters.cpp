#include "stdafx.h"
#include "samplers/pixelcenters.h"
#include "montecarlo.h"
#include "camera.h"

PixelCentersSampler::PixelCentersSampler(int xstart, int xend,
    int ystart, int yend, int subpixelsPerDim, float sopen, float sclose)
    : Sampler(xstart, xend, ystart, yend, subpixelsPerDim*subpixelsPerDim, sopen, sclose) {
    xPos = xPixelStart;
    yPos = yPixelStart;
    nSubpixelsPerDim = subpixelsPerDim;
    nSamples = subpixelsPerDim * subpixelsPerDim;
    
    // Get storage for a pixel's worth of stratified samples
    imageSamples = AllocAligned<float>(5 * nSamples);
    lensSamples = imageSamples + 2 * nSamples;
    timeSamples = lensSamples + 2 * nSamples;

    // Generate random values for lens samples and time samples
    RNG rng(xstart + ystart * (xend - xstart));
    for (int i = 0; i < 3 * nSamples; ++i)
        lensSamples[i] = rng.RandomFloat();

    // Generate subpixel centers for image samples
    int k = 0;
    for (int i = 0; i < nSubpixelsPerDim; ++i) {
        for (int j = 0; j < nSubpixelsPerDim; ++j) {
            imageSamples[k] = xPos + (j + 0.5f) / nSubpixelsPerDim;
            imageSamples[k + 1] = yPos + (i + 0.5f) / nSubpixelsPerDim;
            k += 2;
        }
    }

    samplePos = 0;
}



Sampler *PixelCentersSampler::GetSubSampler(int num, int count) {
    int x0, x1, y0, y1;
    ComputeSubWindow(num, count, &x0, &x1, &y0, &y1);
    if (x0 == x1 || y0 == y1) return NULL;
    return new PixelCentersSampler(x0, x1, y0, y1, nSubpixelsPerDim, shutterOpen, shutterClose);
}



int PixelCentersSampler::GetMoreSamples(Sample *sample, RNG &rng) {
    if (samplePos == nSamples) {
        if (xPixelStart == xPixelEnd || yPixelStart == yPixelEnd)
            return 0;
        if (++xPos == xPixelEnd) {
            xPos = xPixelStart;
            ++yPos;
        }
        if (yPos == yPixelEnd)
            return 0;

        // Generate random values for lens samples and time samples
        for (int i = 0; i < 3 * nSamples; ++i)
            lensSamples[i] = rng.RandomFloat();

        // Generate subpixel centers for image samples
        int k = 0;
        for (int i = 0; i < nSubpixelsPerDim; ++i) {
            for (int j = 0; j < nSubpixelsPerDim; ++j) {
                imageSamples[k] = xPos + (j + 0.5f) / nSubpixelsPerDim;
                imageSamples[k + 1] = yPos + (i + 0.5f) / nSubpixelsPerDim;
                k += 2;
            }
        }
        
        samplePos = 0;
    }
    // Return next \mono{PixelCentersSampler} sample point
    sample->imageX = imageSamples[2*samplePos + 0];
    sample->imageY = imageSamples[2*samplePos + 1];
    sample->lensU = lensSamples[2*samplePos + 0];
    sample->lensV = lensSamples[2*samplePos + 1];
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



PixelCentersSampler *CreatePixelCentersSampler(int subpixelsPerDim,
                                               const Film *film, const Camera *camera) {
    int xstart, xend, ystart, yend;
    film->GetSampleExtent(&xstart, &xend, &ystart, &yend);
    return new PixelCentersSampler(xstart, xend, ystart, yend, subpixelsPerDim,
        camera->shutterOpen, camera->shutterClose);
}


