
/*
pbrt source code Copyright(c) 1998-2012 Matt Pharr and Greg Humphreys.

This file is part of pbrt.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:

- Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.

- Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#if defined(_MSC_VER)
#pragma once
#endif

#ifndef PBRT_INTEGRATORS_GLINTSPATH_H
#define PBRT_INTEGRATORS_GLINTSPATH_H

#include "pbrt.h"
#include "integrator.h"


class GlintsPathIntegrator : public SurfaceIntegrator {
public:
    GlintsPathIntegrator(int subpixelsPerDim, int md,
        int xResolution, int yResolution, int pixelSamples, bool useApproxFirstBounce)
            : filmXResolution(xResolution),
            filmYResolution(yResolution),
            nSubpixelsPerDim(subpixelsPerDim),
            footprintScale(sqrtf(pixelSamples) / subpixelsPerDim),
            useApproxOnFirstBounce(useApproxFirstBounce) {
        maxDepth = md;
    }
    Spectrum Li(const Scene *scene, const Renderer *renderer,
        const RayDifferential &ray, const Intersection &isect,
        const Sample *sample, RNG &rng, MemoryArena &arena) const;
    void RequestSamples(Sampler *sampler, Sample *sample, const Scene *scene);
private:
    int maxDepth;
#define SAMPLE_DEPTH 3
    LightSampleOffsets lightSampleOffsets[SAMPLE_DEPTH];
    int lightNumOffset[SAMPLE_DEPTH];
    BSDFSampleOffsets bsdfSampleOffsets[SAMPLE_DEPTH];
    BSDFSampleOffsets pathSampleOffsets[SAMPLE_DEPTH];

    const int filmXResolution, filmYResolution;
    const int nSubpixelsPerDim;
    // footprintScale is sqrt(pixelSamples) / subpixelsPerDim.  The sqrt(pixelSamples) is to
    // undo the raydifferential scaling that GlintsPassRendererTask::Run() does, giving us a
    // pixel-sized footprint.  The subpixelsPerDim is to scale the pixel-sized footprint
    // to a subpixel-sized footprint.
    const float footprintScale;

    const bool useApproxOnFirstBounce;
};


GlintsPathIntegrator *CreateGlintsPathSurfaceIntegrator(int subpixelsPerDim,
    const ParamSet &params, Film* film, Sampler* sampler);

#endif // PBRT_INTEGRATORS_GLINTSPATH_H
