
#if defined(_MSC_VER)
#pragma once
#endif

#ifndef PBRT_INTEGRATORS_GLINTSDIRECTLIGHTING_H
#define PBRT_INTEGRATORS_GLINTSDIRECTLIGHTING_H


#include "pbrt.h"
#include "integrator.h"
#include "scene.h"


class GlintsDirectLightingIntegrator : public SurfaceIntegrator {
public:
    // GlintsDirectLightingIntegrator Public Methods
    GlintsDirectLightingIntegrator(int md = 5);
    ~GlintsDirectLightingIntegrator();
    Spectrum Li(const Scene *scene, const Renderer *renderer,
        const RayDifferential &ray, const Intersection &isect,
        const Sample *sample, RNG &rng, MemoryArena &arena) const;
    void RequestSamples(Sampler *sampler, Sample *sample, const Scene *scene);
private:
    int maxDepth;
};


GlintsDirectLightingIntegrator *CreateGlintsDirectLightingIntegrator(const ParamSet &params);

#endif // PBRT_INTEGRATORS_GLINTSDIRECTLIGHTING_H
