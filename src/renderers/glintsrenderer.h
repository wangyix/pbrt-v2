#if defined(_MSC_VER)
#pragma once
#endif

#ifndef PBRT_RENDERERS_GLINTSRENDERER_H
#define PBRT_RENDERERS_GLINTSRENDERER_H

#include "pbrt.h"
#include "renderer.h"
#include "glintspassrenderer.h"
#include "../integrators/glintsdirectlighting.h"
#include "../integrators/glintspath.h"
#include "../samplers/pixelcenters.h"

class GlintsRenderer : public Renderer {
public:
    GlintsRenderer(Sampler* pathSampler, Camera* c, VolumeIntegrator* directVi,
        VolumeIntegrator* pathVi, const ParamSet& siParams);
    ~GlintsRenderer();
    void Render(const Scene *scene);

    // These two should never be called
    Spectrum Li(const Scene *scene, const RayDifferential &ray,
        const Sample *sample, RNG &rng, MemoryArena &arena,
        Intersection *isect = NULL, Spectrum *T = NULL) const;
    Spectrum Transmittance(const Scene *scene,
        const RayDifferential &ray, const Sample *sample,
        RNG &rng, MemoryArena &arena) const;
private:
    Camera* camera;
    Sampler* directSampler;
    SurfaceIntegrator* directSurfIntegrator;
    VolumeIntegrator* directVolIntegrator;
    Sampler* pathSampler;
    SurfaceIntegrator* pathSurfIntegrator;
    VolumeIntegrator* pathVolIntegrator;
    
    GlintsPassRenderer* firstRenderer;
    GlintsPassRenderer* secondRenderer;
};

#endif // PBRT_RENDERERS_GLINTSRENDERER_H
