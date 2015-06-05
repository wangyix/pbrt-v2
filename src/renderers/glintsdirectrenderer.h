
#if defined(_MSC_VER)
#pragma once
#endif

#ifndef PBRT_RENDERERS_GLINTSDIRECTRENDERER_H
#define PBRT_RENDERERS_GLINTSDIRECTRENDERER_H

// renderers/samplerrenderer.h*
#include "pbrt.h"
#include "renderer.h"
#include "parallel.h"
#include "../integrators/glintsdirectlighting.h"
#include "../samplers/pixelcenters.h"

// GlintsDirectRenderer Declarations
class GlintsDirectRenderer : public Renderer {
public:
    // GlintsDirectRenderer Public Methods
    GlintsDirectRenderer(PixelCentersSampler *s, Camera *c, GlintsDirectLightingIntegrator* integrator,
        VolumeIntegrator *vi);
    ~GlintsDirectRenderer();

    void Render(const Scene *scene);

    Spectrum Li(const Scene *scene, const RayDifferential &ray,
        const Sample *sample, RNG &rng, MemoryArena &arena,
        Intersection *isect = NULL, Spectrum *T = NULL) const;

    Spectrum Transmittance(const Scene *scene, const RayDifferential &ray,
        const Sample *sample, RNG &rng, MemoryArena &arena) const;
private:
    PixelCentersSampler* sampler;
    Camera* camera;
    GlintsDirectLightingIntegrator* glintsDirectIntegrator;
    VolumeIntegrator* volumeIntegrator;
};



// GlintsDirectRendererTask Declarations
class GlintsDirectRendererTask : public Task {
public:
    // GlintsDirectRendererTask Public Methods
    GlintsDirectRendererTask(const Scene *sc, Renderer *ren, Camera *c,
        ProgressReporter &pr, Sampler *ms, Sample *sam,
        int tn, int tc)
        : reporter(pr)
    {
        scene = sc; renderer = ren; camera = c; mainSampler = ms;
        origSample = sam; taskNum = tn; taskCount = tc;
    }
    void Run();
private:
    // GlintsDirectRendererTask Private Data
    const Scene *scene;
    const Renderer *renderer;
    Camera *camera;
    Sampler *mainSampler;
    ProgressReporter &reporter;
    Sample *origSample;
    int taskNum, taskCount;
};



#endif // PBRT_RENDERERS_GLINTSDIRECTRENDERER_H
