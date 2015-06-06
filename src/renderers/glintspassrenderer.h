
#if defined(_MSC_VER)
#pragma once
#endif

#ifndef PBRT_RENDERERS_GLINTSPASSRENDERER_H
#define PBRT_RENDERERS_GLINTSPASSRENDERER_H

// renderers/samplerrenderer.h*
#include "pbrt.h"
#include "renderer.h"
#include "parallel.h"


// GlintsPassRenderer Declarations
class GlintsPassRenderer : public Renderer {
public:
    // GlintsPassRenderer Public Methods
    GlintsPassRenderer(Sampler *s, Camera *c, SurfaceIntegrator* integrator,
        VolumeIntegrator *vi, bool splat, string prString = "Rendering");
    ~GlintsPassRenderer();

    void Render(const Scene *scene);

    Spectrum Li(const Scene *scene, const RayDifferential &ray,
        const Sample *sample, RNG &rng, MemoryArena &arena,
        Intersection *isect = NULL, Spectrum *T = NULL) const;

    Spectrum Transmittance(const Scene *scene, const RayDifferential &ray,
        const Sample *sample, RNG &rng, MemoryArena &arena) const;
private:
    Sampler* sampler;
    Camera* camera;
    SurfaceIntegrator* surfaceIntegrator;
    VolumeIntegrator* volumeIntegrator;
    const bool splatSamples;
    string reporterString;
};



// GlintsPassRendererTask Declarations
class GlintsPassRendererTask : public Task {
public:
    // GlintsPassRendererTask Public Methods
    GlintsPassRendererTask(const Scene *sc, Renderer *ren, Camera *c,
        ProgressReporter &pr, Sampler *ms, Sample *sam,
        int tn, int tc, bool splat)
        : reporter(pr), splatSamples(splat)
    {
        scene = sc; renderer = ren; camera = c; mainSampler = ms;
        origSample = sam; taskNum = tn; taskCount = tc;
    }
    void Run();
private:
    // GlintsPassRendererTask Private Data
    const Scene *scene;
    const Renderer *renderer;
    Camera *camera;
    Sampler *mainSampler;
    ProgressReporter &reporter;
    Sample *origSample;
    int taskNum, taskCount;
    const bool splatSamples;
};



#endif // PBRT_RENDERERS_GLINTSPASSRENDERER_H
