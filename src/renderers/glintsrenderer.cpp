#include "glintsrenderer.h"
#include "camera.h"
#include "film.h"
#include "spectrum.h"
#include "volume.h"
#include "integrator.h"

#include "../samplers/random.h"

// GlintsRenderer Method Definitions
GlintsRenderer::GlintsRenderer(Sampler* pathS, Camera* c,
    VolumeIntegrator* directVi, VolumeIntegrator* pathVi, const ParamSet& siParams) {
    
    camera = c;
    

    // direct lighting renderer will splat its samples.  This works since we're only using
    // one sample per pixel, which means this sample contribution does not need to be weighted
    // and can simply be added to the film pixel.
    directSampler = CreatePixelCentersSampler(camera->film, camera);
    if (!directSampler) Severe("Unable to create pixel centers sampler.");

    directSurfIntegrator = CreateGlintsDirectLightingIntegrator(siParams);
    if (!directSurfIntegrator) Severe("Unable to create glints direct lighting integrator.");

    directVolIntegrator = directVi;

    firstRenderer = new GlintsPassRenderer(directSampler, camera, directSurfIntegrator, directVolIntegrator,
        true, "Rendering glints direct lighting");


    pathSampler = pathS;

    pathSurfIntegrator = CreateGlintsPathSurfaceIntegrator(siParams, camera->film, pathSampler);
    if (!pathSurfIntegrator) Severe("Unable to create glints path integrator.");

    pathVolIntegrator = pathVi;

    secondRenderer = new GlintsPassRenderer(pathSampler, camera, pathSurfIntegrator, pathVolIntegrator,
        false, "Rendering all other paths");
}

GlintsRenderer::~GlintsRenderer() {
    delete camera;
    delete directSampler;
    delete directSurfIntegrator;
    delete directVolIntegrator;
    delete pathSampler;
    delete pathSurfIntegrator;
    delete pathVolIntegrator;

    delete firstRenderer;
    delete secondRenderer;
}

void GlintsRenderer::Render(const Scene *scene) {
    firstRenderer->Render(scene);
    secondRenderer->Render(scene);
    camera->film->WriteImage();
}

Spectrum GlintsRenderer::Li(const Scene *scene,
    const RayDifferential &ray, const Sample *sample, RNG &rng,
    MemoryArena &arena, Intersection *isect, Spectrum *T) const {
    assert(false);
    return Spectrum();
}

Spectrum GlintsRenderer::Transmittance(const Scene *scene,
    const RayDifferential &ray, const Sample *sample, RNG &rng,
    MemoryArena &arena) const {
    assert(false);
    return Spectrum();
}
