#include "glintsrenderer.h"
#include "camera.h"
#include "film.h"
#include "spectrum.h"
#include "volume.h"
#include "integrator.h"

#include "../samplers/random.h"

// GlintsRenderer Method Definitions
GlintsRenderer::GlintsRenderer(Sampler* pathS, Camera* c,
    VolumeIntegrator* directVi, VolumeIntegrator* pathVi, const ParamSet& rendererParams) {
    
    camera = c;
    
    disableDirectRenderer = rendererParams.FindOneBool("disableDirectRenderer", false);
    disablePathRenderer = rendererParams.FindOneBool("disablePathRenderer", false);

    // direct lighting renderer will splat its samples.
    int directSamplesPerPixel = rendererParams.FindOneInt("directSamplesPerPixel", 1);
    directSampler = CreatePixelCentersSampler(directSamplesPerPixel, camera->film, camera);
    if (!directSampler) Severe("Unable to create pixel centers sampler.");

    directSurfIntegrator = CreateGlintsDirectLightingIntegrator(rendererParams);
    if (!directSurfIntegrator) Severe("Unable to create glints direct lighting integrator.");

    directVolIntegrator = directVi;

    directRenderer = new GlintsPassRenderer(directSampler, camera, directSurfIntegrator, directVolIntegrator,
        true, 1.0f / directSampler->samplesPerPixel, "Rendering glints direct lighting");


    pathSampler = pathS;

    pathSurfIntegrator = CreateGlintsPathSurfaceIntegrator(rendererParams, camera->film, pathSampler);
    if (!pathSurfIntegrator) Severe("Unable to create glints path integrator.");

    pathVolIntegrator = pathVi;

    pathRenderer = new GlintsPassRenderer(pathSampler, camera, pathSurfIntegrator, pathVolIntegrator,
        false, 1.0f, "Rendering all other paths");
}

GlintsRenderer::~GlintsRenderer() {
    delete camera;
    delete directSampler;
    delete directSurfIntegrator;
    delete directVolIntegrator;
    delete pathSampler;
    delete pathSurfIntegrator;
    delete pathVolIntegrator;

    delete directRenderer;
    delete pathRenderer;
}

void GlintsRenderer::Render(const Scene *scene) {
    if (!disableDirectRenderer) directRenderer->Render(scene);
    if (!disablePathRenderer) pathRenderer->Render(scene);
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
