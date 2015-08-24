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
    int subpixelsPerPixel = rendererParams.FindOneInt("subpixelsPerPixel", 1);

    // make sure subpixels per pixel is a square number
    int subpixelsPerDim = Round2Int(sqrtf(subpixelsPerPixel));
    if (subpixelsPerDim * subpixelsPerDim != subpixelsPerPixel) {
        printf("Number of glint subpixels per pixel needs to be a square number!\n");
        exit(1);
    }

    // direct lighting renderer will splat its samples.
    directSampler = CreatePixelCentersSampler(subpixelsPerDim, camera->film, camera);
    if (!directSampler) Severe("Unable to create pixel centers sampler.");

    directSurfIntegrator = CreateGlintsDirectLightingIntegrator(rendererParams);
    if (!directSurfIntegrator) Severe("Unable to create glints direct lighting integrator.");

    directVolIntegrator = directVi;

    directRenderer = new GlintsPassRenderer(directSampler, camera, directSurfIntegrator, directVolIntegrator,
        true, 1.0f / directSampler->samplesPerPixel, "Rendering glints direct lighting");


    pathSampler = pathS;

    pathSurfIntegrator = CreateGlintsPathSurfaceIntegrator(subpixelsPerDim, rendererParams,
                                                           camera->film, pathSampler);
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
