#include "stdafx.h"
#include "integrators/glintspath.h"
#include "scene.h"
#include "intersection.h"
#include "paramset.h"

// GlintsPathIntegrator Method Definitions
void GlintsPathIntegrator::RequestSamples(Sampler *sampler, Sample *sample,
    const Scene *scene) {
    for (int i = 0; i < SAMPLE_DEPTH; ++i) {
        lightSampleOffsets[i] = LightSampleOffsets(1, sample);
        lightNumOffset[i] = sample->Add1D(1);
        bsdfSampleOffsets[i] = BSDFSampleOffsets(1, sample);
        pathSampleOffsets[i] = BSDFSampleOffsets(1, sample);
    }
}


Spectrum GlintsPathIntegrator::Li(const Scene *scene, const Renderer *renderer,
    const RayDifferential &r, const Intersection &isect,
    const Sample *sample, RNG &rng, MemoryArena &arena) const {
    // Declare common path integration variables
    Spectrum pathThroughput = 1., L = 0.;
    RayDifferential ray(r);
    bool specularBounce = false;
    Intersection localIsect;
    const Intersection *isectp = &isect;
    for (int bounces = 0;; ++bounces) {
        // Possibly add emitted light at path vertex
        if (bounces == 0 || specularBounce)
            L += pathThroughput * isectp->Le(-ray.d);

        // Sample illumination from lights to find path contribution
        BSDF *bsdf = isectp->GetBSDF(ray, arena);
        const Point &p = bsdf->dgShading.p;
        const Normal &n = bsdf->dgShading.nn;
        Vector wo = -ray.d;
        if (bounces < SAMPLE_DEPTH)
            L += pathThroughput *
            UniformSampleOneLight(scene, renderer, arena, p, n, wo,
            isectp->rayEpsilon, ray.time, bsdf, sample, rng,
            lightNumOffset[bounces], &lightSampleOffsets[bounces],
            &bsdfSampleOffsets[bounces]);
        else
            L += pathThroughput *
            UniformSampleOneLight(scene, renderer, arena, p, n, wo,
            isectp->rayEpsilon, ray.time, bsdf, sample, rng);

        // Sample BSDF to get new path direction

        // Get _outgoingBSDFSample_ for sampling new path direction
        BSDFSample outgoingBSDFSample;
        if (bounces < SAMPLE_DEPTH)
            outgoingBSDFSample = BSDFSample(sample, pathSampleOffsets[bounces],
            0);
        else
            outgoingBSDFSample = BSDFSample(rng);
        Vector wi;
        float pdf;
        BxDFType flags;
        Spectrum f = bsdf->Sample_f(wo, &wi, outgoingBSDFSample, &pdf,
            BSDF_ALL, &flags);
        if (f.IsBlack() || pdf == 0.)
            break;
        specularBounce = (flags & BSDF_SPECULAR) != 0;
        pathThroughput *= f * AbsDot(wi, n) / pdf;
        ray = RayDifferential(p, wi, ray, isectp->rayEpsilon);

        // Possibly terminate the path
        if (bounces > 3) {
            float continueProbability = min(.5f, pathThroughput.y());
            if (rng.RandomFloat() > continueProbability)
                break;
            pathThroughput /= continueProbability;
        }
        if (bounces == maxDepth)
            break;

        // Find next vertex of path
        if (!scene->Intersect(ray, &localIsect)) {
            if (specularBounce)
                for (uint32_t i = 0; i < scene->lights.size(); ++i)
                    L += pathThroughput * scene->lights[i]->Le(ray);
            break;
        }
        pathThroughput *= renderer->Transmittance(scene, ray, NULL, rng, arena);
        isectp = &localIsect;
    }
    return L;
}


GlintsPathIntegrator *CreateGlintsPathSurfaceIntegrator(const ParamSet &params) {
    int maxDepth = params.FindOneInt("maxdepth", 5);
    return new GlintsPathIntegrator(maxDepth);
}


