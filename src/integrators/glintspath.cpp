#include "stdafx.h"
#include "integrators/glintspath.h"
#include "scene.h"
#include "intersection.h"
#include "paramset.h"
#include "../core/film.h"
#include "../core/sampler.h"

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

    // We need to exclude the paths handled by GlintsDirectLightingIntegrator, which are:
    // E S* G Lp, where E=eye, S=specular, G=glints, Lp=pointlight.
    bool nonSpecularBounceOccurred = false;
    bool glintsMicrofacetDistributionUseApprox;

    // calculate offset of the pixel center from the sample
    float pixelLeft = min(Round2Int(sample->imageX), filmXResolution - 1);
    float pixelTop = min(Round2Int(sample->imageY), filmYResolution - 1);
    float dxToPixelCenter = pixelLeft + 0.5f - sample->imageX;
    float dyToPixelCenter = pixelTop + 0.5f - sample->imageY;


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

        const LightSampleOffsets* lightSampleOffset = NULL;
        const BSDFSampleOffsets* bsdfSampleOffset = NULL;
        if (bounces < SAMPLE_DEPTH) {
            lightSampleOffset = &lightSampleOffsets[bounces];
            bsdfSampleOffset = &bsdfSampleOffsets[bounces];
        }

        if (nonSpecularBounceOccurred) {
            // approximation will be used for the glints bxdfs
            L += pathThroughput *
                UniformSampleOneLightGlints(scene, renderer, arena, p, n, wo,
                isectp->rayEpsilon, ray.time, bsdf, sample, rng,
                lightNumOffset[bounces], lightSampleOffset, bsdfSampleOffset);
        } else {
            // get E S* G Lnp paths, where Lnp = non-pointlight
            // approximation will NOT be used for the glints bxdfs
            L += pathThroughput *
                UniformSampleOneNonPointLightFromGlintsOrOneLightFromNonGlintsMaterial(
                true, true,
                dxToPixelCenter, dyToPixelCenter, footprintScale,
                scene, renderer, arena, p, n, wo,
                isectp->rayEpsilon, ray.time, bsdf, sample, rng,
                lightNumOffset[bounces], lightSampleOffset, bsdfSampleOffset);
            // get E S* D L paths, where D = non-glint,(non-specular)
            L += pathThroughput *
                UniformSampleOneNonPointLightFromGlintsOrOneLightFromNonGlintsMaterial(
                false, false,
                dxToPixelCenter, dyToPixelCenter, footprintScale,
                scene, renderer, arena, p, n, wo,
                isectp->rayEpsilon, ray.time, bsdf, sample, rng,
                lightNumOffset[bounces], lightSampleOffset, bsdfSampleOffset);
        }

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

        if (!specularBounce)
            nonSpecularBounceOccurred = true;
    }
    return L;
}


GlintsPathIntegrator *CreateGlintsPathSurfaceIntegrator(const ParamSet &params,
    Film* film, Sampler* sampler) {
    int maxDepth = params.FindOneInt("maxdepth", 5);
    return new GlintsPathIntegrator(maxDepth, film->xResolution, film->yResolution,
        sampler->samplesPerPixel);
}
