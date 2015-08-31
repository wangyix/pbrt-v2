
#include "stdafx.h"
#include "integrators/glintsdirectlighting.h"
#include "intersection.h"
#include "paramset.h"


GlintsDirectLightingIntegrator::GlintsDirectLightingIntegrator(int md) {
    maxDepth = md;
}


GlintsDirectLightingIntegrator::~GlintsDirectLightingIntegrator() {
}


void GlintsDirectLightingIntegrator::RequestSamples(Sampler *sampler,
    Sample *sample, const Scene *scene) {

}


Spectrum GlintsDirectLightingIntegrator::Li(const Scene *scene,
    const Renderer *renderer, const RayDifferential &ray,
    const Intersection &isect, const Sample *sample, RNG &rng, MemoryArena &arena) const {

    Spectrum L(0.f);

    // Evaluate BSDF at hit point
    BSDF *bsdf = isect.GetBSDF(ray, arena);
    Vector wo = -ray.d;
    const Point &p = bsdf->dgShading.p;
    const Normal &n = bsdf->dgShading.nn;
    
    // if the intersection is not a glinting material nor specular, bail
    if (bsdf->NumComponents(BxDFType(BSDF_GLINTS | BSDF_SPECULAR 
                            | BSDF_REFLECTION | BSDF_TRANSMISSION)) == 0) {
        return Spectrum(0.f);
    }

    // approximation will not be used; footprint needs no modification
    BSDF::SetGlintsMicrofacetBxDFsUseApprox(bsdf, false, 0.0f, 0.0f, 1.0f);

    // Compute direct lighting for _GlintsDirectLightingIntegrator_ integrator
    if (scene->lights.size() > 0) {
        L += UniformSampleAllPointLightsFromGlintsMaterial(
            scene, renderer, arena, p, n, wo,
            isect.rayEpsilon, ray.time, bsdf, sample, rng);
    }
    if (ray.depth + 1 < maxDepth) {
        Vector wi;
        // Trace rays for specular reflection and refraction
        L += SpecularReflect(ray, bsdf, rng, isect, renderer, scene, sample,
            arena);
        L += SpecularTransmit(ray, bsdf, rng, isect, renderer, scene, sample,
            arena);
    }
    L = L.ClampInfs(10.0f);
    return L;
}


GlintsDirectLightingIntegrator *CreateGlintsDirectLightingIntegrator(const ParamSet &params) {
    int maxDepth = params.FindOneInt("maxdepth", 5);
    return new GlintsDirectLightingIntegrator(maxDepth);
}


