
#include "stdafx.h"
#include "integrators/glintsdirectlighting.h"
#include "intersection.h"
#include "paramset.h"


GlintsDirectLightingIntegrator::GlintsDirectLightingIntegrator(LightStrategy st, int md) {
    maxDepth = md;
    strategy = st;
    lightSampleOffsets = NULL;
    bsdfSampleOffsets = NULL;

    scenePointLightsInitialized = false;
}


GlintsDirectLightingIntegrator::~GlintsDirectLightingIntegrator() {
    delete[] lightSampleOffsets;
    delete[] bsdfSampleOffsets;
}


void GlintsDirectLightingIntegrator::RequestSamples(Sampler *sampler,
    Sample *sample, const Scene *scene) {
    if (strategy == SAMPLE_ALL_UNIFORM) {
        // Allocate and request samples for sampling all lights
        uint32_t nLights = scene->lights.size();
        lightSampleOffsets = new LightSampleOffsets[nLights];
        bsdfSampleOffsets = new BSDFSampleOffsets[nLights];
        for (uint32_t i = 0; i < nLights; ++i) {
            const Light *light = scene->lights[i];
            int nSamples = light->nSamples;
            if (sampler) nSamples = sampler->RoundSize(nSamples);
            lightSampleOffsets[i] = LightSampleOffsets(nSamples, sample);
            bsdfSampleOffsets[i] = BSDFSampleOffsets(nSamples, sample);
        }
        lightNumOffset = -1;
    } else {
        // Allocate and request samples for sampling one light
        lightSampleOffsets = new LightSampleOffsets[1];
        lightSampleOffsets[0] = LightSampleOffsets(1, sample);
        lightNumOffset = sample->Add1D(1);
        bsdfSampleOffsets = new BSDFSampleOffsets[1];
        bsdfSampleOffsets[0] = BSDFSampleOffsets(1, sample);
    }
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
    
    // if the intersection is not a glinting material, bail
    if (bsdf->NumComponents(BSDF_GLINTS_ALL) == 0) {
        return Spectrum(0.f);
    }

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
    return L;
}


GlintsDirectLightingIntegrator *CreateGlintsDirectLightingIntegrator(const ParamSet &params) {
    int maxDepth = params.FindOneInt("maxdepth", 5);
    LightStrategy strategy;
    string st = params.FindOneString("strategy", "all");
    if (st == "one") strategy = SAMPLE_ONE_UNIFORM;
    else if (st == "all") strategy = SAMPLE_ALL_UNIFORM;
    else {
        Warning("Strategy \"%s\" for direct lighting unknown. "
            "Using \"all\".", st.c_str());
        strategy = SAMPLE_ALL_UNIFORM;
    }
    return new GlintsDirectLightingIntegrator(strategy, maxDepth);
}


