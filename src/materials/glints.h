#if defined(_MSC_VER)
#pragma once
#endif

#ifndef PBRT_MATERIALS_GLINTS_H
#define PBRT_MATERIALS_GLINTS_H

// materials/glints.h*
#include "pbrt.h"
#include "material.h"
#include "spectrum.h"
#include "../textures/glintsnormalmap.h"

class GlintsMaterial : public Material {
public:
    GlintsMaterial(Reference<GlintsNormalTexture> normal,
        Reference<Texture<Spectrum> > eta,
        Reference<Texture<Spectrum> > k,
        Reference<Texture<float> > rough,
        Reference<Texture<float> > approxRough,
        Reference<Texture<Spectrum> > kd,
        Reference<Texture<Spectrum> > kr,
        Reference<Texture<float> > kr_eta);

    BSDF *GetBSDF(const DifferentialGeometry &dgGeom,
        const DifferentialGeometry &dgShading, MemoryArena &arena) const;

private:
    Reference<GlintsNormalTexture> normalMap;
    Reference<Texture<Spectrum> > eta, k;
    Reference<Texture<float> > roughness;
    Reference<Texture<float> > approxRoughness;

    Reference<Texture<Spectrum> > Kd, Kr;
    Reference<Texture<float> > Kr_eta;
};


GlintsMaterial *CreateGlintsMaterial(const Transform &xform,
    const TextureParams &mp);

#endif // PBRT_MATERIALS_GLINTS_H