#if defined(_MSC_VER)
#pragma once
#endif

#ifndef PBRT_TEXTURES_GLINTSNORMALMAP_H
#define PBRT_TEXTURES_GLINTSNORMALMAP_H

#include "pbrt.h"
#include "texture.h"
#include "paramset.h"
#include <map>

#include "glintsmapdata.h"

class GlintsNormalTexture : public Texture < Spectrum > {
    
public:
    GlintsNormalTexture(TextureMapping2D* m, const string& filename);
    ~GlintsNormalTexture();
    Spectrum Evaluate(const DifferentialGeometry &) const;     // Unused

    const GlintsMapData* getMapData() const { return mapData; }
    const TextureMapping2D* getMapping() const { return mapping; }

    static void ClearCache() {
        auto iter = textures.begin();
        while (iter != textures.end()) {
            delete iter->second;
            ++iter;
        }
        textures.clear();
    }

private:
    TextureMapping2D* mapping;
    GlintsMapData* mapData;


    // used to look up exisiting GlintsMapData objects in the cache
    static GlintsMapData* GetTexture(const string& filename);

    // cache of GlintsMapData objects
    static std::map<string, GlintsMapData*> textures;
};

GlintsNormalTexture* CreateGlintsNormalTexture(const Transform& tex2world,
    const TextureParams &tp);


#endif // PBRT_TEXTURES_GLINTSNORMALMAP_H