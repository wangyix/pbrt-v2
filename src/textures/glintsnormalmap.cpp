#include "glintsnormalmap.h"
#include "imageio.h"
#include <stdio.h>

GlintsNormalTexture::GlintsNormalTexture(TextureMapping2D* m,
                                        const string& filename) {
    mapping = m;
    mapData = GetTexture(filename);
}

GlintsNormalTexture::~GlintsNormalTexture() {
    delete mapping;
}

std::map<string, GlintsMapData*> GlintsNormalTexture::textures;

// Unused
Spectrum GlintsNormalTexture::Evaluate(const DifferentialGeometry &) const {
    return Spectrum();
}



GlintsMapData* GlintsNormalTexture::GetTexture(const string& filename) {
    // look for texture in texture cache
    if (textures.find(filename) != textures.end())
        return textures[filename];

    int width, height;
    RGBSpectrum* texels = ReadImage(filename, &width, &height);
    if (!texels) {
        printf("Unable to open glints normal map file %s!\n", filename.c_str());
        exit(1);
    }
    /*if (width != height || 
        (width & (width - 1)) != 0 || (height & (height - 1)) != 0) {
        printf("Glints normal map %s is %d x %d; dimensions must be equal and a power of 2!\n",
            filename.c_str(), width, height);
        exit(1);
    }*/

    GlintsMapData* ret = new GlintsMapData(texels, width, height);
    delete[] texels;
    
    textures[filename] = ret;
    return ret;
}

GlintsNormalTexture* CreateGlintsNormalTexture(const Transform &tex2world,
    const TextureParams &tp) {
    
    // Initialize 2D texture mapping
    TextureMapping2D* map = NULL;
    string type = tp.FindString("mapping", "uv");
    if (type == "uv") {
        float su = tp.FindFloat("uscale", 1.);
        float sv = tp.FindFloat("vscale", 1.);
        float du = tp.FindFloat("udelta", 0.);
        float dv = tp.FindFloat("vdelta", 0.);
        map = new UVMapping2D(su, sv, du, dv);
    } else if (type == "spherical") map = new SphericalMapping2D(Inverse(tex2world));
    else if (type == "cylindrical") map = new CylindricalMapping2D(Inverse(tex2world));
    else if (type == "planar")
        map = new PlanarMapping2D(tp.FindVector("v1", Vector(1, 0, 0)),
        tp.FindVector("v2", Vector(0, 1, 0)),
        tp.FindFloat("udelta", 0.f), tp.FindFloat("vdelta", 0.f));
    else {
        Error("2D texture mapping \"%s\" unknown", type.c_str());
        map = new UVMapping2D;
    }

    // Initialize GlintsNormalMap parameters
    string filename = tp.FindFilename("filename");
    return new GlintsNormalTexture(map, filename);
}
