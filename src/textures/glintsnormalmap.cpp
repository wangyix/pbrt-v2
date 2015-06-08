#include "glintsnormalmap.h"
#include "imageio.h"
#include "..\core\lodepng.h"
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

    /*int width, height;
    RGBSpectrum* texels = ReadImage(filename, &width, &height);
    if (!texels) {
        printf("Unable to open glints normal map file %s!\n", filename.c_str());
        exit(1);
    }*/
    unsigned width, height;
    vector<unsigned char> image;
    unsigned error = lodepng::decode(image, width, height, filename);
    if (error) {
        printf("Unable to open glints normal map file %s!\n", filename.c_str());
        printf("Make sure it's a png file.\n");
        exit(1);
    }
    GlintsMapData* ret = new GlintsMapData(&image[0], width, height, 4);
    
    //delete[] texels;
    
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
