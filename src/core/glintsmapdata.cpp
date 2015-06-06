#include "glintsmapdata.h"
#include "reflection.h"


ST operator*(float a, const ST& b) {
    return ST(a * b.s, a * b.t);
}

GlintsMapData::GlintsMapData(const RGBSpectrum* texels, int w, int h) {
    width = w;
    height = h;
    data = new float[3 * width * height];
    memcpy(data, texels, 3 * width * height * sizeof(float));

    // normalize all normals
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            float* d = &data[3 * (i*width + j)];
            Vector n(d[0], d[1], d[2]);
            float l = n.Length();
            if (l > 0.0f) {
                n = Normalize(n);
            } else {
                n = Vector(0, 0, 1);
            }
            d[0] = n.x;
            d[1] = n.y;
            d[2] = n.z;
        }
    }
}

GlintsMapData::~GlintsMapData() {
    if (data)
        delete[] data;
}

// evaluates D(s,t) given s,t, pixel footprint, and roughness
float GlintsMapData::D(float s, float t, const GlintsPixelFootprint& footprint,
                        float roughness) const {

    assert(footprint.isValid());

    // PLACEHOLDER: blinn distribution
    float exponent = 1.0f / roughness;
    float z_sq = 1.0f - s*s - t*t;
    if (z_sq <= 0.0f)
        return 0.0f;
    float costhetah = sqrtf(z_sq);
    float D_w = (exponent + 2) * INV_TWOPI * powf(costhetah, exponent);
    return D_w / costhetah;    // convert to D_st
}

//#include "rng.h"    // FOR THE BLINN PLACEHOLDER
//#include "montecarlo.h"





ST GlintsMapData::stAt(int x, int y) const {
    return ST(&data[3 * (y*width + x)]);
}


void GlintsMapData::normalAt(float u, float v, float* s, float* t) const {
    /*// PLACEHOLDER: sample the blinn distribution
    float exponent = 1000.0f;
    static RNG rng;
    float u1 = rng.RandomFloat();
    float u2 = rng.RandomFloat();
    // Compute sampled half-angle vector $\wh$ for Blinn distribution
    float costheta = powf(u1, 1.f / (exponent + 1));
    float sintheta = sqrtf(max(0.f, 1.f - costheta*costheta));
    float phi = u2 * 2.f * M_PI;
    Vector wh = SphericalDirection(sintheta, costheta, phi);
    *s = wh.x;
    *t = wh.y;*/

    // find the four texels near u,v for bilinear interpolation
    int x0 = Floor2Int(u * width - 0.5f);
    int x1 = x0 + 1;
    float tx = (u * width - 0.5f) - x0;
    x0 = Mod(x0, width);
    x1 = Mod(x1, width);
    
    int y0 = Floor2Int(v * height - 0.5f);
    int y1 = y0 + 1;
    float ty = (v * height - 0.5f) - y0;
    y0 = Mod(y0, height);
    y1 = Mod(y1, height);

    assert(tx >= 0.0f && tx <= 1.0f);
    assert(ty >= 0.0f && ty <= 1.0f);
    
    ST a = stAt(x0, y0);    // topleft
    ST b = stAt(x1, y0);    // topright
    ST c = stAt(x0, y1);    // bottomleft
    ST d = stAt(x1, y1);    // bottomright
    ST e = (1.0f - tx)*a + tx*b;    // top lerp
    ST f = (1.0f - tx)*c + tx*d;    // bottom lerp
    ST g = (1.0f - ty)*e + ty*f;    // final lerp

    *s = g.s;
    *t = g.t;

    assert(abs(g.s) <= 1.0f);
    assert(abs(g.t) <= 1.0f);
}
