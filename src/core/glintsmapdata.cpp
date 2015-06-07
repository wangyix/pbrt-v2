#include "glintsmapdata.h"
#include "glintsmath.h"
#include "reflection.h"
#include <Eigen/Dense>

Float2 operator*(float a, const Float2& b) {
    return Float2(a * b.x, a * b.y);
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

    /*// PLACEHOLDER: blinn distribution
    float exponent = 1.0f / roughness;
    float z_sq = 1.0f - s*s - t*t;
    if (z_sq <= 0.0f)
        return 0.0f;
    float costhetah = sqrtf(z_sq);
    float D_w = (exponent + 2) * INV_TWOPI * powf(costhetah, exponent);
    return D_w / costhetah;    // convert to D_st
    */

    // (x,y) represents the centers of the squares formed by neighboring 4 texel samples
    float sum = 0.0;
    for (int y = 1; y < height; y++) {
        for (int x = 1; x < width; x++) {
            // get corners of square in texcoords
            Float2 u0v0(((float)x - 0.5) / width, ((float)y - 0.5) / height);
            Float2 u1v0(((float)x + 0.5) / width, ((float)y - 0.5) / height);
            Float2 u0v1(((float)x - 0.5) / width, ((float)y + 0.5) / height);
            Float2 u1v1(((float)x + 0.5) / width, ((float)y + 0.5) / height);

            // check if any are within 5 stds of footprint center

            // get the 4 samples at the corners of the squares
            Float2 st_u0v0 = stAt(x - 1, y - 1);
            Float2 st_u1v0 = stAt(x, y - 1);
            Float2 st_u0v1 = stAt(x - 1, y);
            Float2 st_u1v1 = stAt(x, y);

            // check if any are within 5 roughness stds of (s,t)

            // upper triangle
            sum += triangleIntegralG(u0v0.x, u1v0.x, u0v0.y, u0v1.y,
                st_u0v0, st_u1v0, st_u0v1,
                s, t, footprint, roughness);
            
            // lower triangle
            sum += triangleIntegralG(u1v1.x, u0v1.x, u1v1.y, u1v0.y,
                st_u1v1, st_u0v1, st_u1v0,
                s, t, footprint, roughness);
        }
    }

    return sum;
}

//#include "rng.h"    // FOR THE BLINN PLACEHOLDER
//#include "montecarlo.h"

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

    Float2 st = normalAt(Float2(u, v));
    *s = st.x;
    *t = st.y;
}



Float2 GlintsMapData::stAt(int x, int y) const {
    return Float2(&data[3 * (y*width + x)]);
}

Float2 GlintsMapData::normalAt(const Float2& uv) const {
    Float2 st;
    normalAt(uv.x, uv.y, &st.x, &st.y);
    return st;

    // find the four texels near u,v for bilinear interpolation
    int x0 = Floor2Int(uv.x * width - 0.5f);
    int x1 = x0 + 1;
    float tx = (uv.x * width - 0.5f) - x0;
    x0 = Mod(x0, width);
    x1 = Mod(x1, width);

    int y0 = Floor2Int(uv.y * height - 0.5f);
    int y1 = y0 + 1;
    float ty = (uv.y * height - 0.5f) - y0;
    y0 = Mod(y0, height);
    y1 = Mod(y1, height);

    assert(tx >= 0.0f && tx <= 1.0f);
    assert(ty >= 0.0f && ty <= 1.0f);

    Float2 a = stAt(x0, y0);    // topleft
    Float2 b = stAt(x1, y0);    // topright
    Float2 c = stAt(x0, y1);    // bottomleft
    Float2 d = stAt(x1, y1);    // bottomright
    Float2 e = (1.0f - tx)*a + tx*b;    // top lerp
    Float2 f = (1.0f - tx)*c + tx*d;    // bottom lerp
    Float2 g = (1.0f - ty)*e + ty*f;    // final lerp

    assert(abs(g.x) <= 1.0f);
    assert(abs(g.y) <= 1.0f);

    return g;
}


#define INV_FOUR_PI_SQ 0.02533029591f

float GlintsMapData::triangleIntegralG(float u0, float u1, float v0, float v1,
    const Float2& st_u0v0, const Float2& st_u1v0, const Float2& st_u0v1,
    float s, float t, const GlintsPixelFootprint& footprint,
    float roughness) const {

    // std of the gaussian pixel reconstruction filter (in pixels)
    const float sigxy = 0.4472135955f;  // sqrt(0.2)
    const float sigr = roughness;       // std of gaussian applied to D(s,t)
    const float sigxy_sq = sigxy * sigxy;
    const float sigr_sq = sigr * sigr;

    // mu, mean of the footprint gaussian (i.e. footprint center)
    float mu1 = footprint.u;
    float mu2 = footprint.v;

    // matrix B.  B transforms (p_xy - mu_xy) to (p_uv - mu_uv)
    // covariance matrix of footprint gaussian is sigxy^2 * B * B'
    Eigen::Matrix2f B;
    B << footprint.dudx, footprint.dudy,
         footprint.dvdx, footprint.dvdy;
    float detB = B.determinant();

    // we need the elements of the inverse of B, defined as D
    Eigen::Matrix2f D = B.inverse();
    float D11 = D(0, 0);
    float D12 = D(0, 1);
    float D21 = D(1, 0);
    float D22 = D(1, 1);


    // calculate A,b of transform s = Au+b between triangle corners and the normals
    // at those corners
    /*Eigen::MatrixXf M(6, 6);
    M <<    u0, v0, 0, 0, 1, 0,
            0, 0, u0, v0, 0, 1,
            u1, v0, 0, 0, 1, 0,
            0, 0, u1, v0, 0, 1,
            u0, v1, 0, 0, 1, 0,
            0, 0, u0, v1, 0, 1;
    Eigen::VectorXf stv(6);
    stv <<  st_u0v0.x,
            st_u0v0.y,
            st_u1v0.x,
            st_u1v0.y,
            st_u0v1.x,
            st_u0v1.y;
    Eigen::VectorXf Abv = M.partialPivLu().solve(stv);
    float A11 = Abv(0);
    float A12 = Abv(1);
    float A21 = Abv(2);
    float A22 = Abv(3);
    float b1 = Abv(4);
    float b2 = Abv(5);*/

    float deltaU = u1 - u0;
    float deltaV = v1 - v0;

    Eigen::Vector2f st_u0v0_e;
    st_u0v0_e << st_u0v0.x, st_u0v0.y;
    Eigen::Vector2f st_u1v0_e;
    st_u1v0_e << st_u1v0.x, st_u1v0.y;
    Eigen::Vector2f st_u0v1_e;
    st_u0v1_e << st_u0v1.x, st_u0v1.y;

    Eigen::Matrix2f A;
    A.col(0) = (st_u1v0_e - st_u0v0_e) / deltaU;
    A.col(1) = (st_u0v1_e - st_u0v0_e) / deltaV;

    Eigen::Vector2f u0v0_e;
    u0v0_e << u0, v0;
    Eigen::Vector2f b = st_u0v0_e - A * u0v0_e;
    
    float A11 = A(0, 0);
    float A12 = A(0, 1);
    float A21 = A(1, 0);
    float A22 = A(1, 1);
    float b1 = b(0);
    float b2 = b(1);


    // calculate constant C in G(u,v) = C*exp(-quadratic(u,v))
    float C = INV_FOUR_PI_SQ / (sigr_sq * sigxy_sq * detB);

    // calculate quadratic coefficients in G(u,v) = C*exp(-quadratic(u,v))
    float D1j_dot_mu = D11*mu1 + D12*mu2;
    float D2j_dot_mu = D21*mu1 + D22*mu2;

    float cuu = 0.5 * (
        (A11*A11 + A21*A21) / sigr_sq
        + (D11*D11 + D21*D21) / sigxy_sq
        );
    float cuv = (
        (A11*A12 + A21*A22) / sigr_sq
        + (D11*D12 + D21*D22) / sigxy_sq
        );
    float cvv = 0.5 * (
        (A12*A12 + A22*A22) / sigr_sq
        + (D12*D12 + D22*D22) / sigxy_sq
        );
    float cu = (
        (A11*b1 + A21*b2) / sigr_sq + 
        (D11*D1j_dot_mu + D21*D2j_dot_mu) / sigxy_sq
        );
    float cv = (
        (A12*b1 + A22*b2) / sigr_sq +
        (D12*D1j_dot_mu + D22*D2j_dot_mu) / sigxy_sq
        );
    float cc = 0.5 * (
        (b1*b1 + b2*b2) / sigr_sq + 
        (D1j_dot_mu*D1j_dot_mu + D2j_dot_mu*D2j_dot_mu) / sigxy_sq
        );

    // calculate triangle integral of G(u,v)
    return C * integral_expquaduv_triangle(cuu, cuv, cvv, cu, cv, cc, u0, u1, v0, v1);
}
