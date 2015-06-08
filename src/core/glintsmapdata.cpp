#include "glintsmapdata.h"
#include "glintsmath.h"
#include "reflection.h"


#define INV_FOUR_PI_SQ 0.02533029591f
#define sigxy 0.4472135955f     // sqrt(0.2)

using namespace Eigen;


GlintsMapData::GlintsMapData(const unsigned char* texels, int w, int h, int channels) {
    width = w;
    height = h;
    if (width != height ||
        (width & (width - 1)) != 0 || (height & (height - 1)) != 0) {
        printf("Glints normal map is %d x %d; dimensions must be equal and a power of 2!\n",
            width, height);
        exit(1);
    }

    // one pixel wide pad on right and bottom side with wrapped values
    data = new float[3 * (width + 1) * (height + 1)];

    // fill out main part of data
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            const unsigned char* u = &texels[channels * (i*width + j)];
            float s = ((float)u[0] - 128.0) / 128.0;
            float t = ((float)u[1] - 128.0) / 128.0;
            float z_sq = 1.0 - s*s - t*t;
            float z;
            if (z_sq > 0.0) {
                z = sqrt(z_sq);
            } else {
                s = 0.0;
                t = 0.0;
                z = 1.0;
            }
            float* d = &data[3 * (i*(width + 1) + j)];
            d[0] = s;
            d[1] = t;
            d[2] = z;
        }
    }
    // fill out one pixel wrap borders
    // bottom border
    for (int j = 0; j < width; j++) {
        float* c = &data[3 * (0*(width + 1) + j)];
        float* d = &data[3 * (height*(width + 1) + j)];
        d[0] = c[0];
        d[1] = c[1];
        d[2] = c[2];
    }
    // right border (and bottom right pixel)
    for (int i = 0; i <= height; i++) {
        float* c = &data[3 * (i * (width + 1) + 0)];
        float* d = &data[3 * (i * (width + 1) + width)];
        d[0] = c[0];
        d[1] = c[1];
        d[2] = c[2];
    }

    buildStMinMaxTree();
}

GlintsMapData::~GlintsMapData() {
    if (data)
        delete[] data;

    for (int i = 0; i < stMinMaxTree.size(); i++) {
        delete[] stMinMaxTree[i];
    }
    stMinMaxTree.clear();
}

// level 0 is width*height
// level 1 is (width/2)*(height/2)
// ...
void GlintsMapData::buildStMinMaxTree() {
    int level = 0;
    int dim = width;
    do {
        STMinMax* thisLevel = new STMinMax[dim*dim];
        if (level == 0) {
            for (int y = 0; y < dim; y++) {
                for (int x = 0; x < dim; x++) {
                    STMinMax* e = &thisLevel[y*dim + x];
                    e->update(stAt(x, y));
                    e->update(stAt(x + 1, y));
                    e->update(stAt(x, y + 1));
                    e->update(stAt(x + 1, y + 1));
                }
            }
        } else {
            STMinMax* levelBelow = stMinMaxTree.back();
            int dimBelow = 2 * dim;
            for (int y = 0; y < dim; y++) {
                for (int x = 0; x < dim; x++) {
                    STMinMax* e = &thisLevel[y*dim + x];
                    e->update(levelBelow[(2 * y)*dimBelow + (2 * x)]);
                    e->update(levelBelow[(2 * y)*dimBelow + (2 * x + 1)]);
                    e->update(levelBelow[(2 * y + 1)*dimBelow + (2 * x)]);
                    e->update(levelBelow[(2 * y + 1)*dimBelow + (2 * x + 1)]);
                }
            }
        }
        dim /= 2;
        stMinMaxTree.push_back(thisLevel);
        level++;
    } while (dim >= 1);
}

// integrates G over texel triangles in a sub-region of the texture
float GlintsMapData::recursiveD(float s, float t, float stCullRadiusSq,
    const Eigen::Vector2f& pqc, const Eigen::Matrix2f& pqToXy, float xyCullRadiusSq,
    int pfrom, int pto, int qfrom, int qto) const {

    int dim = pto - pfrom;
    assert(dim == qto - qfrom);
    if (dim == 1) {
        int p = pfrom;
        int q = qfrom;

    }
    return 0.0;
}




bool cullSquare(const Vector2f& c1, const Vector2f& c2,
    const Vector2f& c3, const Vector2f& c4,
    const Vector2f& center, float r) {

    float r_sq = r*r;
    bool ret = (
        (c1 - center).squaredNorm() >= r_sq
        && (c2 - center).squaredNorm() >= r_sq
        && (c3 - center).squaredNorm() >= r_sq
        && (c4 - center).squaredNorm() >= r_sq
        );
    return ret;
}

#define CULL_RADIUS_STDS 5.0

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
    

    const float xyCullRadius = CULL_RADIUS_STDS * sigxy;
    //const float xyCullRadiusSq = xyCullRadius * xyCullRadius;

    const float stCullRadius = CULL_RADIUS_STDS * roughness;
    //const float stCullRadiusSq = stCullRadius * stCullRadius;


    Vector2f st(s, t);
    /*if (cullSquare(stMin, stMax,
        Vector2f(stMin(0), stMax(1)), Vector2f(stMax(0), stMin(1)),
        st, stCullRadius)) {
        return 0.0;
    }*/


    Vector2f pqc(footprint.u * width, footprint.v * height); // footprint center
    Matrix2f B;                 // (dx,dy) to (dp,dq)
    B << footprint.dudx * width, footprint.dudy * width,
        footprint.dvdx * height, footprint.dvdy * height;
    Matrix2f D = B.inverse();   // (dp,dq) to (dx,dy)


    // (x,y) represents the centers of the squares formed by neighboring 4 texel samples
    float sum = 0.0;
    for (int y = 1; y < height; y++) {
        for (int x = 1; x < width; x++) {
            // get corners of square in texcoords
            Vector2f p0q0(((float)x - 0.5), ((float)y - 0.5));
            Vector2f p1q0(((float)x + 0.5), ((float)y - 0.5));
            Vector2f p0q1(((float)x - 0.5), ((float)y + 0.5));
            Vector2f p1q1(((float)x + 0.5), ((float)y + 0.5));

            // check if any are within 5 stds of footprint center
            if (cullSquare(D * (p0q0 - pqc), D * (p1q0 - pqc),
                D * (p0q1 - pqc), D * (p1q1 - pqc),
                Vector2f(0.0f, 0.0f), xyCullRadius)) {
                continue;
            }

            // get the 4 samples at the corners of the squares
            Vector2f st_p0q0 = stAt(x - 1, y - 1);
            Vector2f st_p1q0 = stAt(x, y - 1);
            Vector2f st_p0q1 = stAt(x - 1, y);
            Vector2f st_p1q1 = stAt(x, y);

            // check if any are within culling radius of (s,t)
            if (cullSquare(st_p0q0, st_p1q0, st_p0q1, st_p1q1, st, stCullRadius)) {
                continue;
            }

return 100.0f;

            // upper triangle
            sum += triangleIntegralG(p0q0(0), p1q0(0), p0q0(1), p0q1(1),
                st_p0q0, st_p1q0, st_p0q1,
                s, t, footprint, roughness);
            
            // lower triangle
            sum += triangleIntegralG(p1q1(0), p0q1(0), p1q1(1), p1q0(1),
                st_p1q1, st_p0q1, st_p1q0,
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

    u -= Floor2Int(u);
    v -= Floor2Int(v);
    assert(u >= 0.0f && u < 1.0f);
    assert(v >= 0.0f && v < 1.0f);

    // find the four texels near u,v for bilinear interpolation
    int x0 = Floor2Int(u * width - 0.5f);
    int x1 = x0 + 1;
    float tx = (u * width - 0.5f) - x0;

    int y0 = Floor2Int(v * height - 0.5f);
    int y1 = y0 + 1;
    float ty = (v * height - 0.5f) - y0;

    assert(tx >= 0.0f && tx <= 1.0f);
    assert(ty >= 0.0f && ty <= 1.0f);

    Vector2f a = stAt(x0, y0);    // topleft
    Vector2f b = stAt(x1, y0);    // topright
    Vector2f c = stAt(x0, y1);    // bottomleft
    Vector2f d = stAt(x1, y1);    // bottomright
    Vector2f e = (1.0f - tx)*a + tx*b;    // top lerp
    Vector2f f = (1.0f - tx)*c + tx*d;    // bottom lerp
    Vector2f g = (1.0f - ty)*e + ty*f;    // final lerp

    assert(abs(g(0)) <= 1.0f);
    assert(abs(g(1)) <= 1.0f);

    *s = g(0);
    *t = g(1);
}


Vector2f GlintsMapData::stAt(int x, int y) const {
    return Vector2f(data[3 * (y*(width + 1) + x)],
                    data[3 * (y*(width + 1) + x) + 1]);
}




float GlintsMapData::triangleIntegralG(float p0, float p1, float q0, float q1,
    const Vector2f& st_p0q0, const Vector2f& st_p1q0, const Vector2f& st_p0q1,
    float s, float t, const GlintsPixelFootprint& footprint,
    float roughness) const {

    // std of the gaussian pixel reconstruction filter (in pixels)
    const float sigr = roughness;       // std of gaussian applied to D(s,t)
    const float sigxy_sq = sigxy * sigxy;
    const float sigr_sq = sigr * sigr;

    // mu, mean of the footprint gaussian (i.e. footprint center)
    float mu1 = footprint.u * width;
    float mu2 = footprint.v * height;

    // matrix B.  B transforms (p_xy - mu_xy) to (p_uv - mu_uv)
    // covariance matrix of footprint gaussian is sigxy^2 * B * B'
    Matrix2f B;
    B << footprint.dudx * width, footprint.dudy * width,
         footprint.dvdx * height, footprint.dvdy * height;
    float detB = B.determinant();

    // we need the elements of the inverse of B, defined as D
    Matrix2f D = B.inverse();
    float D11 = D(0, 0);
    float D12 = D(0, 1);
    float D21 = D(1, 0);
    float D22 = D(1, 1);


    // calculate A,b of transform s = Au+b between triangle corners and the normals
    // at those corners
    float deltaP = p1 - p0;
    float deltaQ = q1 - q0;

    Matrix2f A;
    A.col(0) = (st_p1q0 - st_p0q0) / deltaP;
    A.col(1) = (st_p0q1 - st_p0q0) / deltaQ;
    Vector2f b = st_p0q0 - A * Vector2f(p0, q0);

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
        (A11*b1 + A21*b2) / sigr_sq
        - (D11*D1j_dot_mu + D21*D2j_dot_mu) / sigxy_sq
        );
    float cv = (
        (A12*b1 + A22*b2) / sigr_sq
        - (D12*D1j_dot_mu + D22*D2j_dot_mu) / sigxy_sq
        );
    float cc = 0.5 * (
        (b1*b1 + b2*b2) / sigr_sq
        + (D1j_dot_mu*D1j_dot_mu + D2j_dot_mu*D2j_dot_mu) / sigxy_sq
        );

    // calculate triangle integral of G(u,v)
    float integral = integral_expquaduv_triangle(cuu, cuv, cvv, cu, cv, cc, p0, p1, q0, q1);
    if (isnan(integral)) {
        return 0.0;
    }
    if (integral < 0.0f) {
        return 0.0;
    }
    return C * integral;
}
