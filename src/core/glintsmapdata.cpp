#include "glintsmapdata.h"
#include "glintsmath.h"
#include "reflection.h"

// if true, then the normal texture is set to (s,t)=(0,0) everywhere, so that
// D(s,t) simply becomes the roughness gaussian centered at (0,0).  This can
// be computed analytically and will be compared to the numerically computed result
#define TEST_ACCURACY 0


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
#if TEST_ACCURACY
            float s = 0.0;
            float t = 0.0;
#else
            float s = ((float)u[0] - 128.0) / 128.0;
            float t = ((float)u[1] - 128.0) / 128.0;
#endif
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
    if (x0 == -1) x0 = width - 1;

    int y0 = Floor2Int(v * height - 0.5f);
    int y1 = y0 + 1;
    float ty = (v * height - 0.5f) - y0;
    if (y0 == -1) y0 = height - 1;

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
    const float xyCullRadiusSq = xyCullRadius * xyCullRadius;

    const float stCullRadius = CULL_RADIUS_STDS * roughness;
    const float stCullRadiusSq = stCullRadius * stCullRadius;
    

    Vector2f pqc(footprint.u * width, footprint.v * height); // footprint center
    Matrix2f B;                         // (dx,dy) to (dp,dq)
    B << footprint.dudx * width, footprint.dudy * width,
        footprint.dvdx * height, footprint.dvdy * height;
    if (abs(B.determinant()) < 0.000000001) {
        return 0.0;
    }
    Matrix2f dpq2dxy = B.inverse();     // (dp,dq) to (dx,dy)

    float ret = recursiveD(s, t, stCullRadiusSq, roughness, pqc, dpq2dxy, xyCullRadiusSq,
        0, width, 0, height, stMinMaxTree.size() - 1);

#if TEST_ACCURACY
    if (s*s + t*t < stCullRadiusSq) {
        printf("Computed D(%f, %f) = %f\n", s, t, ret);
        float x_sq = s*s + t*t;
        float sig = roughness;
        float g = 1 / (sig*sig * (2.0*M_PI)) * exp(-x_sq / (2 * sig*sig));
        printf("        should be:   %f\n\n", g);
    }
#endif
    return ret;
}




// integrates G over texel triangles in a sub-region of the texture
// from,to indices refer to cells, or alternatively the top-left texel of the cell
float GlintsMapData::recursiveD(float s, float t, float stCullRadiusSq, float roughness,
    const Vector2f& pqc, const Matrix2f& dpq2dxy, float xyCullRadiusSq,
    int pfrom, int pto, int qfrom, int qto, int level) const {

    int dim = pto - pfrom;
    assert(dim == qto - qfrom);
    assert(dim == Round2Int(pow(2, level)));

    // check if this region of texels is within pixel footprint
    float pLeft = pfrom + 0.5f;
    float pRight = pto + 0.5f;
    float qTop = qfrom + 0.5f;
    float qBottom = qto + 0.5f;
    Vector2f p0q0(pLeft, qTop);
    Vector2f p1q0(pRight, qTop);
    Vector2f p0q1(pLeft, qBottom);
    Vector2f p1q1(pRight, qBottom);
    // first check if footprint center is inside region
    float pc = pqc(0), qc = pqc(1);
    bool pqcInside = (pLeft <= pc && pc <= pRight
        && qTop <= qc && qc <= qBottom);
    // reject region if all 4 centers are outside of culling radius from center
    // and center is outside region
    if (!pqcInside
        && (dpq2dxy*(p0q0 - pqc)).squaredNorm() >= xyCullRadiusSq
        && (dpq2dxy*(p1q0 - pqc)).squaredNorm() >= xyCullRadiusSq
        && (dpq2dxy*(p0q1 - pqc)).squaredNorm() >= xyCullRadiusSq
        && (dpq2dxy*(p1q1 - pqc)).squaredNorm() >= xyCullRadiusSq)
    {
        return 0.0f;
    }

    // now check if this region's span of (s,t) values is within the roughness radius
    const STMinMax* treeLevel = stMinMaxTree[level];
    int treeDim = width / dim;
    int treeX = pfrom / dim;
    int treeY = qfrom / dim;
    const STMinMax& stMinMax = treeLevel[treeY * treeDim + treeX];
    Vector2f stTL(stMinMax.smin, stMinMax.tmin);
    Vector2f stTR(stMinMax.smax, stMinMax.tmin);
    Vector2f stBL(stMinMax.smin, stMinMax.tmax);
    Vector2f stBR(stMinMax.smax, stMinMax.tmax);
    // first check if (s,t) is within minmax region
    bool stInside = (stMinMax.smin <= s && s <= stMinMax.smax
        && stMinMax.tmin <= t && t <= stMinMax.tmax);
    // reject region if all 4 centers are outside of cullring radius from center
    // and center is outside region
    Vector2f st(s, t);
    if (!stInside
        && (stTL - st).squaredNorm() >= stCullRadiusSq
        && (stTR - st).squaredNorm() >= stCullRadiusSq
        && (stBL - st).squaredNorm() >= stCullRadiusSq
        && (stBR - st).squaredNorm() >= stCullRadiusSq)
    {
        return 0.0f;
    }

    float sum = 0.0f;
    if (dim == 1) {
        // BASE CASE: if we've reached a 1x1 region that hasn't been rejected, do
        // the integral over the two triangles

        // get the 4 samples at the corners of the squares
        Vector2f st_p0q0 = stAt(pfrom, qfrom);
        Vector2f st_p1q0 = stAt(pto, qfrom);
        Vector2f st_p0q1 = stAt(pfrom, qto);
        Vector2f st_p1q1 = stAt(pto, qto);

        sum += triangleIntegralG(p0q0(0), p1q0(0), p0q0(1), p0q1(1),
            st_p0q0, st_p1q0, st_p0q1,
            s, t, roughness, pqc, dpq2dxy);

        // lower triangle
        sum += triangleIntegralG(p1q1(0), p0q1(0), p1q1(1), p1q0(1),
            st_p1q1, st_p0q1, st_p1q0,
            s, t, roughness, pqc, dpq2dxy);

    } else {
        // RECURSIVE CASE: call recursiveD on the four subregions
        int pmid = (pfrom + pto) / 2;
        int qmid = (qfrom + qto) / 2;
        sum += recursiveD(s, t, stCullRadiusSq, roughness, pqc, dpq2dxy, xyCullRadiusSq,
            pfrom, pmid, qfrom, qmid, level - 1);
        sum += recursiveD(s, t, stCullRadiusSq, roughness, pqc, dpq2dxy, xyCullRadiusSq,
            pmid, pto, qfrom, qmid, level - 1);
        sum += recursiveD(s, t, stCullRadiusSq, roughness, pqc, dpq2dxy, xyCullRadiusSq,
            pfrom, pmid, qmid, qto, level - 1);
        sum += recursiveD(s, t, stCullRadiusSq, roughness, pqc, dpq2dxy, xyCullRadiusSq,
            pmid, pto, qmid, qto, level - 1);
    }
    return sum;
}




float GlintsMapData::triangleIntegralG(float p0, float p1, float q0, float q1,
    const Vector2f& st_p0q0, const Vector2f& st_p1q0, const Vector2f& st_p0q1,
    float s, float t, float roughness, const Vector2f& pqc, const Matrix2f& dpq2dxy) const {

    // std of the gaussian pixel reconstruction filter (in pixels)
    const double sigr = roughness;       // std of gaussian applied to D(s,t)
    const double sigxy_sq = sigxy * sigxy;
    const double sigr_sq = sigr * sigr;

    // mu, mean of the footprint gaussian (i.e. footprint center)
    double mu1 = (double)pqc(0); //footprint.u * width;
    double mu2 = (double)pqc(1); //footprint.v * height;

    // matrix B.  B transforms (p_xy - mu_xy) to (p_uv - mu_uv)
    // covariance matrix of footprint gaussian is sigxy^2 * B * B'
    //Matrix2f B;
    //B << footprint.dudx * width, footprint.dudy * width,
    //     footprint.dvdx * height, footprint.dvdy * height;
    //float detB = B.determinant();

    // we need the elements of the inverse of B, defined as D
    Matrix2f D = dpq2dxy;   //B.inverse();
    double D11 = (double)D(0, 0);
    double D12 = (double)D(0, 1);
    double D21 = (double)D(1, 0);
    double D22 = (double)D(1, 1);
    double detD = (double)D.determinant();


    // calculate A,b of transform s = Au+b between triangle corners and the normals
    // at those corners
    double deltaP = p1 - p0;
    double deltaQ = q1 - q0;

    Matrix2f A;
    A.col(0) = (st_p1q0 - st_p0q0) / deltaP;
    A.col(1) = (st_p0q1 - st_p0q0) / deltaQ;
    Vector2f b = st_p0q0 - A * Vector2f(p0, q0) - Vector2f(s,t);

    double A11 = (double)A(0, 0);
    double A12 = (double)A(0, 1);
    double A21 = (double)A(1, 0);
    double A22 = (double)A(1, 1);
    double b1 = (double)b(0);
    double b2 = (double)b(1);


    // calculate constant C in G(u,v) = C*exp(-quadratic(u,v))
    double C = INV_FOUR_PI_SQ * abs(detD) / (sigr_sq * sigxy_sq);

    // calculate quadratic coefficients in G(u,v) = C*exp(-quadratic(u,v))
    double D1j_dot_mu = D11*mu1 + D12*mu2;
    double D2j_dot_mu = D21*mu1 + D22*mu2;

    double cuu = 0.5 * (
        (A11*A11 + A21*A21) / sigr_sq
        + (D11*D11 + D21*D21) / sigxy_sq
        );
    double cuv = (
        (A11*A12 + A21*A22) / sigr_sq
        + (D11*D12 + D21*D22) / sigxy_sq
        );
    double cvv = 0.5 * (
        (A12*A12 + A22*A22) / sigr_sq
        + (D12*D12 + D22*D22) / sigxy_sq
        );
    double cu = (
        (A11*b1 + A21*b2) / sigr_sq
        - (D11*D1j_dot_mu + D21*D2j_dot_mu) / sigxy_sq
        );
    double cv = (
        (A12*b1 + A22*b2) / sigr_sq
        - (D12*D1j_dot_mu + D22*D2j_dot_mu) / sigxy_sq
        );
    double cc = 0.5 * (
        (b1*b1 + b2*b2) / sigr_sq
        + (D1j_dot_mu*D1j_dot_mu + D2j_dot_mu*D2j_dot_mu) / sigxy_sq
        );

    // calculate triangle integral of G(u,v)
    double integral_cond = integral_expquaduv_conditioned(cuu, cuv, cvv, cu, cv, cc, p0, p1, q0, q1);
    if (isnan(integral_cond)) {
        return 0.0;
    }
    if (integral_cond < 0.0f) {
        return 0.0;
    }
    return C * integral_cond;
}
