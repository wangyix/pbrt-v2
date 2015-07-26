#include <algorithm>
#include <assert.h>

#include "glintsmath.h"


#define SQRT_PI    1.7724538509055159

#define ERF_HI_ACCR 0

using namespace std;


#if ERF_HI_ACCR == 0

#define ERF_INTERVALS 6
#define ERF_INTERVAL_WIDTH (6.0f / ERF_INTERVALS)
double erfQuadCoeffs[ERF_INTERVALS][3] = {
    { 0.0077718428863119277, 0.043514858915608634, -0.9393799187329831 },
    { 0.18837446996390575, 0.71774488196095576, -0.31333038095266502 },
    { 0.39659792535275584, 1.2392987183024708, 0 },
    { -0.39659792535275645, 1.2392987183024713, 0 },
    { -0.18837446996390733, 0.71774488196095954, 0.31333038095266313 },
    { -0.0077718428863113058, 0.04351485891560556, 0.93937991873298676 }
};

#else

#define ERF_INTERVALS 16
#define ERF_INTERVAL_WIDTH (6.0f / ERF_INTERVALS)
double erfQuadCoeffs[ERF_INTERVALS][3] = {
    { 0.0012538983019272697, 0.0075419385860549483, -0.98863717846218213 },
    { 0.0076089715720849991, 0.040446645348687131, -0.94605274996245303 },
    { 0.034135618362908619, 0.15826869439552815, -0.81524428898560519 },
    { 0.11213125772124799, 0.44746942466203604, -0.5471963393550644 },
    { 0.26515585771745964, 0.90327923257127274, -0.20778697748268607 },
    { 0.43606665548198542, 1.2902452404905687, 0.011240803005794546 },
    { 0.45619726425381479, 1.3319918535353887, 0.031227295355255629 },
    { 0.2007956577269949, 1.152943463473816, 0 },
    { -0.20079565772699451, 1.1529434634738156, 0 },
    { -0.45619726425381718, 1.3319918535353912, -0.031227295355256247 },
    { -0.43606665548197004, 1.2902452404905396, -0.011240803005781086 },
    { -0.26515585771746258, 0.90327923257128129, 0.20778697748267971 },
    { -0.11213125772124254, 0.44746942466201767, 0.54719633935508039 },
    { -0.034135618362910694, 0.15826869439553715, 0.81524428898559553 },
    { -0.0076089715720859038, 0.040446645348691364, 0.94605274996244815 },
    { -0.0012538983019352364, 0.0075419385860994274, 0.98863717846212007 }
};
#endif


/*double erf(double x) {
if (x <= -3.0)
return -1.0;
else if (x >= 3.0)
return 1.0;
else {
// find which interval x belongs to
int i = floor((x + 3.0) / 6.0 * ERF_INTERVALS);
assert(i >= 0 && i < ERF_INTERVALS);
double a = erfQuadCoeffs[i][0];
double b = erfQuadCoeffs[i][1];
double c = erfQuadCoeffs[i][2];
return (a*x + b)*x + c;
}
}*/



struct Linear {
    Linear() : a(0.0), b(0.0) {}
    Linear(double bb) : a(0.0), b(bb) {}
    Linear(double aa, double bb) : a(aa), b(bb) {}
    double operator()(double x) const {
        return a*x + b;
    }
    Linear operator+(double c) const {
        return Linear(a, b + c);
    }
    Linear operator+(const Linear& l) const {
        return Linear(a + l.a, b + l.b);
    }
    Linear operator-(double c) const {
        return Linear(a, b - c);
    }
    Linear operator-() const {
        return Linear(-a, -b);
    }
    Linear operator*(double c) const {
        return Linear(a*c, b*c);
    }
    Linear operator/(double c) const {
        return Linear(a / c, b / c);
    }
    // becomes inverse of linear function
    void invert() {
        assert(a != 0.0);
        b = -b / a;
        a = 1.0 / a;
    }
    Linear inverse() const {
        assert(a != 0.0);
        return Linear(1.0 / a, -b / a);
    }
    double a, b;
};
Linear operator*(double c, const Linear& l) {
    return Linear(c*l.a, c*l.b);
}
Linear operator+(double c, const Linear& l) {
    return Linear(l.a, c + l.b);
}
// find linear function to map [x0,x1] to [y0,y1]
Linear mappingRangeToRange(double x0, double x1, double y0, double y1) {
    assert(x0 != x1);
    return Linear((y1 - y0) / (x1 - x0), (x1*y0 - x0*y1) / (x1 - x0));
}

struct Quadratic {
    Quadratic() : a(0.0), b(0.0), c(0.0) {}
    Quadratic(double aa, double bb, double cc) : a(aa), b(bb), c(cc) {}
    double operator()(double x) const {
        return (a*x + b)*x + c;
    }
    // changes to a quadratic in terms of y
    // given y = dx + de
    Quadratic changeVar(const Linear& y) const {
        return substitute(y.inverse());
    }
    // replaces x with dx + e
    Quadratic substitute(const Linear& f) const {
        double d = f.a, e = f.b;
        return Quadratic(a*d*d, (2 * a*e + b) * d, (a*e + b)*e + c);
    }
    // computes c,d,e so that (cx+d)^2+e = ax^2+bx+c
    // expects a to be positive
    void completeTheSquare(Linear* sq, double* remainder) const {
        assert(a > 0.0);
        sq->a = sqrt(a);
        sq->b = 0.5 * b / sqrt(a);
        *remainder = c - 0.25 * b * b / a;
    }
    Quadratic operator+(const Quadratic& s) const {
        return Quadratic(a + s.a, b + s.b, c + s.c);
    }
    Quadratic operator-(const Quadratic& s) const {
        return Quadratic(a - s.a, b - s.b, c - s.c);
    }
    Quadratic operator*(double s) const {
        return Quadratic(a*s, b*s, c*s);
    }
    Quadratic operator/(double s) const {
        return Quadratic(a / s, b / s, c / s);
    }
    double a, b, c;
};

Quadratic operator*(const Linear& l1, const Linear& l2) {
    return Quadratic(l1.a*l2.a, l1.a*l2.b + l1.b*l2.a, l1.b*l2.b);
}

Quadratic operator-(const Linear& l1, const Quadratic& l2) {
    return Quadratic(-l2.a, l1.a - l2.b, l1.b - l2.c);
}




// represents ax + by + c
struct LinearL {
    LinearL() : a(0.0), b() {}
    LinearL(double aa, const Linear& bb) : a(aa), b(bb) {}
    Linear operator()(double x) const {
        return a*x + b;
    }
    Linear operator()(const Linear& x) const {
        return a*x + b;
    }
    // becomes inverse of linear function
    void invert() {
        assert(a != 0.0);
        b = -b / a;
        a = 1.0 / a;
    }
    LinearL inverse() const {
        assert(a != 0.0);
        return LinearL(1.0 / a, -b / a);
    }
    double a;
    Linear b;
};

// represents ax^2 + byx + cx + dy + e
struct QuadraticL {
    QuadraticL() : a(0.0), b(), c(0.0) {}
    QuadraticL(double aa, const Linear& bb, const Linear& cc) : a(aa), b(bb), c(cc) {}
    QuadraticL(double cxx, double cyx, double cx, double cy, double cc)
        : a(cxx), b(cyx, cx), c(cy, cc) {}
    Linear operator()(double x) const {
        return (a*x + b)*x + c;
    }
    // computes c,d,e so that (cx+d)^2+e = ax^2+bx+c
    // expects a to be positive
    void completeTheSquare(LinearL* sq, Quadratic* remainder) const {
        assert(a > 0.0);
        sq->a = sqrt(a);
        sq->b = 0.5 * b / sqrt(a);
        *remainder = c - 0.25 * b * b / a;
    }
    double a;
    Linear b, c;
};




int getErfIntervalIndex(double x) {
    if (x <= -3.0f)
        return -1;
    else if (x >= 3.0f)
        return ERF_INTERVALS;
    else {
        int i = floor((x + 3.0) / 6.0 * ERF_INTERVALS);
        assert(i >= 0 && i < ERF_INTERVALS);
        return i;
    }
}

// gets the quadratic approximation in interval i of erf function
Quadratic getErfIntervalApprox(int i, double* xfrom, double* xto) {
    if (i < 0) {
        *xfrom = -INFINITY;
        *xto = -3.0f;
        return Quadratic(0.0f, 0.0f, -1.0f);
    } else if (i >= ERF_INTERVALS) {
        *xfrom = 3.0f;
        *xto = INFINITY;
        return Quadratic(0.0f, 0.0f, 1.0f);
    } else {
        *xfrom = -3.0 + i * ERF_INTERVAL_WIDTH;
        *xto = *xfrom + ERF_INTERVAL_WIDTH;
        return Quadratic(erfQuadCoeffs[i][0],
            erfQuadCoeffs[i][1],
            erfQuadCoeffs[i][2]);
    }
}


// integral of exp(c)*quad(x)
double integral_expc_quad(double n, const Quadratic& quad,
    double x0, double x1) {
    double a = quad.a;
    double b = quad.b;
    double c = quad.c;
    double ret = -1.0 / 6.0 * exp(-n) * (x0 - x1) * (
        2 * a * (x0*x0 + x0*x1 + x1*x1)
        + 3 * b * (x0 + x1)
        + 6 * c
        );
    assert(!isnan(ret) && !isinf(ret));
    return ret;
}

// integral of exp(-linear(x))*quad(x)
double integral_explin_quad(const Linear& expLin, const Quadratic& quad,
    double x0, double x1) {
    if (expLin.a == 0.0)
        return integral_expc_quad(expLin.b, quad, x0, x1);

    double a = quad.a;
    double b = quad.b;
    double c = quad.c;
    double m = expLin.a;
    double n = expLin.b;
    double ret = 1 / (m*m*m) * exp(-n)*(
            exp(-m*x0) * (
            a*((m*x0 + 1)*(m*x0 + 1) + 1)
            + m*(b*m*x0 + b + c*m)
            )
            + exp(-m*x1) * (
            -a*((m*x1 + 1)*(m*x1 + 1) + 1)
            - m*(b*m*x1 + b + c*m)
            )
        );
    assert(!isnan(ret) && !isinf(ret));
    return ret;
}

// integral of exp(-quad1(x))*quad2(x)
double integral_expquad_quad(const Quadratic& expQuad, const Quadratic& quad,
    double x0, double x1) {
    // if we get coefficient a < 0 due to rounding error, just clamp it
    // to 0.
    if (expQuad.a <= 0.0f) {
        Linear expLin(expQuad.b, expQuad.c);
        return integral_explin_quad(expLin, quad, x0, x1);
    }
    
    double a = expQuad.a;
    double b = expQuad.b;
    double c = expQuad.c;
    double p = quad.a;
    double q = quad.b;
    double r = quad.c;

    double sqrt_a = sqrt(a);
    double exp_quad_x0 = exp(-expQuad(x0));
    double exp_quad_x1 = exp(-expQuad(x1));
    double ret =
        SQRT_PI * exp(b*b/(4.0*a) - c)
                * (erf(sqrt_a*x1 + b/(2*sqrt_a)) - erf(sqrt_a*x0 + b/(2*sqrt_a)))
                * (r/(2*sqrt_a) + (p-q*b)/(4*a*sqrt_a) + p*b*b/(8*a*a*sqrt_a))
        + (exp_quad_x1 - exp_quad_x0) * (-q/(2*a) + p*b/(4*a*a))
        + (x1*exp_quad_x1 - x0*exp_quad_x0) * (-p/(2*a));

    assert(!isnan(ret) && !isinf(ret));
    return ret;
}

// integral of exp(-quad(x))erf(x)
double integral_expquad_erfx(const Quadratic& expQuad, double x0, double x1) {
    bool boundsFlipped = false;
    if (x0 > x1) {
        double temp = x0;
        x0 = x1;
        x1 = temp;
        boundsFlipped = true;
    }
    int i = getErfIntervalIndex(x0);
    double xfrom, xto;
    double ret = 0.0f;
    Quadratic intervalApprox;
    while (true) {
        intervalApprox = getErfIntervalApprox(i, &xfrom, &xto);
        if (xto >= x1)
            break;
        // integrate from where we are now to the end of this interval
        ret += integral_expquad_quad(expQuad, intervalApprox, x0, xto);
        x0 = xto;
        i++;
    }
    // integrate remainder of range
    ret += integral_expquad_quad(expQuad, intervalApprox, x0, x1);

    if (boundsFlipped)
        ret = -ret;

    assert(!isnan(ret) && !isinf(ret));
    return ret;
}

// integral of exp(-quad(x))erf(constant)
double integral_expquad_erfc(const Quadratic& expQuad, double c,
    double x0, double x1) {
    double ret = integral_expquad_quad(expQuad, Quadratic(0.0, 0.0, erf(c)), x0, x1);
    assert(!isnan(ret) && !isinf(ret));
    return ret;
}

// integral of exp(-quad(x))erf(linear(x))
double integral_expquad_erflin(const Quadratic& expQuad, const Linear& erfLin,
    double x0, double x1) {
    if (abs(erfLin.a) < 0.000001) {
        return integral_expquad_erfc(expQuad, erfLin.b, x0, x1);
    }

    Linear y = erfLin;
    // rewrite integral in terms of y
    // exp(-quad(y))*erf(y)*1/y.a
    Quadratic yExpQuad = expQuad.changeVar(y);
    double dydx = y.a;
    double scale = 1 / dydx;
    double y0 = y(x0), y1 = y(x1);

    double ret = scale * integral_expquad_erfx(yExpQuad, y0, y1);
    assert(!isnan(ret) && !isinf(ret));
    return ret;
}


struct QuadraticUV {
    QuadraticUV() : cuu(0.0), cuv(), cvv(0.0), cu(0.0), cv(0.0), cc(0.0) {}
    QuadraticUV(double cuu, double cuv, double cvv, double cu, double cv, double cc)
        : cuu(cuu), cuv(cuv), cvv(cvv), cu(cu), cv(cv), cc(cc) {}
    double operator()(double u, double v) const {
        return cuu*u*u + cuv*u*v + cvv*v*v + cu*u + cv*v + cc;
    }
    QuadraticUV changeVar(const Linear& s, const Linear& t) const {
        return substitute(s.inverse(), t.inverse());
    }
    // replaces x with dx + e
    QuadraticUV substitute(const Linear& s, const Linear& t) const {
        double a1 = s.a, b1 = s.b;
        double a2 = t.a, b2 = t.b;
        return QuadraticUV(
            a1*a1*cuu,
            a1*a2*cuv,
            a2*a2*cvv,
            a1*cu + 2 * a1*b1*cuu + a1*b2*cuv,
            a2*cv + 2 * a2*b2*cvv + a2*b1*cuv,
            cuu*b1*b1 + cuv*b1*b2 + cvv*b2*b2 + cu*b1 + cv*b2 + cc
            );
    }
    double cuu, cuv, cvv, cu, cv, cc;
};


// integral of exp(-quad(u,v)) dv du over triangle
double integral_expquaduv(double cuu, double cuv, double cvv, double cu, double cv, double cc,
    double u0, double u1, double v0, double v1) {
    // upper bound of inner dv integral
    Linear fu = Linear(v0 - v1, u1*v1 - u0*v0) / (u1 - u0);

    // evaluate dv integral
    // exp(-cuu*u^2) term will be moved out front for now, which gives
    // exp(-cuu*u^2) * int(exp^(-quadL(v)dv, v0, f(u));
    Quadratic expQuadOutside = Quadratic(cuu, 0.0, 0.0);
    QuadraticL expQuadL(cvv, cuv, cv, cu, cc);

    // complete the square on the exp quadratic to express it as 
    // y^2 + r = ax^2+bx+c
    LinearL yL;
    Quadratic rL;
    expQuadL.completeTheSquare(&yL, &rL);

    // we can now write integral in terms of y:
    // exp(-yL^2-rL)*dx/dy = exp(-rL)*dx/dy * exp(-yL);
    double dydx = yL.a;
    expQuadOutside = expQuadOutside + rL;
    Linear yL0 = yL(v0), yL1 = yL(fu);

    // integrating gives: 1/dydx * exp(-quad(U)) * (erf(y1(u)) - erf(y0(u)))
    // adding the du integral outside gives:
    // 1/dydx * ( int(exp(-quad(u)*erf(y1(u))) - int(exp(-quad(u)*erf(y0(u))) )
    double ret = 
    SQRT_PI / (2.0 * dydx) * (
        integral_expquad_erflin(expQuadOutside, yL1, u0, u1) -
        integral_expquad_erflin(expQuadOutside, yL0, u0, u1)
        );

    assert(!isnan(ret) && !isinf(ret));
    return ret;
}

double integral_expquaduv_conditioned(
    double cuu, double cuv, double cvv, double cu, double cv, double cc,
    double u0, double u1, double v0, double v1) {

    QuadraticUV expQuadUv(cuu, cuv, cvv, cu, cv, cc);

    Linear s = mappingRangeToRange(u0, u1, -0.5, 0.5);
    double dsdu = s.a;
    Linear t = mappingRangeToRange(v0, v1, -0.5, 0.5);
    double dtdv = t.a;
    QuadraticUV expQuadSt = expQuadUv.changeVar(s, t);

    double scale = 1.0 / (dsdu * dtdv);
    double s0 = s(u0), s1 = s(u1);
    double t0 = t(v0), t1 = t(v1);


    return scale *
        integral_expquaduv(expQuadSt.cuu, expQuadSt.cuv, expQuadSt.cvv, expQuadSt.cu,
        expQuadSt.cv, expQuadSt.cc, s0, s1, t0, t1);
}
