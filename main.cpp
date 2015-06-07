#include <iostream>
#include <stdio.h>
#include <algorithm>
#include <assert.h>

#define M_PI       3.14159265358979323846f
#define INV_PI     0.31830988618379067154f
#define INV_TWOPI  0.15915494309189533577f
#define INV_FOURPI 0.07957747154594766788f
#define SQRT_PI    1.7724538509055159f

#define ERF_HI_ACCR 1

using namespace std;


#if ERF_HI_ACCR == 0

#define ERF_INTERVALS 6
#define ERF_INTERVAL_WIDTH (6.0f / ERF_INTERVALS)
float erfQuadCoeffs[ERF_INTERVALS][3] = {
    { 0.0077718428863119277, 0.043514858915608634, -0.9393799187329831 },
    { 0.18837446996390575, 0.71774488196095576, -0.31333038095266502 },
    { 0.39659792535275584, 1.2392987183024708, 0 },
    { -0.39659792535275645, 1.2392987183024713, -4.3340321149696856e-17 },
    { -0.18837446996390733, 0.71774488196095954, 0.31333038095266313 },
    { -0.0077718428863113058, 0.04351485891560556, 0.93937991873298676 }
};

#else

#define ERF_INTERVALS 16
#define ERF_INTERVAL_WIDTH (6.0f / ERF_INTERVALS)
float erfQuadCoeffs[ERF_INTERVALS][3] = {
    { 0.0012538983019272697, 0.0075419385860549483, -0.98863717846218213 },
    { 0.0076089715720849991, 0.040446645348687131, -0.94605274996245303 },
    { 0.034135618362908619, 0.15826869439552815, -0.81524428898560519 },
    { 0.11213125772124799, 0.44746942466203604, -0.5471963393550644 },
    { 0.26515585771745964, 0.90327923257127274, -0.20778697748268607 },
    { 0.43606665548198542, 1.2902452404905687, 0.011240803005794546 },
    { 0.45619726425381479, 1.3319918535353887, 0.031227295355255629 },
    { 0.2007956577269949, 1.152943463473816, 0 },
    { -0.20079565772699451, 1.1529434634738156, 7.168330766004533e-17 },
    { -0.45619726425381718, 1.3319918535353912, -0.031227295355256247 },
    { -0.43606665548197004, 1.2902452404905396, -0.011240803005781086 },
    { -0.26515585771746258, 0.90327923257128129, 0.20778697748267971 },
    { -0.11213125772124254, 0.44746942466201767, 0.54719633935508039 },
    { -0.034135618362910694, 0.15826869439553715, 0.81524428898559553 },
    { -0.0076089715720859038, 0.040446645348691364, 0.94605274996244815 },
    { -0.0012538983019352364, 0.0075419385860994274, 0.98863717846212007 }
};
#endif






/*float erf(float x) {
    if (x <= -3.0)
        return -1.0;
    else if (x >= 3.0)
        return 1.0;
    else {
        // find which interval x belongs to
        int i = floor((x + 3.0) / 6.0 * ERF_HI_ACCR_INTERVALS);
        assert(i >= 0 && i < ERF_HI_ACCR_INTERVALS);
        float a = erfQuadCoeffsHiAccr[i][0];
        float b = erfQuadCoeffsHiAccr[i][1];
        float c = erfQuadCoeffsHiAccr[i][2];
        return (a*x + b)*x + c;
    }
}*/



struct Linear {
    Linear() : a(0.0), b(0.0) {}
    Linear(float bb) : a(0.0), b(bb) {}
    Linear(float aa, float bb) : a(aa), b(bb) {}
    float operator()(float x) const {
        return a*x + b;
    }
    Linear operator+(float c) const {
        return Linear(a, b + c);
    }
    Linear operator+(const Linear& l) const {
        return Linear(a + l.a, b + l.b);
    }
    Linear operator-(float c) const {
        return Linear(a, b - c);
    }
    Linear operator-() const {
        return Linear(-a, -b);
    }
    Linear operator*(float c) const {
        return Linear(a*c, b*c);
    }
    Linear operator/(float c) const {
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
    float a, b;
};
Linear operator*(float c, const Linear& l) {
    return Linear(c*l.a, c*l.b);
}
Linear operator+(float c, const Linear& l) {
    return Linear(l.a, c + l.b);
}


struct Quadratic {
    Quadratic() : a(0.0), b(0.0), c(0.0) {}
    Quadratic(float aa, float bb, float cc) : a(aa), b(bb), c(cc) {}
    float operator()(float x) const {
        return (a*x + b)*x + c;
    }
    // changes to a quadratic in terms of y
    // given y = dx + de
    Quadratic changeVar(const Linear& y) const {
        return substitute(y.inverse());
    }
    // replaces x with dx + e
    Quadratic substitute(const Linear& f) const {
        float d = f.a, e = f.b;
        return Quadratic(a*d*d, (2*a*e + b) * d, (a*e + b)*e + c);
    }
    // computes c,d,e so that (cx+d)^2+e = ax^2+bx+c
    // expects a to be positive
    void completeTheSquare(Linear* sq, float* remainder) const {
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
    Quadratic operator*(float s) const {
        return Quadratic(a*s, b*s, c*s);
    }
    Quadratic operator/(float s) const {
        return Quadratic(a / s, b / s, c / s);
    }
    float a, b, c;
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
    LinearL(float aa, const Linear& bb) : a(aa), b(bb) {}
    Linear operator()(float x) const {
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
    float a;
    Linear b;
};

// represents ax^2 + byx + cx + dy + e
struct QuadraticL {
    QuadraticL() : a(0.0), b(), c(0.0) {}
    QuadraticL(float aa, const Linear& bb, const Linear& cc) : a(aa), b(bb), c(cc) {}
    QuadraticL(float cxx, float cyx, float cx, float cy, float cc)
        : a(cxx), b(cyx, cx), c(cy, cc) {}
    Linear operator()(float x) const {
        return (a*x + b)*x + c;
    }
    /*// changes to a quadratic in terms of y
    // given y = dx + de
    QuadraticL changeVar(const LinearL& y) const {
        return substitute(y.inverse());
    }
    // replaces x with dx + e
    QuadraticL substitute(const LinearL& f) const {
        float d = f.a;
        const Linear& e = f.b;
        return QuadraticL(a*d*d, (2*a*e + b) * d, (a*e + b)*e + c);
    }*/
    // computes c,d,e so that (cx+d)^2+e = ax^2+bx+c
    // expects a to be positive
    void completeTheSquare(LinearL* sq, Quadratic* remainder) const {
        assert(a > 0.0);
        sq->a = sqrt(a);
        sq->b = 0.5 * b / sqrt(a);
        *remainder = c - 0.25 * b * b / a;
    }
    float a;
    Linear b, c;
};




int getErfIntervalIndex(float x) {
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
Quadratic getErfIntervalApprox(int i, float* xfrom, float* xto) {
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








// integral of exp(-x^2)
float integral_expx2(float x0, float x1) {
    return 0.5 * SQRT_PI * (erf(x1) - erf(x0));
}
// inegral of exp(-x^2)*x
float integral_expx2_x(float x0, float x1) {
    return 0.5 * (exp(-x0*x0) - exp(-x1*x1));
}
// integral of exp(-x^2)*x^2
float integral_expx2_x2(float x0, float x1) {
    return 0.25 * (SQRT_PI * (erf(x1) - erf(x0)))
        + 0.5 * (exp(-x0*x0)*x0 - exp(-x1*x1)*x1);
}







// integral of exp(-x^2)*quad(x)
float integral_expx2_quad(const Quadratic& quad, float x0, float x1) {
    return quad.a * integral_expx2_x2(x0, x1)
        + quad.b * integral_expx2_x(x0, x1)
        + quad.c * integral_expx2(x0, x1);
}

// integral of exp(c)*quad2(x)
float integral_expc_quad(float n, const Quadratic& quad,
                        float x0, float x1) {
    float a = quad.a;
    float b = quad.b;
    float c = quad.c;
    return -1.0 / 6.0 * exp(-n) * (x0 - x1) * (
            2 * a * (x0*x0 + x0*x1 + x1*x1)
            + 3 * b * (x0 + x1)
            + 6 * c
        );
}

// integral of exp(-linear(x))*quad2(x)
float integral_explin_quad(const Linear& expLin, const Quadratic& quad,
                        float x0, float x1) {
    if (expLin.a == 0.0)
        return integral_expc_quad(expLin.b, quad, x0, x1);

    float a = quad.a;
    float b = quad.b;
    float c = quad.c;
    float m = expLin.a;
    float n = expLin.b;
    return 1/(m*m*m) * exp(-n)*(
        exp(-m*x0) * (
                      a*((m*x0 + 1)*(m*x0 + 1) + 1)
                      + m*(b*m*x0 + b + c*m)
                      )
        + exp(-m*x1) * (
                      -a*((m*x1 + 1)*(m*x1 + 1) + 1)
                      - m*(b*m*x1 + b + c*m)
                      )
        );
}
// integral of exp(-quad1(x))*quad2(x)
float integral_expquad_quad(const Quadratic& expQuad, const Quadratic& quad,
                            float x0, float x1) {
    assert(expQuad.a >= 0.0);

    if (expQuad.a == 0.0f) {
        Linear expLin(expQuad.b, expQuad.c);
        return integral_explin_quad(expLin, quad, x0, x1);
    }

    // complete the square on the exp quadratic to express it as 
    // y^2 + r = ax^2+bx+c
    Linear y;
    float r;
    expQuad.completeTheSquare(&y, &r);

    // rewrite integral in terms of y:
    // exp(-y^2-r)*yQuad(y)*1/y.a dy = exp(-r)/y.a * exp(-y^2)*yQuad(y)
    Quadratic yQuad = quad.changeVar(y);
    float dydx = y.a;
    float scale = exp(-r) / dydx; // take constants outside
    float y0 = y(x0), y1 = y(x1);

    return scale * integral_expx2_quad(yQuad, y0, y1);
}

// integral of exp(-quad(x))erf(x)
float integral_expquad_erfx(const Quadratic& expQuad, float x0, float x1) {
    bool boundsFlipped = false;
    if (x0 > x1) {
        float temp = x0;
        x0 = x1;
        x1 = temp;
        boundsFlipped = true;
    }
    int i = getErfIntervalIndex(x0);
    float xfrom, xto;
    float sum = 0.0f;
    Quadratic intervalApprox;
    while (true) {
        intervalApprox = getErfIntervalApprox(i, &xfrom, &xto);
        if (xto >= x1)
            break;
        // integrate from where we are now to the end of this interval
        sum += integral_expquad_quad(expQuad, intervalApprox, x0, xto);
        x0 = xto;
        i++;
    }
    // integrate remainder of range
    sum += integral_expquad_quad(expQuad, intervalApprox, x0, x1);

    if (boundsFlipped)
        sum = -sum;
    return sum;
}

// integral of exp(-quad(x))erf(linear(x))
float integral_expquad_erflin(const Quadratic& expQuad, const Linear& erfLin,
                            float x0, float x1) {
    Linear y = erfLin;
    // rewrite integral in terms of y
    // exp(-quad(y))*erf(y)*1/y.a
    Quadratic yExpQuad = expQuad.changeVar(y);
    float dydx = y.a;
    float scale = 1 / dydx;
    float y0 = y(x0), y1 = y(x1);

    return scale * integral_expquad_erfx(yExpQuad, y0, y1);
}


// integral of exp(-quad(u,v)) dv du over triangle
float integral_expquaduv(float cuu, float cuv, float cvv, float cu, float cv, float cc,
    float u0, float u1, float v0, float v1) {
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
    float dydx = yL.a;
    expQuadOutside = expQuadOutside + rL;
    Linear yL0 = yL(v0), yL1 = yL(fu);

    // integrating gives: 1/dydx * exp(-quad(U)) * (erf(y1(u)) - erf(y0(u)))
    // adding the du integral outside gives:
    // 1/dydx * ( int(exp(-quad(u)*erf(y1(u))) - int(exp(-quad(u)*erf(y0(u))) )
    return SQRT_PI / (2.0 * dydx) * (
        integral_expquad_erflin(expQuadOutside, yL1, u0, u1) -
        integral_expquad_erflin(expQuadOutside, yL0, u0, u1)
        );
}


int main(void) {

    Quadratic expQuad(1.2, -0.5, -0.3);
    Quadratic quad(0.9, 2.3, 8);
    float x0 = -2, x1 = 3;

    //printf("%f\n", integral_expx2_quad(quad, x0, x1));

    //Linear expLin(0, 3.6);
    //printf("%f\n", integral_explin_quad(expLin, quad, 2, 3));

    //printf("%f\n", integral_expquad_quad(expQuad, quad, x0, x1));

    //printf("%f\n", integral_expquad_erfx(expQuad, x0, x1));

    //Linear erfLin(-1.8, -1.0);
    //printf("%f\n", integral_expquad_erflin(expQuad, erfLin, x0, x1));

    float cuu = 1.0;
    float cuv = 1.5;
    float cvv = 2;
    float cu = -2;
    float cv = 3;
    float cc = -4;

    float u0 = 0.8;
    float u1 = -0.5;
    float v0 = 0.8;
    float v1 = -0.5;

    printf("%f\n", integral_expquaduv(cuu, cuv, cvv, cu, cv, cc, u0, u1, v0, v1));

    return 0;
}


