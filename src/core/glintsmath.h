#if defined(_MSC_VER)
#pragma once
#endif

#ifndef PBRT_CORE_GLINTSMATH_H
#define PBRT_CORE_GLINTSMATH_H

// integral of exp(-(cuu*u^2 + cuv*u*v + cvv*v^2 + cu*u + cv*v + cc))dvdu
// over triangle (u0,v0),(u1,v0),(u0,v1)
double integral_expquaduv_triangle(double cuu, double cuv, double cvv, double cu, double cv, double cc,
    double u0, double u1, double v0, double v1);

#endif // PBRT_CORE_GLINTSMATH_H
