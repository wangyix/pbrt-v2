#if defined(_MSC_VER)
#pragma once
#endif

#ifndef PBRT_CORE_GLINTSMATH_H
#define PBRT_CORE_GLINTSMATH_H

// integral of exp(-(cuu*u^2 + cuv*u*v + cvv*v^2 + cu*u + cv*v + cc))dvdu
// over triangle (u0,v0),(u1,v0),(u0,v1)
float integral_expquaduv_triangle(float cuu, float cuv, float cvv, float cu, float cv, float cc,
    float u0, float u1, float v0, float v1);

#endif // PBRT_CORE_GLINTSMATH_H
