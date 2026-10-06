#pragma once

// Exact geometric predicates (Shewchuk-style adaptive arithmetic: a floating point filter first,
// exact expansion arithmetic when the filter cannot decide).

// > 0 if a, b, c are in counter-clockwise order, < 0 if clockwise, 0 if collinear. The sign is exact.
double orient2d(const double a[2], const double b[2], const double c[2]);

inline int orient2dSign(const double a[2], const double b[2], const double c[2])
{
    const double d = orient2d(a, b, c);
    return d > 0.0 ? 1 : (d < 0.0 ? -1 : 0);
}

// Always uses the exact path (for tests).
double orient2dExact(const double a[2], const double b[2], const double c[2]);
