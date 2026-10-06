#include "param/Predicates.h"

#include <cmath>
#include <vector>

namespace {

using Expansion = std::vector<double>;   // non-overlapping, increasing magnitude, zeros eliminated

inline void twoSum(double a, double b, double& x, double& y)
{
    x = a + b;
    const double bv = x - a;
    const double av = x - bv;
    y = (a - av) + (b - bv);
}

inline void twoDiff(double a, double b, double& x, double& y)
{
    x = a - b;
    const double bv = a - x;
    const double av = x + bv;
    y = (a - av) + (bv - b);
}

inline void twoProduct(double a, double b, double& x, double& y)
{
    x = a * b;
    y = std::fma(a, b, -x);
}

Expansion growExpansion(const Expansion& e, double b)
{
    Expansion h;
    double q = b;
    for (double ei : e) {
        double hh;
        twoSum(q, ei, q, hh);
        if (hh != 0.0) h.push_back(hh);
    }
    if (q != 0.0 || h.empty()) h.push_back(q);
    return h;
}

Expansion expansionSum(Expansion e, const Expansion& f)
{
    for (double fi : f) e = growExpansion(e, fi);
    return e;
}

Expansion scaleExpansion(const Expansion& e, double b)
{
    Expansion h;
    double q, hh;
    twoProduct(e[0], b, q, hh);
    if (hh != 0.0) h.push_back(hh);
    for (size_t i = 1; i < e.size(); ++i) {
        double p1, p0, sum;
        twoProduct(e[i], b, p1, p0);
        twoSum(q, p0, sum, hh);
        if (hh != 0.0) h.push_back(hh);
        twoSum(p1, sum, q, hh);
        if (hh != 0.0) h.push_back(hh);
    }
    if (q != 0.0 || h.empty()) h.push_back(q);
    return h;
}

Expansion product(const Expansion& e, const Expansion& f)
{
    Expansion r = scaleExpansion(e, f[0]);
    for (size_t j = 1; j < f.size(); ++j) r = expansionSum(r, scaleExpansion(e, f[j]));
    return r;
}

Expansion difference(double a, double b)
{
    double x, y;
    twoDiff(a, b, x, y);
    return y != 0.0 ? Expansion{y, x} : Expansion{x};
}

constexpr double kEps = 1.1102230246251565e-16;           // 2^-53
constexpr double kErrBound = (3.0 + 16.0 * kEps) * kEps;   // Shewchuk's ccwerrboundA

} // namespace

double orient2dExact(const double a[2], const double b[2], const double c[2])
{
    const Expansion acx = difference(a[0], c[0]), bcx = difference(b[0], c[0]);
    const Expansion acy = difference(a[1], c[1]), bcy = difference(b[1], c[1]);
    Expansion left = product(acx, bcy);
    Expansion right = product(acy, bcx);
    for (double& x : right) x = -x;
    const Expansion det = expansionSum(left, right);
    return det.back();   // the most significant component carries the sign
}

double orient2d(const double a[2], const double b[2], const double c[2])
{
    const double detleft = (a[0] - c[0]) * (b[1] - c[1]);
    const double detright = (a[1] - c[1]) * (b[0] - c[0]);
    const double det = detleft - detright;
    double detsum;
    if (detleft > 0.0) {
        if (detright <= 0.0) return det;
        detsum = detleft + detright;
    } else if (detleft < 0.0) {
        if (detright >= 0.0) return det;
        detsum = -detleft - detright;
    } else {
        return det;
    }
    const double bound = kErrBound * detsum;
    if (det >= bound || -det >= bound) return det;
    return orient2dExact(a, b, c);
}
