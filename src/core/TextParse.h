#pragma once

// Text number parsing shared by the OBJ and flat mesh readers: no allocations, no locale, a correctly rounded
// fast path for ordinary decimal numbers.

#include <cstdlib>

namespace textparse {

inline bool isSpace(char c) { return c == ' ' || c == '\t' || c == '\r'; }

inline const char* skipSpaces(const char* p, const char* end)
{
    while (p < end && isSpace(*p)) ++p;
    return p;
}

inline const char* skipToken(const char* p, const char* end)
{
    while (p < end && !isSpace(*p) && *p != '\n') ++p;
    return p;
}

// Decimal floating point; exact fast path (one correctly rounded division) for up to 15 significant digits,
// strtod otherwise (and for nan/inf). Advances p; false if no number is there.
inline bool parseDouble(const char*& p, const char* end, double& out)
{
    static const double kPow10[] = {1e0, 1e1, 1e2, 1e3, 1e4, 1e5, 1e6, 1e7, 1e8, 1e9, 1e10, 1e11, 1e12,
                                    1e13, 1e14, 1e15, 1e16, 1e17, 1e18, 1e19, 1e20, 1e21, 1e22};
    const char* q = p;
    bool neg = false;
    if (q < end && (*q == '-' || *q == '+')) { neg = *q == '-'; ++q; }
    unsigned long long mant = 0;
    int digits = 0, frac = 0;
    bool any = false, seenDot = false;
    for (; q < end; ++q) {
        const char c = *q;
        if (c >= '0' && c <= '9') {
            any = true;
            if (digits >= 15) goto slow;   // beyond 15 significant digits the mantissa is no longer exact in a double
            mant = mant * 10 + unsigned(c - '0');
            if (mant) ++digits;
            if (seenDot) ++frac;
        } else if (c == '.' && !seenDot) {
            seenDot = true;
        } else {
            break;
        }
    }
    if (!any) return false;
    if (q < end && (*q == 'e' || *q == 'E')) goto slow;
    if (frac > 22) goto slow;
    {
        const double v = double(mant) / kPow10[frac];   // mant < 1e15 < 2^53 and 10^frac are exact: one rounding, like strtod
        out = neg ? -v : v;
        p = q;
        return true;
    }
slow:
    {
        char* e = nullptr;
        out = std::strtod(p, &e);
        if (e == p) return false;
        p = e;
        return true;
    }
}

// Optional sign and digits.
inline bool parseInt(const char*& p, const char* end, long& out)
{
    const char* q = p;
    bool neg = false;
    if (q < end && (*q == '-' || *q == '+')) { neg = *q == '-'; ++q; }
    if (q >= end || *q < '0' || *q > '9') return false;
    long v = 0;
    while (q < end && *q >= '0' && *q <= '9') { v = v * 10 + (*q - '0'); ++q; }
    out = neg ? -v : v;
    p = q;
    return true;
}

struct Corner { long v = 0, vt = 0; };   // 1-based or negative OBJ indices, 0 = absent

// "v", "v/vt", "v//vn", "v/vt/vn"
inline bool parseCorner(const char*& p, const char* end, Corner& c)
{
    c = Corner();
    if (!parseInt(p, end, c.v)) return false;
    if (p < end && *p == '/') {
        ++p;
        long vt = 0;
        if (parseInt(p, end, vt)) c.vt = vt;
        if (p < end && *p == '/') {
            ++p;
            long vn;
            parseInt(p, end, vn);
        }
    }
    return true;
}

} // namespace textparse
