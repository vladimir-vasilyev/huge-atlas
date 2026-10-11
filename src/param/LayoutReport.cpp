#include "param/LayoutReport.h"

#include <cstdio>

std::string formatLayoutComparison(const UVLayoutStats& o, const UVLayoutStats& n, const MipSafetyResult& mo, const MipSafetyResult& mn,
                                   bool hadOriginal)
{
    std::string out;
    char line[200];
    auto row = [&](const char* name, const char* a, const char* b) {
        std::snprintf(line, sizeof(line), "%-36s %12s %12s\n", name, a, b);
        out += line;
    };
    row("", "original", "new atlas");
    auto num = [&](const char* name, const char* fmt, double x, double y) {
        char a[48], b[48];
        if (hadOriginal) std::snprintf(a, sizeof(a), fmt, x); else std::snprintf(a, sizeof(a), "-");
        std::snprintf(b, sizeof(b), fmt, y);
        row(name, a, b);
    };
    num("charts", "%.0f", o.charts, n.charts);
    num("seams, % of edge length", "%.2f", 100 * o.seamRatio(), 100 * n.seamRatio());
    num("flipped faces", "%.0f", o.flipped, n.flipped);
    num("faces without uv", "%.0f", double(o.facesWithoutUV), double(n.facesWithoutUV));
    num("L2 stretch, mean", "%.3f", o.l2Mean, n.l2Mean);
    num("L2 stretch, p95", "%.3f", o.l2P95, n.l2P95);
    num("L2 stretch, p99", "%.3f", o.l2P99, n.l2P99);
    num("conformal, p95", "%.3f", o.conformalP95, n.conformalP95);
    num("texel density, p05", "%.3f", o.densityP05, n.densityP05);
    num("texel density, p95", "%.3f", o.densityP95, n.densityP95);
    num("texture covered, %", "%.1f", 100 * o.covered, 100 * n.covered);
    num("texels used twice, %", "%.3f", 100 * o.overlap, 100 * n.overlap);
    num("gap between charts, min (texels)", "%.0f", mo.gapMin, mn.gapMin);
    num("gap between charts, 5% (texels)", "%.0f", mo.gapP05, mn.gapP05);
    num("gap between charts, median", "%.0f", mo.gapP50, mn.gapP50);
    num("mip level the 5% gap allows", "%.0f", mo.levelByGapP05, mn.levelByGapP05);
    auto conf = [&](int lv, const MipSafetyResult& r) { return lv < int(r.conflictCharts.size()) ? double(r.conflictCharts[lv]) : -1.0; };
    for (int lv = 0; lv <= 3; ++lv) {
        char nm[64];
        std::snprintf(nm, sizeof(nm), "charts mixed at mip %d", lv);
        num(nm, "%.0f", conf(lv, mo), conf(lv, mn));
    }
    return out;
}
