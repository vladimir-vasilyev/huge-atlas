#include "param/ParamPipeline.h"

#include <cstdio>

const char* paramMethodName(ParamMethod m)
{
    switch (m) {
        case ParamMethod::LinABF:    return "LinABF";
        case ParamMethod::ActiveSet: return "active set";
        case ParamMethod::Cut:       return "cut";
        case ParamMethod::Tutte:     return "Tutte";
        default:                     return "none";
    }
}

namespace {

struct Attempt {
    bool solved = false;               // angles and UV exist
    std::vector<double> angles;
    std::vector<Eigen::Vector2d> uv;
    InjectivityReport report;
    std::string what;

    int problems() const
    {
        return int(report.angles.badCorners.size() + report.angles.foldVertices.size() +
                   report.uv.flippedFaces.size() + report.uv.crossings.size());
    }
};

std::string describe(const InjectivityReport& r)
{
    char buf[160];
    std::snprintf(buf, sizeof(buf), "%zu bad angles, %zu folds, %zu flipped, %zu crossings", r.angles.badCorners.size(),
                  r.angles.foldVertices.size(), r.uv.flippedFaces.size(), r.uv.crossings.size());
    return buf;
}

Attempt attempt(const TriMesh& m, const LinABFResult& lin, const ParamOptions& opt, const char* name)
{
    Attempt a;
    if (!lin.ok) { a.what = std::string(name) + ": " + lin.message; return a; }
    a.angles = lin.angles;
    a.report.angles = checkAngles(m, lin.angles, opt.active.foldDelta);
    const UVResult uv = anglesToUV(m, lin.angles);
    if (!uv.ok) { a.what = std::string(name) + ": UV failed: " + uv.message; return a; }
    a.uv = uv.uv;
    a.report.uv = checkUV(m, uv.uv);
    a.solved = true;
    a.what = std::string(name) + ": " + (a.report.ok() ? "injective" : describe(a.report));
    return a;
}

void finish(PatchParam& out, const Patch& patch, ParamMethod method, Attempt& a)
{
    out.method = method;
    out.patch = patch;
    out.angles = std::move(a.angles);
    out.uv = std::move(a.uv);
    out.report = std::move(a.report);
    out.injective = out.report.ok();
    out.cuts = patch.cuts;
}

} // namespace

PatchParam parameterizePatch(const TriMesh& source, const Patch& original, const ParamOptions& opt)
{
    PatchParam out;
    out.patch = original;
    if (!original.isDisk()) {
        out.log = std::string("not a disk: ") + patchKindName(original.topology.kind);
        out.needsSplit = true;
        return out;
    }

    Patch current = original;
    Attempt best;   // the attempt with the fewest problems, in case nothing is injective
    Patch bestPatch = original;
    ParamMethod bestMethod = ParamMethod::None;
    auto consider = [&](Attempt& a, const Patch& p, ParamMethod method) {
        out.log += a.what + "\n";
        if (!a.solved) return false;
        if (a.report.ok()) { finish(out, p, method, a); return true; }
        if (!best.solved || a.problems() < best.problems()) { best = a; bestPatch = p; bestMethod = method; }
        return false;
    };

    bool done = false;
    for (int cut = 0; cut <= (opt.allowCut ? opt.maxCuts : 0) && !done; ++cut) {
        if (cut > 0) {
            Patch next;
            std::string err;
            if (!cutAtWorstVertex(source, current, opt.cutKappa, next, &err)) { out.log += "cut: " + err + "\n"; break; }
            current = std::move(next);
            out.log += "cut " + std::to_string(cut) + ": slit added\n";
        }
        const ParamMethod plain = cut ? ParamMethod::Cut : ParamMethod::LinABF;
        Attempt a = attempt(current.mesh, solveLinABF(current.mesh, opt.lin), opt, "LinABF");
        if (consider(a, current, plain)) { done = true; break; }
        if (opt.useActiveSet) {
            ActiveSetInfo info;
            Attempt b = attempt(current.mesh, solveLinABFActiveSet(current.mesh, opt.lin, opt.active, &info), opt, "active set");
            if (consider(b, current, cut ? ParamMethod::Cut : ParamMethod::ActiveSet)) { done = true; break; }
        }
    }

    if (!done && opt.allowTutte) {
        std::string err;
        const std::vector<Eigen::Vector2d> uv = tutteEmbedding(original.mesh, &err);
        if (uv.empty()) {
            out.log += "Tutte: " + err + "\n";
        } else {
            Attempt t;
            t.solved = true;
            t.uv = uv;
            t.report.uv = checkUV(original.mesh, uv);
            t.what = std::string("Tutte: ") + (t.report.ok() ? "injective" : describe(t.report));
            done = consider(t, original, ParamMethod::Tutte);
        }
    }

    if (!done && best.solved) {   // nothing injective: keep the least bad attempt for inspection
        finish(out, bestPatch, bestMethod, best);
        out.injective = false;
    }
    if (out.method != ParamMethod::None) {
        out.distortion = computeDistortion(out.patch.mesh, out.uv, &out.perTriangleL2);
        out.needsSplit = !out.injective || (opt.maxStretch > 0.0 && out.distortion.l2Max > opt.maxStretch);
    } else {
        out.needsSplit = true;
    }
    return out;
}
