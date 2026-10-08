#include "app/Preset.h"

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <sstream>

namespace {

// One list of (key, field) pairs serves reading and writing.
template <class F>
void visit(Preset& p, F&& f)
{
    f("vsa.metric", p.metric);
    f("vsa.k", p.k);
    f("vsa.seeding", p.seeding);
    f("vsa.rngSeed", p.rngSeed);
    f("vsa.lloydMaxIterations", p.lloydMaxIterations);
    f("vsa.lloydRelTolerance", p.lloydRelTolerance);
    f("vsa.lloydPatience", p.lloydPatience);
    f("vsa.smoothLambda", p.smoothLambda);
    f("vsa.smoothKappa", p.smoothKappa);
    f("vsa.smoothMaxSweeps", p.smoothMaxSweeps);
    f("patches.minFaces", p.patchMinFaces);
    f("patches.cutMultiBoundary", p.patchCutMultiBoundary);
    f("patches.cutHandles", p.patchCutHandles);
    f("param.useActiveSet", p.param.useActiveSet);
    f("param.allowCut", p.param.allowCut);
    f("param.maxCuts", p.param.maxCuts);
    f("param.cutKappa", p.param.cutKappa);
    f("param.allowTutte", p.param.allowTutte);
    f("param.maxStretch", p.param.maxStretch);
    f("param.maxBadAreaFraction", p.param.maxBadAreaFraction);
    f("linabf.epsilon", p.param.lin.epsilon);
    f("linabf.deficitThreshold", p.param.lin.deficitThreshold);
    f("linabf.maxIterations", p.param.lin.maxIterations);
    f("linabf.wheelTolerance", p.param.lin.wheelTolerance);
    f("activeSet.foldDelta", p.param.active.foldDelta);
    f("activeSet.angleMargin", p.param.active.angleMargin);
    f("activeSet.maxRounds", p.param.active.maxRounds);
    f("refine.policy", p.refinePolicy);
    f("refine.maxDepth", p.refineMaxDepth);
    f("refine.minFaces", p.refineMinFaces);
    f("refine.maxRounds", p.refineMaxRounds);
    f("refine.splitAtOnceAboveFaces", p.refineSplitAtOnceAboveFaces);
    f("refine.smoothLambda", p.refineSmoothLambda);
}

std::string format(int v) { return std::to_string(v); }
std::string format(bool v) { return v ? "1" : "0"; }
std::string format(double v) { char b[40]; std::snprintf(b, sizeof b, "%.17g", v); return b; }

void parse(const std::string& s, int& v) { v = std::atoi(s.c_str()); }
void parse(const std::string& s, bool& v) { v = std::atoi(s.c_str()) != 0; }
void parse(const std::string& s, double& v) { v = std::atof(s.c_str()); }

std::string trim(const std::string& s)
{
    size_t a = 0, b = s.size();
    while (a < b && (s[a] == ' ' || s[a] == '\t' || s[a] == '\r')) ++a;
    while (b > a && (s[b - 1] == ' ' || s[b - 1] == '\t' || s[b - 1] == '\r')) --b;
    return s.substr(a, b - a);
}

} // namespace

bool writePreset(const std::string& path, const Preset& preset, std::string* error)
{
    std::ofstream out(path);
    if (!out) { if (error) *error = "cannot write " + path; return false; }
    out << "# VSA preset: the parameters that made the patch next to this file\n";
    std::istringstream notes(preset.note);
    for (std::string line; std::getline(notes, line);) out << "# " << line << "\n";
    Preset copy = preset;
    visit(copy, [&](const char* key, auto& value) { out << key << " = " << format(value) << "\n"; });
    return bool(out);
}

bool readPreset(const std::string& path, Preset& preset, std::string* error)
{
    std::ifstream in(path);
    if (!in) { if (error) *error = "cannot read " + path; return false; }
    std::map<std::string, std::string> values;
    for (std::string line; std::getline(in, line);) {
        line = trim(line);
        if (line.empty() || line[0] == '#') continue;
        const size_t eq = line.find('=');
        if (eq == std::string::npos) continue;
        values[trim(line.substr(0, eq))] = trim(line.substr(eq + 1));
    }
    visit(preset, [&](const char* key, auto& value) {
        const auto it = values.find(key);
        if (it != values.end()) parse(it->second, value);
    });
    return true;
}
