#pragma once

#include "param/ParamPipeline.h"

#include <string>

// A preset is the set of parameters that made a patch: the VSA that cut it out, the extraction of the patches, the
// parameterization ladder and the refinement. It is saved next to a bad patch (Debug > Save Bad Patch, file name.preset)
// so that the patch can be loaded again and unwrapped with exactly these settings (Debug > Load Bad Patch).
// Text form, one `key = value` per line; unknown keys are ignored and missing ones keep their defaults.
struct Preset {
    // VSA
    int    metric = 0;                  // 0 L2,1  1 L2
    int    k = 100;
    int    seeding = 1;                 // 0 random, 1 farthest point
    int    rngSeed = 0;
    int    lloydMaxIterations = 30;
    double lloydRelTolerance = 1e-4;
    int    lloydPatience = 3;
    double smoothLambda = 0.1, smoothKappa = 0.0;
    int    smoothMaxSweeps = 10;
    int    normalSmoothIterations = 0;
    double normalSmoothSigma = 25.0;
    // extraction of the patches
    int    patchMinFaces = 3;
    bool   patchCutMultiBoundary = true, patchCutHandles = true;
    // parameterization (LinABF, active set, slits, Tutte, limits)
    ParamOptions param;
    // refinement
    int    refinePolicy = 0;            // 0 fix first, 1 split immediately
    int    refineMaxDepth = 10, refineMinFaces = 8, refineMaxRounds = 12, refineSplitAtOnceAboveFaces = 40000;
    double refineSmoothLambda = 0.1;
    // what the preset was saved for: informative, written as comments
    std::string note;
};

bool writePreset(const std::string& path, const Preset& preset, std::string* error = nullptr);
bool readPreset(const std::string& path, Preset& preset, std::string* error = nullptr);
