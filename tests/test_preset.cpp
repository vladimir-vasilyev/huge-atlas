// Preset files: every value survives a write and a read; unknown keys and comments are ignored; missing keys keep the defaults.
#include "app/Preset.h"
#include "TestUtil.h"

#include <fstream>

int main()
{
    Preset p;
    p.metric = 1; p.k = 321; p.seeding = 0; p.rngSeed = 7;
    p.lloydMaxIterations = 11; p.lloydRelTolerance = 3.5e-5; p.lloydPatience = 5;
    p.smoothLambda = 0.37; p.smoothKappa = 1.25; p.smoothMaxSweeps = 17;
    p.patchMinFaces = 9; p.patchCutMultiBoundary = false; p.patchCutHandles = false;
    p.param.useActiveSet = false; p.param.allowCut = false; p.param.maxCuts = 5; p.param.cutKappa = 2.5; p.param.allowTutte = false;
    p.param.maxStretch = 6.5; p.param.maxBadAreaFraction = 0.0123;
    p.param.lin.epsilon = 2e-3; p.param.lin.deficitThreshold = 1.5; p.param.lin.maxIterations = 12; p.param.lin.wheelTolerance = 1e-9;
    p.param.active.foldDelta = 0.07; p.param.active.angleMargin = 0.03; p.param.active.maxRounds = 6;
    p.refinePolicy = 1; p.refineMaxDepth = 3; p.refineMinFaces = 20; p.refineMaxRounds = 4; p.refineSplitAtOnceAboveFaces = 12345;
    p.refineSmoothLambda = 0.21;
    p.note = "patch 12, 345 faces\nnot injective: border crossings";

    const std::string path = "/tmp/vsa_preset_test.preset";
    std::string err;
    CHECK(writePreset(path, p, &err));
    Preset q;   // defaults
    CHECK(readPreset(path, q, &err));
    CHECK(q.metric == 1 && q.k == 321 && q.seeding == 0 && q.rngSeed == 7);
    CHECK(q.lloydMaxIterations == 11 && q.lloydRelTolerance == 3.5e-5 && q.lloydPatience == 5);
    CHECK(q.smoothLambda == 0.37 && q.smoothKappa == 1.25 && q.smoothMaxSweeps == 17);
    CHECK(q.patchMinFaces == 9 && !q.patchCutMultiBoundary && !q.patchCutHandles);
    CHECK(!q.param.useActiveSet && !q.param.allowCut && q.param.maxCuts == 5 && q.param.cutKappa == 2.5 && !q.param.allowTutte);
    CHECK(q.param.maxStretch == 6.5 && q.param.maxBadAreaFraction == 0.0123);
    CHECK(q.param.lin.epsilon == 2e-3 && q.param.lin.deficitThreshold == 1.5 && q.param.lin.maxIterations == 12 && q.param.lin.wheelTolerance == 1e-9);
    CHECK(q.param.active.foldDelta == 0.07 && q.param.active.angleMargin == 0.03 && q.param.active.maxRounds == 6);
    CHECK(q.refinePolicy == 1 && q.refineMaxDepth == 3 && q.refineMinFaces == 20 && q.refineMaxRounds == 4);
    CHECK(q.refineSplitAtOnceAboveFaces == 12345 && q.refineSmoothLambda == 0.21);

    // a hand written file: comments, spaces, an unknown key, only a few values
    {
        std::ofstream o("/tmp/vsa_preset_partial.preset");
        o << "# hand made\n  param.maxStretch   =  9  \nsomething.new = 5\n\nvsa.k=42\n";
    }
    Preset r;
    CHECK(readPreset("/tmp/vsa_preset_partial.preset", r, &err));
    CHECK(r.param.maxStretch == 9.0 && r.k == 42);
    CHECK(r.param.useActiveSet == Preset().param.useActiveSet && r.smoothLambda == Preset().smoothLambda);

    CHECK(!readPreset("/nonexistent/x.preset", r, &err));
    if (failures) { std::printf("%d failures\n", failures); return 1; }
    std::printf("preset: OK\n");
    return 0;
}
