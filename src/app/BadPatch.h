#pragma once

#include "app/Preset.h"
#include "param/FlatPipeline.h"
#include "param/ParamPipeline.h"
#include "param/PatchExtractor.h"

#include <string>
#include <vector>

// Bad patches (not a disk, overlapping itself, too stretched) written as OBJ files with the faulty uv in `vt`, and a preset
// next to each (see Preset.h). Shared by both shells; no interface in here.
struct BadPatchReport {
    int bad = 0;            // bad patches found (the selected one only, when it is bad)
    int saved = 0;
    std::string folder;
    std::string error;
};

// The patches of a regular result. `extracted` (optional, one per patch) are the patches before the slits of the ladder: saving them
// makes loading give the same problem. `selected`: -1 or a patch; when that one is bad, only it is saved.
BadPatchReport saveBadPatches(const std::vector<PatchParam>& params, const std::vector<Patch>* extracted, const Preset& preset,
                              const std::string& folder, int selected);
// The patches of a result of the huge mode (the uv are the packed atlas uv).
BadPatchReport saveBadPatches(const FlatMesh& mesh, const FlatResult& result, const Preset& preset, const std::string& folder, int selected);

// The text for a status line: what was found / saved.
std::string describe(const BadPatchReport& r);
