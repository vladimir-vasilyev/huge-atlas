// Save Bad Patches: a patch that is too stretched goes to an OBJ (v, vt, f) with a preset next to it; both read back.
#include "app/BadPatch.h"
#include "core/MeshPrep.h"
#include "core/ObjLoader.h"
#include "TestUtil.h"

#include <filesystem>
#include <fstream>
#include <sstream>

int main()
{
    TriMesh mesh;
    std::string err;
    CHECK(loadObj(std::string(TEST_DATA_DIR) + "/hemisphere.obj", mesh, nullptr, &err));
    normalizeToUnitBox(mesh);
    ExtractionResult ex = extractPatches(mesh, std::vector<int>(mesh.n_faces(), 0));
    CHECK(ex.patches.size() == 1 && ex.patches[0].isDisk());

    ParamOptions strict;
    strict.maxStretch = 1.0001;   // a hemisphere cannot be flattened this well: the patch is "too stretched"
    PatchParam pp = parameterizePatch(mesh, ex.patches[0], strict);
    CHECK(pp.method != ParamMethod::None && pp.uv.size() == pp.patch.mesh.n_vertices());
    CHECK(pp.needsSplit);

    const std::string folder = "/tmp/vsa_badpatch_test";
    std::filesystem::remove_all(folder);
    Preset preset;
    preset.k = 77;
    const BadPatchReport r = saveBadPatches({pp}, &ex.patches, preset, folder, -1);
    CHECK(r.error.empty() && r.bad == 1 && r.saved == 1);
    CHECK(describe(r).find("saved 1 of 1") != std::string::npos);

    std::string obj;
    for (const auto& e : std::filesystem::directory_iterator(folder))
        if (e.path().extension() == ".obj") obj = e.path().string();
    CHECK(!obj.empty());
    std::ifstream in(obj);
    std::stringstream text;
    text << in.rdbuf();
    CHECK(text.str().find("needs split 1") != std::string::npos);
    size_t vt = 0, f = 0, v = 0;
    std::string line;
    std::istringstream lines(text.str());
    while (std::getline(lines, line)) {
        vt += line.rfind("vt ", 0) == 0;
        v += line.rfind("v ", 0) == 0;
        f += line.rfind("f ", 0) == 0;
    }
    CHECK(f == ex.patches[0].mesh.n_faces() && vt == 3 * f && v == ex.patches[0].mesh.n_vertices());

    Preset back;
    CHECK(readPreset(obj.substr(0, obj.size() - 4) + ".preset", back, &err));
    CHECK(back.k == 77);

    // only the selected bad patch is saved, and a good patch is not a bad patch
    CHECK(saveBadPatches({pp}, nullptr, preset, folder, 0).saved == 1);
    PatchParam good = parameterizePatch(mesh, ex.patches[0], ParamOptions());
    CHECK(saveBadPatches({good}, nullptr, preset, folder, -1).bad == (good.needsSplit || !good.injective ? 1 : 0));
    return 0;
}
