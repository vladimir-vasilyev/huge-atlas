#include "app/BadPatch.h"

#include <cstdio>
#include <ctime>
#include <filesystem>
#include <unordered_map>

namespace {

std::string timestamp()
{
    const std::time_t now = std::time(nullptr);
    std::tm tm{};
    localtime_r(&now, &tm);
    char buf[32];
    std::strftime(buf, sizeof buf, "%Y%m%d-%H%M%S", &tm);
    return buf;
}

// OBJ of one patch: v, vt (the uv of the corners, when there are any) and faces. The first lines say what is wrong with it.
bool writeObj(const std::string& path, const std::vector<std::string>& notes, const std::vector<Eigen::Vector3d>& positions,
              const std::vector<float>& cornerUV, const std::vector<int>& triangles)
{
    FILE* f = std::fopen(path.c_str(), "w");
    if (!f) return false;
    for (const std::string& n : notes) std::fprintf(f, "# %s\n", n.c_str());
    for (const auto& p : positions) std::fprintf(f, "v %.9g %.9g %.9g\n", p.x(), p.y(), p.z());
    const bool uv = cornerUV.size() == triangles.size() * 2;
    if (uv)
        for (size_t c = 0; c < triangles.size(); ++c) std::fprintf(f, "vt %.9g %.9g\n", cornerUV[2 * c], cornerUV[2 * c + 1]);
    for (size_t t = 0; t + 2 < triangles.size(); t += 3) {
        if (uv) std::fprintf(f, "f %d/%zu %d/%zu %d/%zu\n", triangles[t] + 1, t + 1, triangles[t + 1] + 1, t + 2, triangles[t + 2] + 1, t + 3);
        else    std::fprintf(f, "f %d %d %d\n", triangles[t] + 1, triangles[t + 1] + 1, triangles[t + 2] + 1);
    }
    return std::fclose(f) == 0;
}

struct Namer {
    std::filesystem::path dir;
    std::string stamp = timestamp();
    int counter = 0;
    std::string operator()(const char* mode, size_t patch, size_t faces, const char* why)
    {
        char buf[200];
        std::snprintf(buf, sizeof buf, "badpatch_%s_p%zu_%zuf_%s_%s_%03d.obj", mode, patch, faces, why, stamp.c_str(), ++counter);
        return (dir / buf).string();
    }
};

bool prepare(const std::string& folder, Namer& name, BadPatchReport& r)
{
    name.dir = folder;
    std::error_code ec;
    std::filesystem::create_directories(name.dir, ec);
    r.folder = std::filesystem::absolute(name.dir).string();
    if (ec) { r.error = "cannot create " + name.dir.string() + ": " + ec.message(); return false; }
    return true;
}

void writePresetFor(const std::string& objPath, Preset preset, const std::string& note)
{
    preset.note = note;
    std::string err;
    writePreset(objPath.substr(0, objPath.size() - 4) + ".preset", preset, &err);
}

} // namespace

BadPatchReport saveBadPatches(const std::vector<PatchParam>& params, const std::vector<Patch>* extracted, const Preset& preset,
                              const std::string& folder, int selected)
{
    BadPatchReport r;
    Namer name;
    if (!prepare(folder, name, r)) return r;
    auto isBad = [&](size_t i, bool* notDisk, bool* hasUV) {
        const PatchParam& pp = params[i];
        const bool nd = !pp.patch.isDisk();
        const bool uv = !nd && pp.method != ParamMethod::None && pp.uv.size() == pp.patch.mesh.n_vertices();
        if (notDisk) *notDisk = nd;
        if (hasUV) *hasUV = uv;
        return nd || (uv && (pp.needsSplit || !pp.injective));
    };
    const bool onlySelected = selected >= 0 && size_t(selected) < params.size() && isBad(size_t(selected), nullptr, nullptr);
    for (size_t i = 0; i < params.size(); ++i) {
        bool notDisk = false, hasUV = false;
        if (!isBad(i, &notDisk, &hasUV)) continue;
        if (onlySelected && size_t(selected) != i) continue;
        ++r.bad;
        const PatchParam& pp = params[i];
        // what was unfolded: the extracted patch (before the slits of the ladder), so that loading it again gives the same problem
        const Patch* ex = extracted && i < extracted->size() && (*extracted)[i].mesh.n_faces() > 0 ? &(*extracted)[i] : nullptr;
        const TriMesh& m = ex ? ex->mesh : pp.patch.mesh;
        const bool sameAsUnfolded = hasUV && m.n_faces() == pp.patch.mesh.n_faces() && m.n_vertices() == pp.patch.mesh.n_vertices();
        std::vector<Eigen::Vector3d> pos;
        for (auto v : m.vertices()) { const auto q = m.point(v); pos.emplace_back(q[0], q[1], q[2]); }
        std::vector<int> tri;
        std::vector<float> uv;
        for (auto fh : m.faces())
            for (auto v : m.fv_range(fh)) {
                tri.push_back(v.idx());
                if (sameAsUnfolded) { uv.push_back(float(pp.uv[v.idx()].x())); uv.push_back(float(pp.uv[v.idx()].y())); }
            }
        std::vector<std::string> notes{"bad patch of the loaded mesh (uv are the patch uv, not the atlas)"};
        char line[240];
        const size_t faces = pp.patch.faceToGlobal.size();
        if (notDisk)
            std::snprintf(line, sizeof line, "patch %zu: %zu faces, not a disk: %s", i, faces, patchKindName(pp.patch.topology.kind));
        else
            std::snprintf(line, sizeof line,
                          "patch %zu: %zu faces, method %s, injective %d, needs split %d, flipped %d, crossings %zu, folds %zu, L2 mean %.3f max %.3f, bad area share %.4f",
                          i, faces, paramMethodName(pp.method), int(pp.injective), int(pp.needsSplit), pp.distortion.flipped,
                          pp.report.uv.crossings.size(), pp.report.angles.foldVertices.size(), pp.distortion.l2Mean, pp.distortion.l2Max, pp.badAreaFraction);
        notes.push_back(line);
        if (!sameAsUnfolded && hasUV) notes.push_back("the ladder cut slits into this patch: the vt of the bad uv are not saved, load the patch to see them");
        const char* why = notDisk ? "notdisk" : (!pp.injective ? (pp.distortion.flipped ? "flipped" : "crossing") : "stretch");
        const std::string objPath = name("mesh", i, faces, why);
        if (writeObj(objPath, notes, pos, uv, tri)) {
            ++r.saved;
            writePresetFor(objPath, preset, line);
        }
    }
    return r;
}

BadPatchReport saveBadPatches(const FlatMesh& m, const FlatResult& res, const Preset& preset, const std::string& folder, int selected)
{
    BadPatchReport r;
    Namer name;
    if (!prepare(folder, name, r)) return r;
    auto isBad = [&](size_t i) { return res.patches[i].needsSplit || !res.patches[i].injective; };
    const bool onlySelected = selected >= 0 && size_t(selected) < res.patches.size() && isBad(size_t(selected));
    for (size_t i = 0; i < res.patches.size(); ++i) {
        const FlatPatch& p = res.patches[i];
        if (!isBad(i)) continue;
        if (onlySelected && size_t(selected) != i) continue;
        ++r.bad;
        std::vector<Eigen::Vector3d> pos;
        std::vector<int> tri;
        std::vector<float> uv;
        std::unordered_map<uint32_t, int> local;
        for (uint32_t f : p.faces)
            for (int k = 0; k < 3; ++k) {
                const uint32_t v = m.idx[3 * size_t(f) + k];
                auto it = local.find(v);
                if (it == local.end()) {
                    it = local.emplace(v, int(pos.size())).first;
                    pos.emplace_back(m.pos[3 * size_t(v)], m.pos[3 * size_t(v) + 1], m.pos[3 * size_t(v) + 2]);
                }
                tri.push_back(it->second);
                uv.push_back(res.cornerUV[6 * size_t(f) + 2 * k]);
                uv.push_back(res.cornerUV[6 * size_t(f) + 2 * k + 1]);
            }
        char line[200];
        std::vector<std::string> notes{"bad patch of the huge result (uv are the packed atlas uv)"};
        std::snprintf(line, sizeof line, "patch %zu: %zu faces, method %s, injective %d, needs split %d, flipped %d, bad area share %.4f, L2 mean %.3f",
                      i, p.faces.size(), paramMethodName(p.method), int(p.injective), int(p.needsSplit), p.distortion.flipped, p.badAreaFraction,
                      p.distortion.l2Mean);
        notes.push_back(line);
        const std::string objPath = name("huge", i, p.faces.size(), p.injective ? "stretch" : "overlap");
        if (writeObj(objPath, notes, pos, uv, tri)) {
            ++r.saved;
            writePresetFor(objPath, preset, std::string(line) + "\nlifted from a coarse chart in the huge mode; loading it unfolds it again with the ladder");
        }
    }
    return r;
}

std::string describe(const BadPatchReport& r)
{
    if (!r.error.empty()) return r.error;
    if (r.bad == 0) return "no bad patches";
    char buf[400];
    std::snprintf(buf, sizeof buf, "saved %d of %d bad patches to %s", r.saved, r.bad, r.folder.c_str());
    return buf;
}
