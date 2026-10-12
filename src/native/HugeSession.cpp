#include "native/HugeSession.h"

#include "param/FlatAdjacency.h"
#include "param/LayoutReport.h"
#include "param/MipSafety.h"
#include "param/UVMetrics.h"

#include "param/AtlasMask.h"
#include "param/FlatBake.h"
#include "render/ImageFile.h"
#include "render/MeshRenderer.h"

#include <algorithm>
#include <unordered_map>
#include <chrono>
#include <cmath>
#include <filesystem>

size_t HugeSession::estimateFaces(const std::string& path)
{
    std::error_code ec;
    const auto bytes = std::filesystem::file_size(path, ec);
    if (ec) return 0;
    std::string ext = path.size() > 4 ? path.substr(path.size() - 4) : "";
    for (char& c : ext) c = char(std::tolower(static_cast<unsigned char>(c)));
    return size_t(bytes / (ext == ".obj" ? 75 : 40));
}

void HugeSession::clear()
{
    mesh_ = FlatMesh();
    surface_ = FlatSurface();
    result_ = FlatResult();
    atlas_ = AtlasResult();
    sourcePixels_.clear();
    baked_.clear();
    sourceW_ = sourceH_ = 0;
    textureMaterial_ = -1;
    bakedResolution_ = 0;
    resultValid_ = false;
    loaded_ = false;
}

bool HugeSession::setMesh(const float* pos, size_t nV, const uint32_t* idx, size_t nF, std::string* error)
{
    if (nF == 0) { if (error) *error = "the mesh has no faces"; return false; }
    for (size_t i = 0; i < nF * 3; ++i)
        if (idx[i] >= nV) { if (error) *error = "a face refers to a vertex that does not exist"; return false; }
    const std::string path = path_;
    clear();
    path_ = path.empty() ? std::string("<arrays>") : path;
    mesh_.pos.assign(pos, pos + nV * 3);
    mesh_.idx.assign(idx, idx + nF * 3);
    transform_ = normalizeFlatToUnitBox(mesh_);
    loaded_ = true;
    return true;
}

bool HugeSession::setTexture(const std::vector<float>& cornerUV, int w, int h, const unsigned char* rgba, std::string* error)
{
    if (cornerUV.size() != mesh_.faceCount() * 6 || w <= 0 || h <= 0 || !rgba) { if (error) *error = "the texture needs 6 uv values per face and pixels"; return false; }
    surface_.cornerUV = cornerUV;
    surface_.faceMaterial.clear();
    sourcePixels_.assign(rgba, rgba + size_t(w) * h * 4);
    sourceW_ = w;
    sourceH_ = h;
    textureMaterial_ = -1;
    return true;
}

bool HugeSession::load(std::string* error)
{
    const std::string path = path_;
    clear();
    path_ = path;
    if (!loadFlatMesh(path_, mesh_, nullptr, error, &surface_)) return false;
    transform_ = normalizeFlatToUnitBox(mesh_);
    // the texture of the file: the first material with a map_Kd (OBJ) or the TextureFile comment (PLY)
    if (surface_.hasUV()) {
        std::string name = surface_.textureFile;
        for (size_t i = 0; i < surface_.materials.size() && name.empty(); ++i)
            if (!surface_.materials[i].diffuseMap.empty()) { name = surface_.materials[i].diffuseMap; textureMaterial_ = int(i); }
        if (!name.empty() && !loadImageRGBA(surface_.directory + "/" + name, sourceW_, sourceH_, sourcePixels_, nullptr)) sourcePixels_.clear();
    }
    loaded_ = true;
    return true;
}

bool HugeSession::run(const SessionSettings& s, const Progress& progress, std::string* error, const std::atomic<bool>* cancel)
{
    try {
        return runImpl(s, progress, error, cancel);
    } catch (const Cancelled&) {
        resultValid_ = false;
        if (error) *error = "cancelled";
        return false;
    }
}

bool HugeSession::runImpl(const SessionSettings& s, const Progress& progress, std::string* error, const std::atomic<bool>* cancel)
{
    using Clock = std::chrono::steady_clock;
    const auto t0 = Clock::now();
    auto say = [&](const std::string& what) { if (progress) progress(what); };
    if (!loaded_) {
        say("Reading the mesh");
        if (!load(error)) return false;
    } else {
        result_ = FlatResult();   // the file is already in memory: only an earlier result is dropped
        atlas_ = AtlasResult();
        baked_.clear();
        bakedResolution_ = 0;
        resultValid_ = false;
    }

    // the quality: how many patches to start from and how fine the simplified copy is
    static const double kFactor[3] = {0.5, 1.0, 2.0}, kPercent[3] = {5.0, 10.0, 20.0};
    const int q = std::max(0, std::min(2, s.quality));
    const double base = 0.06 * std::sqrt(double(std::max<size_t>(mesh_.faceCount(), 1)));
    CoarseOptions co;
    co.k = s.manualPatches ? std::max(1, s.patches) : int(std::max(8.0, std::min(1000.0, std::round(base * kFactor[q]))));
    co.simplifyPercent = s.dev.simplifyPercent > 0.0 ? s.dev.simplifyPercent : kPercent[q];
    co.lloyd.maxIterations = s.dev.lloydIterations;
    co.smooth.lambda = s.dev.smoothing;
    co.normals.iterations = s.dev.normalIterations;
    co.normals.sigmaDegrees = s.dev.normalSigma;
    co.refine.maxDepth = s.dev.maxDepth;
    co.refine.splitAtOnceAboveFaces = s.dev.splitAtOnce;
    co.refine.param = s.dev.paramOptions();
    co.refine.minFaces = s.dev.minFaces;
    co.refine.maxRounds = s.dev.maxRounds;
    co.refine.policy = s.dev.splitImmediately ? RefinePolicy::SplitImmediately : RefinePolicy::FixFirst;
    k_ = size_t(co.k);
    co.onStage = [&](const std::string& what) { say(what); };
    co.cancel = cancel;
    co.refine.cancel = cancel;
    throwIfCancelled(cancel);
    result_ = runFlatPipeline(mesh_, co);
    throwIfCancelled(cancel);

    if (!packAndBake(s, progress, error)) return false;
    seconds_ = std::chrono::duration<double>(Clock::now() - t0).count();
    return true;
}

bool HugeSession::packAndBake(const SessionSettings& s, const Progress& progress, std::string* error)
{
    auto say = [&](const std::string& what) { if (progress) progress(what); };
    say("Packing the atlas");
    HorizonOptions ho;
    ho.resolution = s.resolution;
    ho.maxMipLevel = s.maxMipLevel;
    atlas_ = packFlatAtlas(result_, ho);   // an earlier packing is undone first
    if (!atlas_.ok) { if (error) *error = "packing failed: " + atlas_.message; return false; }
    resultValid_ = true;
    baked_.clear();
    bakedResolution_ = 0;
    if (!sourcePixels_.empty() && surface_.cornerUV.size() == mesh_.faceCount() * 6) {
        say("Baking the texture");
        BakeSource src{sourceW_, sourceH_, sourcePixels_.data()};
        BakeOptions bo;
        bo.resolution = atlas_.resolution;
        bo.margin = atlas_.margin;
        baked_ = bakeAtlasFlat(mesh_, result_, surface_.cornerUV, surface_.faceMaterial, textureMaterial_, &src, bo);
        bakedResolution_ = bo.resolution;
    }
    return true;
}

bool HugeSession::repack(const SessionSettings& s, const Progress& progress, std::string* error)
{
    if (!resultValid_) { if (error) *error = "there is no atlas to pack again"; return false; }
    return packAndBake(s, progress, error);
}

std::vector<unsigned char> HugeSession::patchRgba() const
{
    std::vector<unsigned char> rgba(result_.patchOfFace.size() * 4, 255);
    std::vector<unsigned char> palette(result_.patches.size() * 3);
    for (size_t p = 0; p < result_.patches.size(); ++p) {
        float c[3];
        AtlasSession::patchColor(int(p), c);
        for (int k = 0; k < 3; ++k) palette[3 * p + k] = (unsigned char)(255.0f * c[k]);
    }
    for (size_t f = 0; f < result_.patchOfFace.size(); ++f) {
        const int p = result_.patchOfFace[f];
        if (p < 0) { rgba[4 * f] = 255; rgba[4 * f + 1] = 60; rgba[4 * f + 2] = 60; continue; }   // faces without a patch: red
        for (int k = 0; k < 3; ++k) rgba[4 * f + k] = palette[3 * size_t(p) + k];
    }
    return rgba;
}

std::vector<float> HugeSession::patchNormalsPerFace() const { return patchNormals(result_.patchOfFace, mesh_); }

std::vector<UvChart> HugeSession::uvCharts() const
{
    std::vector<UvChart> out;
    for (size_t i = 0; i < result_.patches.size() && i < atlas_.uv.size(); ++i) {
        if (atlas_.uv[i].empty()) continue;
        UvChart c;
        c.patch = int(i);
        for (const auto& q : atlas_.uv[i]) { c.uv.push_back(float(q.x())); c.uv.push_back(float(q.y())); }
        for (const auto& e : result_.patches[i].boundaryEdges) { c.segments.push_back(unsigned(e.first)); c.segments.push_back(unsigned(e.second)); }
        out.push_back(std::move(c));
    }
    return out;
}

bool HugeSession::save(const std::string& requested, std::string* error) const
{
    if (!resultValid_) { if (error) *error = "there is no atlas to save"; return false; }
    std::string path = requested;
    if (path.size() < 4 || path.substr(path.size() - 4) != ".ply") path += ".ply";
    std::string textureFile;
    if (!baked_.empty()) {   // the baked atlas next to the mesh; image row 0 is v = 1
        const size_t slash = path.find_last_of('/');
        const std::string dir = slash == std::string::npos ? "." : path.substr(0, slash);
        const std::string stem = path.substr(slash == std::string::npos ? 0 : slash + 1, path.size() - (slash == std::string::npos ? 0 : slash + 1) - 4);
        textureFile = stem + "_atlas.png";
        const int R = bakedResolution_;
        std::vector<unsigned char> flipped(baked_.size());
        const size_t row = size_t(R) * 4;
        for (int y = 0; y < R; ++y)
            std::copy(baked_.begin() + size_t(R - 1 - y) * row, baked_.begin() + size_t(R - y) * row, flipped.begin() + size_t(y) * row);
        std::string pngError;
        if (!writePNG(dir + "/" + textureFile, R, R, flipped, &pngError)) { if (error) *error = "texture: " + pngError; return false; }
    }
    return writeFlatPlyAtlas(path, mesh_, result_, &transform_, error, textureFile);
}

std::string HugeSession::report() const
{
    if (!resultValid_) return "";
    const FlatStats& st = result_.stats;
    const CoarseStats& c = st.coarse;
    char buf[400];
    std::string out;
    std::snprintf(buf, sizeof(buf), "faces %zu, started from %zu patches; simplified copy %zu faces, %d coarse patches (%d bad at the start, %d splits, %d unresolved)\n",
                  mesh_.faceCount(), k_, c.coarseFaces, c.coarsePatches, c.coarseInitialBad, c.coarseSplits, c.coarseUnresolved);
    out += buf;
    std::snprintf(buf, sizeof(buf), "pieces %zu: lifted %zu, through the ordinary pipeline %zu; patches %zu (%zu unresolved, %zu repaired), faces without uv %zu\n",
                  st.pieces, st.liftedPieces, st.fallbackPieces, result_.patches.size(), st.patchesUnresolved, st.patchesRepaired, st.facesWithoutUV);
    out += buf;
    std::snprintf(buf, sizeof(buf), "seconds: simplify %.1f, coarse VSA %.1f, coarse refinement %.1f, adjacency %.1f, cleaning %.1f, lift %.1f, ordinary pipeline %.1f\n",
                  c.simplifySeconds, c.coarseVsaSeconds, c.coarseRefineSeconds, st.adjacencySeconds, st.cleanSeconds, st.liftSeconds, st.fallbackSeconds);
    out += buf;
    std::snprintf(buf, sizeof(buf), "atlas %d x %d, used %.1f%%, total %.1f s\n", atlas_.resolution, atlas_.resolution, 100.0 * atlas_.efficiency, seconds_);
    out += buf;
    return out;
}

std::string HugeSession::compare(std::string* error) const
{
    LayoutComparison c;
    if (!compareData(&c, error)) return "";
    return formatLayoutComparison(c.original, c.ours, c.mipOriginal, c.mipOurs, c.hadOriginal);
}

bool HugeSession::matchOriginal(int* resolution, int* mip, std::string* text) const
{
    if (!loaded_ || surface_.cornerUV.size() != mesh_.faceCount() * 6) { if (text) *text = "the mesh has no texture coordinates to match"; return false; }
    int R = 256;
    while (R < std::max(sourceW_, sourceH_) && R < 8192) R *= 2;
    std::vector<float> original = surface_.cornerUV;
    if (surface_.faceMaterial.size() == mesh_.faceCount())
        for (size_t f = 0; f < mesh_.faceCount(); ++f)
            if (surface_.faceMaterial[f] != textureMaterial_)
                for (int k = 0; k < 6; ++k) original[f * 6 + k] = std::nanf("");
    const MipSafetyResult m = measureMipSafety(mesh_, buildFaceNeighbors(mesh_), original, R, true, 7, false);
    *resolution = R;
    *mip = std::max(0, mipLevelForGap(std::max(0, m.gapP05)));
    char buf[300];
    std::snprintf(buf, sizeof buf, "Original layout: %d charts, gap between charts min %d / 5%% %d / median %d texels (at %d x %d). Atlas set to %d x %d, mip level %d.",
                  m.charts, m.gapMin, m.gapP05, m.gapP50, R, R, R, R, *mip);
    if (text) *text = buf;
    return true;
}

bool HugeSession::compareData(LayoutComparison* out, std::string* error) const
{
    if (!resultValid_) { if (error) *error = "make the atlas first"; return false; }
    const int R = std::max(256, atlas_.resolution);
    const std::vector<int32_t> nbr = buildFaceNeighbors(mesh_);
    std::vector<float> ours = result_.cornerUV, original = surface_.cornerUV;
    const bool hadOriginal = original.size() == mesh_.faceCount() * 6;
    const bool masked = surface_.faceMaterial.size() == mesh_.faceCount();
    for (size_t f = 0; masked && f < mesh_.faceCount(); ++f)
        if (surface_.faceMaterial[f] != textureMaterial_)
            for (int k = 0; k < 6; ++k) {
                ours[f * 6 + k] = std::nanf("");
                if (hadOriginal) original[f * 6 + k] = std::nanf("");
            }
    UVLayoutStats lo, ln = analyzeUVLayout(mesh_, nbr, ours, R);
    MipSafetyResult mo, mn = measureMipSafety(mesh_, nbr, ours, R, true, 7, false);
    if (hadOriginal) {
        lo = analyzeUVLayout(mesh_, nbr, original, R);
        mo = measureMipSafety(mesh_, nbr, original, R, true, 7, false);
    }
    *out = LayoutComparison{lo, ln, mo, mn, hadOriginal};
    return true;
}

Preset HugeSession::preset(const SessionSettings& s) const
{
    Preset p;
    p.k = int(k_);
    p.lloydMaxIterations = s.dev.lloydIterations;
    p.smoothLambda = s.dev.smoothing;
    p.normalSmoothIterations = s.dev.normalIterations;
    p.normalSmoothSigma = s.dev.normalSigma;
    p.param.allowLscm = s.dev.allowLscm;
    p.param.repairNearMisses = s.dev.repairNearMisses;
    p.refineMaxDepth = s.dev.maxDepth;
    p.refineSplitAtOnceAboveFaces = s.dev.splitAtOnce;
    p.refineSmoothLambda = s.dev.smoothing;
    return p;
}

std::vector<PatchRow> HugeSession::patchRows() const
{
    std::vector<PatchRow> rows;
    if (!resultValid_) return rows;
    for (size_t i = 0; i < result_.patches.size(); ++i) {
        const FlatPatch& p = result_.patches[i];
        PatchRow r;
        r.index = int(i);
        r.faces = p.faces.size();
        r.method = paramMethodName(p.method);
        r.l2Mean = p.distortion.l2Mean;
        r.badArea = p.badAreaFraction;
        r.status = p.method == ParamMethod::None ? "no uv" : !p.injective ? (p.distortion.flipped ? "flipped" : "overlap") : p.needsSplit ? "stretch" : "ok";
        r.bad = r.status != "ok";
        rows.push_back(std::move(r));
    }
    return rows;
}

bool HugeSession::patchFrame(int patch, PatchFrame* frame) const
{
    if (!resultValid_ || patch < 0 || size_t(patch) >= result_.patches.size()) return false;
    const FlatPatch& p = result_.patches[size_t(patch)];
    // a sample of the faces is enough for the centre, the normal and the extent of a patch of millions of triangles
    const size_t stride = std::max<size_t>(1, p.faces.size() / 60000);
    std::vector<PatchTriangle> tris;
    for (size_t i = 0; i < p.faces.size(); i += stride) {
        PatchTriangle t;
        for (int c = 0; c < 3; ++c) {
            const size_t v = mesh_.idx[3 * size_t(p.faces[i]) + size_t(c)];
            for (int k = 0; k < 3; ++k) t.p[c][k] = mesh_.pos[3 * v + size_t(k)];
        }
        tris.push_back(t);
    }
    *frame = computePatchFrame(tris);
    if (frame->ok) frame->area *= double(stride);
    return frame->ok;
}

bool HugeSession::patchBorder(int patch, std::vector<float>* segments) const
{
    if (!resultValid_ || patch < 0 || size_t(patch) >= result_.patches.size()) return false;
    const FlatPatch& p = result_.patches[size_t(patch)];
    std::unordered_map<uint64_t, std::pair<std::pair<uint32_t, uint32_t>, int>> edges;
    for (uint32_t f : p.faces)
        for (int k = 0; k < 3; ++k) {
            const uint32_t a = mesh_.idx[3 * size_t(f) + size_t(k)], b = mesh_.idx[3 * size_t(f) + size_t((k + 1) % 3)];
            auto it = edges.emplace((uint64_t(std::min(a, b)) << 32) | std::max(a, b), std::make_pair(std::make_pair(a, b), 0)).first;
            ++it->second.second;
        }
    segments->clear();
    for (const auto& e : edges)
        if (e.second.second == 1)
            for (uint32_t v : {e.second.first.first, e.second.first.second})
                for (int k = 0; k < 3; ++k) segments->push_back(mesh_.pos[3 * size_t(v) + size_t(k)]);
    return true;
}
