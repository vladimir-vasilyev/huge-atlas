#include "native/Session.h"

#include "app/Preset.h"
#include "core/MeshPrep.h"
#include "core/Operations.h"
#include "param/AtlasBake.h"
#include "param/AtlasMask.h"
#include "param/HorizonPacker.h"
#include "param/LayoutReport.h"
#include "param/MipSafety.h"
#include "param/UVMetrics.h"
#include "param/MeshWriters.h"
#include "render/ImageFile.h"
#include "render/MeshRenderer.h"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cmath>

void AtlasSession::patchColor(int index, float* rgb)
{
    const float h = std::fmod(float(index) * 0.61803398875f, 1.0f), s = 0.6f, v = 0.92f;
    const float c = v * s, x = c * (1.0f - std::abs(std::fmod(h * 6.0f, 2.0f) - 1.0f)), m = v - c;
    float r = 0, g = 0, b = 0;
    switch (int(h * 6.0f) % 6) {
        case 0: r = c; g = x; break;
        case 1: r = x; g = c; break;
        case 2: g = c; b = x; break;
        case 3: g = x; b = c; break;
        case 4: r = x; b = c; break;
        default: r = c; b = x; break;
    }
    rgb[0] = r + m; rgb[1] = g + m; rgb[2] = b + m;
}

int AtlasSession::patchesFor(size_t faces, int quality)
{
    static const double kFactor[3] = {0.5, 1.0, 2.0};
    const double base = 0.06 * std::sqrt(double(std::max<size_t>(faces, 1)));
    return int(std::max(8.0, std::min(1000.0, std::round(base * kFactor[std::max(0, std::min(2, quality))]))));
}

bool AtlasSession::setMesh(const float* pos, size_t nV, const uint32_t* idx, size_t nF, std::string* error)
{
    if (nF == 0) { if (error) *error = "the mesh has no faces"; return false; }
    TriMesh m;
    std::vector<TriMesh::VertexHandle> vh(nV);
    for (size_t i = 0; i < nV; ++i) vh[i] = m.add_vertex(Vec3(pos[3 * i], pos[3 * i + 1], pos[3 * i + 2]));
    for (size_t f = 0; f < nF; ++f) {
        for (int k = 0; k < 3; ++k)
            if (idx[3 * f + k] >= nV) { if (error) *error = "a face refers to a vertex that does not exist"; return false; }
        if (!m.add_face(vh[idx[3 * f]], vh[idx[3 * f + 1]], vh[idx[3 * f + 2]]).is_valid()) {
            if (error) *error = "face " + std::to_string(f) + " cannot be added (a non-manifold edge, or a repeated vertex); fix the mesh or use the huge mode";
            return false;
        }
    }
    normalizeToUnitBox(m);
    mesh_ = std::move(m);
    path_.clear();
    result_ = RefineResult();
    atlas_ = AtlasResult();
    resultValid_ = false;
    baked_.clear();
    bakedUV_.clear();
    extras_ = ObjExtras();
    sourcePixels_.clear();
    sourceW_ = sourceH_ = 0;
    textureMaterial_ = -1;
    return true;
}

bool AtlasSession::setTexture(const std::vector<float>& cornerUV, int w, int h, const unsigned char* rgba, std::string* error)
{
    if (cornerUV.size() != mesh_.n_faces() * 6 || w <= 0 || h <= 0 || !rgba) { if (error) *error = "the texture needs 6 uv values per face and pixels"; return false; }
    extras_.cornerUV = cornerUV;
    extras_.faceMaterial.clear();
    sourcePixels_.assign(rgba, rgba + size_t(w) * h * 4);
    sourceW_ = w;
    sourceH_ = h;
    textureMaterial_ = -1;
    return true;
}

std::vector<float> AtlasSession::cornerUVOut() const
{
    if (!resultValid_) return {};
    std::vector<float> uv = bakedUV_;
    for (float& x : uv) if (x < 0.0f) x = std::nanf("");
    return uv;
}

bool AtlasSession::load(const std::string& path, std::string* error)
{
    TriMesh m;
    ObjExtras extras;
    std::string ext = path.size() > 4 ? path.substr(path.size() - 4) : "";
    for (char& c : ext) c = char(std::tolower(static_cast<unsigned char>(c)));
    const bool isObj = ext == ".obj";
    if (isObj ? !loadObj(path, m, &extras, error) : !loadMesh(path, m, error)) return false;
    if (m.n_faces() == 0) { if (error) *error = "the file has no faces"; return false; }
    normalizeToUnitBox(m);
    mesh_ = std::move(m);
    path_ = path;
    result_ = RefineResult();
    atlas_ = AtlasResult();
    resultValid_ = false;
    baked_.clear();
    bakedUV_.clear();

    // the texture of the file: the first material with a diffuse map (the faces of other materials stay gray)
    extras_ = std::move(extras);
    sourcePixels_.clear();
    sourceW_ = sourceH_ = 0;
    textureMaterial_ = -1;
    if (extras_.hasUV()) {
        for (size_t i = 0; i < extras_.materials.size(); ++i) {
            if (extras_.materials[i].diffuseMap.empty()) continue;
            int w = 0, h = 0;
            std::vector<unsigned char> pixels;
            if (loadImageRGBA(extras_.directory + "/" + extras_.materials[i].diffuseMap, w, h, pixels, nullptr)) {
                sourcePixels_ = std::move(pixels);
                sourceW_ = w; sourceH_ = h;
                textureMaterial_ = int(i);
            }
            break;
        }
    }
    return true;
}

// Packs the unfolded patches of result_ into an atlas and bakes the texture of the file into it (resultValid_ is set when it worked).
bool AtlasSession::packAndBake(const SessionSettings& s, std::string* error)
{
    std::vector<AtlasInput> inputs(result_.params.size());
    for (size_t i = 0; i < result_.params.size(); ++i) {
        const PatchParam& p = result_.params[i];
        if (p.method == ParamMethod::None || p.uv.empty()) continue;
        const TriMesh& pm = p.patch.mesh;
        double uvArea = 0.0, area3 = 0.0;
        for (auto f : pm.faces()) {
            Eigen::Vector2d q[3];
            Vec3 v[3];
            int c = 0;
            for (auto fv : pm.fv_range(f)) { q[c] = p.uv[fv.idx()]; v[c] = pm.point(fv); ++c; }
            uvArea += std::abs(0.5 * ((q[1] - q[0]).x() * (q[2] - q[0]).y() - (q[1] - q[0]).y() * (q[2] - q[0]).x()));
            area3 += 0.5 * ((v[1] - v[0]) % (v[2] - v[0])).norm();
        }
        inputs[i] = {&p.uv, uvArea, area3, &pm};
    }
    HorizonOptions ho;
    ho.resolution = s.resolution;
    ho.maxMipLevel = s.maxMipLevel;
    AtlasResult atlas = packHorizon(inputs, ho);
    if (!atlas.ok) { if (error) *error = "packing failed: " + atlas.message; return false; }
    atlas_ = std::move(atlas);
    resultValid_ = true;

    // the atlas as texture coordinates, and the texture of the file baked into it
    std::vector<BakeChart> charts = bakeCharts();
    bakedUV_ = cornerUVFromAtlas(mesh_, charts);
    for (size_t f = 0; f < mesh_.n_faces(); ++f)
        if (std::isnan(bakedUV_[f * 6])) for (int k = 0; k < 6; ++k) bakedUV_[f * 6 + k] = -1.0f;
    baked_.clear();
    if (hasSourceTexture()) {
        BakeSource src{sourceW_, sourceH_, sourcePixels_.data()};
        BakeOptions bo;
        bo.resolution = atlas_.resolution;
        bo.margin = atlas_.margin;
        baked_ = bakeAtlasTexture(mesh_, charts, &src, extras_.cornerUV, extras_.faceMaterial, textureMaterial_, bo);
    }
    return true;
}

bool AtlasSession::repack(const SessionSettings& s, std::string* error)
{
    if (!resultValid_) { if (error) *error = "there is no atlas to pack again"; return false; }
    return packAndBake(s, error);
}

bool AtlasSession::run(const SessionSettings& s, const Progress& progress, const std::atomic<bool>& cancel, std::string* error)
{
    if (!hasMesh()) { if (error) *error = "open a mesh first"; return false; }
    const auto t0 = std::chrono::steady_clock::now();
    resultValid_ = false;
    auto say = [&](double f, const char* what) { if (progress) progress(f, what); };
    auto stop = [&]() { if (cancel) { if (error) *error = "cancelled"; return true; } return false; };

    say(0.02, "Segmenting the mesh");
    FaceData fd = computeFaceData(mesh_);
    if (s.dev.normalIterations > 0) {
        NormalSmoothing ns;
        ns.iterations = s.dev.normalIterations;
        ns.sigmaDegrees = s.dev.normalSigma;
        smoothFaceNormals(mesh_, fd, ns);
    }
    const std::vector<Moments> fm = computeFaceMoments(mesh_, fd);
    const VSAInput in{mesh_, fd, fm};
    const int k = std::min(s.manualPatches ? std::max(1, s.patches) : patchesFor(mesh_.n_faces(), s.quality), int(mesh_.n_faces()));
    VSAState vs;
    vs.metric = Metric::L21;
    vs.smooth = SmoothParams{s.dev.smoothing, 0.0, 10};
    k_ = size_t(k);
    initFarthestSeeds(in, vs, k, 12345);
    LloydParams lp;
    lp.maxIterations = s.dev.lloydIterations;
    runLloyd(in, vs, lp);
    if (stop()) return false;

    say(0.2, "Unfolding the patches");
    RefineOptions ro;
    ro.maxDepth = s.dev.maxDepth;
    ro.splitAtOnceAboveFaces = s.dev.splitAtOnce;
    ro.param.allowLscm = s.dev.allowLscm;
    ro.param.repairNearMisses = s.dev.repairNearMisses;
    Refiner refiner(mesh_, vs.label, ro);
    int rounds = 0;
    while (refiner.step()) {
        ++rounds;
        say(std::min(0.8, 0.2 + 0.1 * rounds), "Unfolding the patches");
        if (stop()) return false;
    }
    RefineResult rr = refiner.takeResult();
    if (stop()) return false;

    result_ = std::move(rr);
    say(0.85, "Packing the atlas");
    if (!packAndBake(s, error)) { resultValid_ = false; return false; }
    if (stop()) { resultValid_ = false; return false; }
    seconds_ = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    say(1.0, "Done");
    return true;
}

std::vector<float> AtlasSession::patchColors() const
{
    std::vector<float> colors(mesh_.n_faces() * 3, 0.72f);
    if (!resultValid_) return colors;
    for (size_t f = 0; f < colors.size() / 3 && f < result_.labels.size(); ++f)
        if (result_.labels[f] >= 0) patchColor(result_.labels[f], &colors[3 * f]);
    return colors;
}

std::vector<float> AtlasSession::patchNormalsPerFace() const
{
    if (!resultValid_) return {};
    const FaceData fd = computeFaceData(mesh_);
    return patchNormals(result_.labels, fd.area, fd.normal);
}

std::vector<BakeChart> AtlasSession::bakeCharts() const
{
    std::vector<BakeChart> charts;
    for (size_t i = 0; i < result_.params.size(); ++i)
        if (i < atlas_.uv.size() && !atlas_.uv[i].empty()) charts.push_back({&result_.params[i].patch, &atlas_.uv[i]});
    return charts;
}

std::vector<UvChart> AtlasSession::uvCharts() const
{
    std::vector<UvChart> out;
    if (!resultValid_) return out;
    for (size_t i = 0; i < result_.params.size(); ++i) {
        if (i >= atlas_.uv.size() || atlas_.uv[i].empty()) continue;
        const Patch& p = result_.params[i].patch;
        UvChart c;
        c.patch = int(i);
        for (const auto& q : atlas_.uv[i]) { c.uv.push_back(float(q.x())); c.uv.push_back(float(q.y())); }
        for (auto f : p.mesh.faces())
            for (auto v : p.mesh.fv_range(f)) c.triangles.push_back(unsigned(v.idx()));
        for (const auto& loop : p.topology.loops) {
            std::vector<unsigned> l;
            for (int v : loop) l.push_back(unsigned(v));
            c.loops.push_back(std::move(l));
        }
        out.push_back(std::move(c));
    }
    return out;
}

bool AtlasSession::save(const std::string& requested, std::string* error) const
{
    if (!resultValid_) { if (error) *error = "there is no atlas to save"; return false; }
    std::string path = requested;
    std::string lower = path;
    for (char& c : lower) c = char(std::tolower(static_cast<unsigned char>(c)));
    const bool endsObj = lower.size() > 4 && lower.compare(lower.size() - 4, 4, ".obj") == 0;
    const bool endsPly = lower.size() > 4 && lower.compare(lower.size() - 4, 4, ".ply") == 0;
    if (!endsObj && !endsPly) path += ".obj";

    const size_t slash = path.find_last_of('/');
    const std::string dir = slash == std::string::npos ? "." : path.substr(0, slash);
    std::string stem = path.substr(slash == std::string::npos ? 0 : slash + 1);
    stem.resize(stem.size() - 4);
    const std::string textureName = stem + "_atlas.png";

    // the texture: the baked one, or a flat color per chart when the mesh has no texture
    const int res = atlas_.resolution > 0 ? atlas_.resolution : 2048;
    std::vector<unsigned char> rgba = baked_;
    if (rgba.empty()) {
        BakeOptions bo;
        bo.resolution = res;
        bo.margin = atlas_.margin;
        rgba = bakeAtlasTexture(mesh_, bakeCharts(), nullptr, {}, {}, -1, bo);
    }
    std::vector<unsigned char> flipped(rgba.size());   // image row 0 is v = 1
    const size_t row = size_t(res) * 4;
    for (int y = 0; y < res; ++y)
        std::copy(rgba.begin() + size_t(res - 1 - y) * row, rgba.begin() + size_t(res - y) * row, flipped.begin() + size_t(y) * row);
    if (!writePNG(dir + "/" + textureName, res, res, flipped, error)) return false;

    std::vector<const Patch*> pv;
    std::vector<const std::vector<Eigen::Vector2d>*> uvv;
    for (size_t i = 0; i < result_.params.size(); ++i) {
        pv.push_back(&result_.params[i].patch);
        uvv.push_back(i < atlas_.uv.size() && !atlas_.uv[i].empty() ? &atlas_.uv[i] : nullptr);
    }
    char meta[200];
    std::snprintf(meta, sizeof(meta), "atlas resolution=%d maxMipLevel=%d gutter=%d margin=%d texelsPerUnit=%.6g", atlas_.resolution,
                  atlas_.maxMipLevel, atlas_.gutter, atlas_.margin, atlas_.texelsPerUnit);
    const int faces = endsPly ? writePlyAtlas(path, pv, uvv, textureName, true, meta) : exportAtlasOBJ(path, mesh_, pv, uvv, meta, textureName);
    if (faces < 0) { if (error) *error = "cannot write " + path; return false; }
    return true;
}

std::string AtlasSession::report() const
{
    if (!resultValid_) return "";
    char buf[300];
    std::string out;
    const RefineStats& st = result_.stats;
    std::snprintf(buf, sizeof(buf), "faces %zu, started from %zu patches, ended with %zu (%d unresolved)\n", mesh_.n_faces(), k_,
                  result_.params.size(), st.unresolved);
    out += buf;
    std::snprintf(buf, sizeof(buf), "refinement: %d rounds, %d bad at the start, %d splits (peel %d, normals %d, crossings %d, farthest %d, flood %d, voronoi %d)\n",
                  st.rounds, st.initialBad, st.splits, st.splitsByPeel, st.splitsByNormals, st.splitsByCrossingHint, st.splitsByFarthest,
                  st.splitsByFlood, st.splitsByVoronoi);
    out += buf;
    int count[8] = {0};
    for (const PatchParam& p : result_.params) ++count[int(p.method)];
    out += "unfolded by:";
    for (int m = 1; m < 8; ++m) {
        if (!count[m]) continue;
        std::snprintf(buf, sizeof(buf), " %s %d,", paramMethodName(ParamMethod(m)), count[m]);
        out += buf;
    }
    out += "\n";
    if (!st.depthHistogram.empty()) {
        out += "depth of splitting:";
        for (size_t d = 0; d < st.depthHistogram.size(); ++d) { std::snprintf(buf, sizeof(buf), " %zu:%d", d, st.depthHistogram[d]); out += buf; }
        out += "\n";
    }
    std::snprintf(buf, sizeof(buf), "atlas %d x %d, gutter %d, used %.1f%%, total %.1f s\n", atlas_.resolution, atlas_.resolution, atlas_.gutter,
                  100.0 * atlas_.efficiency, seconds_);
    out += buf;
    return out;
}

std::string AtlasSession::compare(std::string* error) const
{
    LayoutComparison c;
    if (!compareData(&c, error)) return "";
    return formatLayoutComparison(c.original, c.ours, c.mipOriginal, c.mipOurs, c.hadOriginal);
}

bool AtlasSession::compareData(LayoutComparison* out, std::string* error) const
{
    if (!resultValid_) { if (error) *error = "make the atlas first"; return false; }
    const int R = std::max(256, atlas_.resolution);
    const bool masked = extras_.faceMaterial.size() == mesh_.n_faces();   // only the faces of the texture's material
    const bool hadOriginal = extras_.cornerUV.size() == mesh_.n_faces() * 6;
    std::vector<float> original = extras_.cornerUV, ours = bakedUV_;
    for (float& x : ours) if (x < 0.0f) x = std::nanf("");   // -1 marked "no uv"
    for (size_t f = 0; masked && f < mesh_.n_faces(); ++f)
        if (extras_.faceMaterial[f] != textureMaterial_)
            for (int k = 0; k < 6; ++k) {
                ours[f * 6 + k] = std::nanf("");
                if (hadOriginal) original[f * 6 + k] = std::nanf("");
            }
    UVLayoutStats lo, ln = analyzeUVLayout(mesh_, ours, R);
    MipSafetyResult mo, mn = measureMipSafety(mesh_, ours, R, true, 7, false);
    if (hadOriginal) {
        lo = analyzeUVLayout(mesh_, original, R);
        mo = measureMipSafety(mesh_, original, R, true, 7, false);
    }
    *out = LayoutComparison{lo, ln, mo, mn, hadOriginal};
    return true;
}

Preset AtlasSession::preset(const SessionSettings& s) const
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

bool AtlasSession::loadBadPatch(const std::string& path, std::string* text, std::string* error)
{
    TriMesh m;
    if (!loadObj(path, m, nullptr, error)) return false;
    if (m.n_faces() == 0) { if (error) *error = "the file has no faces"; return false; }
    normalizeToUnitBox(m);
    mesh_ = std::move(m);
    path_ = path;
    result_ = RefineResult();
    atlas_ = AtlasResult();
    resultValid_ = false;
    baked_.clear();
    bakedUV_.clear();
    extras_ = ObjExtras();
    sourcePixels_.clear();
    sourceW_ = sourceH_ = 0;
    badLoaded_ = false;
    bad_ = PatchParam();

    std::string note = "no preset next to the file: the default settings are used";
    const size_t dot = path.find_last_of('.');
    const std::string presetPath = (dot == std::string::npos ? path : path.substr(0, dot)) + ".preset";
    Preset preset;
    std::string perr;
    if (readPreset(presetPath, preset, &perr)) note = "preset applied: " + presetPath;

    PatchOptions po;
    po.minFaces = preset.patchMinFaces;
    po.cutMultiBoundary = preset.patchCutMultiBoundary;
    po.cutHandles = preset.patchCutHandles;
    ExtractionResult ex = extractPatches(mesh_, std::vector<int>(mesh_.n_faces(), 0), po);   // the whole mesh is one region
    char buf[600];
    if (ex.patches.empty() || !ex.patches[0].isDisk() || ex.patches[0].mesh.n_faces() == 0) {
        std::snprintf(buf, sizeof buf, "bad patch loaded (%zu faces): %s. It is %s: nothing to unfold", mesh_.n_faces(), note.c_str(),
                      ex.patches.empty() ? "empty" : patchKindName(ex.patches[0].topology.kind));
        if (text) *text = buf;
        badLoaded_ = false;
        return true;
    }
    bad_ = parameterizePatch(mesh_, ex.patches[0], preset.param);
    badLoaded_ = bad_.method != ParamMethod::None && bad_.uv.size() == bad_.patch.mesh.n_vertices();
    if (badLoaded_)
        std::snprintf(buf, sizeof buf, "bad patch loaded (%zu faces): %s. Unfolded with %s: %s, %d flipped, %zu crossings, %zu folds, L2 mean %.3f, bad area %.4f%s",
                      mesh_.n_faces(), note.c_str(), paramMethodName(bad_.method), bad_.injective ? "injective" : "NOT injective", bad_.distortion.flipped,
                      bad_.report.uv.crossings.size(), bad_.report.angles.foldVertices.size(), bad_.distortion.l2Mean, bad_.badAreaFraction,
                      bad_.needsSplit ? ", needs a split" : "");
    else
        std::snprintf(buf, sizeof buf, "bad patch loaded (%zu faces): %s. Nothing came out of the ladder: %s", mesh_.n_faces(), note.c_str(), bad_.log.c_str());
    if (text) *text = buf;
    return true;
}

UvChart AtlasSession::badPatchChart() const
{
    UvChart c;
    if (!badLoaded_) return c;
    const TriMesh& pm = bad_.patch.mesh;
    double lo[2] = {1e300, 1e300}, hi[2] = {-1e300, -1e300};
    for (const auto& q : bad_.uv)
        for (int k = 0; k < 2; ++k) { lo[k] = std::min(lo[k], q[k]); hi[k] = std::max(hi[k], q[k]); }
    const double extent = std::max(hi[0] - lo[0], hi[1] - lo[1]), s = extent > 0 ? 0.9 / extent : 1.0;
    for (const auto& q : bad_.uv) {
        c.uv.push_back(float(0.05 + (q.x() - lo[0]) * s));
        c.uv.push_back(float(0.05 + (q.y() - lo[1]) * s));
    }
    for (auto f : pm.faces())
        for (auto v : pm.fv_range(f)) c.triangles.push_back(unsigned(v.idx()));
    for (int f : bad_.report.uv.flippedFaces) if (f >= 0) c.flipped.push_back(unsigned(f));
    for (const auto& pr : bad_.report.uv.crossings)
        for (int h : {pr.first, pr.second}) {
            const auto he = pm.halfedge_handle(h);
            c.crossings.push_back(unsigned(pm.from_vertex_handle(he).idx()));
            c.crossings.push_back(unsigned(pm.to_vertex_handle(he).idx()));
        }
    for (const auto& loop : bad_.patch.topology.loops) {
        std::vector<unsigned> l;
        for (int v : loop) l.push_back(unsigned(v));
        c.loops.push_back(std::move(l));
    }
    return c;
}

std::vector<float> AtlasSession::badPatchColors() const
{
    std::vector<float> colors(mesh_.n_faces() * 3, 0.72f);
    if (!badLoaded_) return colors;
    const double maxStretch = 4.0;
    for (size_t f = 0; f < bad_.patch.faceToGlobal.size() && f < bad_.perTriangleL2.size(); ++f) {
        const int g = bad_.patch.faceToGlobal[f];
        if (g < 0 || size_t(g) >= mesh_.n_faces()) continue;
        const float t = float(std::max(0.0, std::min(1.0, (double(bad_.perTriangleL2[f]) - 1.0) / (maxStretch - 1.0))));
        colors[3 * size_t(g)] = 0.25f + 0.7f * t;
        colors[3 * size_t(g) + 1] = 0.55f * (1.0f - t) + 0.2f * t;
        colors[3 * size_t(g) + 2] = 0.9f * (1.0f - t) + 0.2f * t;
    }
    for (int f : bad_.report.uv.flippedFaces)
        if (f >= 0 && size_t(f) < bad_.patch.faceToGlobal.size()) {
            const int g = bad_.patch.faceToGlobal[f];
            if (g >= 0 && size_t(g) < mesh_.n_faces()) { colors[3 * size_t(g)] = 1.0f; colors[3 * size_t(g) + 1] = 0.1f; colors[3 * size_t(g) + 2] = 0.1f; }
        }
    return colors;
}

std::vector<PatchRow> AtlasSession::patchRows() const
{
    std::vector<PatchRow> rows;
    if (!resultValid_) return rows;
    for (size_t i = 0; i < result_.params.size(); ++i) {
        const PatchParam& p = result_.params[i];
        PatchRow r;
        r.index = int(i);
        r.faces = p.patch.faceToGlobal.size();
        r.method = paramMethodName(p.method);
        r.l2Mean = p.distortion.l2Mean;
        r.badArea = p.badAreaFraction;
        if (!p.patch.isDisk()) r.status = "not a disk";
        else if (p.method == ParamMethod::None || p.uv.empty()) r.status = "no uv";
        else if (!p.injective) r.status = p.distortion.flipped ? "flipped" : "overlap";
        else if (p.needsSplit) r.status = "stretch";
        else r.status = "ok";
        r.bad = r.status != "ok";
        rows.push_back(std::move(r));
    }
    return rows;
}

std::string AtlasSession::meshInfo() const
{
    if (mesh_.n_faces() == 0) return "No mesh";
    const MeshDiagnostics d = diagnose(mesh_, computeFaceData(mesh_));
    char buf[400];
    std::snprintf(buf, sizeof buf,
                  "Vertices          %d\nEdges             %d\nFaces             %d\nEuler characteristic %d\nComponents        %d\n"
                  "Boundary loops    %d\nTotal area        %.4f\nDegenerate faces  %d\nNon-manifold verts %d\nIsolated verts    %d",
                  d.vertices, d.edges, d.faces, d.eulerCharacteristic, d.components, d.boundaryLoops, d.totalArea, d.degenerateFaces,
                  d.nonManifoldVertices, d.isolatedVertices);
    return buf;
}
