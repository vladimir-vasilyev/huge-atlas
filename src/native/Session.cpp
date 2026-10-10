#include "native/Session.h"

#include "core/MeshPrep.h"
#include "core/Operations.h"
#include "param/AtlasBake.h"
#include "param/HorizonPacker.h"
#include "render/ImageFile.h"
#include "render/MeshRenderer.h"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cmath>

namespace {

void hueColor(int index, float* rgb)
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

} // namespace

int AtlasSession::patchesFor(size_t faces, int quality)
{
    static const double kFactor[3] = {0.5, 1.0, 2.0};
    const double base = 0.06 * std::sqrt(double(std::max<size_t>(faces, 1)));
    return int(std::max(8.0, std::min(1000.0, std::round(base * kFactor[std::max(0, std::min(2, quality))]))));
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

bool AtlasSession::run(const SessionSettings& s, const Progress& progress, const std::atomic<bool>& cancel, std::string* error)
{
    if (!hasMesh()) { if (error) *error = "open a mesh first"; return false; }
    const auto t0 = std::chrono::steady_clock::now();
    resultValid_ = false;
    auto say = [&](double f, const char* what) { if (progress) progress(f, what); };
    auto stop = [&]() { if (cancel) { if (error) *error = "cancelled"; return true; } return false; };

    say(0.02, "Segmenting the mesh");
    const FaceData fd = computeFaceData(mesh_);
    const std::vector<Moments> fm = computeFaceMoments(mesh_, fd);
    const VSAInput in{mesh_, fd, fm};
    const int k = std::min(s.manualPatches ? std::max(1, s.patches) : patchesFor(mesh_.n_faces(), s.quality), int(mesh_.n_faces()));
    VSAState vs;
    vs.metric = Metric::L21;
    vs.smooth = SmoothParams{0.1, 0.0, 10};
    initFarthestSeeds(in, vs, k, 12345);
    runLloyd(in, vs, LloydParams());
    if (stop()) return false;

    say(0.2, "Unfolding the patches");
    RefineOptions ro;
    ro.maxDepth = 10;
    Refiner refiner(mesh_, vs.label, ro);
    int rounds = 0;
    while (refiner.step()) {
        ++rounds;
        say(std::min(0.8, 0.2 + 0.1 * rounds), "Unfolding the patches");
        if (stop()) return false;
    }
    RefineResult rr = refiner.takeResult();
    if (stop()) return false;

    say(0.85, "Packing the atlas");
    std::vector<AtlasInput> inputs(rr.params.size());
    for (size_t i = 0; i < rr.params.size(); ++i) {
        const PatchParam& p = rr.params[i];
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

    result_ = std::move(rr);
    atlas_ = std::move(atlas);
    resultValid_ = true;
    if (stop()) { resultValid_ = false; return false; }

    // the atlas as texture coordinates, and the texture of the file baked into it
    say(0.92, "Baking the texture");
    std::vector<BakeChart> charts;
    for (size_t i = 0; i < result_.params.size(); ++i)
        if (i < atlas_.uv.size() && !atlas_.uv[i].empty()) charts.push_back({&result_.params[i].patch, &atlas_.uv[i]});
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
        if (getenv("HUGE_NATIVE_DEBUG")) std::fprintf(stderr, "bake: %zu charts, %d x %d source, baked %zu bytes\n", charts.size(), sourceW_, sourceH_, baked_.size());
    }
    seconds_ = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    say(1.0, "Done");
    return true;
}

std::vector<float> AtlasSession::patchColors() const
{
    std::vector<float> colors(mesh_.n_faces() * 3, 0.72f);
    if (!resultValid_) return colors;
    for (size_t f = 0; f < colors.size() / 3 && f < result_.labels.size(); ++f)
        if (result_.labels[f] >= 0) hueColor(result_.labels[f], &colors[3 * f]);
    return colors;
}

std::vector<float> AtlasSession::patchNormalsPerFace() const
{
    if (!resultValid_) return {};
    const FaceData fd = computeFaceData(mesh_);
    return patchNormals(result_.labels, fd.area, fd.normal);
}

bool AtlasSession::save(const std::string& path, std::string* error) const
{
    if (!resultValid_) { if (error) *error = "there is no atlas to save"; return false; }
    std::vector<const Patch*> pv;
    std::vector<const std::vector<Eigen::Vector2d>*> uvv;
    for (size_t i = 0; i < result_.params.size(); ++i) {
        pv.push_back(&result_.params[i].patch);
        uvv.push_back(i < atlas_.uv.size() && !atlas_.uv[i].empty() ? &atlas_.uv[i] : nullptr);
    }
    char meta[200];
    std::snprintf(meta, sizeof(meta), "atlas resolution=%d maxMipLevel=%d gutter=%d margin=%d texelsPerUnit=%.6g", atlas_.resolution,
                  atlas_.maxMipLevel, atlas_.gutter, atlas_.margin, atlas_.texelsPerUnit);
    if (exportAtlasOBJ(path, mesh_, pv, uvv, meta) < 0) { if (error) *error = "cannot write " + path; return false; }
    return true;
}
