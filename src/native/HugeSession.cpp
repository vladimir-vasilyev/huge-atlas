#include "native/HugeSession.h"

#include "param/AtlasMask.h"
#include "param/FlatBake.h"
#include "render/ImageFile.h"
#include "render/MeshRenderer.h"

#include <algorithm>
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
}

bool HugeSession::run(const SessionSettings& s, const Progress& progress, std::string* error)
{
    using Clock = std::chrono::steady_clock;
    const auto t0 = Clock::now();
    auto say = [&](const std::string& what) { if (progress) progress(what); };
    clear();

    say("Reading the mesh");
    if (!loadFlatMesh(path_, mesh_, nullptr, error, &surface_)) return false;
    transform_ = normalizeFlatToUnitBox(mesh_);
    // the texture of the file: the first material with a map_Kd (OBJ) or the TextureFile comment (PLY)
    if (surface_.hasUV()) {
        std::string name = surface_.textureFile;
        for (size_t i = 0; i < surface_.materials.size() && name.empty(); ++i)
            if (!surface_.materials[i].diffuseMap.empty()) { name = surface_.materials[i].diffuseMap; textureMaterial_ = int(i); }
        if (!name.empty()) {
            say("Reading the texture");
            if (!loadImageRGBA(surface_.directory + "/" + name, sourceW_, sourceH_, sourcePixels_, nullptr)) sourcePixels_.clear();
        }
    }

    // the quality: how many patches to start from and how fine the simplified copy is
    static const double kFactor[3] = {0.5, 1.0, 2.0}, kPercent[3] = {5.0, 10.0, 20.0};
    const int q = std::max(0, std::min(2, s.quality));
    const double base = 0.06 * std::sqrt(double(std::max<size_t>(mesh_.faceCount(), 1)));
    CoarseOptions co;
    co.k = s.manualPatches ? std::max(1, s.patches) : int(std::max(8.0, std::min(1000.0, std::round(base * kFactor[q]))));
    co.simplifyPercent = kPercent[q];
    co.lloyd.maxIterations = 30;
    co.onStage = [&](const std::string& what) { say(what); };
    result_ = runFlatPipeline(mesh_, co);

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
