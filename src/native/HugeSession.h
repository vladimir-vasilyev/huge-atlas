#pragma once

#include "core/Cancel.h"
#include "native/Session.h"
#include "param/FlatLoader.h"
#include "param/FlatPipeline.h"

#include <atomic>
#include <string>
#include <vector>

// The huge mode for the native window: a scan of millions of faces goes from the file straight into flat arrays (no half-edge mesh),
// is simplified, segmented and unfolded on the simplified copy and lifted back; the atlas is packed and the texture is baked.
// Nothing here knows about the interface; `run` and `repack` are meant for a thread of their own. `run` can be cancelled (between its stages).
class HugeSession {
public:
    using Progress = std::function<void(const std::string& what)>;

    // faces a file will probably have, from its size (an OBJ line of a face is about 75 bytes, a binary PLY face about 40)
    static size_t estimateFaces(const std::string& path);

    void setFile(const std::string& path) { path_ = path; clear(); }
    const std::string& path() const { return path_; }
    void clear();

    // reads the file (and its texture) into flat arrays so that it can be shown; `run` reuses what is loaded
    bool load(std::string* error);
    // The mesh from arrays instead of a file (positions x y z per vertex, three indices per face); the old texture is optional
    // (`cornerUV`: 6 floats per face, `rgba`: width x height, row 0 = v = 0).
    bool setMesh(const float* positions, size_t vertexCount, const uint32_t* indices, size_t faceCount, std::string* error);
    bool setTexture(const std::vector<float>& cornerUV, int width, int height, const unsigned char* rgba, std::string* error);
    const std::vector<int32_t>& patchOfFace() const { return result_.patchOfFace; }
    bool loaded() const { return loaded_; }
    bool hasSourceTexture() const { return sourceW_ > 0 && !sourcePixels_.empty() && surface_.hasUV(); }
    const std::vector<float>& sourceCornerUV() const { return surface_.cornerUV; }
    const unsigned char* sourcePixels() const { return sourcePixels_.data(); }
    int sourceWidth() const { return sourceW_; }
    int sourceHeight() const { return sourceH_; }

    // `cancel` (optional) stops the run between its stages (and between the rounds of the refinement): false and the error "cancelled"
    bool run(const SessionSettings& settings, const Progress& progress, std::string* error, const std::atomic<bool>* cancel = nullptr);
    bool repack(const SessionSettings& settings, const Progress& progress, std::string* error);

    bool hasResult() const { return resultValid_; }
    const FlatMesh& mesh() const { return mesh_; }
    const std::vector<float>& cornerUV() const { return result_.cornerUV; }
    size_t faceCount() const { return mesh_.faceCount(); }
    size_t patchCount() const { return result_.patches.size(); }
    size_t unresolvedPatches() const { return result_.stats.patchesUnresolved; }
    size_t facesWithoutUV() const { return result_.stats.facesWithoutUV; }
    double efficiency() const { return atlas_.efficiency; }
    int resolution() const { return atlas_.resolution; }
    int gutter() const { return atlas_.gutter; }
    double seconds() const { return seconds_; }

    std::vector<unsigned char> patchRgba() const;      // rgba per face: every patch in its own color
    std::vector<float> patchNormalsPerFace() const;    // 3 floats per face
    bool hasBakedTexture() const { return !baked_.empty(); }
    int bakedResolution() const { return bakedResolution_; }
    const unsigned char* bakedPixels() const { return baked_.data(); }
    std::vector<UvChart> uvCharts() const;             // the borders of the charts in the atlas (the huge mode keeps no triangles)

    std::string report() const;                        // what the last run did, as text for the developer mode
    std::string compare(std::string* error) const;     // the original layout and the new atlas side by side
    bool compareData(LayoutComparison* out, std::string* error) const;   // the same, as numbers

    // PLY with the atlas UV, and `<name>_atlas.png` next to it when there is a baked texture
    bool save(const std::string& path, std::string* error) const;

private:
    bool runImpl(const SessionSettings& settings, const Progress& progress, std::string* error, const std::atomic<bool>* cancel);
    bool packAndBake(const SessionSettings& settings, const Progress& progress, std::string* error);

    std::string path_;
    FlatMesh mesh_;
    FlatSurface surface_;
    FlatTransform transform_;
    FlatResult result_;
    AtlasResult atlas_;
    std::vector<unsigned char> sourcePixels_;
    int sourceW_ = 0, sourceH_ = 0, textureMaterial_ = -1;
    std::vector<unsigned char> baked_;
    int bakedResolution_ = 0;
    bool resultValid_ = false;
    bool loaded_ = false;
    double seconds_ = 0.0;
    size_t k_ = 0;
};
