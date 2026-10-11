#pragma once

#include "native/Session.h"
#include "param/FlatLoader.h"
#include "param/FlatPipeline.h"

#include <string>
#include <vector>

// The huge mode for the native window: a scan of millions of faces goes from the file straight into flat arrays (no half-edge mesh),
// is simplified, segmented and unfolded on the simplified copy and lifted back; the atlas is packed and the texture is baked.
// Nothing here knows about the interface; `run` and `repack` are meant for a thread of their own. They cannot be cancelled.
class HugeSession {
public:
    using Progress = std::function<void(const std::string& what)>;

    // faces a file will probably have, from its size (an OBJ line of a face is about 75 bytes, a binary PLY face about 40)
    static size_t estimateFaces(const std::string& path);

    void setFile(const std::string& path) { path_ = path; clear(); }
    const std::string& path() const { return path_; }
    void clear();

    bool run(const SessionSettings& settings, const Progress& progress, std::string* error);
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
    double seconds() const { return seconds_; }

    std::vector<unsigned char> patchRgba() const;      // rgba per face: every patch in its own color
    std::vector<float> patchNormalsPerFace() const;    // 3 floats per face
    bool hasBakedTexture() const { return !baked_.empty(); }
    int bakedResolution() const { return bakedResolution_; }
    const unsigned char* bakedPixels() const { return baked_.data(); }
    std::vector<UvChart> uvCharts() const;             // the borders of the charts in the atlas (the huge mode keeps no triangles)

    std::string report() const;                        // what the last run did, as text for the developer mode
    std::string compare(std::string* error) const;     // the original layout and the new atlas side by side

    // PLY with the atlas UV, and `<name>_atlas.png` next to it when there is a baked texture
    bool save(const std::string& path, std::string* error) const;

private:
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
    double seconds_ = 0.0;
    size_t k_ = 0;
};
