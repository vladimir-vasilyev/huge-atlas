#pragma once

#include "core/MeshTypes.h"
#include "core/ObjLoader.h"
#include "param/AtlasBake.h"
#include "param/AtlasPacker.h"
#include "param/Refinement.h"

#include <atomic>
#include <functional>
#include <string>
#include <vector>

// The work behind the native window: load a mesh, make the atlas, give the colors of the patches, save. Nothing here knows about the
// user interface; `run` is meant to be called from a thread of its own.
struct SessionSettings {
    int  quality = 1;               // 0 fast, 1 balanced, 2 detailed
    bool manualPatches = false;
    int  patches = 100;
    int  resolution = 2048;
    int  maxMipLevel = 3;
};

// One chart of the atlas for the UV view: its vertices in the atlas (u, v in [0, 1]), its triangles and the loops of its border.
struct UvChart {
    int patch = 0;
    std::vector<float> uv;                          // u, v per vertex
    std::vector<unsigned> triangles;                // three vertex indices per face
    std::vector<std::vector<unsigned>> loops;       // the border of the chart, vertex indices
};

class AtlasSession {
public:
    using Progress = std::function<void(double fraction, const std::string& what)>;

    bool load(const std::string& path, std::string* error);
    bool hasMesh() const { return mesh_.n_faces() > 0; }
    const TriMesh& mesh() const { return mesh_; }
    const std::string& path() const { return path_; }

    // Blocking; checks `cancel` between the steps. false + message when cancelled or failed.
    bool run(const SessionSettings& settings, const Progress& progress, const std::atomic<bool>& cancel, std::string* error);

    bool hasResult() const { return resultValid_; }
    size_t patchCount() const { return result_.params.size(); }
    int unresolvedPatches() const { return result_.stats.unresolved; }
    double efficiency() const { return atlas_.efficiency; }
    int resolution() const { return atlas_.resolution; }
    int gutter() const { return atlas_.gutter; }
    double seconds() const { return seconds_; }

    // rgb per face: every patch in its own color
    std::vector<float> patchColors() const;

    // The texture of the file (an OBJ with vt and a material with a diffuse map) and its bake into the new atlas.
    bool hasSourceTexture() const { return sourceW_ > 0; }
    bool hasBakedTexture() const { return resultValid_ && !baked_.empty(); }
    int bakedResolution() const { return atlas_.resolution; }
    const unsigned char* bakedPixels() const { return baked_.data(); }
    // the atlas as texture coordinates of the corners of every face (6 floats per face; u = -1: the face has none)
    const std::vector<float>& atlasFaceUV() const { return bakedUV_; }
    // the normal of the patch of every face (3 floats per face): the direction in which the patches move in the presentation
    std::vector<float> patchNormalsPerFace() const;

    // the charts as they lie in the atlas (for the UV view)
    std::vector<UvChart> uvCharts() const;
    // the color of a patch (the same in the 3D view and in the UV view)
    static void patchColor(int patch, float* rgb);

    // OBJ (with a material) or PLY by the extension, with the atlas UV, and `<name>_atlas.png` next to it: the texture of the file baked
    // into the atlas, or a flat color per chart to paint over when the mesh has no texture.
    bool save(const std::string& path, std::string* error) const;

    // how many patches the quality asks for on a mesh of this size
    static int patchesFor(size_t faces, int quality);

private:
    std::vector<BakeChart> bakeCharts() const;
    TriMesh mesh_;
    std::string path_;
    RefineResult result_;
    AtlasResult atlas_;
    bool resultValid_ = false;
    double seconds_ = 0.0;

    ObjExtras extras_;
    std::vector<unsigned char> sourcePixels_;
    int sourceW_ = 0, sourceH_ = 0, textureMaterial_ = -1;
    std::vector<unsigned char> baked_;
    std::vector<float> bakedUV_;
};
