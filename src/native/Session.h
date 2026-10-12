#pragma once

#include "core/MeshTypes.h"
#include "core/ObjLoader.h"
#include "app/Preset.h"
#include "core/PatchFrame.h"
#include "param/LayoutReport.h"
#include "param/AtlasBake.h"
#include "param/AtlasPacker.h"
#include "param/Refinement.h"

#include <atomic>
#include <functional>
#include <string>
#include <vector>

// The work behind the native window: load a mesh, make the atlas, give the colors of the patches, save. Nothing here knows about the
// user interface; `run` is meant to be called from a thread of its own.
// What the developer mode adds: the knobs of the pipeline (the defaults are the values the ordinary mode uses).
struct DeveloperOptions {
    double smoothing = 0.1;          // border smoothing of the VSA regions (lambda)
    int    normalIterations = 0;     // smoothing of the normal field before the VSA (0 = off)
    double normalSigma = 25.0;       // ... and its angle (degrees)
    int    lloydIterations = 30;
    int    maxDepth = 10;            // how many times a patch may be split again
    int    splitAtOnce = 40000;      // patches above this many faces are split at once when plain LinABF fails (0 = never)
    bool   allowLscm = true;         // LSCM before Tutte
    bool   repairNearMisses = true;  // a map with a few flips is repaired locally
    double simplifyPercent = 0.0;    // huge mode: size of the simplified copy in percent (0 = by quality)
    // refinement (the splitting of the bad patches) and the unfolding ladder
    int    minFaces = 8;             // both parts of a split need at least this many faces
    int    maxRounds = 12;
    bool   splitImmediately = false; // only plain LinABF, split at once when it fails (faster, a bit more patches)
    bool   useActiveSet = true;
    bool   allowCut = true;          // slits
    bool   allowTutte = true;
    double maxStretch = 4.0;         // a triangle stretched more than this counts as bad

    ParamOptions paramOptions() const
    {
        ParamOptions p;
        p.allowLscm = allowLscm;
        p.repairNearMisses = repairNearMisses;
        p.useActiveSet = useActiveSet;
        p.allowCut = allowCut;
        p.allowTutte = allowTutte;
        p.maxStretch = maxStretch;
        return p;
    }
};

struct SessionSettings {
    int  quality = 1;               // 0 fast, 1 balanced, 2 detailed
    bool manualPatches = false;
    int  patches = 100;
    int  resolution = 2048;
    int  maxMipLevel = 3;
    DeveloperOptions dev;
};

// One chart of the atlas for the UV view: its vertices in the atlas (u, v in [0, 1]), its triangles and the loops of its border.
struct UvChart {
    int patch = 0;
    std::vector<float> uv;                          // u, v per vertex
    std::vector<unsigned> triangles;                // three vertex indices per face
    std::vector<std::vector<unsigned>> loops;       // the border of the chart, vertex indices
    std::vector<unsigned> segments;                 // border edges as pairs of vertex indices (the huge mode keeps only these)
    std::vector<unsigned> flipped;                  // faces (index in `triangles` / 3) whose uv are flipped: drawn red
    std::vector<unsigned> crossings;                // pairs of vertex indices: the border edges that cross another one: drawn magenta
};

// One line of the patch table of the developer mode.
struct PatchRow {
    int index = 0;
    size_t faces = 0;
    std::string method;
    double l2Mean = 0.0;
    double badArea = 0.0;
    std::string status;     // ok / stretch / overlap / flipped / not a disk / no uv
    bool bad = false;
};

class AtlasSession {
public:
    using Progress = std::function<void(double fraction, const std::string& what)>;

    bool load(const std::string& path, std::string* error);
    // The mesh from arrays (positions x y z per vertex, three vertex indices per face). Faces keep their numbers: a mesh the half-edge
    // structure cannot take as it is (non-manifold edges, repeated vertices in a face) is refused. `cornerUV` (6 floats per face,
    // optional) and `rgba` (width x height, optional, row 0 = v = 0) are the old texture that is baked into the atlas.
    bool setMesh(const float* positions, size_t vertexCount, const uint32_t* indices, size_t faceCount, std::string* error);
    bool setTexture(const std::vector<float>& cornerUV, int width, int height, const unsigned char* rgba, std::string* error);
    // atlas.faceUV: 6 floats per face, NaN for a face without a chart
    std::vector<float> cornerUVOut() const;
    // the number of the patch of every face (-1: none)
    const std::vector<int>& patchOfFace() const { return result_.labels; }
    bool hasMesh() const { return mesh_.n_faces() > 0; }
    const TriMesh& mesh() const { return mesh_; }
    const std::string& path() const { return path_; }
    // the diagnostics of the open mesh as text lines (vertices, edges, components, boundary loops, degenerate faces ...)
    std::string meshInfo() const;

    // Blocking; checks `cancel` between the steps. false + message when cancelled or failed.
    bool run(const SessionSettings& settings, const Progress& progress, const std::atomic<bool>& cancel, std::string* error);

    bool hasResult() const { return resultValid_; }
    size_t patchCount() const { return result_.params.size(); }
    int unresolvedPatches() const { return result_.stats.unresolved; }
    double efficiency() const { return atlas_.efficiency; }
    const AtlasResult& atlas() const { return atlas_; }
    int resolution() const { return atlas_.resolution; }
    int gutter() const { return atlas_.gutter; }
    double seconds() const { return seconds_; }

    // rgb per face: every patch in its own color
    std::vector<float> patchColors() const;

    // The texture of the file (an OBJ with vt and a material with a diffuse map) and its bake into the new atlas.
    bool hasSourceTexture() const { return sourceW_ > 0; }
    // the texture of the file and its coordinates as they are (6 floats per face), to show the mesh before there is an atlas
    const std::vector<float>& sourceCornerUV() const { return extras_.cornerUV; }
    const unsigned char* sourcePixels() const { return sourcePixels_.data(); }
    int sourceWidth() const { return sourceW_; }
    int sourceHeight() const { return sourceH_; }
    bool hasBakedTexture() const { return resultValid_ && !baked_.empty(); }
    int bakedResolution() const { return atlas_.resolution; }
    const unsigned char* bakedPixels() const { return baked_.data(); }
    // the atlas as texture coordinates of the corners of every face (6 floats per face; u = -1: the face has none)
    const std::vector<float>& atlasFaceUV() const { return bakedUV_; }
    // the normal of the patch of every face (3 floats per face): the direction in which the patches move in the presentation
    std::vector<float> patchNormalsPerFace() const;

    // The atlas packed again with another texture size / mip levels (and the texture baked again): nothing before the packing is run again.
    bool repack(const SessionSettings& settings, std::string* error);

    // the charts as they lie in the atlas (for the UV view)
    std::vector<UvChart> uvCharts() const;

    // Regions to start from instead of the segmentation of the run (the Segmentation Lab); dropped when another mesh is opened.
    void setStartLabels(std::vector<int> labels) { startLabels_ = std::move(labels); }
    bool hasStartLabels() const { return startLabels_.size() == mesh_.n_faces() && mesh_.n_faces() > 0; }

    // The unfolded patches of the last run, for the bad patches (Debug > Save Bad Patches).
    const std::vector<PatchParam>& params() const { return result_.params; }
    std::vector<PatchRow> patchRows() const;
    bool patchFrame(int patch, PatchFrame* frame) const;
    // The border of a patch as segments (A, B: 6 floats each, in the coordinates of the mesh): the edges that only one of its faces has.
    bool patchBorder(int patch, std::vector<float>* segments) const;   // where a patch is and which way it faces (for the camera)
    const std::vector<int>& faceToPatch() const { return result_.labels; }
    Preset preset(const SessionSettings& settings) const;   // the settings of the run as a preset

    // Debug > Load Bad Patch: an OBJ saved by Save Bad Patches becomes the mesh and its only patch; the preset next to it (same name,
    // .preset) says how to unfold it. `text` gets what came out; the chart and the colors are for the views.
    bool loadBadPatch(const std::string& path, std::string* text, std::string* error);
    bool badPatchLoaded() const { return badLoaded_; }
    // The unfolding of a patch of the result: its uv chart (with the flipped faces and crossings) and a text. `again`: unfold it again with
    // the ladder options of these settings (the atlas is not changed).
    bool inspectPatch(int index, const DeveloperOptions* again, UvChart* chart, std::string* text) const;
    UvChart badPatchChart() const;                 // uv scaled into the square, with the flipped faces and the crossings
    std::vector<float> badPatchColors() const;     // per face: blue where the stretch is 1, red where it is bad
    // the color of a patch (the same in the 3D view and in the UV view)
    static void patchColor(int patch, float* rgb);

    // OBJ (with a material) or PLY by the extension, with the atlas UV, and `<name>_atlas.png` next to it: the texture of the file baked
    // into the atlas, or a flat color per chart to paint over when the mesh has no texture.
    bool save(const std::string& path, std::string* error) const;

    // What the last run did, as text for the developer mode: patches, the methods that unfolded them, splits, packing.
    std::string report() const;
    // The original layout and the new atlas side by side (needs the atlas; without UV in the file only the atlas is measured).
    std::string compare(std::string* error) const;
    bool compareData(LayoutComparison* out, std::string* error) const;   // the same, as numbers
    // The coverage of the atlas as an RGBA PNG (the alpha channel is the occupancy).
    bool writeMask(const std::string& path, std::string* error) const;
    // The texture size and the mip level that fit the layout of the file: the atlas can be made like the original.
    bool matchOriginal(int* resolution, int* mip, std::string* text) const;

    // how many patches the quality asks for on a mesh of this size
    static int patchesFor(size_t faces, int quality);

private:
    bool packAndBake(const SessionSettings& settings, std::string* error);
    std::vector<BakeChart> bakeCharts() const;
    TriMesh mesh_;
    std::vector<int> startLabels_;
    PatchParam bad_;
    bool badLoaded_ = false;
    std::string path_;
    RefineResult result_;
    AtlasResult atlas_;
    bool resultValid_ = false;
    double seconds_ = 0.0;
    size_t k_ = 0;

    ObjExtras extras_;
    std::vector<unsigned char> sourcePixels_;
    int sourceW_ = 0, sourceH_ = 0, textureMaterial_ = -1;
    std::vector<unsigned char> baked_;
    std::vector<float> bakedUV_;
};
