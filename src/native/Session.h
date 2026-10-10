#pragma once

#include "core/MeshTypes.h"
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

    // OBJ with the atlas UV (a vt per patch vertex, seams get their own vt)
    bool save(const std::string& path, std::string* error) const;

    // how many patches the quality asks for on a mesh of this size
    static int patchesFor(size_t faces, int quality);

private:
    TriMesh mesh_;
    std::string path_;
    RefineResult result_;
    AtlasResult atlas_;
    bool resultValid_ = false;
    double seconds_ = 0.0;
};
