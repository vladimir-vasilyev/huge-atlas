// Python module `huge_atlas`: UV atlases for triangle meshes (segmentation, unfolding, packing with room for mip levels, baking).
// The arrays are numpy arrays; long runs release the GIL, report progress, and stop on Ctrl+C or when the `cancel` object says so.
#include <pybind11/functional.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "native/HugeSession.h"
#include "native/Session.h"
#include "param/FlatLoader.h"
#include "render/ImageFile.h"

#include <chrono>
#include <cmath>
#include <deque>
#include <memory>
#include <mutex>
#include <thread>

namespace py = pybind11;

namespace {

struct Settings {
    int quality = 1;
    py::object patches = py::none();
    int textureSize = 2048;
    int mipLevels = 3;
    DeveloperOptions dev;
};

SessionSettings toSession(const Settings& s)
{
    SessionSettings out;
    out.quality = std::max(0, std::min(2, s.quality));
    if (!s.patches.is_none()) { out.manualPatches = true; out.patches = std::max(1, s.patches.cast<int>()); }
    out.resolution = s.textureSize;
    out.maxMipLevel = s.mipLevels;
    out.dev = s.dev;
    return out;
}

template <class T> py::array_t<T> copyOut(const std::vector<T>& v, std::vector<py::ssize_t> shape)
{
    py::array_t<T> a(shape);
    std::copy(v.begin(), v.end(), a.mutable_data());
    return a;
}

// Runs `job` on a thread of its own; this thread stays with Python: it passes progress messages to `progress`, looks at Ctrl+C and at
// `cancel` (an object with is_set(), like threading.Event) and stops the job through the flag.
template <class Job>
void runInterruptible(const py::object& progress, const py::object& cancel, std::atomic<bool>& flag, std::mutex& mutex,
                      std::deque<std::string>& messages, Job job)
{
    std::atomic<bool> done{false};
    std::thread worker([&] { job(); done = true; });
    bool interrupted = false;
    {
        // the worker needs no GIL; this thread takes it only to talk to Python
        py::gil_scoped_release release;
        while (!done) {
            std::this_thread::sleep_for(std::chrono::milliseconds(40));
            py::gil_scoped_acquire acquire;
            if (PyErr_CheckSignals() != 0) {
                interrupted = true;
                flag = true;
                break;
            }
            if (!cancel.is_none() && py::bool_(cancel.attr("is_set")())) flag = true;
            std::deque<std::string> take;
            { std::lock_guard<std::mutex> lock(mutex); take.swap(messages); }
            if (!progress.is_none())
                for (const std::string& m : take) progress(m);
        }
        worker.join();
    }
    if (interrupted) throw py::error_already_set();   // KeyboardInterrupt is still set
}

class Atlas {
public:
    Atlas(py::array_t<float, py::array::c_style | py::array::forcecast> positions,
          py::array_t<uint32_t, py::array::c_style | py::array::forcecast> faces, py::object uv, py::object texture, const Settings& settings,
          py::object huge, py::object progress, py::object cancel)
        : settings_(settings), nF_(0)
    {
        if (positions.ndim() != 2 || positions.shape(1) != 3) throw std::invalid_argument("positions must have the shape (N, 3)");
        if (faces.ndim() != 2 || faces.shape(1) != 3) throw std::invalid_argument("faces must have the shape (F, 3)");
        nF_ = size_t(faces.shape(0));
        const size_t nV = size_t(positions.shape(0));
        huge_ = huge.is_none() ? nF_ > 3000000 : huge.cast<bool>();

        std::vector<float> cornerUV;
        int tw = 0, th = 0;
        const unsigned char* pixels = nullptr;
        py::array_t<unsigned char, py::array::c_style | py::array::forcecast> tex;
        if (!uv.is_none() || !texture.is_none()) {
            if (uv.is_none() || texture.is_none()) throw std::invalid_argument("uv and texture go together");
            auto u = py::array_t<float, py::array::c_style | py::array::forcecast>::ensure(uv);
            if (!u || u.size() != py::ssize_t(nF_ * 6)) throw std::invalid_argument("uv must have the shape (F, 3, 2): the old texture coordinates of the corners");
            cornerUV.assign(u.data(), u.data() + u.size());
            tex = py::array_t<unsigned char, py::array::c_style | py::array::forcecast>::ensure(texture);
            if (!tex || tex.ndim() != 3 || tex.shape(2) != 4) throw std::invalid_argument("texture must have the shape (H, W, 4), uint8, row 0 = v = 0");
            th = int(tex.shape(0));
            tw = int(tex.shape(1));
            pixels = tex.data();
        }

        std::string err;
        if (huge_) {
            hugeSession_ = std::make_unique<HugeSession>();
            if (!hugeSession_->setMesh(positions.data(), nV, faces.data(), nF_, &err)) throw std::invalid_argument(err);
            if (pixels && !hugeSession_->setTexture(cornerUV, tw, th, pixels, &err)) throw std::invalid_argument(err);
        } else {
            session_ = std::make_unique<AtlasSession>();
            if (!session_->setMesh(positions.data(), nV, faces.data(), nF_, &err)) throw std::invalid_argument(err);
            if (pixels && !session_->setTexture(cornerUV, tw, th, pixels, &err)) throw std::invalid_argument(err);
        }
        run(progress, cancel);
    }

    // The same mesh again with other settings.
    void rerun(const Settings& s, py::object progress, py::object cancel) { settings_ = s; run(progress, cancel); }

    // Another texture size or mip levels: only the packing and the baking are done again.
    void repack(int textureSize, py::object mipLevels)
    {
        settings_.textureSize = textureSize;
        if (!mipLevels.is_none()) settings_.mipLevels = mipLevels.cast<int>();
        std::string err;
        const SessionSettings s = toSession(settings_);
        bool ok;
        {
            py::gil_scoped_release release;
            ok = huge_ ? hugeSession_->repack(s, nullptr, &err) : session_->repack(s, &err);
        }
        if (!ok) throw std::runtime_error(err);
    }

    bool isHuge() const { return huge_; }
    size_t faceCount() const { return nF_; }
    size_t patchCount() const { return huge_ ? hugeSession_->patchCount() : session_->patchCount(); }

    py::array_t<float> cornerUV() const
    {
        if (huge_) return copyOut(hugeSession_->cornerUV(), {py::ssize_t(nF_), 3, 2});
        return copyOut(session_->cornerUVOut(), {py::ssize_t(nF_), 3, 2});
    }

    py::array_t<int32_t> chartId() const
    {
        std::vector<int32_t> id(nF_, -1);
        if (huge_) id = hugeSession_->patchOfFace();
        else for (size_t f = 0; f < nF_ && f < session_->patchOfFace().size(); ++f) id[f] = session_->patchOfFace()[f];
        return copyOut(id, {py::ssize_t(nF_)});
    }

    py::object texture() const
    {
        const bool has = huge_ ? hugeSession_->hasBakedTexture() : session_->hasBakedTexture();
        if (!has) return py::none();
        const int R = huge_ ? hugeSession_->bakedResolution() : session_->bakedResolution();
        const unsigned char* p = huge_ ? hugeSession_->bakedPixels() : session_->bakedPixels();
        py::array_t<unsigned char> a({py::ssize_t(R), py::ssize_t(R), py::ssize_t(4)});
        std::copy(p, p + size_t(R) * R * 4, a.mutable_data());
        return std::move(a);
    }

    py::dict metrics() const
    {
        py::dict d;
        d["mode"] = huge_ ? "huge" : "regular";
        d["faces"] = nF_;
        d["patches"] = patchCount();
        d["unresolved_patches"] = huge_ ? hugeSession_->unresolvedPatches() : size_t(std::max(0, session_->unresolvedPatches()));
        d["faces_without_uv"] = huge_ ? hugeSession_->facesWithoutUV() : size_t(0);
        d["texture_size"] = huge_ ? hugeSession_->resolution() : session_->resolution();
        d["efficiency"] = huge_ ? hugeSession_->efficiency() : session_->efficiency();
        d["gutter_texels"] = huge_ ? hugeSession_->gutter() : session_->gutter();
        d["mip_levels"] = settings_.mipLevels;
        d["seconds"] = huge_ ? hugeSession_->seconds() : session_->seconds();
        d["has_texture"] = huge_ ? hugeSession_->hasBakedTexture() : session_->hasBakedTexture();
        return d;
    }

    std::string report() const { return huge_ ? hugeSession_->report() : session_->report(); }

    // The layout measured: the new atlas, and the old one when uv and texture were given.
    py::dict layout() const
    {
        LayoutComparison c;
        std::string err;
        const bool ok = huge_ ? hugeSession_->compareData(&c, &err) : session_->compareData(&c, &err);
        if (!ok) throw std::runtime_error(err);
        py::dict out;
        out["new"] = layoutDict(c.ours, c.mipOurs);
        out["original"] = c.hadOriginal ? py::object(layoutDict(c.original, c.mipOriginal)) : py::none();
        return out;
    }

    std::string compare() const
    {
        std::string err;
        const std::string text = huge_ ? hugeSession_->compare(&err) : session_->compare(&err);
        if (text.empty()) throw std::runtime_error(err);
        return text;
    }

    void save(const std::string& path) const
    {
        std::string err;
        const bool ok = huge_ ? hugeSession_->save(path, &err) : session_->save(path, &err);
        if (!ok) throw std::runtime_error(err);
    }

private:
    static py::dict layoutDict(const UVLayoutStats& l, const MipSafetyResult& m)
    {
        py::dict d;
        d["charts"] = l.charts;
        d["seam_ratio"] = l.seamRatio();
        d["flipped_faces"] = l.flipped;
        d["faces_without_uv"] = l.facesWithoutUV;
        d["l2_mean"] = l.l2Mean;
        d["l2_p95"] = l.l2P95;
        d["l2_p99"] = l.l2P99;
        d["conformal_p95"] = l.conformalP95;
        d["density_p05"] = l.densityP05;
        d["density_p50"] = l.densityP50;
        d["density_p95"] = l.densityP95;
        d["covered"] = l.covered;
        d["overlap"] = l.overlap;
        d["safe_mip_level"] = m.safeLevel;
        d["gap_min_texels"] = m.gapMin;
        d["gap_p05_texels"] = m.gapP05;
        d["level_by_gap_p05"] = m.levelByGapP05;
        return d;
    }

    void run(py::object progress, py::object cancel)
    {
        const SessionSettings s = toSession(settings_);
        std::atomic<bool> flag{false};
        std::mutex mutex;
        std::deque<std::string> messages;
        std::string err;
        bool ok = false;
        auto say = [&](const std::string& what) { std::lock_guard<std::mutex> lock(mutex); if (messages.empty() || messages.back() != what) messages.push_back(what); };
        runInterruptible(progress, cancel, flag, mutex, messages, [&] {
            if (huge_) ok = hugeSession_->run(s, say, &err, &flag);
            else ok = session_->run(s, [&](double, const std::string& what) { say(what); }, flag, &err);
        });
        if (!ok) {
            if (err == "cancelled") throw std::runtime_error("cancelled");
            throw std::runtime_error(err.empty() ? "the atlas could not be made" : err);
        }
    }

    Settings settings_;
    size_t nF_;
    bool huge_ = false;
    std::unique_ptr<AtlasSession> session_;
    std::unique_ptr<HugeSession> hugeSession_;
};

Settings makeSettings(int quality, py::object patches, int textureSize, int mipLevels, double smoothing, int normalIterations,
                      double normalSigma, int lloydIterations, int maxDepth, int splitAtOnce, bool allowLscm, bool repairNearMisses,
                      double simplifyPercent)
{
    Settings s;
    s.quality = quality;
    s.patches = patches;
    s.textureSize = textureSize;
    s.mipLevels = mipLevels;
    s.dev.smoothing = smoothing;
    s.dev.normalIterations = normalIterations;
    s.dev.normalSigma = normalSigma;
    s.dev.lloydIterations = lloydIterations;
    s.dev.maxDepth = maxDepth;
    s.dev.splitAtOnce = splitAtOnce;
    s.dev.allowLscm = allowLscm;
    s.dev.repairNearMisses = repairNearMisses;
    s.dev.simplifyPercent = simplifyPercent;
    return s;
}

// A mesh file as arrays: positions (N, 3), faces (F, 3), the texture coordinates of the corners (F, 3, 2) and the texture (H, W, 4)
// of the file when it has them (None otherwise). The positions are the ones of the file, not normalized.
py::dict loadMeshFile(const std::string& path)
{
    FlatMesh mesh;
    FlatSurface surface;
    std::string err;
    bool ok;
    {
        py::gil_scoped_release release;
        ok = loadFlatMesh(path, mesh, nullptr, &err, &surface);
    }
    if (!ok) throw std::runtime_error(err);
    py::dict d;
    d["positions"] = copyOut(mesh.pos, {py::ssize_t(mesh.vertexCount()), 3});
    d["faces"] = copyOut(mesh.idx, {py::ssize_t(mesh.faceCount()), 3});
    d["uv"] = py::none();
    d["texture"] = py::none();
    if (surface.hasUV()) {
        d["uv"] = copyOut(surface.cornerUV, {py::ssize_t(mesh.faceCount()), 3, 2});
        std::string name = surface.textureFile;
        for (size_t i = 0; i < surface.materials.size() && name.empty(); ++i) name = surface.materials[i].diffuseMap;
        int w = 0, h = 0;
        std::vector<unsigned char> pixels;
        if (!name.empty() && loadImageRGBA(surface.directory + "/" + name, w, h, pixels, nullptr))
            d["texture"] = copyOut(pixels, {py::ssize_t(h), py::ssize_t(w), 4});
    }
    return d;
}

} // namespace

PYBIND11_MODULE(huge_atlas, m)
{
    m.doc() = "UV atlases for triangle meshes: VSA segmentation, LinABF unfolding, packing with room for mip levels, texture baking.";

    py::class_<Atlas>(m, "Atlas", "The result of make_atlas.")
        .def_property_readonly("is_huge", &Atlas::isHuge, "True when the flat-array pipeline for big scans was used")
        .def_property_readonly("corner_uv", &Atlas::cornerUV, "(F, 3, 2) float32: the atlas coordinates of the corners of the faces, in [0, 1]; NaN for a face without a chart")
        .def_property_readonly("chart_id", &Atlas::chartId, "(F,) int32: the chart (patch) of every face, -1 for none")
        .def_property_readonly("texture", &Atlas::texture, "(R, R, 4) uint8, row 0 = v = 0: the old texture baked into the atlas, or None")
        .def_property_readonly("patch_count", &Atlas::patchCount)
        .def("metrics", &Atlas::metrics, "A dict of numbers: patches, efficiency (share of the texture that is used), gutter, seconds ...")
        .def("layout", &Atlas::layout, "Distortion, seams and mip safety of the new atlas (and of the old layout, if uv and texture were given)")
        .def("report", &Atlas::report, "What the run did, as text")
        .def("compare", &Atlas::compare, "The old layout and the new atlas side by side, as text")
        .def("repack", &Atlas::repack, py::arg("texture_size"), py::arg("mip_levels") = py::none(),
             "Pack and bake again with another texture size / mip levels; the segmentation and the unfolding are kept")
        .def("rerun", [](Atlas& a, int quality, py::object patches, int textureSize, int mipLevels, double smoothing, int normalIterations,
                         double normalSigma, int lloydIterations, int maxDepth, int splitAtOnce, bool allowLscm, bool repairNearMisses,
                         double simplifyPercent, py::object progress, py::object cancel) {
                 a.rerun(makeSettings(quality, patches, textureSize, mipLevels, smoothing, normalIterations, normalSigma, lloydIterations, maxDepth,
                                      splitAtOnce, allowLscm, repairNearMisses, simplifyPercent), progress, cancel);
             },
             py::arg("quality") = 1, py::arg("patches") = py::none(), py::arg("texture_size") = 2048, py::arg("mip_levels") = 3,
             py::arg("smoothing") = 0.1, py::arg("normal_iterations") = 0, py::arg("normal_sigma") = 25.0, py::arg("lloyd_iterations") = 30,
             py::arg("max_depth") = 10, py::arg("split_at_once") = 40000, py::arg("allow_lscm") = true, py::arg("repair_near_misses") = true,
             py::arg("simplify_percent") = 0.0, py::arg("progress") = py::none(), py::arg("cancel") = py::none(),
             "The whole run again on the same mesh with other settings (the arguments as in make_atlas)")
        .def("save", &Atlas::save, py::arg("path"), "Write OBJ (+MTL) or PLY with the atlas coordinates and <name>_atlas.png next to it");

    m.def("make_atlas",
          [](py::array_t<float, py::array::c_style | py::array::forcecast> positions,
             py::array_t<uint32_t, py::array::c_style | py::array::forcecast> faces, py::object uv, py::object texture, int quality,
             py::object patches, int textureSize, int mipLevels, double smoothing, int normalIterations, double normalSigma,
             int lloydIterations, int maxDepth, int splitAtOnce, bool allowLscm, bool repairNearMisses, double simplifyPercent,
             py::object huge, py::object progress, py::object cancel) {
              const Settings s = makeSettings(quality, patches, textureSize, mipLevels, smoothing, normalIterations, normalSigma, lloydIterations,
                                              maxDepth, splitAtOnce, allowLscm, repairNearMisses, simplifyPercent);
              return std::make_unique<Atlas>(positions, faces, uv, texture, s, huge, progress, cancel);
          },
          py::arg("positions"), py::arg("faces"), py::arg("uv") = py::none(), py::arg("texture") = py::none(), py::arg("quality") = 1,
          py::arg("patches") = py::none(), py::arg("texture_size") = 2048, py::arg("mip_levels") = 3, py::arg("smoothing") = 0.1,
          py::arg("normal_iterations") = 0, py::arg("normal_sigma") = 25.0, py::arg("lloyd_iterations") = 30, py::arg("max_depth") = 10,
          py::arg("split_at_once") = 40000, py::arg("allow_lscm") = true, py::arg("repair_near_misses") = true,
          py::arg("simplify_percent") = 0.0, py::arg("huge") = py::none(), py::arg("progress") = py::none(), py::arg("cancel") = py::none(),
          R"doc(Make a UV atlas for a triangle mesh.

positions  (N, 3) float: vertices.      faces  (F, 3) int: three vertex indices per face.
uv, texture  optional, together: (F, 3, 2) old texture coordinates of the corners and the (H, W, 4) uint8 texture (row 0 = v = 0);
             the texture is baked into the new atlas.
quality  0 fast, 1 balanced, 2 detailed: how many patches to start from.   patches  set the number of patches yourself.
texture_size, mip_levels  size of the atlas and how many mip levels must stay clean (the gap between charts follows from it).
huge  None = decided by the size (more than 3M faces use the flat-array pipeline); True / False to force.
progress  a function that gets a message for every stage.   cancel  an object with is_set() (threading.Event); Ctrl+C works too.
Returns an Atlas. The mesh must be manifold in the regular mode; use huge=True for a mesh that is not.)doc");

    m.def("load_mesh", &loadMeshFile, py::arg("path"),
          "Read OBJ / PLY: a dict with positions (N, 3), faces (F, 3), uv (F, 3, 2) and texture (H, W, 4), the last two or None");
}
