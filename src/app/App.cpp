#include "app/App.h"

#include "render/GL.h"   // strictly before GLFW (GLFW_INCLUDE_NONE is set in CMake)
#include "param/AtlasMask.h"   // writePNG
#include "param/MeshWriters.h"
#include "render/ImageFile.h"

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <ImGuiFileDialog.h>

#include <algorithm>
#include <cfloat>
#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace {

void glfwErrorCallback(int code, const char* description)
{
    std::fprintf(stderr, "[GLFW] error %d: %s\n", code, description);
}

const char* glString(GLenum name)
{
    const GLubyte* s = glGetString(name);
    return s ? reinterpret_cast<const char*>(s) : "?";
}

} // namespace

bool App::init(int width, int height, const char* title)
{
    glfwSetErrorCallback(glfwErrorCallback);
    if (!glfwInit())
        return false;

    // macOS: 4.1 at most, Core + forward-compatible is mandatory
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
    glfwWindowHint(GLFW_COCOA_RETINA_FRAMEBUFFER, GLFW_TRUE);
    glfwWindowHint(GLFW_SAMPLES, 0);   // the 3D scene renders into its own multisample target (see SceneFramebuffer)
    glfwWindowHint(GLFW_DEPTH_BITS, 24);

    window_ = glfwCreateWindow(width, height, title, nullptr, nullptr);
    if (!window_) {
        glfwTerminate();
        return false;
    }
    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1);

    glVersion_   = glString(GL_VERSION);
    glRenderer_  = glString(GL_RENDERER);
    glslVersion_ = glString(GL_SHADING_LANGUAGE_VERSION);
    std::printf("OpenGL %s | %s | GLSL %s\n",
                glVersion_.c_str(), glRenderer_.c_str(), glslVersion_.c_str());

    // ---- ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
#ifdef IMGUI_HAS_DOCK
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
#endif
    ImGui::StyleColorsDark();

    if (!ImGui_ImplGlfw_InitForOpenGL(window_, true) ||
        !ImGui_ImplOpenGL3_Init("#version 410")) {
        std::fprintf(stderr, "ImGui backend init failed\n");
        return false;
    }
    imguiReady_ = true;

    // ---- Test geometry
    std::string log;
    if (!triangle_.init(&log)) {
        std::fprintf(stderr, "DebugTriangle shader error:\n%s\n", log.c_str());
        return false;
    }

    if (!meshRenderer_.init(&log)) {
        std::fprintf(stderr, "MeshRenderer shader error:\n%s\n", log.c_str());
        return false;
    }
    showTriangle_ = false;
    rotate_ = false;
    const char* envMesh = std::getenv("VSA_MESH");   // testing hooks: VSA_MESH, VSA_K, VSA_AUTO_ATLAS
    openMesh(envMesh ? envMesh : "data/cube.obj");   // default file; on failure only a status message
    if (const char* envK = std::getenv("VSA_K")) vsaK_ = std::max(1, std::atoi(envK));
    if (const char* envW = std::getenv("VSA_WIREFRAME")) wireframe_ = std::atoi(envW) != 0;
    if (const char* envM = std::getenv("VSA_MSAA")) msaaSamples_ = std::atoi(envM);
    if (const char* envC = std::getenv("VSA_CULL")) cullBackFaces_ = std::atoi(envC) != 0;

    glEnable(GL_DEPTH_TEST);
    GL_CHECK();
    return true;
}

struct App::HugeJob {
    std::string path;
    CoarseOptions options;
    HorizonOptions horizon;
    std::thread worker;
    std::atomic<bool> done{false};
    std::mutex mutex;
    std::string stage = "starting...";   // guarded by mutex
    std::string error;                   // guarded by mutex; set when the run failed
    FlatMesh mesh;
    FlatSurface surface;                 // old texture coordinates and materials of the file
    FlatTransform transform;
    FlatResult result;
    std::vector<unsigned char> baked;    // the atlas texture, RGBA, row 0 = v = 0 (empty: the file had no texture)
    int bakedResolution = 0;
    std::string textureName;             // of the old texture (for the messages)
    bool preloaded = false;              // mesh, surface and source texture were filled from the loaded TriMesh
    std::vector<unsigned char> sourcePixels;   // the old texture, RGBA, row 0 = v = 0
    int sourceW = 0, sourceH = 0, textureMaterial = -1;
    AtlasResult atlas;
    FlatLoadInfo loadInfo;
    double loadSeconds = 0, pipelineSeconds = 0, packSeconds = 0, bakeSeconds = 0;
    std::string warning;                 // guarded by mutex
    bool saved = false;
    ~HugeJob() { if (worker.joinable()) worker.join(); }
    void say(const std::string& s) { std::lock_guard<std::mutex> lock(mutex); stage = s; }
};

void App::run()
{
    // VSA_AUTO_ATLAS=1: press "Make atlas" by itself, print the outcome and quit (end-to-end check without clicks)
    const bool autoAtlas = std::getenv("VSA_AUTO_ATLAS") != nullptr;
    int autoFrame = 0, autoDoneFrame = 0;
    bool autoDone = false;
    // VSA_BENCH_FRAMES=N: render N frames, print the average frame time and quit
    const char* benchEnv = std::getenv("VSA_BENCH_FRAMES");
    const int benchFrames = benchEnv ? std::atoi(benchEnv) : 0;
    int benchDone = 0;
    std::chrono::steady_clock::time_point benchStart;
    // VSA_AUTO_HUGE=file: "Make atlas (huge)" on that file by itself, save the PLY to VSA_AUTO_SAVE, print the outcome and quit
    const char* autoHuge = std::getenv("VSA_AUTO_HUGE");
    int hugeFrame = 0;
    while (!glfwWindowShouldClose(window_)) {
        glfwPollEvents();
        if (autoHuge) {
            if (hugeFrame == 2) {
                if (const char* envK = std::getenv("VSA_K")) hugeK_ = std::max(1, std::atoi(envK));
                if (const char* envF = std::getenv("VSA_HUGE_FACES")) hugeCoarseFaces_ = std::max(1000, std::atoi(envF));
                if (std::string(autoHuge) == "loaded") startHugeFromLoaded();   // VSA_AUTO_HUGE=loaded: the mesh of VSA_MESH
                else startHuge(autoHuge);
            }
            if (hugeFrame > 3 && hugeJob_ && hugeJob_->done && hugeJob_->worker.joinable() == false && hugeJob_->saved == false) {
                if (const char* view = std::getenv("VSA_HUGE_VIEW")) { hugeView_ = std::atoi(view); setHugeColors(); }
                std::printf("AUTO HUGE: %s\n", hugeStatus_.c_str());
                if (const char* save = std::getenv("VSA_AUTO_SAVE")) {
                    const bool ok = saveHuge(save);
                    std::printf("AUTO HUGE SAVE: %s (%s)\n", ok ? "ok" : "failed", hugeStatus_.c_str());
                }
                hugeJob_->saved = true;
                if (!std::getenv("VSA_SCREENSHOT")) glfwSetWindowShouldClose(window_, GLFW_TRUE);
            }
            ++hugeFrame;
        }
        if (benchFrames > 0) {
            if (benchDone == 5) benchStart = std::chrono::steady_clock::now();   // skip the first frames
            if (benchDone == 5 + benchFrames) {
                const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - benchStart).count();
                std::printf("FRAMES: %d frames, %.2f ms per frame (%.1f fps), mesh draw %.2f ms on the GPU\n", benchFrames,
                            ms / benchFrames, 1000.0 * benchFrames / ms, gpuDrawMsAvg_);
                std::printf("SETTINGS: msaa=%d cull=%d wireframe=%d faces=%zu\n", msaaSamples_, int(cullBackFaces_), int(wireframe_),
                            meshRenderer_.faceCount());
                glfwSetWindowShouldClose(window_, GLFW_TRUE);
            }
            ++benchDone;
        }
        if (autoAtlas) {
            if (autoFrame == 2) startMakeAtlas();
            if (autoFrame > 3 && atlasStage_ == AtlasStage::Idle && !autoDone) {
                std::printf("AUTO ATLAS: %s\n", makeAtlasStatus_.c_str());
                if (std::getenv("VSA_AUTO_BAKE") && atlasValid_) {   // bake the atlas texture, compare with the original layout
                    measureLayouts();
                    const UVLayoutStats &o = layoutOriginal_, &n = layoutOurs_;
                    std::printf("AUTO LAYOUT original: charts %d seams %.2f%% flipped %d L2 mean %.3f p95 %.3f conformal p95 %.3f density [%.2f, %.2f] covered %.1f%%\n",
                                o.charts, 100 * o.seamRatio(), o.flipped, o.l2Mean, o.l2P95, o.conformalP95, o.densityP05, o.densityP95, 100 * o.covered);
                    std::printf("AUTO LAYOUT new     : charts %d seams %.2f%% flipped %d L2 mean %.3f p95 %.3f conformal p95 %.3f density [%.2f, %.2f] covered %.1f%%\n",
                                n.charts, 100 * n.seamRatio(), n.flipped, n.l2Mean, n.l2P95, n.conformalP95, n.densityP05, n.densityP95, 100 * n.covered);
                    std::printf("AUTO RETEXTURE: PSNR %.1f dB (%.1f dB against the full resolution source), %zu samples\n", retexture_.psnr,
                                retexture_.psnrFull, retexture_.samples);
                    colorMode_ = 11;
                    uploadMesh();
                }
                if (const char* save = std::getenv("VSA_AUTO_SAVE")) {   // then File > Save As to that path
                    performSave(save);
                    std::printf("AUTO SAVE: %s\n", status_.c_str());
                }
                autoDone = true;
                autoDoneFrame = autoFrame;
            }
            // with a screenshot requested the window stays for a few frames after the pipeline, then it is taken and we quit
            if (autoDone && !std::getenv("VSA_SCREENSHOT")) glfwSetWindowShouldClose(window_, GLFW_TRUE);
            ++autoFrame;
        }

        // Minimized window: do not spin frames needlessly
        if (glfwGetWindowAttrib(window_, GLFW_ICONIFIED)) {
            glfwWaitEventsTimeout(0.1);
            continue;
        }

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        drawUI();
        ImGui::Render();

        // On Retina the framebuffer size in pixels != the window size in points
        int fbW = 0, fbH = 0;
        glfwGetFramebufferSize(window_, &fbW, &fbH);
        renderScene(fbW, fbH);

        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        if (const char* shot = std::getenv("VSA_SCREENSHOT")) {   // testing hook: write the 8th frame to a PNG and quit
            static int frame = 0;
            const bool hugeReady = !autoHuge || (hugeJob_ && hugeJob_->done && hugeShown_);
            if ((!autoAtlas || autoDone) && hugeReady && ++frame == 8) {
                std::vector<unsigned char> px(size_t(fbW) * fbH * 4), flipped(px.size());
                glReadPixels(0, 0, fbW, fbH, GL_RGBA, GL_UNSIGNED_BYTE, px.data());
                for (int y = 0; y < fbH; ++y)
                    std::copy(px.begin() + size_t(fbH - 1 - y) * fbW * 4, px.begin() + size_t(fbH - y) * fbW * 4,
                              flipped.begin() + size_t(y) * fbW * 4);
                for (size_t i = 3; i < flipped.size(); i += 4) flipped[i] = 255;
                writePNG(shot, fbW, fbH, flipped);
                glfwSetWindowShouldClose(window_, GLFW_TRUE);
            }
        }
        glfwSwapBuffers(window_);
    }
}

void App::openMesh(const std::string& path)
{
    hugeShown_ = false;
    const auto t0 = std::chrono::steady_clock::now();
    TriMesh m;
    std::string err;
    ObjExtras extras;
    const bool isObj = path.size() > 4 && [&] {
        std::string ext = path.substr(path.size() - 4);
        for (char& c : ext) c = char(std::tolower(static_cast<unsigned char>(c)));
        return ext == ".obj";
    }();
    if (isObj ? !loadObj(path, m, &extras, &err) : !loadMesh(path, m, &err)) {
        status_ = err;
        return;
    }
    const double readSeconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    normalizeToUnitBox(m);
    mesh_ = std::move(m);
    faceData_ = computeFaceData(mesh_);
    diag_ = diagnose(mesh_, faceData_);
    faceWeights_.assign(mesh_.n_faces(), 1.0f);
    faceMoments_ = computeFaceMoments(mesh_, faceData_);
    wholeMesh_ = Moments();
    for (const Moments& fm : faceMoments_)
        wholeMesh_ += fm;
    vsa_ = VSAState();
    vsa_.metric = vsaMetric_ == 0 ? Metric::L21 : Metric::L2;
    vsaAdded_ = vsaSteps_ = 0;
    job_.reset();
    lloydStatus_.clear();
    stopRefine(true);
    refineHasStats_ = false;
    patchesRefined_ = false;
    patches_ = ExtractionResult();
    patchesValid_ = false;
    selectedPatch_ = -1;
    linabf_.clear();
    linabfSolved_.clear();
    uv_.clear();
    uvStats_.clear();
    uvDone_.clear();
    params_.clear();
    paramDone_.clear();
    atlasValid_ = false;
    paramStatus_.clear();
    camera_.reset();

    // the texture of the file (if it has uv and a material with a diffuse map)
    objExtras_ = std::move(extras);
    hasSourceTexture_ = false;
    textureMaterial_ = -1;
    textureNote_.clear();
    sourcePixels_.clear();
    sourceW_ = sourceH_ = 0;
    bakedRgba_.clear();
    bakedVersion_ = ~0u;
    textureOwner_ = 0;
    layoutMeasured_ = false;
    meshPath_ = path;
    savePath_.clear();
    if (objExtras_.hasUV()) {
        for (size_t i = 0; i < objExtras_.materials.size(); ++i) {
            if (objExtras_.materials[i].diffuseMap.empty()) continue;
            int w = 0, h = 0;
            std::vector<unsigned char> pixels;
            std::string imageError;
            const std::string image = objExtras_.directory + "/" + objExtras_.materials[i].diffuseMap;
            if (loadImageRGBA(image, w, h, pixels, &imageError) && meshRenderer_.setTexture(w, h, pixels.data())) {
                hasSourceTexture_ = true;
                textureOwner_ = 1;
                sourcePixels_ = std::move(pixels);
                sourceW_ = w;
                sourceH_ = h;
                textureMaterial_ = int(i);
                char note[256];
                std::snprintf(note, sizeof(note), "texture %s (%d x %d)", objExtras_.materials[i].diffuseMap.c_str(), w, h);
                textureNote_ = note;
            } else {
                textureNote_ = imageError;
            }
            break;   // one texture is shown; faces of other materials stay gray
        }
    }

    // big scans: the expensive settings off by default (the user can switch them on)
    const size_t faces = mesh_.n_faces();
    cullBackFaces_ = faces > 500000;
    if (faces > 500000) { wireframe_ = false; msaaSamples_ = std::min(msaaSamples_, 2); }
    if (faces > 1500000) msaaSamples_ = 0;
    colorMode_ = hasSourceTexture_ ? 10 : 0;
    meshRenderer_.invalidateGeometry();
    uploadMesh();
    const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    std::printf("loaded %s: %zu vertices, %zu faces in %.2f s (read %.2f s)%s%s\n", path.c_str(), mesh_.n_vertices(), faces,
                seconds, readSeconds, hasSourceTexture_ ? ", " : "", textureNote_.c_str());
    status_ = "loaded: " + path + (textureNote_.empty() ? "" : " | " + textureNote_);
}

namespace {

void hueColor(int index, float out[3])
{
    // golden-ratio hue steps give well-separated colors for any number of regions
    const float h = std::fmod(float(index) * 0.61803399f, 1.0f) * 6.0f;
    const float x = 1.0f - std::fabs(std::fmod(h, 2.0f) - 1.0f);
    const float c = 0.85f, m = 0.1f;
    float r = 0, g = 0, b = 0;
    switch (int(h)) {
        case 0: r = c; g = x; break;
        case 1: r = x; g = c; break;
        case 2: g = c; b = x; break;
        case 3: g = x; b = c; break;
        case 4: r = x; b = c; break;
        default: r = c; b = x; break;
    }
    out[0] = r + m; out[1] = g + m; out[2] = b + m;
}

// weight 1 is neutral gray-blue, heavier goes to red, lighter to blue (log scale)
void weightColor(float w, float out[3])
{
    const float neutral[3] = {0.55f, 0.58f, 0.65f};
    const float lw = std::max(-3.0f, std::min(4.0f, std::log2(std::max(w, 1e-6f))));
    static const float heavy[3] = {1.0f, 0.2f, 0.1f}, light[3] = {0.1f, 0.25f, 0.95f};
    const float* target = lw >= 0 ? heavy : light;
    const float t = lw >= 0 ? lw / 4.0f : -lw / 3.0f;
    for (int k = 0; k < 3; ++k) out[k] = neutral[k] + (target[k] - neutral[k]) * t;
}

} // namespace

void App::uploadMesh()
{
    if (hugeShown_) return;   // the 3D view belongs to the huge result until another mesh is opened
    std::vector<float> colors;
    const size_t nF = mesh_.n_faces();
    if (colorMode_ == 1) {
        colors.resize(nF * 3);
        for (size_t i = 0; i < nF; ++i)
            for (int k = 0; k < 3; ++k)
                colors[i * 3 + k] = float(faceData_.normal[i][k] * 0.5 + 0.5);
    } else if (colorMode_ == 2 && vsa_.label.size() == nF) {
        colors.resize(nF * 3);
        for (size_t i = 0; i < nF; ++i) {
            const int l = vsa_.label[i];
            if (l < 0) colors[i * 3] = colors[i * 3 + 1] = colors[i * 3 + 2] = 0.3f;
            else hueColor(l, &colors[i * 3]);
        }
    } else if (colorMode_ == 3 && faceWeights_.size() == nF) {
        colors.resize(nF * 3);
        for (size_t i = 0; i < nF; ++i)
            weightColor(faceWeights_[i], &colors[i * 3]);
    }
    else if ((colorMode_ == 4 || colorMode_ == 5) && patchesValid_ && patches_.faceToPatch.size() == nF) {
        colors.resize(nF * 3);
        for (size_t i = 0; i < nF; ++i) {
            const int pi = patches_.faceToPatch[i];
            float* c = &colors[i * 3];
            if (pi < 0) { c[0] = c[1] = c[2] = 0.3f; continue; }
            const Patch& p = patches_.patches[pi];
            if (colorMode_ == 4) {
                hueColor(pi, c);
            } else if (p.isDisk()) {
                const float disk[3] = {0.35f, 0.75f, 0.4f}, cut[3] = {0.3f, 0.7f, 0.85f};
                for (int k = 0; k < 3; ++k) c[k] = p.cuts ? cut[k] : disk[k];
            } else {
                c[0] = 0.9f; c[1] = 0.25f; c[2] = 0.2f;
            }
            if (pi == selectedPatch_) c[0] = c[1] = c[2] = 1.0f;
        }
    }

    else if (colorMode_ == 6 && patchesValid_ && patches_.faceToPatch.size() == nF) {
        colors.assign(nF * 3, 0.3f);
        for (size_t pi = 0; pi < patches_.patches.size() && pi < linabf_.size(); ++pi) {
            if (!linabfSolved_[pi] || !linabf_[pi].ok) continue;
            const Patch& p = patchAt(pi);
            const std::vector<double> orig = originalAngles(p.mesh);
            for (auto lf : p.mesh.faces()) {
                double e = 0.0;
                for (int k = 0; k < 3; ++k)
                    e = std::max(e, std::abs(linabf_[pi].angles[3 * lf.idx() + k] - orig[3 * lf.idx() + k]));
                const float t = float(std::min(1.0, e / 0.3));   // 0.3 rad (about 17 degrees) saturates
                float* c = &colors[size_t(p.faceToGlobal[lf.idx()]) * 3];
                c[0] = 0.35f + 0.6f * t;
                c[1] = 0.65f - 0.45f * t;
                c[2] = 0.45f - 0.3f * t;
            }
        }
    }

    std::vector<float> faceUV;
    if (colorMode_ == 11 && bakedVersion_ == atlasVersion_ && bakedCornerUV_.size() == nF * 6) {
        faceUV = bakedCornerUV_;   // the atlas uv; faces without a chart get u = -1 (no texture)
        for (size_t f = 0; f < nF; ++f)
            if (std::isnan(faceUV[f * 6])) for (int k = 0; k < 6; ++k) faceUV[f * 6 + k] = -1.0f;
        if (textureOwner_ != 2) { meshRenderer_.setTexture(bakedResolution_, bakedResolution_, bakedRgba_.data()); textureOwner_ = 2; }
    }
    if (colorMode_ == 10 && hasSourceTexture_ && textureOwner_ != 1 && !sourcePixels_.empty()) {
        meshRenderer_.setTexture(sourceW_, sourceH_, sourcePixels_.data());
        textureOwner_ = 1;
    }
    if (colorMode_ == 10 && hasSourceTexture_ && objExtras_.cornerUV.size() == nF * 6) {
        // the uv of the file; faces of other materials get u = -1 (no texture)
        faceUV = objExtras_.cornerUV;
        for (size_t f = 0; f < nF; ++f)
            if (objExtras_.faceMaterial.size() != nF || objExtras_.faceMaterial[f] != textureMaterial_)
                for (int k = 0; k < 6; ++k) faceUV[f * 6 + k] = -1.0f;
    }
    if ((colorMode_ == 7 || colorMode_ == 8 || colorMode_ == 9) && patchesValid_ && params_.size() == patches_.patches.size() &&
        patches_.faceToPatch.size() == nF) {
        colors.assign(nF * 3, 0.3f);
        if (colorMode_ == 8) faceUV.assign(nF * 6, 0.0f);
        for (size_t pi = 0; pi < params_.size(); ++pi) {
            if (!paramDone_[pi] || params_[pi].method == ParamMethod::None) continue;
            const PatchParam& pp = params_[pi];
            const TriMesh& pm = pp.patch.mesh;
            std::vector<char> flagged;
            if (colorMode_ == 9) {
                flagged.assign(pm.n_faces(), 0);
                for (int f : pp.report.uv.flippedFaces) flagged[f] = 1;
                for (const auto& cr : pp.report.uv.crossings)
                    for (int he : {cr.first, cr.second}) {
                        const auto f = pm.face_handle(pm.opposite_halfedge_handle(TriMesh::HalfedgeHandle(he)));
                        if (f.is_valid()) flagged[f.idx()] = 1;
                    }
                for (int v : pp.report.angles.foldVertices)
                    for (auto f : pm.vf_range(TriMesh::VertexHandle(v))) flagged[f.idx()] = 1;
            }
            for (auto lf : pm.faces()) {
                const size_t g = size_t(pp.patch.faceToGlobal[lf.idx()]);
                float* c = &colors[g * 3];
                if (colorMode_ == 7) {
                    const float v = pp.perTriangleL2[lf.idx()];
                    if (v < 0.0f) { c[0] = 0.9f; c[1] = 0.1f; c[2] = 0.1f; continue; }
                    const float t = std::max(0.0f, std::min(1.0f, (v - 1.0f) / 2.0f));   // 1 = isometric, 3 = red
                    c[0] = 0.2f + 0.75f * t; c[1] = 0.55f - 0.3f * t; c[2] = 0.9f - 0.75f * t;
                } else if (colorMode_ == 8) {
                    c[0] = c[1] = c[2] = 0.8f;
                    if (atlasValid_ && pi < atlas_.uv.size() && !atlas_.uv[pi].empty()) {
                        int k = 0;
                        for (auto fv : pm.fv_range(lf)) {
                            faceUV[g * 6 + k * 2] = float(atlas_.uv[pi][fv.idx()].x());
                            faceUV[g * 6 + k * 2 + 1] = float(atlas_.uv[pi][fv.idx()].y());
                            ++k;
                        }
                    }
                } else {
                    if (flagged[lf.idx()]) { c[0] = 0.95f; c[1] = 0.1f; c[2] = 0.1f; }
                    else if (pp.needsSplit) { c[0] = 0.95f; c[1] = 0.6f; c[2] = 0.15f; }
                    else { c[0] = 0.3f; c[1] = 0.75f; c[2] = 0.4f; }
                }
            }
        }
    }

    // exploded view: every patch moves away from the mesh center
    std::vector<float> offsets;
    const std::vector<Vec3> po = patchExplodeOffsets();
    if (!po.empty()) {
        offsets.assign(nF * 3, 0.0f);
        for (size_t i = 0; i < nF; ++i) {
            const int pi = patches_.faceToPatch[i];
            if (pi < 0) continue;
            for (int k = 0; k < 3; ++k) offsets[i * 3 + k] = float(po[pi][k]);
        }
    }
    meshRenderer_.upload(mesh_, colors, offsets, faceUV);
    rebuildBoundaryLines();
    boundaryStatsValid_ = vsa_.label.size() == nF;
    if (boundaryStatsValid_) boundaryStats_ = computeBoundaryStats(mesh_, vsa_.label);
}

std::vector<Vec3> App::patchExplodeOffsets() const
{
    const size_t nF = mesh_.n_faces();
    if (explode_ <= 0.0f || !patchesValid_ || patches_.faceToPatch.size() != nF) return {};
    std::vector<Vec3> center(patches_.patches.size(), Vec3(0, 0, 0));
    std::vector<double> area(patches_.patches.size(), 0.0);
    for (size_t i = 0; i < nF; ++i) {
        const int pi = patches_.faceToPatch[i];
        if (pi < 0) continue;
        center[pi] += faceData_.centroid[i] * faceData_.area[i];
        area[pi] += faceData_.area[i];
    }
    for (size_t p = 0; p < center.size(); ++p)
        center[p] = area[p] > 0.0 ? center[p] / area[p] * double(explode_) : Vec3(0, 0, 0);
    return center;
}

// Boundary of every patch: seams between patches (black), slits cut into a patch (magenta).
// The mesh border counts as a seam.
void App::rebuildBoundaryLines()
{
    std::vector<float> segs;
    if (showBoundaries_ && patchesValid_ && patches_.faceToPatch.size() == mesh_.n_faces()) {
        const std::vector<Vec3> po = patchExplodeOffsets();
        for (size_t pi = 0; pi < patches_.patches.size(); ++pi) {
            const Patch& p = patchAt(pi);
            const Vec3 off = po.empty() ? Vec3(0, 0, 0) : po[pi];
            for (auto h : p.mesh.halfedges()) {
                if (!p.mesh.is_boundary(h)) continue;
                const auto u = p.mesh.from_vertex_handle(h), w = p.mesh.to_vertex_handle(h);

                // a slit: the same edge of the source mesh has faces of this very patch on both sides
                bool slit = false;
                const auto gh = mesh_.find_halfedge(TriMesh::VertexHandle(p.vertexToGlobal[u.idx()]),
                                                    TriMesh::VertexHandle(p.vertexToGlobal[w.idx()]));
                if (gh.is_valid()) {
                    const auto f1 = mesh_.face_handle(gh), f2 = mesh_.face_handle(mesh_.opposite_halfedge_handle(gh));
                    slit = f1.is_valid() && f2.is_valid() &&
                           patches_.faceToPatch[f1.idx()] == int(pi) && patches_.faceToPatch[f2.idx()] == int(pi);
                }
                const Vec3 a = p.mesh.point(u) + off, b = p.mesh.point(w) + off;
                const float col[3] = {slit ? 0.95f : 0.02f, slit ? 0.1f : 0.02f, slit ? 0.85f : 0.02f};
                segs.insert(segs.end(), {float(a[0]), float(a[1]), float(a[2]), float(b[0]), float(b[1]),
                                         float(b[2]), col[0], col[1], col[2]});
            }
        }
    }
    meshRenderer_.uploadLines(segs);
}

void App::extractPatchesNow()
{
    patches_ = extractPatches(mesh_, vsa_.label, patchOptions_);
    const size_t n = patches_.patches.size();
    linabf_.assign(n, LinABFResult());
    linabfSolved_.assign(n, 0);
    uv_.assign(n, UVResult());
    uvStats_.assign(n, UVStats());
    uvDone_.assign(n, 0);
    params_.assign(n, PatchParam());
    paramDone_.assign(n, 0);
    atlasValid_ = false;
    paramStatus_.clear();
    patchesValid_ = true;
    patchedLabels_ = vsa_.label;
    patchesRefined_ = false;
    selectedPatch_ = -1;
    colorMode_ = 5;
    uploadMesh();
}

void App::drawPatchPanel()
{
    if (!ImGui::Begin("Patches")) { ImGui::End(); return; }
    if (vsa_.label.size() != mesh_.n_faces()) {
        ImGui::TextUnformatted("Run a partition first (VSA panel)");
        ImGui::End();
        return;
    }
    ImGui::InputInt("Merge regions smaller than", &patchOptions_.minFaces);
    patchOptions_.minFaces = std::max(1, patchOptions_.minFaces);
    ImGui::Checkbox("Cut multi-boundary patches into disks", &patchOptions_.cutMultiBoundary);
    if (ImGui::Button("Extract patches")) extractPatchesNow();
    if (!patchesValid_) { ImGui::End(); return; }

    if (patchesRefined_)
        ImGui::TextColored(ImVec4(0.5f, 0.85f, 0.6f, 1.0f), "refined: patches were split after the VSA partition");
    else if (patchedLabels_ != vsa_.label)
        ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.3f, 1.0f), "stale: the partition changed since extraction");
    if (ImGui::SliderFloat("Explode", &explode_, 0.0f, 1.0f)) uploadMesh();
    if (ImGui::Checkbox("Show patch boundaries", &showBoundaries_)) rebuildBoundaryLines();
    if (showBoundaries_)
        ImGui::SliderFloat("Boundary width (px)", &boundaryWidth_, 1.0f, 8.0f, "%.1f");
    ImGui::TextDisabled("black: seams   magenta: cuts");
    if (ImGui::RadioButton("Patches", &colorMode_, 4)) uploadMesh();
    ImGui::SameLine();
    if (ImGui::RadioButton("Patch kind", &colorMode_, 5)) uploadMesh();

    int cutPatches = 0;
    for (const Patch& p : patches_.patches) cutPatches += p.cuts > 0;
    ImGui::Text("%d patches, %d disks (%d after cuts), %d regions merged",
                int(patches_.patches.size()), patches_.disks, cutPatches, patches_.mergedRegions);

    if (ImGui::BeginTable("patches", 8, ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY | ImGuiTableFlags_SizingFixedFit,
                          ImVec2(0, 220))) {
        const char* heads[] = {"#", "Faces", "Verts", "Chi", "Loops", "Genus", "Cuts", "Kind"};
        for (const char* h : heads) ImGui::TableSetupColumn(h);
        ImGui::TableSetupScrollFreeze(0, 1);
        ImGui::TableHeadersRow();
        for (int i = 0; i < int(patches_.patches.size()); ++i) {
            const Patch& p = patches_.patches[i];
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            char id[16];
            std::snprintf(id, sizeof(id), "%d", i);
            if (ImGui::Selectable(id, selectedPatch_ == i, ImGuiSelectableFlags_SpanAllColumns)) {
                selectedPatch_ = selectedPatch_ == i ? -1 : i;
                uploadMesh();
            }
            ImGui::TableSetColumnIndex(1); ImGui::Text("%d", p.topology.faces);
            ImGui::TableSetColumnIndex(2); ImGui::Text("%d", p.topology.vertices);
            ImGui::TableSetColumnIndex(3); ImGui::Text("%d", p.topology.euler);
            ImGui::TableSetColumnIndex(4); ImGui::Text("%d", p.topology.boundaryLoops);
            ImGui::TableSetColumnIndex(5); ImGui::Text("%d", p.topology.genus);
            ImGui::TableSetColumnIndex(6); ImGui::Text("%d", p.cuts);
            ImGui::TableSetColumnIndex(7); ImGui::TextUnformatted(patchKindName(p.topology.kind));
        }
        ImGui::EndTable();
    }
    if (selectedPatch_ >= 0 && selectedPatch_ < int(patches_.patches.size()) &&
        !patches_.patches[selectedPatch_].note.empty())
        ImGui::TextWrapped("Patch %d: %s", selectedPatch_, patches_.patches[selectedPatch_].note.c_str());
    ImGui::End();
}

void App::solvePatches(bool onlySelected)
{
    const auto t0 = std::chrono::steady_clock::now();
    for (size_t i = 0; i < patches_.patches.size(); ++i) {
        if (onlySelected && int(i) != selectedPatch_) continue;
        if (!patches_.patches[i].isDisk()) continue;
        linabf_[i] = solveLinABF(patchAt(i).mesh, linabfParams_);
        linabfSolved_[i] = 1;
        uvDone_[i] = 0;
    }
    linabfSeconds_ = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    colorMode_ = 6;
    uploadMesh();
}

void App::computeUV(bool onlySelected)
{
    for (size_t i = 0; i < patches_.patches.size(); ++i) {
        if (onlySelected && int(i) != selectedPatch_) continue;
        if (!linabfSolved_[i] || !linabf_[i].ok) continue;
        const TriMesh& m = patchAt(i).mesh;
        uv_[i] = uvGreedy_ ? anglesToUVGreedy(m, linabf_[i].angles) : anglesToUV(m, linabf_[i].angles);
        uvStats_[i] = uv_[i].ok ? analyzeUV(m, uv_[i].uv, linabf_[i].angles) : UVStats();
        uvDone_[i] = 1;
    }
}

void App::drawHugeAtlas()
{
    const FlatResult& r = hugeJob_->result;
    const AtlasResult& atlas = hugeJob_->atlas;
    ImGui::Checkbox("Edges", &uvShowEdges_);
    ImGui::SameLine();
    if (ImGui::Button("Fit view")) { uvZoom_ = 1.0f; uvPan_ = ImVec2(0, 0); }
    ImGui::Text("Atlas of the huge result: %zu patches, %d x %d, efficiency %.1f%%%s", r.patches.size(), atlas.resolution,
                atlas.resolution, 100.0 * atlas.efficiency, hugeJob_->baked.empty() ? "" : ", baked texture");
    const ImVec2 avail = ImGui::GetContentRegionAvail();
    const ImVec2 origin = ImGui::GetCursorScreenPos();
    const ImVec2 canvas(std::max(avail.x, 64.0f), std::max(avail.y, 64.0f));
    ImGui::InvisibleButton("hugeuvcanvas", canvas, ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight);
    if (ImGui::IsItemHovered()) {
        const ImGuiIO& io = ImGui::GetIO();
        if (io.MouseWheel != 0.0f) uvZoom_ = std::max(0.05f, uvZoom_ * std::pow(1.15f, io.MouseWheel));
        if (ImGui::IsMouseDragging(ImGuiMouseButton_Left) || ImGui::IsMouseDragging(ImGuiMouseButton_Right)) {
            uvPan_.x += io.MouseDelta.x;
            uvPan_.y += io.MouseDelta.y;
        }
    }
    const float scale = 0.92f * std::min(canvas.x, canvas.y) * uvZoom_;
    auto toScreen = [&](double u, double v) {   // the unit square, v points up
        return ImVec2(origin.x + canvas.x * 0.5f + uvPan_.x + float(u - 0.5) * scale,
                      origin.y + canvas.y * 0.5f + uvPan_.y - float(v - 0.5) * scale);
    };
    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->PushClipRect(origin, ImVec2(origin.x + canvas.x, origin.y + canvas.y), true);
    dl->AddRectFilled(origin, ImVec2(origin.x + canvas.x, origin.y + canvas.y), IM_COL32(24, 26, 32, 255));
    const bool haveTexture = !hugeJob_->baked.empty() && meshRenderer_.hasTexture();
    if (haveTexture)   // GL row 0 is v = 0: v = 1 is the top of the picture
        dl->AddImage(ImTextureID(uintptr_t(meshRenderer_.textureId())), toScreen(0, 1), toScreen(1, 0), ImVec2(0, 1), ImVec2(1, 0));
    dl->AddRect(toScreen(0, 1), toScreen(1, 0), IM_COL32(120, 120, 130, 255));
    if (uvShowEdges_ || !haveTexture) {
        for (size_t i = 0; i < r.patches.size() && i < atlas.uv.size(); ++i) {
            const auto& pts = atlas.uv[i];
            if (pts.empty()) continue;
            float rgb[3];
            hueColor(int(i), rgb);
            const ImU32 col = haveTexture ? IM_COL32(250, 220, 90, 255) : IM_COL32(int(rgb[0] * 255), int(rgb[1] * 255), int(rgb[2] * 255), 255);
            for (const auto& e : r.patches[i].boundaryEdges)
                dl->AddLine(toScreen(pts[e.first].x(), pts[e.first].y()), toScreen(pts[e.second].x(), pts[e.second].y()), col, 1.0f);
        }
    }
    dl->PopClipRect();
}

void App::drawUvPanel()
{
    ImGui::SetNextWindowSize(ImVec2(520, 520), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("UV view")) { ImGui::End(); return; }

    if (hugeShown_ && hugeJob_ && hugeJob_->done) {   // the atlas of the huge result: texture and chart borders
        drawHugeAtlas();
        ImGui::End();
        return;
    }
    const int sel = selectedPatch_;
    const bool haveParam = patchesValid_ && sel >= 0 && sel < int(params_.size()) && paramDone_[sel] &&
                           params_[sel].method != ParamMethod::None;
    const bool haveStage8 = patchesValid_ && sel >= 0 && sel < int(uv_.size()) && uvDone_[sel] && uv_[sel].ok;
    const bool canAtlas = atlasValid_ && !atlas_.uv.empty();
    if (canAtlas) {
        ImGui::Checkbox("Atlas", &uvShowAtlas_);
        ImGui::SameLine();
    } else {
        uvShowAtlas_ = false;
    }
    ImGui::Checkbox("Edges", &uvShowEdges_);
    ImGui::SameLine();
    if (ImGui::Button("Fit view")) { uvZoom_ = 1.0f; uvPan_ = ImVec2(0, 0); }

    // What to draw: list of (mesh, uv, color index) plus bounds
    struct Layer { const TriMesh* mesh; const std::vector<Eigen::Vector2d>* uv; int colorIndex; const PatchParam* pp; };
    std::vector<Layer> layers;
    if (uvShowAtlas_) {
        for (size_t i = 0; i < params_.size(); ++i)
            if (paramDone_[i] && i < atlas_.uv.size() && !atlas_.uv[i].empty())
                layers.push_back({&params_[i].patch.mesh, &atlas_.uv[i], int(i), &params_[i]});
    } else if (haveParam) {
        layers.push_back({&params_[sel].patch.mesh, &params_[sel].uv, sel, &params_[sel]});
    } else if (haveStage8) {
        layers.push_back({&patchAt(sel).mesh, &uv_[sel].uv, sel, nullptr});
    }
    if (layers.empty()) {
        ImGui::TextUnformatted("Select a patch and parameterize it (Parameterize panel)");
        ImGui::End();
        return;
    }

    if (!uvShowAtlas_ && haveParam) {
        const PatchParam& pp = params_[sel];
        ImGui::Text("Patch %d: %s%s, %d flipped, %zu crossings, %zu folds, %d cuts", sel, paramMethodName(pp.method),
                    pp.injective ? "" : " (NOT injective)", pp.distortion.flipped, pp.report.uv.crossings.size(),
                    pp.report.angles.foldVertices.size(), pp.cuts);
        ImGui::Text("L2 stretch: mean %.3f max %.3f   conformal max %.2f   area ratio [%.2f, %.2f]%s",
                    pp.distortion.l2Mean, pp.distortion.l2Max, pp.distortion.conformalMax, pp.distortion.areaRatioMin,
                    pp.distortion.areaRatioMax, pp.needsSplit ? "   NEEDS SPLIT" : "");
    } else if (uvShowAtlas_) {
        ImGui::Text("Atlas: %zu patches, efficiency %.1f%%", layers.size(), 100.0 * atlas_.efficiency);
    } else if (haveStage8) {
        const UVStats& st = uvStats_[sel];
        ImGui::Text("Patch %d: %d triangles, %d flipped, angle deviation max %.1e", sel, st.triangles, st.flipped,
                    st.maxAngleDeviation);
    }

    Eigen::Vector2d lo(1e300, 1e300), hi(-1e300, -1e300);
    for (const Layer& l : layers)
        for (const auto& p : *l.uv) { lo = lo.cwiseMin(p); hi = hi.cwiseMax(p); }
    if (uvShowAtlas_) { lo = Eigen::Vector2d(0, 0); hi = Eigen::Vector2d(1, 1); }
    const Eigen::Vector2d size = (hi - lo).cwiseMax(1e-12);

    const ImVec2 avail = ImGui::GetContentRegionAvail();
    const ImVec2 origin = ImGui::GetCursorScreenPos();
    const ImVec2 canvas(std::max(avail.x, 64.0f), std::max(avail.y, 64.0f));
    ImGui::InvisibleButton("uvcanvas", canvas, ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight);
    if (ImGui::IsItemHovered()) {
        const ImGuiIO& io = ImGui::GetIO();
        if (io.MouseWheel != 0.0f) uvZoom_ = std::max(0.05f, uvZoom_ * std::pow(1.15f, io.MouseWheel));
        if (ImGui::IsMouseDragging(ImGuiMouseButton_Left) || ImGui::IsMouseDragging(ImGuiMouseButton_Right)) {
            uvPan_.x += io.MouseDelta.x;
            uvPan_.y += io.MouseDelta.y;
        }
    }
    const float scale = float(0.92 * std::min(canvas.x / size.x(), canvas.y / size.y())) * uvZoom_;
    const Eigen::Vector2d mid = 0.5 * (lo + hi);
    auto toScreen = [&](const Eigen::Vector2d& p) {   // v points up on the screen
        return ImVec2(origin.x + canvas.x * 0.5f + uvPan_.x + float(p.x() - mid.x()) * scale,
                      origin.y + canvas.y * 0.5f + uvPan_.y - float(p.y() - mid.y()) * scale);
    };

    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->PushClipRect(origin, ImVec2(origin.x + canvas.x, origin.y + canvas.y), true);
    dl->AddRectFilled(origin, ImVec2(origin.x + canvas.x, origin.y + canvas.y), IM_COL32(24, 26, 32, 255));
    if (uvShowAtlas_)   // the unit square
        dl->AddRect(toScreen({0, 1}), toScreen({1, 0}), IM_COL32(120, 120, 130, 255));

    for (const Layer& l : layers) {
        const TriMesh& m = *l.mesh;
        const std::vector<Eigen::Vector2d>& uv = *l.uv;
        float rgb[3];
        hueColor(l.colorIndex, rgb);
        const ImU32 fill = uvShowAtlas_ ? IM_COL32(int(rgb[0] * 200), int(rgb[1] * 200), int(rgb[2] * 200), 255)
                                         : IM_COL32(70, 110, 150, 255);
        for (auto f : m.faces()) {
            Eigen::Vector2d p[3];
            for (auto h : m.fh_range(f)) p[cornerIndex(m, h) % 3] = uv[m.to_vertex_handle(h).idx()];
            const ImVec2 a = toScreen(p[0]), b = toScreen(p[1]), c = toScreen(p[2]);
            const bool flipped = signedArea(p[0], p[1], p[2]) <= 0.0;
            dl->AddTriangleFilled(a, b, c, flipped ? IM_COL32(220, 50, 40, 255) : fill);
            if (uvShowEdges_) {
                const ImU32 col = IM_COL32(15, 20, 28, 200);
                dl->AddLine(a, b, col);
                dl->AddLine(b, c, col);
                dl->AddLine(c, a, col);
            }
        }
        for (auto h : m.halfedges()) {   // patch boundary on top
            if (!m.is_boundary(h)) continue;
            dl->AddLine(toScreen(uv[m.from_vertex_handle(h).idx()]), toScreen(uv[m.to_vertex_handle(h).idx()]),
                        IM_COL32(250, 220, 90, 255), uvShowAtlas_ ? 1.0f : 2.0f);
        }
        if (l.pp && !uvShowAtlas_) {   // problems of the shown patch
            for (const auto& cr : l.pp->report.uv.crossings)
                for (int he : {cr.first, cr.second}) {
                    const auto h = TriMesh::HalfedgeHandle(he);
                    dl->AddLine(toScreen(uv[m.from_vertex_handle(h).idx()]), toScreen(uv[m.to_vertex_handle(h).idx()]),
                                IM_COL32(255, 60, 220, 255), 3.5f);
                }
            for (int v : l.pp->report.angles.foldVertices)
                dl->AddCircle(toScreen(uv[v]), 8.0f, IM_COL32(255, 150, 30, 255), 16, 2.5f);
        }
    }
    if (uvShowAtlas_ && !atlas_.horizon.empty()) {   // the final horizon of the horizon packer
        const int n = int(atlas_.horizon.size());
        const int step = std::max(1, n / 512);
        ImVec2 prev = toScreen({0.0, double(atlas_.horizon[0])});
        for (int x = step; x < n; x += step) {
            const ImVec2 cur = toScreen({double(x) / n, double(atlas_.horizon[x])});
            dl->AddLine(prev, cur, IM_COL32(255, 140, 40, 255), 1.5f);
            prev = cur;
        }
    }
    dl->PopClipRect();
    ImGui::End();
}

void App::runParam(bool onlySelected)
{
    const auto t0 = std::chrono::steady_clock::now();
    std::vector<int> work;
    for (size_t i = 0; i < patches_.patches.size(); ++i) {
        if (onlySelected && int(i) != selectedPatch_) continue;
        if (!patches_.patches[i].isDisk()) continue;
        work.push_back(int(i));
    }
    std::sort(work.begin(), work.end(), [&](int a, int b) {   // the biggest first, so that the threads finish together
        return patches_.patches[a].faceToGlobal.size() > patches_.patches[b].faceToGlobal.size();
    });
    ThreadPool::shared().parallelFor(int(work.size()), [&](int t) {
        const int i = work[t];
        params_[i] = parameterizePatch(mesh_, patches_.patches[i], paramOptions_);
        paramDone_[i] = 1;
    });
    const int done = int(work.size());
    paramSeconds_ = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    atlasValid_ = false;
    char buf[96];
    std::snprintf(buf, sizeof(buf), "parameterized %d patches in %.2f s", done, paramSeconds_);
    paramStatus_ = buf;
    colorMode_ = 9;
    uploadMesh();
}

void App::packAtlasNow()
{
    std::vector<AtlasInput> in(params_.size());
    for (size_t i = 0; i < params_.size(); ++i) {
        if (!paramDone_[i] || params_[i].method == ParamMethod::None) continue;
        const TriMesh& m = params_[i].patch.mesh;
        double uvArea = 0.0, area3 = 0.0;
        for (auto f : m.faces()) {
            Eigen::Vector2d q[3];
            Vec3 p[3];
            for (auto h : m.fh_range(f)) {
                const int k = cornerIndex(m, h) % 3;
                q[k] = params_[i].uv[m.to_vertex_handle(h).idx()];
                p[k] = m.point(m.to_vertex_handle(h));
            }
            uvArea += std::abs(signedArea(q[0], q[1], q[2]));
            area3 += 0.5 * ((p[1] - p[0]) % (p[2] - p[0])).norm();
        }
        in[i] = {&params_[i].uv, uvArea, area3, &m};
    }
    atlas_ = atlasMethod_ == 0 ? packHorizon(in, horizonOptions_) : packAtlas(in, atlasOptions_);
    if (!atlas_.ok) {
        paramStatus_ = "packing failed: " + atlas_.message;
        atlasValid_ = false;
        return;
    }
    atlasValid_ = true;
    ++atlasVersion_;
    layoutMeasured_ = false;
    uvShowAtlas_ = true;
    colorMode_ = 8;
    uploadMesh();
}

void App::startRefine()
{
    refineOptions_.param = paramOptions_;
    refineOptions_.patch = patchOptions_;
    refineOptions_.metric = vsaMetric_ == 0 ? Metric::L21 : Metric::L2;
    stopRefine(true);
    refineRounds_ = 0;
    refiner_ = std::make_unique<Refiner>(mesh_, patches_.faceToPatch, refineOptions_);
    refineHasStats_ = false;
    paramStatus_.clear();
}

void App::applyRefineResult()
{
    RefineResult r = refiner_->takeResult();   // moved: no copy of the patches
    patches_ = std::move(r.extraction);
    params_ = std::move(r.params);
    const size_t n = patches_.patches.size();
    paramDone_.assign(n, 0);
    for (size_t i = 0; i < n; ++i) paramDone_[i] = params_[i].method != ParamMethod::None || !patches_.patches[i].isDisk();
    linabf_.assign(n, LinABFResult());
    linabfSolved_.assign(n, 0);
    uv_.assign(n, UVResult());
    uvStats_.assign(n, UVStats());
    uvDone_.assign(n, 0);
    patchedLabels_ = std::move(r.labels);
    patchesValid_ = true;
    patchesRefined_ = true;
    atlasValid_ = false;
    selectedPatch_ = -1;
    refineStats_ = r.stats;
    refineHasStats_ = true;
    colorMode_ = 9;
    uploadMesh();
}

// One round of the refinement takes tens of seconds on a big scan: it runs on a worker thread, a round at a time, and the
// window stays alive (a modal window shows the progress and Cancel; it also keeps the rest of the interface away from the
// shared thread pool while the round runs).
void App::refineTick()
{
    if (!refiner_) return;
    if (refineBusy_) return;                         // a round is running
    if (refineThread_.joinable()) {
        refineThread_.join();
        if (refineStop_) { refiner_.reset(); refineStop_ = false; return; }
        if (refineMore_) { ++refineRounds_; }        // go on with the next round below
        else { applyRefineResult(); refiner_.reset(); return; }
    }
    refineBusy_ = true;
    refineThread_ = std::thread([this]() {
        refineMore_ = refiner_->step();
        refineBusy_ = false;
    });
}

// Stops the refinement: the round in progress finishes first, then the refiner is dropped (wait: block until that happens).
void App::stopRefine(bool wait)
{
    if (!refiner_) return;
    if (refineThread_.joinable() || refineBusy_) {
        refineStop_ = true;
        if (!wait) return;
        if (refineThread_.joinable()) refineThread_.join();
        refineBusy_ = false;
        refineStop_ = false;
    }
    refiner_.reset();
}

void App::drawRefineModal()
{
    if (!refiner_) return;
    ImGui::OpenPopup("Refining##modal");
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Always, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(ImVec2(440, 0), ImGuiCond_Always);
    if (ImGui::BeginPopupModal("Refining##modal", nullptr, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove)) {
        ImGui::Text("Parameterizing the patches and splitting the bad ones: round %d", refineRounds_ + 1);
        ImGui::ProgressBar(-float(ImGui::GetTime()), ImVec2(-1, 0), "");
        ImGui::TextDisabled("A round takes seconds to minutes on a big scan.");
        if (refineStop_) ImGui::TextUnformatted("Cancelling after this round...");
        else if (ImGui::Button("Cancel", ImVec2(-1, 0))) {
            refineStop_ = true;
            paramStatus_ = "refinement stopped";
            if (atlasStage_ == AtlasStage::Refine) { atlasStage_ = AtlasStage::Idle; makeAtlasStatus_ = "cancelled"; }
        }
        ImGui::EndPopup();
    }
}

void App::drawRefinePanel()
{
    refineTick();
    if (!ImGui::Begin("Refinement")) { ImGui::End(); return; }
    if (!patchesValid_ || params_.size() != patches_.patches.size()) {
        ImGui::TextUnformatted("Extract patches first (Patches panel)");
        ImGui::End();
        return;
    }
    const bool running = refiner_ != nullptr;

    int pol = refineOptions_.policy == RefinePolicy::FixFirst ? 0 : 1;
    ImGui::BeginDisabled(running);
    if (ImGui::RadioButton("Fix first (ladder, then split)", &pol, 0)) refineOptions_.policy = RefinePolicy::FixFirst;
    if (ImGui::RadioButton("Split immediately (plain LinABF)", &pol, 1)) refineOptions_.policy = RefinePolicy::SplitImmediately;
    ImGui::SliderInt("Max depth", &refineOptions_.maxDepth, 0, 10);
    ImGui::SliderInt("Min faces per part", &refineOptions_.minFaces, 2, 200);
    ImGui::SliderInt("Max rounds", &refineOptions_.maxRounds, 1, 30);
    {
        float lam = float(refineOptions_.smooth.lambda);
        if (ImGui::SliderFloat("Local smoothing lambda", &lam, 0.0f, 1.0f, "%.3f", ImGuiSliderFlags_Logarithmic))
            refineOptions_.smooth.lambda = lam;
    }
    ImGui::TextDisabled("Bad: not a disk, not injective, or L2 stretch above the Parameterize limit (%.1f)",
                        paramOptions_.maxStretch);
    if (ImGui::Button("Refine bad patches")) startRefine();
    ImGui::EndDisabled();
    if (running) {
        ImGui::SameLine();
        ImGui::Text("round %d running...", refineRounds_ + 1);
    }

    if (refineHasStats_) {
        const RefineStats& st = refineStats_;
        ImGui::SeparatorText("Last result");
        ImGui::Text("%d rounds, %.2f s: %d bad patches at the start, %d unresolved at the end", st.rounds, st.seconds,
                    st.initialBad, st.unresolved);
        ImGui::Text("%d splits: crossing hint %d, farthest %d, flood %d, Voronoi %d", st.splits, st.splitsByCrossingHint,
                    st.splitsByFarthest, st.splitsByFlood, st.splitsByVoronoi);
        ImGui::Text("%zu patches", patches_.patches.size());
        std::string hist = "patches by depth:";
        for (size_t d = 0; d < st.depthHistogram.size(); ++d)
            if (st.depthHistogram[d] > 0) hist += "  d" + std::to_string(d) + "=" + std::to_string(st.depthHistogram[d]);
        ImGui::TextUnformatted(hist.c_str());
        int byMethod[5] = {0, 0, 0, 0, 0};
        for (const PatchParam& p : params_) ++byMethod[int(p.method)];
        ImGui::Text("LinABF %d, active set %d, slit %d, Tutte %d, not parameterized %d", byMethod[int(ParamMethod::LinABF)],
                    byMethod[int(ParamMethod::ActiveSet)], byMethod[int(ParamMethod::Cut)], byMethod[int(ParamMethod::Tutte)],
                    byMethod[int(ParamMethod::None)]);
    }
    ImGui::End();
}

void App::drawParamPanel()
{
    if (!ImGui::Begin("Parameterize")) { ImGui::End(); return; }
    if (!patchesValid_ || params_.size() != patches_.patches.size()) {
        ImGui::TextUnformatted("Extract patches first (Patches panel)");
        ImGui::End();
        return;
    }

    ImGui::SeparatorText("Repair ladder");
    ImGui::Checkbox("Active set (pin angles, limit boundary fans)", &paramOptions_.useActiveSet);
    {
        float d = float(paramOptions_.active.foldDelta), mg = float(paramOptions_.active.angleMargin);
        if (ImGui::SliderFloat("Fold delta (rad)", &d, 0.0f, 0.5f)) paramOptions_.active.foldDelta = d;
        if (ImGui::SliderFloat("Angle margin (rad)", &mg, 0.001f, 0.2f)) paramOptions_.active.angleMargin = mg;
    }
    ImGui::Checkbox("Slits from the worst vertex", &paramOptions_.allowCut);
    ImGui::SliderInt("Max slits", &paramOptions_.maxCuts, 0, 6);
    ImGui::Checkbox("Tutte fallback (always injective)", &paramOptions_.allowTutte);
    {
        float ms = float(paramOptions_.maxStretch);
        if (ImGui::SliderFloat("Split above L2 stretch", &ms, 0.0f, 20.0f)) paramOptions_.maxStretch = ms;
    }

    ImGui::BeginDisabled(selectedPatch_ < 0);
    if (ImGui::Button("Parameterize selected")) runParam(true);
    ImGui::EndDisabled();
    ImGui::SameLine();
    if (ImGui::Button("Parameterize all disks")) runParam(false);

    // summary
    int done = 0, injective = 0, split = 0, byMethod[5] = {0, 0, 0, 0, 0};
    double worst = 0.0, sumL2 = 0.0;
    for (size_t i = 0; i < params_.size(); ++i) {
        if (!paramDone_[i]) continue;
        const PatchParam& pp = params_[i];
        ++done;
        injective += pp.injective;
        split += pp.needsSplit;
        ++byMethod[int(pp.method)];
        if (pp.method != ParamMethod::None) { worst = std::max(worst, pp.distortion.l2Max); sumL2 += pp.distortion.l2Mean; }
    }
    ImGui::Separator();
    if (!paramStatus_.empty()) ImGui::TextUnformatted(paramStatus_.c_str());
    if (done > 0) {
        ImGui::Text("%d patches: %d injective, %d need a split", done, injective, split);
        ImGui::Text("LinABF %d, active set %d, slit %d, Tutte %d, failed %d", byMethod[int(ParamMethod::LinABF)],
                    byMethod[int(ParamMethod::ActiveSet)], byMethod[int(ParamMethod::Cut)], byMethod[int(ParamMethod::Tutte)],
                    byMethod[int(ParamMethod::None)]);
        ImGui::Text("L2 stretch: mean of patches %.3f, worst triangle %.2f", sumL2 / std::max(1, done), worst);
        if (ImGui::RadioButton("Stretch", &colorMode_, 7)) uploadMesh();
        ImGui::SameLine();
        if (ImGui::RadioButton("Problems", &colorMode_, 9)) uploadMesh();
        ImGui::SameLine();
        if (ImGui::RadioButton("UV checker", &colorMode_, 8)) uploadMesh();
    }

    if (done > 0 && ImGui::BeginTable("params", 7, ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY | ImGuiTableFlags_SizingFixedFit,
                                      ImVec2(0, 180))) {
        const char* heads[] = {"#", "Method", "Inj.", "Flips", "Cross", "L2 mean", "L2 max"};
        for (const char* h : heads) ImGui::TableSetupColumn(h);
        ImGui::TableSetupScrollFreeze(0, 1);
        ImGui::TableHeadersRow();
        for (int i = 0; i < int(params_.size()); ++i) {
            if (!paramDone_[i]) continue;
            const PatchParam& pp = params_[i];
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            char id[16];
            std::snprintf(id, sizeof(id), "%d", i);
            if (ImGui::Selectable(id, selectedPatch_ == i, ImGuiSelectableFlags_SpanAllColumns)) {
                selectedPatch_ = selectedPatch_ == i ? -1 : i;
                uploadMesh();
            }
            ImGui::TableSetColumnIndex(1); ImGui::TextUnformatted(paramMethodName(pp.method));
            ImGui::TableSetColumnIndex(2); ImGui::TextUnformatted(pp.injective ? (pp.needsSplit ? "split" : "yes") : "NO");
            ImGui::TableSetColumnIndex(3); ImGui::Text("%d", pp.distortion.flipped);
            ImGui::TableSetColumnIndex(4); ImGui::Text("%zu", pp.report.uv.crossings.size());
            ImGui::TableSetColumnIndex(5); ImGui::Text("%.3f", pp.distortion.l2Mean);
            ImGui::TableSetColumnIndex(6); ImGui::Text("%.2f", pp.distortion.l2Max);
        }
        ImGui::EndTable();
    }
    if (selectedPatch_ >= 0 && selectedPatch_ < int(params_.size()) && paramDone_[selectedPatch_] &&
        !params_[selectedPatch_].log.empty()) {
        ImGui::SeparatorText("Log of the selected patch");
        ImGui::TextWrapped("%s", params_[selectedPatch_].log.c_str());
    }

    ImGui::SeparatorText("Atlas");
    ImGui::RadioButton("Horizon pack (texel exact, mip aware)", &atlasMethod_, 0);
    ImGui::RadioButton("Shelf pack (quick)", &atlasMethod_, 1);
    if (atlasMethod_ == 0) {
        HorizonOptions& ho = horizonOptions_;
        static const int kResolutions[] = {256, 512, 1024, 2048, 4096, 8192};
        int ri = 3;
        for (int i = 0; i < 6; ++i) if (kResolutions[i] == ho.resolution) ri = i;
        char label[16];
        std::snprintf(label, sizeof(label), "%d", ho.resolution);
        if (ImGui::BeginCombo("Texture size", label)) {
            for (int i = 0; i < 6; ++i) {
                std::snprintf(label, sizeof(label), "%d x %d", kResolutions[i], kResolutions[i]);
                if (ImGui::Selectable(label, i == ri)) ho.resolution = kResolutions[i];
            }
            ImGui::EndCombo();
        }
        int maxLevel = 0;
        while ((1 << (maxLevel + 1)) <= ho.resolution) ++maxLevel;
        maxLevel = std::max(0, maxLevel - 2);
        ho.maxMipLevel = std::min(ho.maxMipLevel, maxLevel);
        ImGui::SliderInt("Max mip level", &ho.maxMipLevel, 0, maxLevel);
        ImGui::SliderInt("Extra margin (texels)", &ho.extraTexels, 0, 16);
        ImGui::SliderInt("Atlas border (texels)", &ho.atlasBorder, 0, 32);
        ImGui::SliderInt("Rotations tried per chart", &ho.rotations, 1, 16);
        ImGui::Checkbox("Fill holes under overhangs", &ho.fillHoles);
        if (ho.fillHoles) {
            ImGui::SliderInt("Rotations tried per hole", &ho.holeRotations, 1, 16);
            ImGui::SliderInt("Threads (0 = all cores)", &ho.threads, 0, 16);
        }
        ImGui::Checkbox("Longest first by diameter (else by area)", &ho.sortByDiameter);
        const int gutter = chartGutter(ho.maxMipLevel, ho.extraTexels);
        ImGui::Text("Gap between charts: %d texels (%.2f%% of the side); each chart reaches %d texels into it", gutter,
                    100.0 * gutter / ho.resolution, chartMargin(ho.maxMipLevel, ho.extraTexels));
        ImGui::TextDisabled("Bake: extrude every chart border by that many texels into the gutter.");
    } else {
        float g = float(atlasOptions_.gutter);
        if (ImGui::SliderFloat("Gutter", &g, 0.0f, 0.05f, "%.3f")) atlasOptions_.gutter = g;
        ImGui::Checkbox("Rotate to minimum rectangle", &atlasOptions_.rotate);
    }
    ImGui::SliderInt("Checker squares", &checkerSquares_, 8, 200, "%d across the model");
    ImGui::TextDisabled("The UV checker follows the atlas: equal squares on the surface, whole texels of the atlas.");
    ImGui::BeginDisabled(done == 0);
    if (ImGui::Button("Pack atlas")) packAtlasNow();
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::BeginDisabled(!atlasValid_);
    if (ImGui::Button("Export OBJ...")) exportObjRequested_ = true;
    ImGui::SameLine();
    if (ImGui::Button("Export mask PNG...")) exportMaskRequested_ = true;
    ImGui::EndDisabled();
    if (atlasValid_) {
        ImGui::Text("Efficiency %.1f%%", 100.0 * atlas_.efficiency);
        if (atlas_.resolution > 0)
            ImGui::Text("%d x %d, density %.1f texels per unit, gutter %d", atlas_.resolution, atlas_.resolution,
                        atlas_.texelsPerUnit, atlas_.gutter);
    }

    ImGui::End();
}

void App::cancelMakeAtlas(const char* why)
{
    atlasJob_.reset();
    if (atlasStage_ == AtlasStage::Refine) stopRefine(false);
    atlasStage_ = AtlasStage::Idle;
    makeAtlasStatus_ = why;
}

void App::startMakeAtlas()
{
    if (!meshRenderer_.hasMesh()) return;
    job_.reset();
    stopRefine(true);
    atlasJob_.reset();

    // a fresh partition with the settings of the VSA panel (the paint weights count)
    vsa_ = VSAState();
    vsa_.metric = vsaMetric_ == 0 ? Metric::L21 : Metric::L2;
    vsa_.smooth = smoothParams_;
    if (seedMethod_ == 0) initRandomSeeds(vsaInput(), vsa_, vsaK_, uint64_t(vsaRngSeed_));
    else                  initFarthestSeeds(vsaInput(), vsa_, vsaK_, uint64_t(vsaRngSeed_));
    vsaAdded_ = vsaSteps_ = 0;
    lloydStatus_.clear();

    patches_ = ExtractionResult();
    patchesValid_ = false;
    patchesRefined_ = false;
    selectedPatch_ = -1;
    linabf_.clear(); linabfSolved_.clear();
    uv_.clear(); uvStats_.clear(); uvDone_.clear();
    params_.clear(); paramDone_.clear();
    atlasValid_ = false;
    refineHasStats_ = false;
    paramStatus_.clear();

    atlasJob_ = std::make_unique<RefineJob>(makeAtlasTeleport_ ? RefineJob::Kind::Teleport : RefineJob::Kind::Lloyd,
                                            lloydParams_, teleportParams_);
    atlasStage_ = AtlasStage::Lloyd;
    makeAtlasIter_ = 0;
    makeAtlasStatus_ = "1/4 partition...";
    colorMode_ = 2;
    uploadMesh();
}

void App::makeAtlasTick()
{
    if (atlasStage_ == AtlasStage::Idle) return;
    if (!meshRenderer_.hasMesh()) { cancelMakeAtlas("cancelled: no mesh"); return; }
    char buf[256];

    if (atlasStage_ == AtlasStage::Lloyd) {
        if (!atlasJob_) { cancelMakeAtlas("cancelled"); return; }
        const bool more = atlasJob_->step(vsaInput(), vsa_);
        ++makeAtlasIter_;
        ++vsaSteps_;
        uploadMesh();
        std::snprintf(buf, sizeof(buf), "1/4 partition into %d regions: iteration %d, energy %.5g", vsa_.k(),
                      makeAtlasIter_, totalEnergy(vsa_));
        makeAtlasStatus_ = buf;
        if (more) return;

        atlasJob_.reset();
        extractPatchesNow();     // 2/4
        startRefine();           // 3/4: refinement parameterizes every patch and splits the bad ones
        atlasStage_ = AtlasStage::Refine;
        makeAtlasStatus_ = "3/4 patches extracted, parameterizing...";
        return;
    }

    // Refine: refineTick() (in the Refinement panel code) steps the refiner and applies its result
    if (refiner_) {
        std::snprintf(buf, sizeof(buf), "3/4 parameterizing and refining: round %d, %zu patches",
                      refineRounds_ + 1, patches_.patches.size());
        makeAtlasStatus_ = buf;
        return;
    }
    if (!patchesRefined_) { cancelMakeAtlas("stopped"); return; }

    makeAtlasStatus_ = "4/4 packing the atlas...";
    atlasMethod_ = 0;   // the one-button pipeline always uses the horizon packer
    packAtlasNow();
    atlasStage_ = AtlasStage::Idle;
    if (!atlasValid_) {
        int notDisks = 0, unparameterized = 0;
        for (size_t i = 0; i < patches_.patches.size(); ++i) {
            notDisks += !patches_.patches[i].isDisk();
            unparameterized += params_[i].method == ParamMethod::None;
        }
        std::snprintf(buf, sizeof(buf),
                      "no patch could be flattened (%d of %zu patches are not disks, %d have no UV): %s. The mesh may be too small "
                      "for the limits: raise k, or lower 'Min faces per part' in the Refinement panel.",
                      notDisks, patches_.patches.size(), unparameterized, paramStatus_.c_str());
        makeAtlasStatus_ = buf;
        return;
    }

    size_t withUV = 0;
    for (size_t i = 0; i < params_.size(); ++i)
        if (i < atlas_.uv.size() && !atlas_.uv[i].empty()) withUV += params_[i].patch.faceToGlobal.size();
    std::snprintf(buf, sizeof(buf),
                  "done: %zu patches (%d still bad), %d x %d atlas, efficiency %.1f%%, gutter %d texels, %zu of %zu faces have UV",
                  patches_.patches.size(), refineStats_.unresolved, atlas_.resolution, atlas_.resolution,
                  100.0 * atlas_.efficiency, atlas_.gutter, withUV, mesh_.n_faces());
    makeAtlasStatus_ = buf;
}

// ---------------------------------------------------------------------------------------------- huge meshes

void App::startHuge(const std::string& path)
{
    if (hugeJob_ && !hugeJob_->done) return;
    if (atlasStage_ != AtlasStage::Idle || job_) { hugeStatus_ = "finish or cancel the running job first"; return; }
    auto job = std::make_shared<HugeJob>();
    job->path = path;
    hugeFromLoaded_ = false;
    runHugeJob(job);
}

// The mesh that is loaded (File > Open) goes through the huge pipeline as it is: flat arrays and the texture are filled from it.
void App::startHugeFromLoaded()
{
    if (hugeJob_ && !hugeJob_->done) return;
    if (atlasStage_ != AtlasStage::Idle || job_) { hugeStatus_ = "finish or cancel the running job first"; return; }
    if (!meshRenderer_.hasMesh() || mesh_.n_faces() == 0 || (hugeShown_ && !hugeFromLoaded_)) return;
    auto job = std::make_shared<HugeJob>();
    job->path = "(the loaded mesh)";
    job->preloaded = true;
    hugeFromLoaded_ = true;
    job->mesh = flatFromTriMesh(mesh_);        // corner k of a face = the k-th vertex of fv_range, as in objExtras_
    job->transform = FlatTransform();          // the loaded mesh is already normalized: the saved coordinates are the normalized ones
    if (objExtras_.hasUV() && objExtras_.cornerUV.size() == mesh_.n_faces() * 6) {
        job->surface.cornerUV = objExtras_.cornerUV;
        job->surface.materials = objExtras_.materials;
        job->surface.directory = objExtras_.directory;
        if (objExtras_.faceMaterial.size() == mesh_.n_faces()) {
            job->surface.faceMaterial.resize(mesh_.n_faces());
            for (size_t f = 0; f < mesh_.n_faces(); ++f) {
                const int m = objExtras_.faceMaterial[f];
                job->surface.faceMaterial[f] = m < 0 ? uint8_t(255) : uint8_t(std::min(m, 254));
            }
        }
    }
    if (hasSourceTexture_ && !sourcePixels_.empty() && job->surface.hasUV()) {
        job->sourcePixels = sourcePixels_;
        job->sourceW = sourceW_;
        job->sourceH = sourceH_;
        job->textureMaterial = textureMaterial_;
        job->textureName = "the texture of the loaded mesh";
    }
    runHugeJob(job);
}

void App::runHugeJob(const std::shared_ptr<HugeJob>& job)
{
    job->options.simplify.targetFaces = size_t(std::max(1000, hugeCoarseFaces_));
    job->options.k = std::max(1, hugeK_);
    job->options.lloyd.maxIterations = 30;
    job->horizon = horizonOptions_;
    HugeJob* raw = job.get();
    job->options.onStage = [raw](const std::string& s) { raw->say(s); };
    job->worker = std::thread([raw]() {
        using Clock = std::chrono::steady_clock;
        auto since = [](Clock::time_point t) { return std::chrono::duration<double>(Clock::now() - t).count(); };
        std::string err;
        auto t = Clock::now();
        if (!raw->preloaded) {
            raw->say("reading the mesh...");
            if (!loadFlatMesh(raw->path, raw->mesh, &raw->loadInfo, &err, &raw->surface)) {
                std::lock_guard<std::mutex> lock(raw->mutex);
                raw->error = err;
                raw->done = true;
                return;
            }
            raw->transform = normalizeFlatToUnitBox(raw->mesh);
            // the old texture: the first material with a map_Kd (OBJ) or the TextureFile comment (PLY)
            if (raw->surface.hasUV()) {
                std::string name = raw->surface.textureFile;
                for (size_t i = 0; i < raw->surface.materials.size() && name.empty(); ++i)
                    if (!raw->surface.materials[i].diffuseMap.empty()) { name = raw->surface.materials[i].diffuseMap; raw->textureMaterial = int(i); }
                if (!name.empty()) {
                    raw->say("reading the texture...");
                    std::string texErr;
                    if (loadImageRGBA(raw->surface.directory + "/" + name, raw->sourceW, raw->sourceH, raw->sourcePixels, &texErr)) raw->textureName = name;
                    else { std::lock_guard<std::mutex> lock(raw->mutex); raw->warning = "texture " + name + " not read: " + texErr; raw->sourcePixels.clear(); }
                }
            }
        }
        raw->loadSeconds = since(t);
        t = Clock::now();
        raw->result = runFlatPipeline(raw->mesh, raw->options);
        raw->pipelineSeconds = since(t);
        raw->say("packing the atlas...");
        t = Clock::now();
        raw->atlas = packFlatAtlas(raw->result, raw->horizon);
        raw->packSeconds = since(t);
        if (!raw->atlas.ok) {
            std::lock_guard<std::mutex> lock(raw->mutex);
            raw->error = "packing failed: " + raw->atlas.message;
        } else if (!raw->sourcePixels.empty()) {
            raw->say("baking the atlas texture...");
            t = Clock::now();
            BakeSource src{raw->sourceW, raw->sourceH, raw->sourcePixels.data()};
            BakeOptions bo;
            bo.resolution = raw->atlas.resolution;
            bo.margin = raw->atlas.margin;
            raw->baked = bakeAtlasFlat(raw->mesh, raw->result, raw->surface.cornerUV, raw->surface.faceMaterial, raw->textureMaterial, &src, bo);
            raw->bakedResolution = bo.resolution;
            raw->bakeSeconds = since(t);
            std::vector<float>().swap(raw->surface.cornerUV);   // the old uv and the old texture are not needed any more
            std::vector<uint8_t>().swap(raw->surface.faceMaterial);
            std::vector<unsigned char>().swap(raw->sourcePixels);
        }
        raw->done = true;
    });
    hugeJob_ = job;
    hugeStatus_.clear();
}

bool App::saveHuge(const std::string& path)
{
    if (!hugeJob_ || !hugeJob_->done) return false;
    bool ok = false;
    {
        std::string err;
        const auto t0 = std::chrono::steady_clock::now();
        std::string textureFile;
        if (!hugeJob_->baked.empty()) {   // the baked atlas next to the mesh; image row 0 is v = 1
            const size_t slash = path.find_last_of('/');
            const std::string dir = slash == std::string::npos ? "." : path.substr(0, slash);
            const std::string stem = path.substr(slash == std::string::npos ? 0 : slash + 1, path.size() - (slash == std::string::npos ? 0 : slash + 1) - 4);
            textureFile = stem + "_atlas.png";
            const int R = hugeJob_->bakedResolution;
            std::vector<unsigned char> flipped(hugeJob_->baked.size());
            const size_t row = size_t(R) * 4;
            for (int y = 0; y < R; ++y)
                std::copy(hugeJob_->baked.begin() + size_t(R - 1 - y) * row, hugeJob_->baked.begin() + size_t(R - y) * row, flipped.begin() + size_t(y) * row);
            std::string pngErr;
            if (!writePNG(dir + "/" + textureFile, R, R, flipped, &pngErr)) { err = "texture: " + pngErr; textureFile.clear(); }
        }
        if (err.empty() && writeFlatPlyAtlas(path, hugeJob_->mesh, hugeJob_->result, &hugeJob_->transform, &err, textureFile)) {
            char buf[300];
            std::snprintf(buf, sizeof(buf), "saved %s in %.0f s", path.c_str(),
                      std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count());
            hugeStatus_ = buf; ok = true;
        } else {
            hugeStatus_ = "save failed: " + err;
        }
    }
    status_ = hugeStatus_;
    return ok;
}

void App::drawHugeUI()
{
    // modal window while the job runs: the rest of the interface stays untouched (the pipeline owns the thread pool)
    if (hugeJob_ && !hugeJob_->done) {
        ImGui::OpenPopup("Huge mesh##progress");
        const ImVec2 c = ImGui::GetMainViewport()->GetCenter();
        ImGui::SetNextWindowPos(c, ImGuiCond_Always, ImVec2(0.5f, 0.5f));
        ImGui::SetNextWindowSize(ImVec2(460, 0), ImGuiCond_Always);
        if (ImGui::BeginPopupModal("Huge mesh##progress", nullptr, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove)) {
            std::string stage;
            { std::lock_guard<std::mutex> lock(hugeJob_->mutex); stage = hugeJob_->stage; }
            ImGui::TextWrapped("%s", hugeJob_->path.c_str());
            ImGui::Separator();
            ImGui::TextUnformatted(stage.c_str());
            const float t = float(ImGui::GetTime());
            ImGui::ProgressBar(-t, ImVec2(-1, 0), "");
            ImGui::TextDisabled("This cannot be cancelled. Large scans take minutes.");
            ImGui::EndPopup();
        }
    } else if (hugeJob_ && hugeJob_->done && !hugeJob_->saved && hugeJob_->worker.joinable()) {
        hugeJob_->worker.join();
        std::string err;
        { std::lock_guard<std::mutex> lock(hugeJob_->mutex); err = hugeJob_->error; }
        if (!err.empty()) {
            hugeStatus_ = "huge mesh failed: " + err;
            status_ = hugeStatus_;
        } else {
            const FlatStats& s = hugeJob_->result.stats;
            char buf[560];
            std::snprintf(buf, sizeof(buf),
                          "huge mesh: %zu faces -> %zu patches, %zu faces without UV, atlas %d x %d, efficiency %.1f%%%s "
                          "(read %.0f s, pipeline %.0f s, pack %.0f s, bake %.0f s)",
                          hugeJob_->mesh.faceCount(), hugeJob_->result.patches.size(), s.facesWithoutUV, hugeJob_->atlas.resolution,
                          hugeJob_->atlas.resolution, 100.0 * hugeJob_->atlas.efficiency,
                          hugeJob_->baked.empty() ? "" : ", texture baked", hugeJob_->loadSeconds, hugeJob_->pipelineSeconds,
                          hugeJob_->packSeconds, hugeJob_->bakeSeconds);
            hugeStatus_ = buf;
            { std::lock_guard<std::mutex> lock(hugeJob_->mutex); if (!hugeJob_->warning.empty()) hugeStatus_ += "\n" + hugeJob_->warning; }
            status_ = hugeStatus_;
            std::printf("%s\n", hugeStatus_.c_str());
            showHugeResult();
        }
    }

    // file dialogs
    static const char* kHugeOpen = "HugeOpenKey";
    static const char* kHugeSave = "HugeSaveKey";
    const ImVec2 vp = ImGui::GetMainViewport()->Size;
    if (hugeOpenRequested_) {
        hugeOpenRequested_ = false;
        IGFD::FileDialogConfig cfg;
        cfg.path = lastDir_;
        cfg.countSelectionMax = 1;
        cfg.flags = ImGuiFileDialogFlags_Modal;
        ImGuiFileDialog::Instance()->OpenDialog(kHugeOpen, "Open a huge mesh (read straight into flat arrays)", "Meshes{.ply,.obj}", cfg);
    }
    if (ImGuiFileDialog::Instance()->Display(kHugeOpen, ImGuiWindowFlags_NoCollapse, ImVec2(vp.x * 0.5f, vp.y * 0.5f))) {
        if (ImGuiFileDialog::Instance()->IsOk()) {
            lastDir_ = ImGuiFileDialog::Instance()->GetCurrentPath();
            startHuge(ImGuiFileDialog::Instance()->GetFilePathName());
        }
        ImGuiFileDialog::Instance()->Close();
    }
    if (hugeSaveRequested_) {
        hugeSaveRequested_ = false;
        IGFD::FileDialogConfig cfg;
        cfg.path = lastDir_;
        cfg.countSelectionMax = 1;
        cfg.flags = ImGuiFileDialogFlags_Modal | ImGuiFileDialogFlags_ConfirmOverwrite;
        ImGuiFileDialog::Instance()->OpenDialog(kHugeSave, "Save the huge mesh with atlas UV (PLY)", ".ply", cfg);
    }
    if (ImGuiFileDialog::Instance()->Display(kHugeSave, ImGuiWindowFlags_NoCollapse, ImVec2(vp.x * 0.5f, vp.y * 0.5f))) {
        if (ImGuiFileDialog::Instance()->IsOk() && hugeJob_ && hugeJob_->done) {
            lastDir_ = ImGuiFileDialog::Instance()->GetCurrentPath();
            std::string path = ImGuiFileDialog::Instance()->GetFilePathName();
            if (path.size() < 4 || path.substr(path.size() - 4) != ".ply") path += ".ply";
            saveHuge(path);
            status_ = hugeStatus_;
        }
        ImGuiFileDialog::Instance()->Close();
    }
}

// The checker follows the atlas: every square has the same size on the surface (the packer gives all charts the same texel
// density), about 1 / squares of the size of the model (the models are normalized to a unit box), and with the texel based
// packer a whole number of texels of the atlas.
static float atlasCheckerCells(const AtlasResult& a, int squares)
{
    squares = std::max(4, squares);
    if (a.resolution > 0 && a.texelsPerUnit > 0.0) {
        const double texelsPerSquare = std::max(2.0, std::round(a.texelsPerUnit / squares));
        return float(std::max(1.0, std::floor(a.resolution / texelsPerSquare)));
    }
    if (a.side > 0.0) return float(std::max(4.0, std::round(squares * a.side)));   // shelf packer: the uv unit is `side` long in 3D
    return 32.0f;
}

float App::hugeCheckerCells() const
{
    if (!hugeJob_ || !hugeJob_->done) return 32.0f;
    return atlasCheckerCells(hugeJob_->atlas, checkerSquares_);
}

void App::setHugeColors()
{
    if (!hugeJob_ || !hugeJob_->done) return;
    const FlatResult& r = hugeJob_->result;
    std::vector<unsigned char> rgba;
    if (hugeView_ == 0) {
        rgba.assign(r.patchOfFace.size() * 4, 255);
        // a color per patch from the golden ratio hue walk
        std::vector<unsigned char> palette(r.patches.size() * 3);
        for (size_t p = 0; p < r.patches.size(); ++p) {
            const float h = std::fmod(float(p) * 0.61803398875f, 1.0f), s = 0.55f, v = 0.9f;
            const float c = v * s, x = c * (1.0f - std::abs(std::fmod(h * 6.0f, 2.0f) - 1.0f)), m = v - c;
            float rgb[3] = {0, 0, 0};
            switch (int(h * 6.0f) % 6) {
                case 0: rgb[0] = c; rgb[1] = x; break;
                case 1: rgb[0] = x; rgb[1] = c; break;
                case 2: rgb[1] = c; rgb[2] = x; break;
                case 3: rgb[1] = x; rgb[2] = c; break;
                case 4: rgb[0] = x; rgb[2] = c; break;
                default: rgb[0] = c; rgb[2] = x; break;
            }
            for (int k = 0; k < 3; ++k) palette[3 * p + k] = (unsigned char)(255.0f * (rgb[k] + m));
        }
        for (size_t f = 0; f < r.patchOfFace.size(); ++f) {
            const int p = r.patchOfFace[f];
            if (p < 0) { rgba[4 * f] = 255; rgba[4 * f + 1] = 40; rgba[4 * f + 2] = 40; continue; }   // no patch: red
            for (int k = 0; k < 3; ++k) rgba[4 * f + k] = palette[3 * size_t(p) + k];
        }
    }
    meshRenderer_.setFaceColorsRgba(rgba);   // empty = gray
}

void App::showHugeResult()
{
    if (!hugeJob_ || !hugeJob_->done) return;
    meshRenderer_.uploadFlat(hugeJob_->mesh, hugeJob_->result.cornerUV);
    meshRenderer_.uploadLines({});
    if (!hugeJob_->baked.empty() && hugeJob_->bakedResolution > 0) {
        meshRenderer_.setTexture(hugeJob_->bakedResolution, hugeJob_->bakedResolution, hugeJob_->baked.data());
        hugeView_ = 3;   // the retextured model is what one wants to see
    } else if (hugeView_ == 3) {
        hugeView_ = 0;
    }
    hugeShown_ = true;
    cullBackFaces_ = true;
    wireframe_ = false;
    msaaSamples_ = 0;
    camera_.reset();
    setHugeColors();
}

void App::drawHugeButtons()
{
    const bool busy = hugeJob_ && !hugeJob_->done;
    ImGui::BeginDisabled(busy);
    ImGui::InputInt("Simplified mesh, faces", &hugeCoarseFaces_, 50000, 200000);
    hugeCoarseFaces_ = std::max(10000, hugeCoarseFaces_);
    ImGui::InputInt("Patches to start from", &hugeK_, 100, 500);
    hugeK_ = std::max(1, hugeK_);
    // a loaded mesh is used as it is (flat arrays and texture are filled from it); with nothing loaded the file dialog opens
    const bool haveLoaded = meshRenderer_.hasMesh() && mesh_.n_faces() > 0 && (!hugeShown_ || hugeFromLoaded_);
    if (ImGui::Button(haveLoaded ? "Make atlas (huge)" : "Make atlas (huge)...", ImVec2(haveLoaded ? -130.0f : -1.0f, 34))) {
        if (haveLoaded) startHugeFromLoaded();
        else hugeOpenRequested_ = true;
    }
    const bool tipItem = ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled);
    if (haveLoaded) {
        ImGui::SameLine();
        if (ImGui::Button("from a file...", ImVec2(-1, 34))) hugeOpenRequested_ = true;
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Process another mesh straight from a PLY or OBJ file (read into flat arrays)");
    }
    ImGui::EndDisabled();
    // the explanation appears once per run of the program: the first time the mouse rests on the button
    if (tipItem && !hugeTipShown_) {
        ImGui::SetTooltip("%s",
            "Huge mode for dense scans (tens of millions of faces).\n"
            "Works without the half-edge mesh, on compact arrays (about 100 bytes per face): with a mesh\n"
            "loaded this button takes it and its texture straight from memory, no file dialog (\"from a file\"\n"
            "reads a PLY / OBJ directly). The mesh is simplified, segmented and unwrapped on the simplified\n"
            "copy, the charts are lifted back onto the full mesh, the ones that do not lift are repaired,\n"
            "and the atlas is packed. If there is a texture it is moved to the new atlas. The result is shown\n"
            "in the 3D view and the UV view and saved as PLY + atlas PNG. It cannot be cancelled.");
        hugeTipHovered_ = true;
    } else if (hugeTipHovered_) {
        hugeTipShown_ = true;
    }
    if (hugeJob_ && hugeJob_->done && hugeJob_->error.empty()) {
        if (hugeShown_) {
            ImGui::TextUnformatted("3D view:");
            ImGui::SameLine();
            int v = hugeView_;
            bool changed = ImGui::RadioButton("patches", &v, 0);
            ImGui::SameLine();
            changed |= ImGui::RadioButton("atlas checker", &v, 1);
            ImGui::SameLine();
            changed |= ImGui::RadioButton("plain", &v, 2);
            if (!hugeJob_->baked.empty()) {
                ImGui::SameLine();
                changed |= ImGui::RadioButton("texture", &v, 3);
            }
            if (v == 1) {
                ImGui::SetNextItemWidth(-1.0f);
                ImGui::SliderInt("##squares", &checkerSquares_, 8, 200, "%d squares across the model");
            }
            if (changed) { hugeView_ = v; setHugeColors(); }
        } else if (ImGui::Button("Show the huge result in 3D")) {
            showHugeResult();
        }
        ImGui::BeginDisabled(busy);
        if (ImGui::Button("Save the huge result (PLY)...")) hugeSaveRequested_ = true;
        ImGui::EndDisabled();
    }
    if (!hugeStatus_.empty()) ImGui::TextWrapped("%s", hugeStatus_.c_str());
}

void App::drawMakeAtlasPanel()
{
    ImGui::SetNextWindowPos(ImVec2(12, 36), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(440, 520), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("Make atlas")) { ImGui::End(); return; }
    const bool hugeBusy = hugeJob_ && !hugeJob_->done;
    if (!meshRenderer_.hasMesh()) {
        ImGui::TextUnformatted("Open a mesh first (File > Open), or use the huge mode below");
        ImGui::Separator();
        drawHugeButtons();
        ImGui::End();
        return;
    }
    const bool running = atlasStage_ != AtlasStage::Idle;

    ImGui::BeginDisabled(running);
    ImGui::InputInt("Patches to start from (k)", &vsaK_);
    vsaK_ = std::max(1, std::min(vsaK_, int(mesh_.n_faces())));
    {
        HorizonOptions& ho = horizonOptions_;
        static const int kResolutions[] = {256, 512, 1024, 2048, 4096, 8192};
        char label[24];
        std::snprintf(label, sizeof(label), "%d x %d", ho.resolution, ho.resolution);
        if (ImGui::BeginCombo("Texture size", label)) {
            for (int r : kResolutions) {
                std::snprintf(label, sizeof(label), "%d x %d", r, r);
                if (ImGui::Selectable(label, r == ho.resolution)) ho.resolution = r;
            }
            ImGui::EndCombo();
        }
        int maxLevel = 0;
        while ((1 << (maxLevel + 1)) <= ho.resolution) ++maxLevel;
        maxLevel = std::max(0, maxLevel - 2);
        ho.maxMipLevel = std::min(ho.maxMipLevel, maxLevel);
        ImGui::SliderInt("Max mip level", &ho.maxMipLevel, 0, maxLevel);
        ImGui::TextDisabled("gap between charts: %d texels", chartGutter(ho.maxMipLevel, ho.extraTexels));
    }
    ImGui::Checkbox("Improve the partition with teleportation (slower)", &makeAtlasTeleport_);
    ImGui::EndDisabled();

    if (!running) {
        if (ImGui::Button("Make atlas", ImVec2(-1, 40))) startMakeAtlas();
    } else {
        if (ImGui::Button("Cancel", ImVec2(-1, 40))) cancelMakeAtlas("cancelled");
        const float stage = atlasStage_ == AtlasStage::Lloyd ? 0.15f : 0.55f;
        ImGui::ProgressBar(stage, ImVec2(-1, 0), "");
    }
    if (!makeAtlasStatus_.empty()) ImGui::TextWrapped("%s", makeAtlasStatus_.c_str());
    ImGui::Separator();
    drawHugeButtons();
    ImGui::Separator();

    if (atlasValid_ && !running) {
        ImGui::SeparatorText("Retexture and compare");
        if (ImGui::Button("Bake atlas texture")) bakeAtlasPreview();
        ImGui::SameLine();
        ImGui::BeginDisabled(!objExtras_.hasUV());
        if (ImGui::Button("Compare with the original layout")) measureLayouts();
        ImGui::EndDisabled();
        ImGui::TextDisabled("Save bakes the source texture into the new atlas by itself.");
        if (layoutMeasured_ && bakedVersion_ == atlasVersion_) {
            const UVLayoutStats& o = layoutOriginal_;
            const UVLayoutStats& n = layoutOurs_;
            if (ImGui::BeginTable("layouts", 3, ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingFixedFit)) {
                ImGui::TableSetupColumn("");
                ImGui::TableSetupColumn("original");
                ImGui::TableSetupColumn("new atlas");
                ImGui::TableHeadersRow();
                auto row = [&](const char* name, const char* a, const char* b) {
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0); ImGui::TextUnformatted(name);
                    ImGui::TableSetColumnIndex(1); ImGui::TextUnformatted(a);
                    ImGui::TableSetColumnIndex(2); ImGui::TextUnformatted(b);
                };
                char a[64], b[64];
                auto num = [&](const char* name, const char* fmt, double x, double y) {
                    std::snprintf(a, sizeof(a), fmt, x);
                    std::snprintf(b, sizeof(b), fmt, y);
                    row(name, a, b);
                };
                num("charts", "%.0f", o.charts, n.charts);
                num("seams, % of edge length", "%.2f", 100 * o.seamRatio(), 100 * n.seamRatio());
                num("flipped faces", "%.0f", o.flipped, n.flipped);
                num("faces without uv", "%.0f", double(o.facesWithoutUV), double(n.facesWithoutUV));
                num("L2 stretch, mean", "%.3f", o.l2Mean, n.l2Mean);
                num("L2 stretch, p95", "%.3f", o.l2P95, n.l2P95);
                num("L2 stretch, p99", "%.3f", o.l2P99, n.l2P99);
                num("conformal, p95", "%.3f", o.conformalP95, n.conformalP95);
                num("texel density, p05", "%.3f", o.densityP05, n.densityP05);
                num("texel density, p95", "%.3f", o.densityP95, n.densityP95);
                num("texture covered, %", "%.1f", 100 * o.covered, 100 * n.covered);
                num("texels used twice, %", "%.3f", 100 * o.overlap, 100 * n.overlap);
                ImGui::EndTable();
            }
            if (retexture_.samples > 0)
                ImGui::Text("Retexturing: PSNR %.1f dB (%.1f dB against the full resolution source), %zu samples", retexture_.psnr,
                            retexture_.psnrFull, retexture_.samples);
            ImGui::TextDisabled("Distortion is measured after scaling the uv to the 3D area: 1 is ideal. Lower stretch, a tighter density, fewer seams are better.");
        }
        if (bakedVersion_ == atlasVersion_ && ImGui::RadioButton("View the baked atlas texture", &colorMode_, 11)) uploadMesh();
    }

    ImGui::BeginDisabled(running);
    if (ImGui::Button("Save")) requestSave();
    ImGui::SameLine();
    if (ImGui::Button("Save As...")) saveAsRequested_ = true;
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::TextDisabled("%s", atlasValid_ ? "OBJ / PLY with atlas UV and texture" : "make an atlas to save UV and texture");
    ImGui::BeginDisabled(!atlasValid_ || running);
    if (ImGui::Button("Export OBJ...")) exportObjRequested_ = true;
    ImGui::SameLine();
    if (ImGui::Button("Export mask PNG...")) exportMaskRequested_ = true;
    ImGui::EndDisabled();
    if (!paramStatus_.empty()) ImGui::TextWrapped("%s", paramStatus_.c_str());
    ImGui::TextDisabled("Other settings come from the VSA, Patches, Parameterize and Refinement panels.");
    ImGui::End();
}

std::vector<BakeChart> App::bakeCharts() const
{
    std::vector<BakeChart> charts;
    for (size_t i = 0; i < params_.size(); ++i)
        if (paramDone_[i] && i < atlas_.uv.size() && !atlas_.uv[i].empty()) charts.push_back({&params_[i].patch, &atlas_.uv[i]});
    return charts;
}

// Retexturing: the source texture, looked up through the old uv, baked into the layout of the new atlas.
void App::bakeAtlasPreview()
{
    if (!atlasValid_) return;
    const auto t0 = std::chrono::steady_clock::now();
    const std::vector<BakeChart> charts = bakeCharts();
    BakeOptions bo;
    bo.resolution = atlas_.resolution > 0 ? atlas_.resolution : 2048;
    bo.margin = atlas_.resolution > 0 ? atlas_.margin : chartMargin(3, 0);
    BakeSource src{sourceW_, sourceH_, sourcePixels_.empty() ? nullptr : sourcePixels_.data()};
    const bool haveSource = hasSourceTexture_ && src.rgba;
    bakedRgba_ = bakeAtlasTexture(mesh_, charts, haveSource ? &src : nullptr, objExtras_.cornerUV, objExtras_.faceMaterial,
                                  textureMaterial_, bo);
    bakedResolution_ = bo.resolution;
    bakedCornerUV_ = cornerUVFromAtlas(mesh_, charts);
    bakedVersion_ = atlasVersion_;
    textureOwner_ = 0;   // the renderer still holds another texture: uploadMesh() puts the baked one there
    colorMode_ = 11;
    uploadMesh();
    char buf[200];
    std::snprintf(buf, sizeof(buf), "baked the atlas texture %d x %d%s in %.1f s", bo.resolution, bo.resolution,
                  haveSource ? " from the source texture" : " (no source texture: chart colors)",
                  std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count());
    status_ = buf;
}

void App::measureLayouts()
{
    if (!atlasValid_) return;
    if (bakedVersion_ != atlasVersion_) bakeAtlasPreview();
    const int R = std::max(256, bakedResolution_);
    // compare the faces that carry the source texture (other materials have no layout to compare)
    std::vector<float> original = objExtras_.cornerUV, ours = bakedCornerUV_;
    if (original.size() == mesh_.n_faces() * 6 && objExtras_.faceMaterial.size() == mesh_.n_faces())
        for (size_t f = 0; f < mesh_.n_faces(); ++f)
            if (objExtras_.faceMaterial[f] != textureMaterial_)
                for (int k = 0; k < 6; ++k) { original[f * 6 + k] = std::nanf(""); ours[f * 6 + k] = std::nanf(""); }
    layoutOriginal_ = objExtras_.hasUV() ? analyzeUVLayout(mesh_, original, R) : UVLayoutStats();
    layoutOurs_ = analyzeUVLayout(mesh_, ours, R);
    retexture_ = RetextureReport();
    if (hasSourceTexture_ && !sourcePixels_.empty()) {
        BakeSource src{sourceW_, sourceH_, sourcePixels_.data()};
        retexture_ = measureRetexture(mesh_, objExtras_.cornerUV, src, objExtras_.faceMaterial, textureMaterial_, bakedCornerUV_,
                                      bakedRgba_, bakedResolution_, std::max<size_t>(1, mesh_.n_faces() / 300000));
    }
    layoutMeasured_ = true;
}

void App::requestSave()
{
    if (savePath_.empty()) saveAsRequested_ = true;
    else performSave(savePath_);
}

// Writes the mesh as OBJ or PLY (by the extension). With an atlas: the atlas UV in the mesh and the atlas texture
// `<name>_atlas.png` next to it, baked from the source texture (or a per-chart colour mask if the mesh has none).
bool App::performSave(const std::string& requested)
{
    std::string path = requested;
    std::string lower = path;
    for (char& c : lower) c = char(std::tolower(static_cast<unsigned char>(c)));
    const bool endsObj = lower.size() > 4 && lower.compare(lower.size() - 4, 4, ".obj") == 0;
    const bool endsPly = lower.size() > 4 && lower.compare(lower.size() - 4, 4, ".ply") == 0;
    if (!endsObj && !endsPly) path += ".obj";
    const bool ply = endsPly;
    const auto t0 = std::chrono::steady_clock::now();
    char buf[400];

    if (!atlasValid_ || params_.empty()) {   // no atlas: the geometry alone
        const bool ok = ply ? writePlyMesh(path, mesh_, binaryPly_) : writeObjMesh(path, mesh_);
        std::snprintf(buf, sizeof(buf), ok ? "saved the mesh without UV (make an atlas first to save UV and texture): %s"
                                           : "save failed: %s", path.c_str());
        status_ = buf;
        if (ok) savePath_ = path;
        return ok;
    }

    const size_t slash = path.find_last_of('/');
    const std::string dir = slash == std::string::npos ? "." : path.substr(0, slash);
    std::string stem = path.substr(slash == std::string::npos ? 0 : slash + 1);
    stem.resize(stem.size() - 4);
    const std::string textureName = stem + "_atlas.png";

    std::vector<BakeChart> charts;
    std::vector<const Patch*> pv;
    std::vector<const std::vector<Eigen::Vector2d>*> uvv;
    size_t withUV = 0;
    for (size_t i = 0; i < params_.size(); ++i) {
        const bool has = paramDone_[i] && i < atlas_.uv.size() && !atlas_.uv[i].empty();
        pv.push_back(&params_[i].patch);
        uvv.push_back(has ? &atlas_.uv[i] : nullptr);
        if (has) {
            charts.push_back({&params_[i].patch, &atlas_.uv[i]});
            withUV += params_[i].patch.faceToGlobal.size();
        }
    }

    BakeOptions bo;
    bo.resolution = atlas_.resolution > 0 ? atlas_.resolution : 2048;   // the shelf packer has no texel grid
    bo.margin = atlas_.resolution > 0 ? atlas_.margin : chartMargin(3, 0);
    const bool haveSource = hasSourceTexture_ && !sourcePixels_.empty();
    if (bakedVersion_ != atlasVersion_) {   // the texture of the current atlas is not baked yet
        BakeSource src{sourceW_, sourceH_, sourcePixels_.empty() ? nullptr : sourcePixels_.data()};
        bakedRgba_ = bakeAtlasTexture(mesh_, charts, haveSource ? &src : nullptr, objExtras_.cornerUV, objExtras_.faceMaterial,
                                      textureMaterial_, bo);
        bakedResolution_ = bo.resolution;
        bakedCornerUV_ = cornerUVFromAtlas(mesh_, charts);
        bakedVersion_ = atlasVersion_;
        textureOwner_ = textureOwner_ == 2 ? 0 : textureOwner_;
    }
    const std::vector<unsigned char>& rgba = bakedRgba_;
    std::vector<unsigned char> flipped(rgba.size());   // image row 0 is v = 1
    const size_t row = size_t(bo.resolution) * 4;
    for (int y = 0; y < bo.resolution; ++y)
        std::copy(rgba.begin() + size_t(bo.resolution - 1 - y) * row, rgba.begin() + size_t(bo.resolution - y) * row,
                  flipped.begin() + size_t(y) * row);
    std::string err;
    if (!writePNG(dir + "/" + textureName, bo.resolution, bo.resolution, flipped, &err)) {
        status_ = "texture save failed: " + err;
        return false;
    }

    char meta[200];
    std::snprintf(meta, sizeof(meta), "atlas resolution=%d maxMipLevel=%d gutter=%d margin=%d texelsPerUnit=%.6g",
                  bo.resolution, atlas_.maxMipLevel, atlas_.gutter, bo.margin, atlas_.texelsPerUnit);
    const int faces = ply ? writePlyAtlas(path, pv, uvv, textureName, binaryPly_, meta)
                          : exportAtlasOBJ(path, mesh_, pv, uvv, meta, textureName);
    if (faces < 0) {
        status_ = "save failed: " + path;
        return false;
    }
    savePath_ = path;
    const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    std::snprintf(buf, sizeof(buf), "saved %s with %d of %zu faces and the %s atlas texture %s (%d x %d) in %.1f s",
                  path.c_str(), faces, mesh_.n_faces(), haveSource ? "baked" : "mask", textureName.c_str(), bo.resolution,
                  bo.resolution, seconds);
    status_ = buf;
    std::printf("%s\n", buf);
    (void)withUV;
    return true;
}

void App::drawExportDialogs()
{
    constexpr const char* kSaveKey = "SaveAsDlg";
    if (saveAsRequested_) {
        saveAsRequested_ = false;
        std::string name = meshPath_;
        const size_t slash = name.find_last_of('/');
        if (slash != std::string::npos) name = name.substr(slash + 1);
        const size_t dot = name.find_last_of('.');
        if (dot != std::string::npos) name.resize(dot);
        IGFD::FileDialogConfig cfg;
        cfg.path = lastDir_;
        cfg.fileName = (name.empty() ? std::string("mesh") : name) + "_atlas.obj";
        cfg.countSelectionMax = 1;
        cfg.flags = ImGuiFileDialogFlags_Modal | ImGuiFileDialogFlags_ConfirmOverwrite;
        ImGuiFileDialog::Instance()->OpenDialog(kSaveKey, "Save mesh (OBJ or PLY)", "Meshes{.obj,.ply}", cfg);
    }
    {
        const ImVec2 vpSize = ImGui::GetMainViewport()->Size;
        if (ImGuiFileDialog::Instance()->Display(kSaveKey, ImGuiWindowFlags_NoCollapse, ImVec2(vpSize.x * 0.5f, vpSize.y * 0.5f))) {
            if (ImGuiFileDialog::Instance()->IsOk()) {
                lastDir_ = ImGuiFileDialog::Instance()->GetCurrentPath();
                performSave(ImGuiFileDialog::Instance()->GetFilePathName());
            }
            ImGuiFileDialog::Instance()->Close();
        }
    }

    constexpr const char* kMaskKey = "ExportMaskDlg";
    if (exportMaskRequested_) {
        exportMaskRequested_ = false;
        IGFD::FileDialogConfig cfg;
        cfg.path = lastDir_;
        cfg.fileName = "atlas_mask.png";
        cfg.countSelectionMax = 1;
        cfg.flags = ImGuiFileDialogFlags_Modal | ImGuiFileDialogFlags_ConfirmOverwrite;
        ImGuiFileDialog::Instance()->OpenDialog(kMaskKey, "Export occupancy mask (alpha channel)", ".png", cfg);
    }
    {
        const ImVec2 vpSize = ImGui::GetMainViewport()->Size;
        if (ImGuiFileDialog::Instance()->Display(kMaskKey, ImGuiWindowFlags_NoCollapse, ImVec2(vpSize.x * 0.5f, vpSize.y * 0.5f))) {
            if (ImGuiFileDialog::Instance()->IsOk()) {
                lastDir_ = ImGuiFileDialog::Instance()->GetCurrentPath();
                std::vector<MaskChart> mc;
                for (size_t i = 0; i < params_.size(); ++i)
                    if (paramDone_[i] && i < atlas_.uv.size() && !atlas_.uv[i].empty())
                        mc.push_back({&params_[i].patch.mesh, &atlas_.uv[i]});
                const int res = atlas_.resolution > 0 ? atlas_.resolution : 2048;   // the shelf packer has no texel grid: 2048
                std::string err;
                const std::string path = ImGuiFileDialog::Instance()->GetFilePathName();
                char buf[256];
                if (writeAtlasPNG(path, mc, res, true, &err))
                    std::snprintf(buf, sizeof(buf), "mask written: %s (%d x %d, alpha = occupancy)", path.c_str(), res, res);
                else
                    std::snprintf(buf, sizeof(buf), "mask export failed: %s", err.c_str());
                paramStatus_ = buf;
            }
            ImGuiFileDialog::Instance()->Close();
        }
    }

    constexpr const char* kExportKey = "ExportObjDlg";
    if (exportObjRequested_) {
        exportObjRequested_ = false;
        IGFD::FileDialogConfig cfg;
        cfg.path = lastDir_;
        cfg.fileName = "atlas.obj";
        cfg.countSelectionMax = 1;
        cfg.flags = ImGuiFileDialogFlags_Modal | ImGuiFileDialogFlags_ConfirmOverwrite;
        ImGuiFileDialog::Instance()->OpenDialog(kExportKey, "Export OBJ with UV", ".obj", cfg);
    }
    const ImVec2 vp = ImGui::GetMainViewport()->Size;
    if (ImGuiFileDialog::Instance()->Display(kExportKey, ImGuiWindowFlags_NoCollapse, ImVec2(vp.x * 0.5f, vp.y * 0.5f))) {
        if (ImGuiFileDialog::Instance()->IsOk()) {
            lastDir_ = ImGuiFileDialog::Instance()->GetCurrentPath();
            std::vector<const Patch*> pv;
            std::vector<const std::vector<Eigen::Vector2d>*> uvv;
            for (size_t i = 0; i < params_.size(); ++i) {
                pv.push_back(&params_[i].patch);
                uvv.push_back(paramDone_[i] && i < atlas_.uv.size() && !atlas_.uv[i].empty() ? &atlas_.uv[i] : nullptr);
            }
            const std::string path = ImGuiFileDialog::Instance()->GetFilePathName();
            char meta[200] = "";
            if (atlas_.resolution > 0)
                std::snprintf(meta, sizeof(meta), "atlas resolution=%d maxMipLevel=%d gutter=%d margin=%d texelsPerUnit=%.6g",
                              atlas_.resolution, atlas_.maxMipLevel, atlas_.gutter, atlas_.margin, atlas_.texelsPerUnit);
            const int faces = exportAtlasOBJ(path, mesh_, pv, uvv, meta);
            char buf[256];
            if (faces < 0) std::snprintf(buf, sizeof(buf), "export failed: %s", path.c_str());
            else std::snprintf(buf, sizeof(buf), "exported %d of %d faces to %s", faces, int(mesh_.n_faces()), path.c_str());
            paramStatus_ = buf;
        }
        ImGuiFileDialog::Instance()->Close();
    }
}

void App::drawLinabfPanel()
{
    if (!ImGui::Begin("LinABF")) { ImGui::End(); return; }
    if (!patchesValid_ || linabf_.size() != patches_.patches.size()) {
        ImGui::TextUnformatted("Extract patches first (Patches panel)");
        ImGui::End();
        return;
    }
    ImGui::InputInt("Max linearization steps", &linabfParams_.maxIterations);
    linabfParams_.maxIterations = std::max(1, linabfParams_.maxIterations);
    {
        float tol = float(linabfParams_.wheelTolerance), th = float(linabfParams_.deficitThreshold);
        if (ImGui::InputFloat("Wheel tolerance", &tol, 0.0f, 0.0f, "%.1e")) linabfParams_.wheelTolerance = tol;
        if (ImGui::SliderFloat("Fan deficit threshold", &th, 0.1f, 6.0f)) linabfParams_.deficitThreshold = th;
    }
    ImGui::BeginDisabled(selectedPatch_ < 0);
    if (ImGui::Button("Solve selected")) solvePatches(true);
    ImGui::EndDisabled();
    ImGui::SameLine();
    if (ImGui::Button("Solve all disks")) solvePatches(false);
    ImGui::SameLine();
    if (ImGui::RadioButton("Angle error", &colorMode_, 6)) uploadMesh();

    ImGui::Checkbox("Greedy (debug) instead of least squares", &uvGreedy_);
    ImGui::BeginDisabled(selectedPatch_ < 0);
    if (ImGui::Button("UV selected")) computeUV(true);
    ImGui::EndDisabled();
    ImGui::SameLine();
    if (ImGui::Button("UV all solved")) computeUV(false);
    {
        int done = 0, withFlips = 0, flipped = 0;
        for (size_t i = 0; i < uvDone_.size(); ++i)
            if (uvDone_[i] && uv_[i].ok) { ++done; flipped += uvStats_[i].flipped; withFlips += uvStats_[i].flipped > 0; }
        if (done > 0)
            ImGui::Text("UV for %d patches: %d have flipped triangles (%d in total)", done, withFlips, flipped);
    }

    int solved = 0, valid = 0, failed = 0, rescaled = 0;
    double tri = 0, vert = 0, wheel = 0, err = 0;
    for (size_t i = 0; i < linabf_.size(); ++i) {
        if (!linabfSolved_[i]) continue;
        const LinABFResult& r = linabf_[i];
        ++solved;
        if (!r.ok) { ++failed; continue; }
        valid += r.anglesValid;
        rescaled += r.fansRescaled;
        tri = std::max(tri, r.triangleResidual);
        vert = std::max(vert, r.vertexResidual);
        if (r.anglesValid) wheel = std::max(wheel, r.wheelResidual);
        err = std::max(err, r.angularError);
    }
    ImGui::Separator();
    ImGui::Text("Solved %d patches in %.3f s: %d valid, %d with invalid angles, %d failed", solved, linabfSeconds_,
                valid, solved - valid - failed, failed);
    if (solved > 0) {
        ImGui::Text("Max residuals: triangle %.1e, vertex %.1e, wheel %.1e", tri, vert, wheel);
        ImGui::Text("Max angular error ||e||/3nt: %.2e   fans re-estimated: %d", err, rescaled);
    }
    if (selectedPatch_ >= 0 && selectedPatch_ < int(linabf_.size()) && linabfSolved_[selectedPatch_]) {
        const LinABFResult& r = linabf_[selectedPatch_];
        ImGui::SeparatorText("Selected patch");
        if (!r.ok) {
            ImGui::TextWrapped("failed: %s", r.message.c_str());
        } else {
            ImGui::Text("%d triangles, %d interior vertices, %d constraints, %d steps", r.triangles,
                        r.interiorVertices, r.rows, r.iterations);
            ImGui::Text("angles in [%.4f, %.4f]  %s", r.minAngle, r.maxAngle, r.anglesValid ? "valid" : "INVALID");
            ImGui::Text("residuals: tri %.1e vert %.1e wheel %.1e", r.triangleResidual, r.vertexResidual,
                        r.wheelResidual);
            if (!r.message.empty()) ImGui::TextWrapped("%s", r.message.c_str());
        }
    }
    ImGui::End();
}

const Patch& App::patchAt(size_t i) const
{
    if (i < params_.size() && i < paramDone_.size() && paramDone_[i] && params_[i].patch.mesh.n_faces() > 0) return params_[i].patch;
    return patches_.patches[i];
}

void App::rebuildMoments()
{
    faceMoments_ = computeFaceMoments(mesh_, faceData_);   // the unweighted moments are cheap to recompute: no second copy is kept
    wholeMesh_ = Moments();
    for (size_t i = 0; i < faceMoments_.size(); ++i) {
        if (faceWeights_[i] != 1.0f) faceMoments_[i] *= double(faceWeights_[i]);
        wholeMesh_ += faceMoments_[i];
    }
    if (vsa_.label.size() == mesh_.n_faces())
        fitProxies(vsaInput(), vsa_);   // also recomputes the region moments
}

void App::startJob(RefineJob::Kind kind)
{
    job_ = std::make_unique<RefineJob>(kind, lloydParams_, teleportParams_, vsaK_);
    lloydStatus_.clear();
    colorMode_ = 2;
}

void App::jobTick()
{
    if (!job_) return;
    if (vsa_.k() == 0) { job_.reset(); return; }
    const bool more = job_->step(vsaInput(), vsa_);
    ++vsaSteps_;
    uploadMesh();
    if (more) return;

    char buf[160];
    std::snprintf(buf, sizeof(buf), "finished: %d iterations, %d teleports, k = %d, %s",
                  job_->iterations(), job_->teleports(), vsa_.k(),
                  job_->lastRunConverged() ? "converged" : "stopped at max iterations");
    lloydStatus_ = buf;
    job_.reset();
}

void App::drawVsaPanel()
{
    jobTick();
    if (!ImGui::Begin("VSA")) { ImGui::End(); return; }
    if (!meshRenderer_.hasMesh()) {
        ImGui::TextUnformatted("No mesh loaded");
        ImGui::End();
        return;
    }

    auto resetState = [&] {
        job_.reset();
        lloydStatus_.clear();
        vsaAdded_ = vsaSteps_ = 0;
    };
    auto refresh = [&] { if (colorMode_ == 2) uploadMesh(); };
    const bool running = job_ != nullptr;
    const bool partitioned = vsa_.label.size() == mesh_.n_faces();

    const char* metrics[] = {"L2,1", "L2"};
    if (ImGui::Combo("Metric", &vsaMetric_, metrics, 2)) {
        vsa_ = VSAState();
        vsa_.metric = vsaMetric_ == 0 ? Metric::L21 : Metric::L2;
        resetState();
        refresh();
    }
    ImGui::InputInt("Proxies (k)", &vsaK_);
    vsaK_ = std::max(1, std::min(vsaK_, int(mesh_.n_faces())));

    const char* methods[] = {"Random", "Farthest point"};
    ImGui::Combo("Seeding", &seedMethod_, methods, 2);
    ImGui::InputInt("Random seed", &vsaRngSeed_);
    if (ImGui::Button("Seed")) {
        vsa_.metric = vsaMetric_ == 0 ? Metric::L21 : Metric::L2;
        if (seedMethod_ == 0) initRandomSeeds(vsaInput(), vsa_, vsaK_, uint64_t(vsaRngSeed_));
        else                  initFarthestSeeds(vsaInput(), vsa_, vsaK_, uint64_t(vsaRngSeed_));
        resetState();
        refresh();
    }

    ImGui::SeparatorText("Lloyd");
    ImGui::BeginDisabled(vsa_.k() == 0 || running);
    if (ImGui::Button("Partition")) {
        vsaAdded_ = partition(vsaInput(), vsa_);
        ++vsaSteps_;
        colorMode_ = 2;
        uploadMesh();
    }
    ImGui::SameLine();
    if (ImGui::Button("Fit")) {
        fitProxies(vsaInput(), vsa_);
        refresh();
    }
    ImGui::SameLine();
    if (ImGui::Button("Step")) {   // one Lloyd iteration: partition + fit
        vsaAdded_ = lloydStep(vsaInput(), vsa_).added;
        ++vsaSteps_;
        colorMode_ = 2;
        uploadMesh();
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    if (!running) {
        ImGui::BeginDisabled(vsa_.k() == 0);
        if (ImGui::Button("Run")) startJob(RefineJob::Kind::Lloyd);
        ImGui::EndDisabled();
    } else if (ImGui::Button("Stop")) {
        job_.reset();
        lloydStatus_ = "stopped";
    }

    ImGui::InputInt("Max iterations", &lloydParams_.maxIterations);
    lloydParams_.maxIterations = std::max(1, lloydParams_.maxIterations);
    ImGui::InputInt("Patience", &lloydParams_.patience);
    lloydParams_.patience = std::max(1, lloydParams_.patience);
    {
        float tol = float(lloydParams_.relTolerance);
        if (ImGui::InputFloat("Rel. tolerance", &tol, 0.0f, 0.0f, "%.1e"))
            lloydParams_.relTolerance = std::max(0.0f, tol);
    }

    ImGui::SeparatorText("Border smoothing");
    {
        float lam = float(smoothParams_.lambda), kap = float(smoothParams_.kappa);
        if (ImGui::SliderFloat("Lambda (perimeter)", &lam, 0.0f, 1.0f, "%.3f", ImGuiSliderFlags_Logarithmic))
            smoothParams_.lambda = lam;
        if (ImGui::SliderFloat("Kappa (crease)", &kap, 0.0f, 4.0f, "%.2f"))
            smoothParams_.kappa = kap;
        ImGui::SliderInt("Max sweeps", &smoothParams_.maxSweeps, 1, 30);
        ImGui::TextDisabled("lambda = 0: off. Applied after every partition in Run / Step.");
    }
    vsa_.smooth = smoothParams_;
    ImGui::BeginDisabled(!partitioned || running);
    if (ImGui::Button("Smooth now")) {
        const SmoothStats st = smoothLabels(vsaInput(), vsa_);
        char buf[96];
        std::snprintf(buf, sizeof(buf), "smoothing: %d flips in %d sweeps, perimeter %.3f -> %.3f", st.flips,
                      st.sweeps, st.perimeterBefore, st.perimeterAfter);
        lloydStatus_ = buf;
        colorMode_ = 2;
        uploadMesh();
    }
    ImGui::EndDisabled();

    ImGui::SeparatorText("Proxy operations");
    ImGui::BeginDisabled(!partitioned || running);
    if (ImGui::Button("Insert")) {
        if (insertProxy(vsaInput(), vsa_)) { colorMode_ = 2; uploadMesh(); }
    }
    ImGui::SameLine();
    if (ImGui::Button("Delete cheapest")) {
        RegionPair m;
        if (findBestMerge(vsaInput(), vsa_, m) && deleteProxy(vsaInput(), vsa_, m.a, m.b)) {
            colorMode_ = 2;
            uploadMesh();
        }
    }
    ImGui::SameLine();
    if (ImGui::Button("Teleport")) {
        if (teleportProxy(vsaInput(), vsa_, teleportParams_.ratio)) { colorMode_ = 2; uploadMesh(); }
        else lloydStatus_ = "teleport: no admissible move";
    }
    ImGui::EndDisabled();
    ImGui::BeginDisabled(running);
    if (ImGui::Button("Grow to k")) {
        if (vsa_.k() == 0) {
            vsa_.metric = vsaMetric_ == 0 ? Metric::L21 : Metric::L2;
            initFarthestSeeds(vsaInput(), vsa_, 1, uint64_t(vsaRngSeed_));
        }
        startJob(RefineJob::Kind::Grow);
    }
    ImGui::SameLine();
    ImGui::BeginDisabled(vsa_.k() < 3);
    if (ImGui::Button("Run + teleport")) startJob(RefineJob::Kind::Teleport);
    ImGui::EndDisabled();
    ImGui::EndDisabled();
    {
        float ratio = float(teleportParams_.ratio);
        if (ImGui::SliderFloat("Teleport ratio", &ratio, 0.05f, 1.0f))
            teleportParams_.ratio = ratio;
        ImGui::InputInt("Max teleports", &teleportParams_.maxTeleports);
        teleportParams_.maxTeleports = std::max(1, teleportParams_.maxTeleports);
    }

    ImGui::SeparatorText("Weights");
    ImGui::Checkbox("Paint (Alt+drag rotates)", &paintMode_);
    ImGui::SliderInt("Brush radius (px)", &brushRadius_, 2, 100);
    ImGui::SliderFloat("Brush weight", &brushWeight_, 0.1f, 20.0f, "%.2f", ImGuiSliderFlags_Logarithmic);
    ImGui::Checkbox("Erase (set to 1)", &eraseMode_);
    if (ImGui::Button("Reset weights")) {
        std::fill(faceWeights_.begin(), faceWeights_.end(), 1.0f);
        rebuildMoments();
        if (colorMode_ == 2 || colorMode_ == 3) uploadMesh();
    }

    ImGui::Separator();
    if (!lloydStatus_.empty())
        ImGui::TextUnformatted(lloydStatus_.c_str());
    if (running) ImGui::TextUnformatted("running...");
    ImGui::Text("Proxies : %d", vsa_.k());
    ImGui::Text("Partitions run : %d", vsaSteps_);
    if (vsaAdded_ > 0)
        ImGui::Text("Added for unseeded components: %d", vsaAdded_);
    if (partitioned)
        ImGui::Text("Energy : %.6f", totalEnergy(vsa_));
    if (boundaryStatsValid_ && partitioned)
        ImGui::Text("Borders: length %.3f, teeth %d, spikes %d, reflex %d", boundaryStats_.perimeter,
                    boundaryStats_.teeth, boundaryStats_.spikeVertices, boundaryStats_.reflexVertices);
    if (!vsa_.energyHistory.empty()) {
        const std::vector<float> h(vsa_.energyHistory.begin(), vsa_.energyHistory.end());
        char overlay[48];
        std::snprintf(overlay, sizeof(overlay), "E = %.5g", h.back());
        ImGui::PlotLines("##energy", h.data(), int(h.size()), 0, overlay, FLT_MAX, FLT_MAX, ImVec2(-1, 80));
    }
    ImGui::End();
}

void App::paintWeights(int fbW, int fbH)
{
    ImGuiIO& io = ImGui::GetIO();
    int winW = 0, winH = 0;
    glfwGetWindowSize(window_, &winW, &winH);
    if (winW <= 0 || winH <= 0 || fbH <= 0) return;
    const float scale = float(fbW) / float(winW);

    // brush outline
    ImGui::GetForegroundDrawList()->AddCircle(io.MousePos, float(brushRadius_) / scale,
                                              IM_COL32(255, 255, 255, 200), 32, 1.5f);

    if (!ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
        if (painted_) {   // end of a stroke: apply the weights to the moments and refit once
            painted_ = false;
            rebuildMoments();
            if (colorMode_ == 2) uploadMesh();
        }
        return;
    }
    float vp[16];
    camera_.viewProj(float(fbW) / float(fbH), vp);
    std::vector<int> ids;
    meshRenderer_.pickFaces(vp, fbW, fbH, int(io.MousePos.x * scale), fbH - 1 - int(io.MousePos.y * scale),
                            brushRadius_, ids);
    if (ids.empty()) return;
    const float w = eraseMode_ ? 1.0f : brushWeight_;
    for (int f : ids) faceWeights_[f] = w;
    painted_ = true;
    if (colorMode_ == 3) uploadMesh();
}

void App::handleSceneInput()
{
    ImGuiIO& io = ImGui::GetIO();
    if (io.WantCaptureMouse) {
        dragging_ = panning_ = false;
        return;
    }
    int w, h;
    glfwGetWindowSize(window_, &w, &h);   // ImGui coordinates are in window points
    const double x = io.MousePos.x, y = io.MousePos.y;

    // In paint mode the left button paints; Alt+left still rotates
    const bool painting = paintMode_ && !io.KeyAlt && meshRenderer_.hasMesh();
    if (painting) {
        int fbW, fbH;
        glfwGetFramebufferSize(window_, &fbW, &fbH);
        paintWeights(fbW, fbH);
    }

    if (ImGui::IsMouseClicked(ImGuiMouseButton_Left) && !painting) {
        dragging_ = true;
        camera_.beginRotate(x, y, w, h);
    }
    if (ImGui::IsMouseClicked(ImGuiMouseButton_Right) || ImGui::IsMouseClicked(ImGuiMouseButton_Middle))
        panning_ = true;
    if (!ImGui::IsMouseDown(ImGuiMouseButton_Left)) dragging_ = false;
    if (!ImGui::IsMouseDown(ImGuiMouseButton_Right) && !ImGui::IsMouseDown(ImGuiMouseButton_Middle)) panning_ = false;

    if (dragging_) camera_.rotateTo(x, y, w, h);
    if (panning_) camera_.pan(io.MouseDelta.x, io.MouseDelta.y, h);
    if (io.MouseWheel != 0.0f) camera_.zoom(io.MouseWheel);
}

void App::drawMainMenu()
{
    constexpr const char* kDialogKey = "OpenMeshDlg";
    bool openRequested = false;

    if (ImGui::BeginMainMenuBar()) {
        if (ImGui::BeginMenu("File")) {
            if (ImGui::MenuItem("Open...", "Cmd+O"))
                openRequested = true;
            ImGui::Separator();
            if (ImGui::MenuItem("Save", "Cmd+S", false, meshRenderer_.hasMesh())) requestSave();
            if (ImGui::MenuItem("Save As...", "Cmd+Shift+S", false, meshRenderer_.hasMesh())) saveAsRequested_ = true;
            ImGui::MenuItem("Binary PLY", nullptr, &binaryPly_);
            ImGui::Separator();
            if (ImGui::MenuItem("Quit", "Cmd+Q"))
                glfwSetWindowShouldClose(window_, GLFW_TRUE);
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("View")) {
            ImGui::MenuItem("Wireframe", nullptr, &wireframe_);
            if (ImGui::MenuItem("Reset camera"))
                camera_.reset();
            ImGui::Separator();
            ImGui::MenuItem("ImGui demo", nullptr, &showDemo_);
            ImGui::EndMenu();
        }
        ImGui::EndMainMenuBar();
    }

    // Cmd+O on macOS (ImGui maps Super to Cmd)
    if (ImGui::IsKeyChordPressed(ImGuiMod_Ctrl | ImGuiKey_O) || ImGui::IsKeyChordPressed(ImGuiMod_Super | ImGuiKey_O))
        openRequested = true;

    if (ImGui::IsKeyChordPressed(ImGuiMod_Super | ImGuiKey_S) && meshRenderer_.hasMesh()) requestSave();
    if (ImGui::IsKeyChordPressed(ImGuiMod_Super | ImGuiMod_Shift | ImGuiKey_S) && meshRenderer_.hasMesh()) saveAsRequested_ = true;

    if (openRequested) {
        IGFD::FileDialogConfig cfg;
        cfg.path = lastDir_;
        cfg.countSelectionMax = 1;
        cfg.flags = ImGuiFileDialogFlags_Modal;
        ImGuiFileDialog::Instance()->OpenDialog(kDialogKey, "Open mesh",
                                                "Meshes{.obj,.off,.ply,.stl},.*", cfg);
    }

    const ImVec2 vp = ImGui::GetMainViewport()->Size;
    if (ImGuiFileDialog::Instance()->Display(kDialogKey, ImGuiWindowFlags_NoCollapse,
                                             ImVec2(vp.x * 0.5f, vp.y * 0.5f))) {
        if (ImGuiFileDialog::Instance()->IsOk()) {
            lastDir_ = ImGuiFileDialog::Instance()->GetCurrentPath();
            openMesh(ImGuiFileDialog::Instance()->GetFilePathName());
        }
        ImGuiFileDialog::Instance()->Close();
    }
}

void App::drawUI()
{
    ImGuiIO& io = ImGui::GetIO();

#ifdef IMGUI_HAS_DOCK
    // The central area is transparent: the 3D scene shows through it
  #if IMGUI_VERSION_NUM >= 19100
    ImGui::DockSpaceOverViewport(0, ImGui::GetMainViewport(), ImGuiDockNodeFlags_PassthruCentralNode);
  #else
    ImGui::DockSpaceOverViewport(ImGui::GetMainViewport(), ImGuiDockNodeFlags_PassthruCentralNode);
  #endif
#endif

    ImGui::SetNextWindowPos(ImVec2(12, 36), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(360, 0), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Info")) {
        ImGui::Text("OpenGL : %s", glVersion_.c_str());
        ImGui::Text("GPU    : %s", glRenderer_.c_str());
        ImGui::Text("GLSL   : %s", glslVersion_.c_str());
        ImGui::Separator();

        int winW, winH, fbW, fbH;
        glfwGetWindowSize(window_, &winW, &winH);
        glfwGetFramebufferSize(window_, &fbW, &fbH);
        ImGui::Text("Window      : %d x %d pt", winW, winH);
        ImGui::Text("Framebuffer : %d x %d px (scale %.1f)", fbW, fbH,
                    winW > 0 ? float(fbW) / float(winW) : 0.0f);
        ImGui::Text("Frame       : %.2f ms (%.0f FPS)", 1000.0f / io.Framerate, io.Framerate);
        ImGui::Text("Mesh draw   : %.2f ms on the GPU (%zu faces, %zu vertices)", gpuDrawMsAvg_, meshRenderer_.faceCount(),
                    meshRenderer_.vertexCount());
        ImGui::Text("ImGui wants mouse: %s", io.WantCaptureMouse ? "yes" : "no");
        ImGui::Separator();

        ImGui::ColorEdit3("Background", clearColor_);
        ImGui::Checkbox("Debug triangle", &showTriangle_);
        ImGui::SameLine();
        ImGui::Checkbox("Rotate", &rotate_);
        ImGui::Checkbox("ImGui demo", &showDemo_);
    }
    ImGui::End();

    drawMainMenu();

    if (ImGui::Begin("Mesh")) {
        ImGui::TextWrapped("%s", status_.c_str());
        ImGui::Separator();
        ImGui::Checkbox("Wireframe", &wireframe_);
        ImGui::Checkbox("Cull back faces (faster on scans)", &cullBackFaces_);
        {
            static const char* kAA[] = {"Off", "2x", "4x", "8x"};
            int idx = msaaSamples_ >= 8 ? 3 : msaaSamples_ >= 4 ? 2 : msaaSamples_ >= 2 ? 1 : 0;
            if (ImGui::Combo("Anti-aliasing (MSAA)", &idx, kAA, 4)) msaaSamples_ = idx == 0 ? 0 : (1 << idx);
        }
        if (!textureNote_.empty()) ImGui::TextDisabled("%s", textureNote_.c_str());
        if (ImGui::RadioButton("Gray", &colorMode_, 0)) uploadMesh();
        ImGui::SameLine();
        if (ImGui::RadioButton("Face normals", &colorMode_, 1)) uploadMesh();
        ImGui::SameLine();
        ImGui::BeginDisabled(!hasSourceTexture_);
        if (ImGui::RadioButton("Texture", &colorMode_, 10)) uploadMesh();
        ImGui::EndDisabled();
        ImGui::SameLine();
        if (ImGui::RadioButton("Regions", &colorMode_, 2)) uploadMesh();
        ImGui::SameLine();
        if (ImGui::RadioButton("Weights", &colorMode_, 3)) uploadMesh();
        if (ImGui::Button("Reset camera")) camera_.reset();
        ImGui::Separator();
        if (meshRenderer_.hasMesh()) {
            ImGui::Text("Vertices : %d", diag_.vertices);
            ImGui::Text("Edges    : %d", diag_.edges);
            ImGui::Text("Faces    : %d", diag_.faces);
            ImGui::Text("Euler chi: %d", diag_.eulerCharacteristic);
            ImGui::Text("Components     : %d", diag_.components);
            ImGui::Text("Boundary loops : %d", diag_.boundaryLoops);
            ImGui::Text("Total area     : %.4f", diag_.totalArea);
            ImGui::Separator();
            ImGui::Text("Degenerate faces   : %d", diag_.degenerateFaces);
            ImGui::Text("Non-manifold verts : %d", diag_.nonManifoldVertices);
            ImGui::Text("Isolated verts     : %d", diag_.isolatedVertices);
            ImGui::Separator();
            ImGui::TextUnformatted("Best single proxy for the whole mesh");
            for (Metric me : {Metric::L21, Metric::L2}) {
                const Proxy p = fitProxy(me, wholeMesh_);
                ImGui::Text("%-4s N=(%.3f %.3f %.3f) err=%.5f", metricName(me),
                            p.N[0], p.N[1], p.N[2], metricError(me, wholeMesh_, p));
            }
        }
    }
    ImGui::End();

    makeAtlasTick();
    drawMakeAtlasPanel();
    drawVsaPanel();
    drawPatchPanel();
    drawLinabfPanel();
    drawParamPanel();
    drawRefinePanel();
    drawUvPanel();
    drawExportDialogs();
    drawHugeUI();
    drawRefineModal();

    handleSceneInput();

    if (showDemo_)
        ImGui::ShowDemoWindow(&showDemo_);

    if (rotate_)
        angle_ = std::fmod(angle_ + io.DeltaTime, 6.28318530718f);
}

void App::renderScene(int fbWidth, int fbHeight)
{
    scene_.begin(fbWidth, fbHeight, msaaSamples_);   // the default framebuffer if no multisampling
    glViewport(0, 0, fbWidth, fbHeight);
    glEnable(GL_DEPTH_TEST);
    glClearColor(clearColor_[0], clearColor_[1], clearColor_[2], 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (showTriangle_ && fbHeight > 0)
        triangle_.draw(angle_, float(fbWidth) / float(fbHeight));

    if (meshRenderer_.hasMesh() && fbHeight > 0) {
        float vp[16], v[16];
        camera_.viewProj(float(fbWidth) / float(fbHeight), vp);
        camera_.view(v);

        // GPU time of the mesh draw, read one frame late so that nothing stalls
        if (!gpuQuery_[0]) glGenQueries(2, gpuQuery_);
        const int cur = gpuQueryFrame_ & 1, prev = cur ^ 1;
        if (gpuQueryFrame_ > 1) {
            GLint available = 0;
            glGetQueryObjectiv(gpuQuery_[prev], GL_QUERY_RESULT_AVAILABLE, &available);
            if (available) {
                GLuint64 ns = 0;
                glGetQueryObjectui64v(gpuQuery_[prev], GL_QUERY_RESULT, &ns);
                gpuDrawMs_ = double(ns) * 1e-6;
                gpuDrawMsAvg_ = gpuDrawMsAvg_ == 0.0 ? gpuDrawMs_ : 0.95 * gpuDrawMsAvg_ + 0.05 * gpuDrawMs_;
            }
        }
        glBeginQuery(GL_TIME_ELAPSED, gpuQuery_[cur]);
        meshRenderer_.draw(vp, v, wireframe_, hugeShown_ ? (hugeView_ == 1 ? hugeCheckerCells() : 0.0f) : (colorMode_ == 8 ? atlasCheckerCells(atlas_, checkerSquares_) : 0.0f),
                           hugeShown_ ? hugeView_ == 3 : (colorMode_ == 10 || colorMode_ == 11), cullBackFaces_);
        glEndQuery(GL_TIME_ELAPSED);
        ++gpuQueryFrame_;

        if (meshRenderer_.hasLines())
            meshRenderer_.drawLines(vp, fbWidth, fbHeight, boundaryWidth_);
    }
    scene_.end();
}

void App::shutdown()
{
    stopRefine(true);
    // Delete GL resources while the context is alive
    if (window_) {
        glfwMakeContextCurrent(window_);
        triangle_.destroy();
        meshRenderer_.destroy();
        scene_.destroy();
        if (gpuQuery_[0]) { glDeleteQueries(2, gpuQuery_); gpuQuery_[0] = gpuQuery_[1] = 0; }
    }
    if (imguiReady_) {
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();
        imguiReady_ = false;
    }
    if (window_) {
        glfwDestroyWindow(window_);
        window_ = nullptr;
    }
    glfwTerminate();
}
