#include "app/App.h"

#include "render/GL.h"   // strictly before GLFW (GLFW_INCLUDE_NONE is set in CMake)

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <ImGuiFileDialog.h>

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstdio>

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
    glfwWindowHint(GLFW_SAMPLES, 4);
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
    openMesh("data/cube.obj");   // default file; on failure only a status message

    glEnable(GL_DEPTH_TEST);
    GL_CHECK();
    return true;
}

void App::run()
{
    while (!glfwWindowShouldClose(window_)) {
        glfwPollEvents();

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
        glfwSwapBuffers(window_);
    }
}

void App::openMesh(const std::string& path)
{
    TriMesh m;
    std::string err;
    if (!loadMesh(path, m, &err)) {
        status_ = err;
        return;
    }
    normalizeToUnitBox(m);
    mesh_ = std::move(m);
    faceData_ = computeFaceData(mesh_);
    diag_ = diagnose(mesh_, faceData_);
    baseMoments_ = computeFaceMoments(mesh_, faceData_);
    faceWeights_.assign(mesh_.n_faces(), 1.0f);
    faceMoments_ = baseMoments_;
    wholeMesh_ = Moments();
    for (const Moments& fm : faceMoments_)
        wholeMesh_ += fm;
    vsa_ = VSAState();
    vsa_.metric = vsaMetric_ == 0 ? Metric::L21 : Metric::L2;
    vsaAdded_ = vsaSteps_ = 0;
    job_.reset();
    lloydStatus_.clear();
    patches_ = ExtractionResult();
    patchesValid_ = false;
    selectedPatch_ = -1;
    camera_.reset();
    uploadMesh();
    status_ = "loaded: " + path;
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
    meshRenderer_.upload(mesh_, colors, offsets);
    rebuildBoundaryLines();
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
            const Patch& p = patches_.patches[pi];
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
    if (ImGui::Button("Extract patches")) {
        patches_ = extractPatches(mesh_, vsa_.label, patchOptions_);
        patchesValid_ = true;
        patchedLabels_ = vsa_.label;
        selectedPatch_ = -1;
        colorMode_ = 5;
        uploadMesh();
    }
    if (!patchesValid_) { ImGui::End(); return; }

    if (patchedLabels_ != vsa_.label)
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

void App::rebuildMoments()
{
    faceMoments_ = baseMoments_;
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
        if (ImGui::RadioButton("Gray", &colorMode_, 0)) uploadMesh();
        ImGui::SameLine();
        if (ImGui::RadioButton("Face normals", &colorMode_, 1)) uploadMesh();
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

    drawVsaPanel();
    drawPatchPanel();

    handleSceneInput();

    if (showDemo_)
        ImGui::ShowDemoWindow(&showDemo_);

    if (rotate_)
        angle_ = std::fmod(angle_ + io.DeltaTime, 6.28318530718f);
}

void App::renderScene(int fbWidth, int fbHeight)
{
    glViewport(0, 0, fbWidth, fbHeight);
    glClearColor(clearColor_[0], clearColor_[1], clearColor_[2], 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (showTriangle_ && fbHeight > 0)
        triangle_.draw(angle_, float(fbWidth) / float(fbHeight));

    if (meshRenderer_.hasMesh() && fbHeight > 0) {
        float vp[16], v[16];
        camera_.viewProj(float(fbWidth) / float(fbHeight), vp);
        camera_.view(v);
        meshRenderer_.draw(vp, v, wireframe_, 1.0f);
        if (meshRenderer_.hasLines())
            meshRenderer_.drawLines(vp, fbWidth, fbHeight, boundaryWidth_);
    }
}

void App::shutdown()
{
    // Delete GL resources while the context is alive
    if (window_) {
        glfwMakeContextCurrent(window_);
        triangle_.destroy();
        meshRenderer_.destroy();
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
