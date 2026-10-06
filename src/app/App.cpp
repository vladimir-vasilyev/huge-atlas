#include "app/App.h"

#include "render/GL.h"   // strictly before GLFW (GLFW_INCLUDE_NONE is set in CMake)

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <ImGuiFileDialog.h>

#include <algorithm>
#include <cfloat>
#include <chrono>
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

    else if (colorMode_ == 6 && patchesValid_ && patches_.faceToPatch.size() == nF) {
        colors.assign(nF * 3, 0.3f);
        for (size_t pi = 0; pi < patches_.patches.size() && pi < linabf_.size(); ++pi) {
            if (!linabfSolved_[pi] || !linabf_[pi].ok) continue;
            const Patch& p = patches_.patches[pi];
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
        linabf_.assign(patches_.patches.size(), LinABFResult());
        linabfSolved_.assign(patches_.patches.size(), 0);
        uv_.assign(patches_.patches.size(), UVResult());
        uvStats_.assign(patches_.patches.size(), UVStats());
        uvDone_.assign(patches_.patches.size(), 0);
        params_.assign(patches_.patches.size(), PatchParam());
        paramDone_.assign(patches_.patches.size(), 0);
        atlasValid_ = false;
        paramStatus_.clear();
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

void App::solvePatches(bool onlySelected)
{
    const auto t0 = std::chrono::steady_clock::now();
    for (size_t i = 0; i < patches_.patches.size(); ++i) {
        if (onlySelected && int(i) != selectedPatch_) continue;
        if (!patches_.patches[i].isDisk()) continue;
        linabf_[i] = solveLinABF(patches_.patches[i].mesh, linabfParams_);
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
        const TriMesh& m = patches_.patches[i].mesh;
        uv_[i] = uvGreedy_ ? anglesToUVGreedy(m, linabf_[i].angles) : anglesToUV(m, linabf_[i].angles);
        uvStats_[i] = uv_[i].ok ? analyzeUV(m, uv_[i].uv, linabf_[i].angles) : UVStats();
        uvDone_[i] = 1;
    }
}

void App::drawUvPanel()
{
    ImGui::SetNextWindowSize(ImVec2(520, 520), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("UV view")) { ImGui::End(); return; }

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
        layers.push_back({&patches_.patches[sel].mesh, &uv_[sel].uv, sel, nullptr});
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
    dl->PopClipRect();
    ImGui::End();
}

void App::runParam(bool onlySelected)
{
    const auto t0 = std::chrono::steady_clock::now();
    int done = 0;
    for (size_t i = 0; i < patches_.patches.size(); ++i) {
        if (onlySelected && int(i) != selectedPatch_) continue;
        if (!patches_.patches[i].isDisk()) continue;
        params_[i] = parameterizePatch(mesh_, patches_.patches[i], paramOptions_);
        paramDone_[i] = 1;
        ++done;
    }
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
        in[i] = {&params_[i].uv, uvArea, area3};
    }
    atlas_ = packAtlas(in, atlasOptions_);
    atlasValid_ = true;
    uvShowAtlas_ = true;
    colorMode_ = 8;
    uploadMesh();
}

void App::drawParamPanel()
{
    constexpr const char* kExportKey = "ExportObjDlg";
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
    {
        float g = float(atlasOptions_.gutter);
        if (ImGui::SliderFloat("Gutter", &g, 0.0f, 0.05f, "%.3f")) atlasOptions_.gutter = g;
        ImGui::Checkbox("Rotate to minimum rectangle", &atlasOptions_.rotate);
        ImGui::SliderFloat("Checker cells", &checkerCells_, 4.0f, 128.0f, "%.0f");
    }
    ImGui::BeginDisabled(done == 0);
    if (ImGui::Button("Pack atlas")) packAtlasNow();
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::BeginDisabled(!atlasValid_);
    bool openExport = ImGui::Button("Export OBJ...");
    ImGui::EndDisabled();
    if (atlasValid_) ImGui::Text("Efficiency %.1f%%", 100.0 * atlas_.efficiency);

    if (openExport) {
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
            const int faces = exportAtlasOBJ(path, mesh_, pv, uvv);
            char buf[256];
            if (faces < 0) std::snprintf(buf, sizeof(buf), "export failed: %s", path.c_str());
            else std::snprintf(buf, sizeof(buf), "exported %d of %d faces to %s", faces, int(mesh_.n_faces()), path.c_str());
            paramStatus_ = buf;
        }
        ImGuiFileDialog::Instance()->Close();
    }
    ImGui::End();
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
    drawLinabfPanel();
    drawParamPanel();
    drawUvPanel();

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
        meshRenderer_.draw(vp, v, wireframe_, 1.0f, colorMode_ == 8 ? checkerCells_ : 0.0f);
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
