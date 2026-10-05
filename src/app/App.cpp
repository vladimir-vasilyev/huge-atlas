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
    faceMoments_ = computeFaceMoments(mesh_, faceData_);
    wholeMesh_ = Moments();
    for (const Moments& fm : faceMoments_)
        wholeMesh_ += fm;
    vsa_ = VSAState();
    vsa_.metric = vsaMetric_ == 0 ? Metric::L21 : Metric::L2;
    vsaAdded_ = vsaSteps_ = 0;
    lloydRunning_ = false;
    lloydStatus_.clear();
    camera_.reset();
    uploadMesh();
    status_ = "loaded: " + path;
}

void App::uploadMesh()
{
    std::vector<float> colors;
    if (colorMode_ == 1) {
        colors.resize(mesh_.n_faces() * 3);
        for (size_t i = 0; i < mesh_.n_faces(); ++i)
            for (int k = 0; k < 3; ++k)
                colors[i * 3 + k] = float(faceData_.normal[i][k] * 0.5 + 0.5);
    }
    else if (colorMode_ == 2 && vsa_.label.size() == mesh_.n_faces()) {
        colors.resize(mesh_.n_faces() * 3);
        for (size_t i = 0; i < mesh_.n_faces(); ++i) {
            const int l = vsa_.label[i];
            if (l < 0) { colors[i * 3] = colors[i * 3 + 1] = colors[i * 3 + 2] = 0.3f; continue; }
            // golden-ratio hue steps give well-separated colors for any number of regions
            const float h = std::fmod(float(l) * 0.61803399f, 1.0f) * 6.0f;
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
            colors[i * 3] = r + m; colors[i * 3 + 1] = g + m; colors[i * 3 + 2] = b + m;
        }
    }
    meshRenderer_.upload(mesh_, colors);
}

void App::lloydTick()
{
    if (!lloydRunning_ || vsa_.k() == 0) { lloydRunning_ = false; return; }
    const LloydStepInfo info = lloydStep(vsaInput(), vsa_);
    vsaAdded_ = info.added;
    ++vsaSteps_;
    ++lloydIter_;
    uploadMesh();
    if (lloydConverged(vsa_, lloydParams_, info)) {
        lloydRunning_ = false;
        lloydStatus_ = "converged after " + std::to_string(lloydIter_) + " iterations";
    } else if (lloydIter_ >= lloydParams_.maxIterations) {
        lloydRunning_ = false;
        lloydStatus_ = "stopped: max iterations (" + std::to_string(lloydIter_) + ")";
    }
}

void App::drawVsaPanel()
{
    lloydTick();
    if (!ImGui::Begin("VSA")) { ImGui::End(); return; }
    if (!meshRenderer_.hasMesh()) {
        ImGui::TextUnformatted("No mesh loaded");
        ImGui::End();
        return;
    }

    const char* metrics[] = {"L2,1", "L2"};
    if (ImGui::Combo("Metric", &vsaMetric_, metrics, 2)) {
        vsa_ = VSAState();
        vsa_.metric = vsaMetric_ == 0 ? Metric::L21 : Metric::L2;
        vsaAdded_ = vsaSteps_ = 0;
        lloydRunning_ = false;
        lloydStatus_.clear();
        if (colorMode_ == 2) uploadMesh();
    }
    ImGui::InputInt("Proxies (k)", &vsaK_);
    vsaK_ = std::max(1, std::min(vsaK_, int(mesh_.n_faces())));
    ImGui::InputInt("Random seed", &vsaRngSeed_);

    auto refresh = [&] { if (colorMode_ == 2) uploadMesh(); };
    if (ImGui::Button("Seed")) {
        vsa_.metric = vsaMetric_ == 0 ? Metric::L21 : Metric::L2;
        initRandomSeeds(vsaInput(), vsa_, vsaK_, uint64_t(vsaRngSeed_));
        vsaAdded_ = vsaSteps_ = 0;
        lloydRunning_ = false;
        lloydStatus_.clear();
        refresh();
    }
    ImGui::SameLine();
    const bool seeded = vsa_.k() > 0;
    ImGui::BeginDisabled(!seeded);
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
    ImGui::SameLine();
    if (!lloydRunning_) {
        if (ImGui::Button("Run")) {
            lloydRunning_ = true;
            lloydIter_ = 0;
            lloydStatus_.clear();
            colorMode_ = 2;
        }
    } else if (ImGui::Button("Stop")) {
        lloydRunning_ = false;
        lloydStatus_ = "stopped";
    }
    ImGui::EndDisabled();

    ImGui::InputInt("Max iterations", &lloydParams_.maxIterations);
    lloydParams_.maxIterations = std::max(1, lloydParams_.maxIterations);
    ImGui::InputInt("Patience", &lloydParams_.patience);
    lloydParams_.patience = std::max(1, lloydParams_.patience);
    {
        float tol = float(lloydParams_.relTolerance);
        if (ImGui::InputFloat("Rel. tolerance", &tol, 0.0f, 0.0f, "%.1e"))
            lloydParams_.relTolerance = std::max(0.0f, tol);
    }

    ImGui::Separator();
    if (!lloydStatus_.empty())
        ImGui::TextUnformatted(lloydStatus_.c_str());
    ImGui::Text("Proxies : %d", vsa_.k());
    ImGui::Text("Partitions run : %d", vsaSteps_);
    if (vsaAdded_ > 0)
        ImGui::Text("Added for unseeded components: %d", vsaAdded_);
    if (vsa_.label.size() == mesh_.n_faces())
        ImGui::Text("Energy : %.6f", totalEnergy(vsa_));
    if (!vsa_.energyHistory.empty()) {
        const std::vector<float> h(vsa_.energyHistory.begin(), vsa_.energyHistory.end());
        char overlay[48];
        std::snprintf(overlay, sizeof(overlay), "E = %.5g", h.back());
        ImGui::PlotLines("##energy", h.data(), int(h.size()), 0, overlay, FLT_MAX, FLT_MAX, ImVec2(-1, 80));
    }
    ImGui::End();
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

    if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
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
