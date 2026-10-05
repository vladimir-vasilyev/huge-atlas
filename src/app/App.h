#pragma once

#include "core/MeshPrep.h"
#include "core/Partition.h"
#include "core/Proxy.h"
#include "render/Camera.h"
#include "render/DebugTriangle.h"
#include "render/MeshRenderer.h"

#include <string>

struct GLFWwindow;

// Window, main loop, ImGui. Algorithmic modules (VSA, LinABF) do not live here:
// App only owns their state and calls them from the UI.
class App {
public:
    bool init(int width, int height, const char* title);
    void run();
    void shutdown();

private:
    void drawUI();
    void drawMainMenu();
    void renderScene(int fbWidth, int fbHeight);
    void handleSceneInput();
    void openMesh(const std::string& path);
    void uploadMesh();
    void drawVsaPanel();
    VSAInput vsaInput() const { return {mesh_, faceData_, faceMoments_}; }

    GLFWwindow* window_ = nullptr;
    bool imguiReady_ = false;

    // Context info, collected once
    std::string glVersion_, glRenderer_, glslVersion_;

    // Stage 0 UI state
    float clearColor_[3] = {0.16f, 0.17f, 0.20f};
    bool  showDemo_ = false;
    bool  showTriangle_ = true;
    bool  rotate_ = true;
    float angle_ = 0.0f;

    DebugTriangle triangle_;

    // Stage 1: mesh and its display
    TriMesh          mesh_;
    FaceData         faceData_;
    MeshDiagnostics  diag_;
    std::vector<Moments> faceMoments_;
    Moments          wholeMesh_;       // moments of all faces (stage 2 check)
    MeshRenderer     meshRenderer_;
    Camera           camera_;
    std::string      lastDir_ = ".";
    std::string      status_;
    bool             wireframe_ = true;
    int              colorMode_ = 0;   // 0 - gray, 1 - face normal, 2 - VSA regions
    // Stage 3: VSA
    VSAState         vsa_;
    int              vsaK_ = 6;
    int              vsaRngSeed_ = 1;
    int              vsaMetric_ = 0;       // 0 - L2,1  1 - L2
    int              vsaAdded_ = 0;        // proxies added by the last partition for unseeded components
    int              vsaSteps_ = 0;
    LloydParams      lloydParams_;
    bool             lloydRunning_ = false;
    int              lloydIter_ = 0;       // iterations done in the current run
    std::string      lloydStatus_;
    void lloydTick();                      // one Lloyd step of a running loop (called once per frame)

    bool             dragging_ = false;
    bool             panning_ = false;
};
