#pragma once

#include "core/MeshPrep.h"
#include "core/Operations.h"
#include "core/Partition.h"
#include "core/Proxy.h"
#include "param/PatchExtractor.h"
#include "render/Camera.h"
#include "render/DebugTriangle.h"
#include "render/MeshRenderer.h"

#include <memory>
#include <string>
#include <vector>

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
    void rebuildMoments();                 // face moments = base moments * weights; refit if partitioned
    void startJob(RefineJob::Kind kind);
    void jobTick();                        // one step of the running job (once per frame)
    void paintWeights(int fbW, int fbH);
    void drawPatchPanel();
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
    int              colorMode_ = 0;   // 0 - gray, 1 - face normal, 2 - VSA regions, 3 - weights, 4 - patches, 5 - patch kind
    // Stage 3: VSA
    VSAState         vsa_;
    int              vsaK_ = 6;
    int              vsaRngSeed_ = 1;
    int              vsaMetric_ = 0;       // 0 - L2,1  1 - L2
    int              vsaAdded_ = 0;        // proxies added by the last partition for unseeded components
    int              vsaSteps_ = 0;
    LloydParams      lloydParams_;
    TeleportParams   teleportParams_;
    int              seedMethod_ = 1;      // 0 - random, 1 - farthest point
    std::unique_ptr<RefineJob> job_;       // running Lloyd / grow / teleport job
    std::string      lloydStatus_;

    // Stage 5: per-face weights (error density), painted with a brush
    std::vector<Moments> baseMoments_;     // unweighted
    std::vector<float>   faceWeights_;
    bool             paintMode_ = false;
    bool             eraseMode_ = false;
    int              brushRadius_ = 20;    // framebuffer pixels
    float            brushWeight_ = 4.0f;
    bool             painted_ = false;     // weights changed during the current stroke

    // Stage 6: patches
    ExtractionResult patches_;
    PatchOptions     patchOptions_;
    bool             patchesValid_ = false;
    std::vector<int> patchedLabels_;       // labels the patches were extracted from (to detect a stale result)
    int              selectedPatch_ = -1;
    float            explode_ = 0.0f;

    bool             dragging_ = false;
    bool             panning_ = false;
};
