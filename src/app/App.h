#pragma once

#include "core/MeshPrep.h"
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
    Moments          wholeMesh_;       // moments of all faces (stage 2 check)
    MeshRenderer     meshRenderer_;
    Camera           camera_;
    std::string      lastDir_ = ".";
    std::string      status_;
    bool             wireframe_ = true;
    int              colorMode_ = 0;   // 0 - gray, 1 - face normal
    bool             dragging_ = false;
    bool             panning_ = false;
};
