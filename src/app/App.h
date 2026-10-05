#pragma once

#include "core/MeshPrep.h"
#include "render/Camera.h"
#include "render/DebugTriangle.h"
#include "render/MeshRenderer.h"

#include <string>

struct GLFWwindow;

// Окно, главный цикл, ImGui. Алгоритмические модули (VSA, LinABF) сюда не попадают:
// App только владеет их состоянием и вызывает их из UI.
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

    // Информация о контексте, собирается один раз
    std::string glVersion_, glRenderer_, glslVersion_;

    // Состояние UI этапа 0
    float clearColor_[3] = {0.16f, 0.17f, 0.20f};
    bool  showDemo_ = false;
    bool  showTriangle_ = true;
    bool  rotate_ = true;
    float angle_ = 0.0f;

    DebugTriangle triangle_;

    // Этап 1: меш и его отображение
    TriMesh          mesh_;
    FaceData         faceData_;
    MeshDiagnostics  diag_;
    MeshRenderer     meshRenderer_;
    Camera           camera_;
    std::string      lastDir_ = ".";
    std::string      status_;
    bool             wireframe_ = true;
    int              colorMode_ = 0;   // 0 — серый, 1 — по нормали грани
    bool             dragging_ = false;
    bool             panning_ = false;
};
