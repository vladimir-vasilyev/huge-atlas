#pragma once

#include "render/DebugTriangle.h"

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
    void renderScene(int fbWidth, int fbHeight);

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
};
