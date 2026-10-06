// Offscreen check of ID-buffer picking: needs an OpenGL 4.1 context (hidden GLFW window).
#include "TestUtil.h"
#include "render/Camera.h"
#include "render/MeshRenderer.h"

#include <GLFW/glfw3.h>

#include <array>

int main()
{
    if (!glfwInit()) { std::printf("SKIP: no GLFW\n"); return 0; }
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
    GLFWwindow* win = glfwCreateWindow(400, 300, "pick", nullptr, nullptr);
    if (!win) { std::printf("SKIP: no GL context\n"); glfwTerminate(); return 0; }
    glfwMakeContextCurrent(win);

    TriMesh m;
    addCube(m, 0.0);
    normalizeToUnitBox(m);

    MeshRenderer mr;
    std::string log;
    CHECK(mr.init(&log));
    mr.upload(m);

    int fbW, fbH;
    glfwGetFramebufferSize(win, &fbW, &fbH);
    Camera cam;                       // looks along -z at the origin: the +z side (faces 2, 3) faces us
    float vp[16];
    cam.viewProj(float(fbW) / float(fbH), vp);

    std::vector<int> ids;
    mr.pickFaces(vp, fbW, fbH, fbW / 2, fbH / 2, 0, ids);
    CHECK(ids.size() == 1 && (ids[0] == 2 || ids[0] == 3));

    ids.clear();   // the +z square is split along a diagonal through its center
    mr.pickFaces(vp, fbW, fbH, fbW / 2, fbH / 2, 4, ids);
    CHECK(ids.size() == 2 && ids[0] == 2 && ids[1] == 3);

    ids.clear();   // the corner of the window sees no mesh
    mr.pickFaces(vp, fbW, fbH, 2, 2, 1, ids);
    CHECK(ids.empty());

    ids.clear();   // a disc partly outside the framebuffer must not crash and still finds the cube
    mr.pickFaces(vp, fbW, fbH, 0, 0, fbW, ids);
    CHECK(!ids.empty());
    for (int f : ids) CHECK(f >= 0 && f < 12);

    // thick line layer: a red horizontal segment through the window center, 6 px wide
    {
        const float identity[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
        mr.uploadLines({-0.5f, 0.f, 0.f, 0.5f, 0.f, 0.f, 1.f, 0.f, 0.f});
        CHECK(mr.hasLines());
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        glViewport(0, 0, fbW, fbH);
        glClearColor(1.f, 1.f, 1.f, 1.f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glEnable(GL_DEPTH_TEST);
        mr.drawLines(identity, fbW, fbH, 6.0f);
        auto pixel = [&](int x, int y) {
            unsigned char c[4] = {0, 0, 0, 0};
            glReadPixels(x, y, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, c);
            return std::array<int, 3>{c[0], c[1], c[2]};
        };
        const auto center = pixel(fbW / 2, fbH / 2);
        CHECK(center[0] > 200 && center[1] < 60 && center[2] < 60);          // on the line
        const auto above = pixel(fbW / 2, fbH / 2 + 10);
        CHECK(above[0] > 240 && above[1] > 240 && above[2] > 240);           // 10 px away: background
        const auto outside = pixel(fbW / 2 + int(0.6f * fbW / 2), fbH / 2);
        CHECK(outside[0] > 240 && outside[1] > 240);                          // beyond the segment end
        mr.uploadLines({});
        CHECK(!mr.hasLines());
    }

    mr.destroy();
    glfwDestroyWindow(win);
    glfwTerminate();
    std::printf(failures ? "FAILED (%d)\n" : "OK\n", failures);
    return failures ? 1 : 0;
}
