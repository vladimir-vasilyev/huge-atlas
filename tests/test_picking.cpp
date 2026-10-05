// Offscreen check of ID-buffer picking: needs an OpenGL 4.1 context (hidden GLFW window).
#include "TestUtil.h"
#include "render/Camera.h"
#include "render/MeshRenderer.h"

#include <GLFW/glfw3.h>

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

    mr.destroy();
    glfwDestroyWindow(win);
    glfwTerminate();
    std::printf(failures ? "FAILED (%d)\n" : "OK\n", failures);
    return failures ? 1 : 0;
}
