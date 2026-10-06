#pragma once

#include "render/GL.h"

// An offscreen multisample target for the 3D scene, resolved onto the default framebuffer, so that
// the anti-aliasing can be switched at run time (the sample count of the window itself is fixed when
// the window is created). With fewer than two samples the scene is drawn to the default framebuffer.
class SceneFramebuffer {
public:
    ~SceneFramebuffer() { destroy(); }

    // Binds the target (created or resized as needed). false: no offscreen target, draw to the default framebuffer.
    bool begin(int width, int height, int samples);
    // Resolves the target onto the default framebuffer and binds that. A no-op if begin() returned false.
    void end();
    void destroy();

    static int maxSamples();   // needs a current context

private:
    GLuint fbo_ = 0, color_ = 0, depth_ = 0;
    int width_ = 0, height_ = 0, samples_ = 0;
    bool active_ = false;
};
