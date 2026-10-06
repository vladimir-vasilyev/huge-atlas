#include "render/SceneFramebuffer.h"

#include <algorithm>

int SceneFramebuffer::maxSamples()
{
    GLint n = 0;
    glGetIntegerv(GL_MAX_SAMPLES, &n);
    return n;
}

bool SceneFramebuffer::begin(int w, int h, int samples)
{
    active_ = false;
    if (samples < 2 || w <= 0 || h <= 0) return false;
    samples = std::min(samples, maxSamples());
    if (samples < 2) return false;

    if (!fbo_ || w != width_ || h != height_ || samples != samples_) {
        if (!fbo_) {
            glGenFramebuffers(1, &fbo_);
            glGenRenderbuffers(1, &color_);
            glGenRenderbuffers(1, &depth_);
        }
        glBindRenderbuffer(GL_RENDERBUFFER, color_);
        glRenderbufferStorageMultisample(GL_RENDERBUFFER, samples, GL_RGBA8, w, h);
        glBindRenderbuffer(GL_RENDERBUFFER, depth_);
        glRenderbufferStorageMultisample(GL_RENDERBUFFER, samples, GL_DEPTH_COMPONENT24, w, h);
        glBindRenderbuffer(GL_RENDERBUFFER, 0);
        glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, color_);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depth_);
        width_ = w;
        height_ = h;
        samples_ = samples;
    }
    glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        return false;
    }
    active_ = true;
    return true;
}

void SceneFramebuffer::end()
{
    if (!active_) return;
    glBindFramebuffer(GL_READ_FRAMEBUFFER, fbo_);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
    glBlitFramebuffer(0, 0, width_, height_, 0, 0, width_, height_, GL_COLOR_BUFFER_BIT, GL_NEAREST);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    active_ = false;
}

void SceneFramebuffer::destroy()
{
    if (fbo_) { glDeleteFramebuffers(1, &fbo_); fbo_ = 0; }
    if (color_) { glDeleteRenderbuffers(1, &color_); color_ = 0; }
    if (depth_) { glDeleteRenderbuffers(1, &depth_); depth_ = 0; }
    width_ = height_ = samples_ = 0;
    active_ = false;
}
