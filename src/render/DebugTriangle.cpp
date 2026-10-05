#include "render/DebugTriangle.h"

namespace {

const char* kVS = R"(#version 410 core
layout(location = 0) in vec2 aPos;
layout(location = 1) in vec3 aColor;
uniform float uAngle;
uniform float uAspect;
out vec3 vColor;
void main() {
    float c = cos(uAngle), s = sin(uAngle);
    vec2 p = vec2(c * aPos.x - s * aPos.y, s * aPos.x + c * aPos.y);
    p.x /= uAspect;                     // без искажения при любом соотношении сторон
    gl_Position = vec4(p, 0.0, 1.0);
    vColor = aColor;
}
)";

const char* kFS = R"(#version 410 core
in vec3 vColor;
out vec4 fragColor;
void main() { fragColor = vec4(vColor, 1.0); }
)";

} // namespace

bool DebugTriangle::init(std::string* errorLog)
{
    if (!shader_.build(kVS, kFS, errorLog))
        return false;
    uAngle_  = shader_.uniform("uAngle");
    uAspect_ = shader_.uniform("uAspect");

    // x, y, r, g, b
    const float verts[] = {
         0.0f,   0.6f,  1.0f, 0.3f, 0.3f,
        -0.52f, -0.3f,  0.3f, 1.0f, 0.3f,
         0.52f, -0.3f,  0.3f, 0.3f, 1.0f,
    };

    glGenVertexArrays(1, &vao_);
    glGenBuffers(1, &vbo_);
    glBindVertexArray(vao_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(verts), verts, GL_STATIC_DRAW);

    const GLsizei stride = 5 * sizeof(float);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(0));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(2 * sizeof(float)));

    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    GL_CHECK();
    return true;
}

void DebugTriangle::draw(float angle, float aspect) const
{
    if (!vao_) return;
    shader_.use();
    glUniform1f(uAngle_, angle);
    glUniform1f(uAspect_, aspect > 0.0f ? aspect : 1.0f);
    glBindVertexArray(vao_);
    glDrawArrays(GL_TRIANGLES, 0, 3);
    glBindVertexArray(0);
    GL_CHECK();
}

void DebugTriangle::destroy()
{
    if (vbo_) { glDeleteBuffers(1, &vbo_); vbo_ = 0; }
    if (vao_) { glDeleteVertexArrays(1, &vao_); vao_ = 0; }
    shader_.destroy();
}
