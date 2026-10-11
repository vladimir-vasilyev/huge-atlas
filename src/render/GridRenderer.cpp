#include "render/GridRenderer.h"

#include <cmath>
#include <vector>

namespace {

const char* kVS = R"(#version 410 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec4 aColor;   // rgb: tint (negative = use the line color), a: strength
uniform mat4 uVP;
out vec3 vPos;
out vec4 vColor;
void main() {
    vPos = aPos;
    vColor = aColor;
    gl_Position = uVP * vec4(aPos, 1.0);
}
)";

const char* kFS = R"(#version 410 core
in vec3 vPos;
in vec4 vColor;
uniform vec3 uLine;
out vec4 frag;
void main() {
    float fade = 1.0 - smoothstep(2.0, 5.0, length(vPos.xz));
    vec3 rgb = vColor.r < 0.0 ? uLine : vColor.rgb;
    frag = vec4(rgb, vColor.a * fade);
}
)";

} // namespace

bool GridRenderer::init(std::string* errorLog)
{
    if (!shader_.build(kVS, kFS, errorLog)) return false;
    uVP_ = shader_.uniform("uVP");
    uLine_ = shader_.uniform("uLine");

    std::vector<float> v;   // x y z r g b a
    auto line = [&](float x0, float y0, float z0, float x1, float y1, float z1, float r, float g, float b, float a) {
        const float p[14] = {x0, y0, z0, r, g, b, a, x1, y1, z1, r, g, b, a};
        v.insert(v.end(), p, p + 14);
    };
    const int n = 50;   // 0.1 * 50 = 5 units to each side
    for (int i = -n; i <= n; ++i) {
        if (i == 0) continue;   // the axes are drawn separately
        const float t = 0.1f * float(i), a = (i % 10 == 0) ? 0.45f : 0.16f;
        line(t, 0, -0.1f * n, t, 0, 0.1f * n, -1, -1, -1, a);
        line(-0.1f * n, 0, t, 0.1f * n, 0, t, -1, -1, -1, a);
    }
    line(-5, 0, 0, 5, 0, 0, 0.90f, 0.30f, 0.30f, 0.90f);   // X
    line(0, 0, -5, 0, 0, 5, 0.30f, 0.50f, 0.95f, 0.90f);   // Z
    line(0, 0, 0, 0, 1, 0, 0.35f, 0.80f, 0.40f, 0.90f);    // Y, up from the origin
    vertexCount_ = GLsizei(v.size() / 7);

    glGenVertexArrays(1, &vao_);
    glGenBuffers(1, &vbo_);
    glBindVertexArray(vao_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(v.size() * sizeof(float)), v.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 7 * sizeof(float), nullptr);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 7 * sizeof(float), reinterpret_cast<void*>(3 * sizeof(float)));
    glBindVertexArray(0);
    return true;
}

void GridRenderer::destroy()
{
    if (vbo_) glDeleteBuffers(1, &vbo_);
    if (vao_) glDeleteVertexArrays(1, &vao_);
    vbo_ = vao_ = 0;
    shader_.destroy();
}

void GridRenderer::draw(const float viewProj[16], bool darkBackground) const
{
    if (!vao_) return;
    shader_.use();
    glUniformMatrix4fv(uVP_, 1, GL_FALSE, viewProj);
    if (darkBackground) glUniform3f(uLine_, 0.85f, 0.87f, 0.92f);
    else glUniform3f(uLine_, 0.12f, 0.13f, 0.16f);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glBindVertexArray(vao_);
    glDrawArrays(GL_LINES, 0, vertexCount_);
    glBindVertexArray(0);
    glDisable(GL_BLEND);
}
