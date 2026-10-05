#include "render/MeshRenderer.h"

namespace {

const char* kVS = R"(#version 410 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aBary;
layout(location = 2) in vec3 aColor;
uniform mat4 uVP;
uniform mat4 uView;
out vec3 vBary;
out vec3 vColor;
out vec3 vViewPos;
void main() {
    vBary = aBary;
    vColor = aColor;
    vViewPos = (uView * vec4(aPos, 1.0)).xyz;
    gl_Position = uVP * vec4(aPos, 1.0);
}
)";

const char* kFS = R"(#version 410 core
in vec3 vBary;
in vec3 vColor;
in vec3 vViewPos;
uniform float uWire;       // 0 or 1
uniform float uWireWidth;  // in pixels
out vec4 fragColor;
void main() {
    // flat normal from position derivatives in view space
    vec3 n = normalize(cross(dFdx(vViewPos), dFdy(vViewPos)));
    float diff = abs(n.z) * 0.75 + 0.25;       // headlight, two-sided
    vec3 col = vColor * diff;
    if (uWire > 0.5) {
        vec3 d = fwidth(vBary);
        vec3 a = smoothstep(vec3(0.0), d * uWireWidth, vBary);
        float edge = 1.0 - min(min(a.x, a.y), a.z);
        col = mix(col, vec3(0.05), edge * 0.85);
    }
    fragColor = vec4(col, 1.0);
}
)";

} // namespace

bool MeshRenderer::init(std::string* errorLog)
{
    if (!shader_.build(kVS, kFS, errorLog)) return false;
    uVP_        = shader_.uniform("uVP");
    uView_      = shader_.uniform("uView");
    uWire_      = shader_.uniform("uWire");
    uWireWidth_ = shader_.uniform("uWireWidth");
    glGenVertexArrays(1, &vao_);
    glGenBuffers(1, &vbo_);
    return true;
}

void MeshRenderer::upload(const TriMesh& mesh, const std::vector<float>& faceColors)
{
    std::vector<float> buf;
    buf.reserve(mesh.n_faces() * 3 * 9);
    static const float bary[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    for (auto f : mesh.faces()) {
        float c[3] = {0.72f, 0.74f, 0.78f};
        if (faceColors.size() >= size_t(f.idx() + 1) * 3)
            for (int k = 0; k < 3; ++k) c[k] = faceColors[f.idx() * 3 + k];
        int i = 0;
        for (auto v : mesh.fv_range(f)) {
            const auto p = mesh.point(v);
            buf.insert(buf.end(), {float(p[0]), float(p[1]), float(p[2]),
                                   bary[i][0], bary[i][1], bary[i][2], c[0], c[1], c[2]});
            ++i;
        }
    }
    vertexCount_ = GLsizei(buf.size() / 9);

    glBindVertexArray(vao_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(buf.size() * sizeof(float)), buf.data(), GL_STATIC_DRAW);
    const GLsizei stride = 9 * sizeof(float);
    for (int a = 0; a < 3; ++a) {
        glEnableVertexAttribArray(a);
        glVertexAttribPointer(a, 3, GL_FLOAT, GL_FALSE, stride,
                              reinterpret_cast<void*>(a * 3 * sizeof(float)));
    }
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    GL_CHECK();
}

void MeshRenderer::draw(const float vp[16], const float view[16], bool wireframe, float wireWidth) const
{
    if (!vertexCount_) return;
    shader_.use();
    glUniformMatrix4fv(uVP_, 1, GL_FALSE, vp);
    glUniformMatrix4fv(uView_, 1, GL_FALSE, view);
    glUniform1f(uWire_, wireframe ? 1.0f : 0.0f);
    glUniform1f(uWireWidth_, wireWidth);
    glBindVertexArray(vao_);
    glDrawArrays(GL_TRIANGLES, 0, vertexCount_);
    glBindVertexArray(0);
    GL_CHECK();
}

void MeshRenderer::destroy()
{
    if (vbo_) { glDeleteBuffers(1, &vbo_); vbo_ = 0; }
    if (vao_) { glDeleteVertexArrays(1, &vao_); vao_ = 0; }
    shader_.destroy();
    vertexCount_ = 0;
}
