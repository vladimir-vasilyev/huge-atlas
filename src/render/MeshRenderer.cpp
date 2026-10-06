#include "render/MeshRenderer.h"

#include <algorithm>

namespace {

const char* kVS = R"(#version 410 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aBary;
layout(location = 2) in vec3 aColor;
layout(location = 3) in vec2 aUV;
uniform mat4 uVP;
uniform mat4 uView;
out vec3 vBary;
out vec3 vColor;
out vec3 vViewPos;
out vec2 vUV;
void main() {
    vBary = aBary;
    vUV = aUV;
    vColor = aColor;
    vViewPos = (uView * vec4(aPos, 1.0)).xyz;
    gl_Position = uVP * vec4(aPos, 1.0);
}
)";

const char* kFS = R"(#version 410 core
in vec3 vBary;
in vec3 vColor;
in vec3 vViewPos;
in vec2 vUV;
uniform float uChecker;    // number of checker cells across the unit square; 0 = off
uniform float uWire;       // 0 or 1
uniform float uWireWidth;  // in pixels
out vec4 fragColor;
void main() {
    // flat normal from position derivatives in view space
    vec3 n = normalize(cross(dFdx(vViewPos), dFdy(vViewPos)));
    float diff = abs(n.z) * 0.75 + 0.25;       // headlight, two-sided
    vec3 col = vColor * diff;
    if (uChecker > 0.5) {
        vec2 c = floor(vUV * uChecker);
        float k = mod(c.x + c.y, 2.0);
        col = col * mix(0.45, 1.1, k);
    }
    if (uWire > 0.5) {
        vec3 d = fwidth(vBary);
        vec3 a = smoothstep(vec3(0.0), d * uWireWidth, vBary);
        float edge = 1.0 - min(min(a.x, a.y), a.z);
        col = mix(col, vec3(0.05), edge * 0.85);
    }
    fragColor = vec4(col, 1.0);
}
)";

const char* kPickVS = R"(#version 410 core
layout(location = 0) in vec3 aPos;
uniform mat4 uVP;
flat out int vId;
void main() {
    vId = gl_VertexID / 3 + 1;      // vertices are unrolled: three per face; 0 means "no face"
    gl_Position = uVP * vec4(aPos, 1.0);
}
)";

const char* kPickFS = R"(#version 410 core
flat in int vId;
out vec4 fragColor;
void main() {
    fragColor = vec4(float(vId & 255) / 255.0, float((vId >> 8) & 255) / 255.0,
                     float((vId >> 16) & 255) / 255.0, 1.0);
}
)";

const char* kLineVS = R"(#version 410 core
layout(location = 0) in vec3 aA;
layout(location = 1) in vec3 aB;
layout(location = 2) in vec2 aCorner;   // x: 0 at A, 1 at B; y: side -1 / +1
layout(location = 3) in vec3 aColor;
uniform mat4 uVP;
uniform vec2 uViewport;                 // framebuffer size in pixels
uniform float uWidth;                   // line width in pixels
out vec3 vColor;
void main() {
    vec4 a = uVP * vec4(aA, 1.0);
    vec4 b = uVP * vec4(aB, 1.0);
    vec2 sa = a.xy / a.w * uViewport * 0.5;
    vec2 sb = b.xy / b.w * uViewport * 0.5;
    vec2 d = sb - sa;
    float len = length(d);
    vec2 dir = len > 1e-6 ? d / len : vec2(1.0, 0.0);
    vec2 n = vec2(-dir.y, dir.x);
    vec4 c = mix(a, b, aCorner.x);
    vec2 offPx = n * aCorner.y * uWidth * 0.5 + dir * (aCorner.x * 2.0 - 1.0) * uWidth * 0.5;
    c.xy += offPx / (uViewport * 0.5) * c.w;
    c.z -= 2e-4 * c.w;                  // pull towards the camera so the lines win against the surface
    gl_Position = c;
    vColor = aColor;
}
)";

const char* kLineFS = R"(#version 410 core
in vec3 vColor;
out vec4 fragColor;
void main() { fragColor = vec4(vColor, 1.0); }
)";

} // namespace

bool MeshRenderer::init(std::string* errorLog)
{
    if (!shader_.build(kVS, kFS, errorLog)) return false;
    uVP_        = shader_.uniform("uVP");
    uView_      = shader_.uniform("uView");
    uWire_      = shader_.uniform("uWire");
    uWireWidth_ = shader_.uniform("uWireWidth");
    uChecker_   = shader_.uniform("uChecker");
    if (!lineShader_.build(kLineVS, kLineFS, errorLog)) return false;
    uLineVP_       = lineShader_.uniform("uVP");
    uLineViewport_ = lineShader_.uniform("uViewport");
    uLineWidth_    = lineShader_.uniform("uWidth");
    glGenVertexArrays(1, &lineVao_);
    glGenBuffers(1, &lineVbo_);
    if (!pickShader_.build(kPickVS, kPickFS, errorLog)) return false;
    uPickVP_ = pickShader_.uniform("uVP");
    glGenVertexArrays(1, &vao_);
    glGenBuffers(1, &vbo_);
    return true;
}

void MeshRenderer::upload(const TriMesh& mesh, const std::vector<float>& faceColors,
                          const std::vector<float>& faceOffsets, const std::vector<float>& faceUV)
{
    std::vector<float> buf;
    buf.reserve(mesh.n_faces() * 3 * 11);
    static const float bary[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    for (auto f : mesh.faces()) {
        float c[3] = {0.72f, 0.74f, 0.78f};
        if (faceColors.size() >= size_t(f.idx() + 1) * 3)
            for (int k = 0; k < 3; ++k) c[k] = faceColors[f.idx() * 3 + k];
        float o[3] = {0.f, 0.f, 0.f};
        if (faceOffsets.size() >= size_t(f.idx() + 1) * 3)
            for (int k = 0; k < 3; ++k) o[k] = faceOffsets[f.idx() * 3 + k];
        int i = 0;
        for (auto v : mesh.fv_range(f)) {
            const auto p = mesh.point(v);
            float uv[2] = {0.f, 0.f};
            if (faceUV.size() >= size_t(f.idx() + 1) * 6)
                for (int k = 0; k < 2; ++k) uv[k] = faceUV[f.idx() * 6 + i * 2 + k];
            buf.insert(buf.end(), {float(p[0]) + o[0], float(p[1]) + o[1], float(p[2]) + o[2],
                                   bary[i][0], bary[i][1], bary[i][2], c[0], c[1], c[2], uv[0], uv[1]});
            ++i;
        }
    }
    vertexCount_ = GLsizei(buf.size() / 11);

    glBindVertexArray(vao_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(buf.size() * sizeof(float)), buf.data(), GL_STATIC_DRAW);
    const GLsizei stride = 11 * sizeof(float);
    for (int a = 0; a < 3; ++a) {
        glEnableVertexAttribArray(a);
        glVertexAttribPointer(a, 3, GL_FLOAT, GL_FALSE, stride,
                              reinterpret_cast<void*>(a * 3 * sizeof(float)));
    }
    glEnableVertexAttribArray(3);
    glVertexAttribPointer(3, 2, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(9 * sizeof(float)));
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    GL_CHECK();
}

void MeshRenderer::draw(const float vp[16], const float view[16], bool wireframe, float wireWidth,
                        float checkerCells) const
{
    if (!vertexCount_) return;
    shader_.use();
    glUniformMatrix4fv(uVP_, 1, GL_FALSE, vp);
    glUniformMatrix4fv(uView_, 1, GL_FALSE, view);
    glUniform1f(uWire_, wireframe ? 1.0f : 0.0f);
    glUniform1f(uWireWidth_, wireWidth);
    glUniform1f(uChecker_, checkerCells);
    glBindVertexArray(vao_);
    glDrawArrays(GL_TRIANGLES, 0, vertexCount_);
    glBindVertexArray(0);
    GL_CHECK();
}

void MeshRenderer::uploadLines(const std::vector<float>& segs)
{
    // 6 vertices per segment: A(3) B(3) corner(2) color(3) = 11 floats
    static const float corners[6][2] = {{0, -1}, {0, 1}, {1, 1}, {0, -1}, {1, 1}, {1, -1}};
    std::vector<float> buf;
    buf.reserve(segs.size() / 9 * 6 * 11);
    for (size_t i = 0; i + 9 <= segs.size(); i += 9)
        for (const auto& c : corners) {
            buf.insert(buf.end(), segs.begin() + i, segs.begin() + i + 6);
            buf.push_back(c[0]);
            buf.push_back(c[1]);
            buf.insert(buf.end(), segs.begin() + i + 6, segs.begin() + i + 9);
        }
    lineVertexCount_ = GLsizei(buf.size() / 11);

    glBindVertexArray(lineVao_);
    glBindBuffer(GL_ARRAY_BUFFER, lineVbo_);
    glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(buf.size() * sizeof(float)), buf.data(), GL_DYNAMIC_DRAW);
    const GLsizei stride = 11 * sizeof(float);
    const int sizes[4] = {3, 3, 2, 3};
    size_t offset = 0;
    for (int a = 0; a < 4; ++a) {
        glEnableVertexAttribArray(a);
        glVertexAttribPointer(a, sizes[a], GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(offset * sizeof(float)));
        offset += sizes[a];
    }
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    GL_CHECK();
}

void MeshRenderer::drawLines(const float vp[16], int fbW, int fbH, float widthPx) const
{
    if (!lineVertexCount_) return;
    lineShader_.use();
    glUniformMatrix4fv(uLineVP_, 1, GL_FALSE, vp);
    glUniform2f(uLineViewport_, float(fbW), float(fbH));
    glUniform1f(uLineWidth_, widthPx);
    glBindVertexArray(lineVao_);
    glDrawArrays(GL_TRIANGLES, 0, lineVertexCount_);
    glBindVertexArray(0);
    GL_CHECK();
}

void MeshRenderer::ensurePickTarget(int w, int h)
{
    if (pickFbo_ && w == pickW_ && h == pickH_) return;
    if (!pickFbo_) {
        glGenFramebuffers(1, &pickFbo_);
        glGenTextures(1, &pickColor_);
        glGenRenderbuffers(1, &pickDepth_);
    }
    glBindTexture(GL_TEXTURE_2D, pickColor_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glBindTexture(GL_TEXTURE_2D, 0);
    glBindRenderbuffer(GL_RENDERBUFFER, pickDepth_);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, w, h);
    glBindRenderbuffer(GL_RENDERBUFFER, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, pickFbo_);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, pickColor_, 0);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, pickDepth_);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    pickW_ = w;
    pickH_ = h;
    GL_CHECK();
}

void MeshRenderer::pickFaces(const float vp[16], int fbW, int fbH, int cx, int cy, int radius,
                             std::vector<int>& out)
{
    if (!vertexCount_ || fbW <= 0 || fbH <= 0) return;
    ensurePickTarget(fbW, fbH);

    glBindFramebuffer(GL_FRAMEBUFFER, pickFbo_);
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        return;
    }
    glViewport(0, 0, fbW, fbH);
    glDisable(GL_BLEND);
    glDisable(GL_SCISSOR_TEST);
    glEnable(GL_DEPTH_TEST);
    glClearColor(0.f, 0.f, 0.f, 0.f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    pickShader_.use();
    glUniformMatrix4fv(uPickVP_, 1, GL_FALSE, vp);
    glBindVertexArray(vao_);
    glDrawArrays(GL_TRIANGLES, 0, vertexCount_);
    glBindVertexArray(0);

    const int x0 = std::max(cx - radius, 0), x1 = std::min(cx + radius, fbW - 1);
    const int y0 = std::max(cy - radius, 0), y1 = std::min(cy + radius, fbH - 1);
    if (x1 >= x0 && y1 >= y0) {
        const int w = x1 - x0 + 1, h = y1 - y0 + 1;
        std::vector<unsigned char> px(size_t(w) * h * 4);
        glPixelStorei(GL_PACK_ALIGNMENT, 1);
        glReadPixels(x0, y0, w, h, GL_RGBA, GL_UNSIGNED_BYTE, px.data());
        const size_t first = out.size();
        for (int y = 0; y < h; ++y)
            for (int x = 0; x < w; ++x) {
                const int dx = x0 + x - cx, dy = y0 + y - cy;
                if (dx * dx + dy * dy > radius * radius) continue;
                const unsigned char* c = &px[(size_t(y) * w + x) * 4];
                const int id = c[0] | (c[1] << 8) | (c[2] << 16);
                if (id > 0) out.push_back(id - 1);
            }
        std::sort(out.begin() + first, out.end());
        out.erase(std::unique(out.begin() + first, out.end()), out.end());
    }
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    GL_CHECK();
}

void MeshRenderer::destroy()
{
    if (lineVbo_) { glDeleteBuffers(1, &lineVbo_); lineVbo_ = 0; }
    if (lineVao_) { glDeleteVertexArrays(1, &lineVao_); lineVao_ = 0; }
    lineShader_.destroy();
    lineVertexCount_ = 0;
    if (pickFbo_) { glDeleteFramebuffers(1, &pickFbo_); pickFbo_ = 0; }
    if (pickColor_) { glDeleteTextures(1, &pickColor_); pickColor_ = 0; }
    if (pickDepth_) { glDeleteRenderbuffers(1, &pickDepth_); pickDepth_ = 0; }
    pickShader_.destroy();
    pickW_ = pickH_ = 0;
    if (vbo_) { glDeleteBuffers(1, &vbo_); vbo_ = 0; }
    if (vao_) { glDeleteVertexArrays(1, &vao_); vao_ = 0; }
    shader_.destroy();
    vertexCount_ = 0;
}
