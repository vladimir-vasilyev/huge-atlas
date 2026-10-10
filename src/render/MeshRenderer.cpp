#include "render/MeshRenderer.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <unordered_map>

namespace {

const char* kVS = R"(#version 410 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec2 aUV;
uniform mat4 uVP;
uniform mat4 uView;
out vec3 vViewPos;
out vec2 vUV;
void main() {
    vUV = aUV;
    vViewPos = (uView * vec4(aPos, 1.0)).xyz;
    gl_Position = uVP * vec4(aPos, 1.0);
}
)";

const char* kFS = R"(#version 410 core
in vec3 vViewPos;
in vec2 vUV;
uniform samplerBuffer uFaceColors;   // rgba8 per face
uniform float uUseFaceColors;        // 0: the default gray
uniform float uChecker;              // number of checker cells across the unit square; 0 = off
uniform float uUseTexture;           // 1: sample uTex where the uv is valid (u >= 0)
uniform sampler2D uTex;
out vec4 fragColor;
void main() {
    vec3 base = uUseFaceColors > 0.5 ? texelFetch(uFaceColors, gl_PrimitiveID).rgb : vec3(0.72, 0.74, 0.78);
    // flat normal from position derivatives in view space
    vec3 n = normalize(cross(dFdx(vViewPos), dFdy(vViewPos)));
    float diff = abs(n.z) * 0.75 + 0.25;       // headlight, two-sided
    vec3 col = base * diff;
    if (uUseTexture > 0.5) {
        vec3 tex = texture(uTex, vUV).rgb;      // uniform control flow: the derivatives stay defined
        if (vUV.x >= 0.0) col = tex;            // the scan texture is shown unlit
    }
    if (uChecker > 0.5) {
        vec2 c = floor(vUV * uChecker);
        float k = mod(c.x + c.y, 2.0);
        col = col * mix(0.45, 1.1, k);
    }
    fragColor = vec4(col, 1.0);
}
)";

const char* kGhostVS = R"(#version 410 core
layout(location = 0) in vec3 aPos;
out vec3 vPos;
void main() { vPos = aPos; gl_Position = vec4(aPos, 1.0); }
)";

const char* kGhostGS = R"(#version 410 core
layout(triangles) in;
layout(triangle_strip, max_vertices = 3) out;
in vec3 vPos[];
uniform mat4 uVP;
uniform mat4 uView;
uniform float uOffset;
uniform samplerBuffer uPatchNormals;   // rgba8 per face: the normal of the face's patch, (n * 0.5 + 0.5)
out vec3 vViewPos;
void main() {
    vec3 n = texelFetch(uPatchNormals, gl_PrimitiveIDIn).xyz * 2.0 - 1.0;
    for (int i = 0; i < 3; ++i) {
        vec3 p = vPos[i] + n * uOffset;
        vViewPos = (uView * vec4(p, 1.0)).xyz;
        gl_Position = uVP * vec4(p, 1.0);
        gl_PrimitiveID = gl_PrimitiveIDIn;
        EmitVertex();
    }
    EndPrimitive();
}
)";

const char* kGhostFS = R"(#version 410 core
in vec3 vViewPos;
uniform samplerBuffer uFaceColors;
uniform float uUseFaceColors;
uniform float uAlpha;
out vec4 fragColor;
void main() {
    vec3 base = uUseFaceColors > 0.5 ? texelFetch(uFaceColors, gl_PrimitiveID).rgb : vec3(0.72, 0.74, 0.78);
    vec3 n = normalize(cross(dFdx(vViewPos), dFdy(vViewPos)));
    float facing = abs(n.z);
    // glassy look: lighter and more opaque towards the silhouette
    vec3 col = base * (0.65 + 0.35 * facing) + vec3(0.18) * (1.0 - facing);
    fragColor = vec4(col, uAlpha * (0.45 + 0.55 * (1.0 - facing)));
}
)";

const char* kWireVS = R"(#version 410 core
layout(location = 0) in vec3 aPos;
uniform mat4 uVP;
void main() { gl_Position = uVP * vec4(aPos, 1.0); }
)";

const char* kWireFS = R"(#version 410 core
out vec4 fragColor;
void main() { fragColor = vec4(0.04, 0.04, 0.05, 1.0); }
)";

const char* kPickVS = R"(#version 410 core
layout(location = 0) in vec3 aPos;
uniform mat4 uVP;
void main() { gl_Position = uVP * vec4(aPos, 1.0); }
)";

const char* kPickFS = R"(#version 410 core
out vec4 fragColor;
void main() {
    int id = gl_PrimitiveID + 1;    // 0 means "no face"
    fragColor = vec4(float(id & 255) / 255.0, float((id >> 8) & 255) / 255.0,
                     float((id >> 16) & 255) / 255.0, 1.0);
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

namespace {

uint64_t hashFloats(const std::vector<float>& v)
{
    uint64_t h = 1469598103934665603ULL;
    const unsigned char* p = reinterpret_cast<const unsigned char*>(v.data());
    const size_t n = v.size() * sizeof(float);
    size_t i = 0;
    for (; i + 8 <= n; i += 8) {
        uint64_t w;
        std::memcpy(&w, p + i, 8);
        h = (h ^ w) * 1099511628211ULL;
        h ^= h >> 29;
    }
    for (; i < n; ++i) h = (h ^ p[i]) * 1099511628211ULL;
    return h ^ (uint64_t(n) << 1);
}

struct VertexKey {
    int   v;
    float f[5];   // u, w, offset xyz
    bool operator==(const VertexKey& o) const { return v == o.v && std::memcmp(f, o.f, sizeof(f)) == 0; }
};

struct VertexKeyHash {
    size_t operator()(const VertexKey& k) const
    {
        uint64_t h = uint64_t(uint32_t(k.v)) * 0x9E3779B97F4A7C15ULL;
        for (float x : k.f) {
            uint32_t b;
            std::memcpy(&b, &x, 4);
            h = (h ^ b) * 0xff51afd7ed558ccdULL;
            h ^= h >> 32;
        }
        return size_t(h);
    }
};

} // namespace

bool MeshRenderer::init(std::string* errorLog)
{
    if (!shader_.build(kVS, kFS, errorLog)) return false;
    uVP_            = shader_.uniform("uVP");
    uView_          = shader_.uniform("uView");
    uChecker_       = shader_.uniform("uChecker");
    uUseTexture_    = shader_.uniform("uUseTexture");
    uUseFaceColors_ = shader_.uniform("uUseFaceColors");
    uFaceColors_    = shader_.uniform("uFaceColors");
    uTex_           = shader_.uniform("uTex");
    if (!ghostShader_.build(kGhostVS, kGhostFS, errorLog, kGhostGS)) return false;
    uGhostVP_        = ghostShader_.uniform("uVP");
    uGhostView_      = ghostShader_.uniform("uView");
    uGhostOffset_    = ghostShader_.uniform("uOffset");
    uGhostAlpha_     = ghostShader_.uniform("uAlpha");
    uGhostColors_    = ghostShader_.uniform("uFaceColors");
    uGhostNormals_   = ghostShader_.uniform("uPatchNormals");
    uGhostUseColors_ = ghostShader_.uniform("uUseFaceColors");
    glGenBuffers(1, &patchNormBuffer_);
    glGenTextures(1, &patchNormTexture_);
    if (!wireShader_.build(kWireVS, kWireFS, errorLog)) return false;
    uWireVP_ = wireShader_.uniform("uVP");
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
    glGenBuffers(1, &ibo_);
    glGenBuffers(1, &colorBuffer_);
    glGenTextures(1, &colorTexture_);
    {   // a valid texture buffer even before any colors exist (the driver warns about an empty one)
        const unsigned char white[4] = {255, 255, 255, 255};
        glBindBuffer(GL_TEXTURE_BUFFER, colorBuffer_);
        glBufferData(GL_TEXTURE_BUFFER, 4, white, GL_DYNAMIC_DRAW);
        glBindTexture(GL_TEXTURE_BUFFER, colorTexture_);
        glTexBuffer(GL_TEXTURE_BUFFER, GL_RGBA8, colorBuffer_);
        glBindTexture(GL_TEXTURE_BUFFER, 0);
        glBindBuffer(GL_TEXTURE_BUFFER, 0);
    }
    return true;
}

void MeshRenderer::buildGeometry(const TriMesh& mesh, const std::vector<float>& offsets, const std::vector<float>& uv)
{
    const size_t nF = mesh.n_faces(), nV = mesh.n_vertices();
    std::vector<uint32_t> indices(nF * 3);
    std::vector<float> verts;   // x y z u v

    if (uv.empty() && offsets.empty()) {
        // the vertices of the mesh as they are
        verts.assign(nV * 5, 0.0f);
        for (auto v : mesh.vertices()) {
            const auto p = mesh.point(v);
            float* d = &verts[size_t(v.idx()) * 5];
            d[0] = float(p[0]); d[1] = float(p[1]); d[2] = float(p[2]);
        }
        for (auto f : mesh.faces()) {
            int k = 0;
            for (auto v : mesh.fv_range(f)) indices[size_t(f.idx()) * 3 + k++] = uint32_t(v.idx());
        }
    } else {
        // weld the corners with equal (vertex, uv, offset)
        std::unordered_map<VertexKey, uint32_t, VertexKeyHash> table;
        table.reserve(nV + nV / 2);
        verts.reserve(nV * 5 + nV);
        for (auto f : mesh.faces()) {
            float off[3] = {0.f, 0.f, 0.f};
            if (offsets.size() >= size_t(f.idx() + 1) * 3)
                for (int k = 0; k < 3; ++k) off[k] = offsets[size_t(f.idx()) * 3 + k];
            int c = 0;
            for (auto v : mesh.fv_range(f)) {
                VertexKey key;
                key.v = v.idx();
                key.f[0] = key.f[1] = 0.0f;
                if (uv.size() >= size_t(f.idx() + 1) * 6) { key.f[0] = uv[size_t(f.idx()) * 6 + c * 2]; key.f[1] = uv[size_t(f.idx()) * 6 + c * 2 + 1]; }
                key.f[2] = off[0]; key.f[3] = off[1]; key.f[4] = off[2];
                auto it = table.find(key);
                if (it == table.end()) {
                    it = table.emplace(key, uint32_t(verts.size() / 5)).first;
                    const auto p = mesh.point(v);
                    verts.insert(verts.end(), {float(p[0]) + off[0], float(p[1]) + off[1], float(p[2]) + off[2], key.f[0], key.f[1]});
                }
                indices[size_t(f.idx()) * 3 + c] = it->second;
                ++c;
            }
        }
    }
    createGeometry(verts, indices.data(), indices.size());
}

void MeshRenderer::createGeometry(const std::vector<float>& verts, const uint32_t* indices, size_t indexCount)
{
    vertexCount_ = verts.size() / 5;
    indexCount_ = GLsizei(indexCount);

    glBindVertexArray(vao_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(verts.size() * sizeof(float)), verts.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), reinterpret_cast<void*>(0));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), reinterpret_cast<void*>(3 * sizeof(float)));
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo_);   // stays bound to the VAO
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, GLsizeiptr(indexCount * sizeof(uint32_t)), indices, GL_STATIC_DRAW);
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    GL_CHECK();
}

void MeshRenderer::uploadColors(size_t faces, const std::vector<float>& colors)
{
    useFaceColors_ = colors.size() >= faces * 3 && faces > 0;
    if (!useFaceColors_) return;
    std::vector<unsigned char> bytes(faces * 4, 255);
    for (size_t i = 0; i < faces; ++i)
        for (int k = 0; k < 3; ++k) {
            const float c = std::max(0.0f, std::min(1.0f, colors[i * 3 + k]));
            bytes[i * 4 + k] = (unsigned char)(c * 255.0f + 0.5f);
        }
    glBindBuffer(GL_TEXTURE_BUFFER, colorBuffer_);
    glBufferData(GL_TEXTURE_BUFFER, GLsizeiptr(bytes.size()), bytes.data(), GL_DYNAMIC_DRAW);
    glBindTexture(GL_TEXTURE_BUFFER, colorTexture_);
    glTexBuffer(GL_TEXTURE_BUFFER, GL_RGBA8, colorBuffer_);
    glBindTexture(GL_TEXTURE_BUFFER, 0);
    glBindBuffer(GL_TEXTURE_BUFFER, 0);
    GL_CHECK();
}

void MeshRenderer::uploadColorBytes(size_t faces, const std::vector<unsigned char>& rgba)
{
    useFaceColors_ = rgba.size() >= faces * 4 && faces > 0;
    if (!useFaceColors_) return;
    glBindBuffer(GL_TEXTURE_BUFFER, colorBuffer_);
    glBufferData(GL_TEXTURE_BUFFER, GLsizeiptr(faces * 4), rgba.data(), GL_DYNAMIC_DRAW);
    glBindTexture(GL_TEXTURE_BUFFER, colorTexture_);
    glTexBuffer(GL_TEXTURE_BUFFER, GL_RGBA8, colorBuffer_);
    glBindTexture(GL_TEXTURE_BUFFER, 0);
    glBindBuffer(GL_TEXTURE_BUFFER, 0);
    GL_CHECK();
}

void MeshRenderer::setFaceColorsRgba(const std::vector<unsigned char>& faceRgba) { uploadColorBytes(faceCount_, faceRgba); }

void MeshRenderer::uploadFlat(const FlatMesh& mesh, const std::vector<float>& cornerUV, const std::vector<unsigned char>& faceRgba)
{
    const size_t nV = mesh.vertexCount(), nF = mesh.faceCount();
    const bool haveUV = cornerUV.size() == nF * 6;
    std::vector<float> verts;
    std::vector<uint32_t> welded;   // only with uv: the indices of the welded vertices
    if (!haveUV) {
        verts.assign(nV * 5, 0.0f);
        for (size_t v = 0; v < nV; ++v) {
            verts[5 * v] = mesh.pos[3 * v];
            verts[5 * v + 1] = mesh.pos[3 * v + 1];
            verts[5 * v + 2] = mesh.pos[3 * v + 2];
        }
    } else {
        // corners grouped by vertex; one GPU vertex per distinct uv of a vertex
        std::vector<uint32_t> start(nV + 1, 0);
        for (uint32_t v : mesh.idx) ++start[v + 1];
        for (size_t v = 0; v < nV; ++v) start[v + 1] += start[v];
        std::vector<uint32_t> corners(mesh.idx.size());
        {
            std::vector<uint32_t> fill(start.begin(), start.end() - 1);
            for (size_t c = 0; c < mesh.idx.size(); ++c) corners[fill[mesh.idx[c]]++] = uint32_t(c);
        }
        welded.assign(mesh.idx.size(), 0);
        verts.reserve((nV + nV / 8) * 5);
        for (size_t v = 0; v < nV; ++v) {
            const size_t first = verts.size() / 5;
            for (uint32_t i = start[v]; i < start[v + 1]; ++i) {
                const uint32_t c = corners[i];
                float u = cornerUV[2 * size_t(c)], w = cornerUV[2 * size_t(c) + 1];
                if (!(u == u) || !(w == w)) { u = 0.f; w = 0.f; }
                size_t at = first;
                while (at < verts.size() / 5 && !(verts[5 * at + 3] == u && verts[5 * at + 4] == w)) ++at;
                if (at == verts.size() / 5) verts.insert(verts.end(), {mesh.pos[3 * v], mesh.pos[3 * v + 1], mesh.pos[3 * v + 2], u, w});
                welded[c] = uint32_t(at);
            }
            if (start[v] == start[v + 1]) verts.insert(verts.end(), {mesh.pos[3 * v], mesh.pos[3 * v + 1], mesh.pos[3 * v + 2], 0.f, 0.f});
        }
    }
    createGeometry(verts, haveUV ? welded.data() : mesh.idx.data(), mesh.idx.size());
    faceCount_ = nF;
    builtVersion_ = ~0u;   // the next upload(TriMesh) rebuilds
    builtFaces_ = 0;
    uploadColorBytes(nF, faceRgba);
}

void MeshRenderer::upload(const TriMesh& mesh, const std::vector<float>& faceColors,
                          const std::vector<float>& faceOffsets, const std::vector<float>& faceUV)
{
    const uint64_t uvHash = faceUV.empty() ? 0 : hashFloats(faceUV);
    const uint64_t offHash = faceOffsets.empty() ? 0 : hashFloats(faceOffsets);
    const bool rebuild = builtVersion_ != geometryVersion_ || builtFaces_ != mesh.n_faces() ||
                         builtVertices_ != mesh.n_vertices() || builtUvHash_ != uvHash || builtOffsetHash_ != offHash ||
                         faceUV.empty() != (builtUvHash_ == 0) || faceOffsets.empty() != (builtOffsetHash_ == 0);
    if (rebuild) {
        buildGeometry(mesh, faceOffsets, faceUV);
        builtVersion_ = geometryVersion_;
        builtFaces_ = mesh.n_faces();
        builtVertices_ = mesh.n_vertices();
        builtUvHash_ = uvHash;
        builtOffsetHash_ = offHash;
    }
    faceCount_ = mesh.n_faces();
    uploadColors(faceCount_, faceColors);
}

bool MeshRenderer::setTexture(int w, int h, const unsigned char* rgba)
{
    if (w <= 0 || h <= 0 || !rgba) return false;
    if (!texture_) glGenTextures(1, &texture_);
    glBindTexture(GL_TEXTURE_2D, texture_);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, rgba);
    glGenerateMipmap(GL_TEXTURE_2D);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    // anisotropic filtering (GL_EXT_texture_filter_anisotropic), when the driver has it
    while (glGetError() != GL_NO_ERROR) {}
    GLfloat maxAniso = 0.0f;
    glGetFloatv(0x84FF, &maxAniso);   // GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT
    if (glGetError() == GL_NO_ERROR && maxAniso > 1.0f)
        glTexParameterf(GL_TEXTURE_2D, 0x84FE, std::min(8.0f, maxAniso));   // GL_TEXTURE_MAX_ANISOTROPY_EXT
    glBindTexture(GL_TEXTURE_2D, 0);
    GL_CHECK();
    return true;
}

std::vector<float> patchNormals(const std::vector<int>& label, const std::vector<double>& area, const std::vector<Vec3>& normal)
{
    int maxLabel = -1;
    for (int l : label) maxLabel = std::max(maxLabel, l);
    std::vector<double> sum(3 * size_t(maxLabel + 1), 0.0);
    for (size_t f = 0; f < label.size(); ++f)
        if (label[f] >= 0)
            for (int k = 0; k < 3; ++k) sum[3 * size_t(label[f]) + k] += area[f] * normal[f][k];
    for (size_t p = 0; p * 3 < sum.size(); ++p) {
        const double len = std::sqrt(sum[3 * p] * sum[3 * p] + sum[3 * p + 1] * sum[3 * p + 1] + sum[3 * p + 2] * sum[3 * p + 2]);
        for (int k = 0; k < 3; ++k) sum[3 * p + k] = len > 1e-300 ? sum[3 * p + k] / len : 0.0;
    }
    std::vector<float> out(3 * label.size(), 0.0f);
    for (size_t f = 0; f < label.size(); ++f)
        if (label[f] >= 0)
            for (int k = 0; k < 3; ++k) out[3 * f + k] = float(sum[3 * size_t(label[f]) + k]);
    return out;
}

std::vector<float> patchNormals(const std::vector<int>& label, const FlatMesh& m)
{
    int maxLabel = -1;
    for (int l : label) maxLabel = std::max(maxLabel, l);
    std::vector<double> sum(3 * size_t(maxLabel + 1), 0.0);
    auto P = [&](size_t f, int k) { return &m.pos[3 * size_t(m.idx[3 * f + k])]; };
    for (size_t f = 0; f < label.size(); ++f) {
        if (label[f] < 0) continue;
        const float *a = P(f, 0), *b = P(f, 1), *c = P(f, 2);
        const double e1[3] = {double(b[0]) - a[0], double(b[1]) - a[1], double(b[2]) - a[2]};
        const double e2[3] = {double(c[0]) - a[0], double(c[1]) - a[1], double(c[2]) - a[2]};
        sum[3 * size_t(label[f])]     += e1[1] * e2[2] - e1[2] * e2[1];   // the cross product is the normal times twice the area
        sum[3 * size_t(label[f]) + 1] += e1[2] * e2[0] - e1[0] * e2[2];
        sum[3 * size_t(label[f]) + 2] += e1[0] * e2[1] - e1[1] * e2[0];
    }
    for (size_t p = 0; p * 3 < sum.size(); ++p) {
        const double len = std::sqrt(sum[3 * p] * sum[3 * p] + sum[3 * p + 1] * sum[3 * p + 1] + sum[3 * p + 2] * sum[3 * p + 2]);
        for (int k = 0; k < 3; ++k) sum[3 * p + k] = len > 1e-300 ? sum[3 * p + k] / len : 0.0;
    }
    std::vector<float> out(3 * label.size(), 0.0f);
    for (size_t f = 0; f < label.size(); ++f)
        if (label[f] >= 0)
            for (int k = 0; k < 3; ++k) out[3 * f + k] = float(sum[3 * size_t(label[f]) + k]);
    return out;
}

void MeshRenderer::setPatchNormals(const std::vector<float>& n)
{
    const size_t faces = n.size() / 3;
    std::vector<unsigned char> bytes(faces * 4);
    for (size_t f = 0; f < faces; ++f) {
        for (int k = 0; k < 3; ++k) bytes[4 * f + k] = (unsigned char)std::lround(std::max(0.0f, std::min(1.0f, n[3 * f + k] * 0.5f + 0.5f)) * 255.0f);
        bytes[4 * f + 3] = 255;
    }
    if (bytes.empty()) bytes.assign(4, 0);
    glBindBuffer(GL_TEXTURE_BUFFER, patchNormBuffer_);
    glBufferData(GL_TEXTURE_BUFFER, GLsizeiptr(bytes.size()), bytes.data(), GL_STATIC_DRAW);
    glBindTexture(GL_TEXTURE_BUFFER, patchNormTexture_);
    glTexBuffer(GL_TEXTURE_BUFFER, GL_RGBA8, patchNormBuffer_);
    glBindTexture(GL_TEXTURE_BUFFER, 0);
    glBindBuffer(GL_TEXTURE_BUFFER, 0);
    patchNormalCount_ = faces;
    GL_CHECK();
}

void MeshRenderer::drawGhosts(const float vp[16], const float view[16], float offset, float alpha, bool cullBackFaces) const
{
    if (!hasPatchNormals() || !indexCount_) return;
    ghostShader_.use();
    glUniformMatrix4fv(uGhostVP_, 1, GL_FALSE, vp);
    glUniformMatrix4fv(uGhostView_, 1, GL_FALSE, view);
    glUniform1f(uGhostOffset_, offset);
    glUniform1f(uGhostAlpha_, alpha);
    glUniform1f(uGhostUseColors_, useFaceColors_ ? 1.0f : 0.0f);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_BUFFER, colorTexture_);
    glUniform1i(uGhostColors_, 0);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_BUFFER, patchNormTexture_);
    glUniform1i(uGhostNormals_, 1);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);   // the translucent shells do not hide each other or what is behind them
    if (cullBackFaces) { glEnable(GL_CULL_FACE); glCullFace(GL_BACK); }
    glBindVertexArray(vao_);
    glDrawElements(GL_TRIANGLES, indexCount_, GL_UNSIGNED_INT, nullptr);
    glBindVertexArray(0);
    glDisable(GL_CULL_FACE);
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_BUFFER, 0);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_BUFFER, 0);
    GL_CHECK();
}

void MeshRenderer::draw(const float vp[16], const float view[16], bool wireframe, float checkerCells,
                        bool useTexture, bool cullBackFaces) const
{
    if (!faceCount_ || !indexCount_) return;
    shader_.use();
    glUniformMatrix4fv(uVP_, 1, GL_FALSE, vp);
    glUniformMatrix4fv(uView_, 1, GL_FALSE, view);
    glUniform1f(uChecker_, checkerCells);
    glUniform1f(uUseFaceColors_, useFaceColors_ ? 1.0f : 0.0f);
    glUniform1f(uUseTexture_, (useTexture && texture_) ? 1.0f : 0.0f);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_BUFFER, colorTexture_);
    glUniform1i(uFaceColors_, 0);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, texture_);
    glUniform1i(uTex_, 1);

    if (cullBackFaces) { glEnable(GL_CULL_FACE); glCullFace(GL_BACK); }
    glBindVertexArray(vao_);
    if (wireframe) {
        glEnable(GL_POLYGON_OFFSET_FILL);   // push the fill back so the edges win the depth test
        glPolygonOffset(1.0f, 1.0f);
    }
    glDrawElements(GL_TRIANGLES, indexCount_, GL_UNSIGNED_INT, nullptr);
    if (wireframe) {
        glDisable(GL_POLYGON_OFFSET_FILL);
        wireShader_.use();
        glUniformMatrix4fv(uWireVP_, 1, GL_FALSE, vp);
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glDrawElements(GL_TRIANGLES, indexCount_, GL_UNSIGNED_INT, nullptr);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }
    glDisable(GL_CULL_FACE);
    glBindVertexArray(0);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, 0);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_BUFFER, 0);
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
    if (!faceCount_ || fbW <= 0 || fbH <= 0) return;
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
    glDrawElements(GL_TRIANGLES, indexCount_, GL_UNSIGNED_INT, nullptr);
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
    if (ibo_) { glDeleteBuffers(1, &ibo_); ibo_ = 0; }
    if (vao_) { glDeleteVertexArrays(1, &vao_); vao_ = 0; }
    if (colorBuffer_) { glDeleteBuffers(1, &colorBuffer_); colorBuffer_ = 0; }
    if (colorTexture_) { glDeleteTextures(1, &colorTexture_); colorTexture_ = 0; }
    if (texture_) { glDeleteTextures(1, &texture_); texture_ = 0; }
    if (patchNormBuffer_) { glDeleteBuffers(1, &patchNormBuffer_); patchNormBuffer_ = 0; }
    if (patchNormTexture_) { glDeleteTextures(1, &patchNormTexture_); patchNormTexture_ = 0; }
    patchNormalCount_ = 0;
    ghostShader_.destroy();
    shader_.destroy();
    wireShader_.destroy();
    faceCount_ = vertexCount_ = 0;
    indexCount_ = 0;
}
