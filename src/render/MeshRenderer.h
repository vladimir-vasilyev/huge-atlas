#pragma once

#include "core/MeshTypes.h"
#include "render/GL.h"
#include "render/Shader.h"

#include <string>
#include <vector>

// Mesh with unrolled vertices (3 per face): flat shading, per-face color,
// wireframe via barycentric coordinates in the fragment shader.
class MeshRenderer {
public:
    bool init(std::string* errorLog);
    void destroy();

    // faceColors: rgb per face (indexed by face.idx()); an empty array means gray.
    void upload(const TriMesh& mesh, const std::vector<float>& faceColors = {});

    void draw(const float viewProj[16], const float view[16], bool wireframe, float wireWidth) const;

    bool hasMesh() const { return vertexCount_ > 0; }

    // Picking by an ID buffer: the mesh is rendered with the face index as color into an offscreen
    // target of the framebuffer size; the pixels in a disc are decoded.
    // (cx, cy) is the disc center in framebuffer pixels, origin at the bottom-left.
    // Appends the unique ids of the visible faces within `radius` pixels to `out`.
    void pickFaces(const float viewProj[16], int fbWidth, int fbHeight, int cx, int cy, int radius,
                   std::vector<int>& out);

private:
    void ensurePickTarget(int w, int h);

    ShaderProgram shader_, pickShader_;
    GLuint pickFbo_ = 0, pickColor_ = 0, pickDepth_ = 0;
    int pickW_ = 0, pickH_ = 0;
    GLint uPickVP_ = -1;
    GLuint vao_ = 0, vbo_ = 0;
    GLsizei vertexCount_ = 0;
    GLint uVP_ = -1, uView_ = -1, uWire_ = -1, uWireWidth_ = -1;
};
