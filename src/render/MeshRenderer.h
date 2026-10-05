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

private:
    ShaderProgram shader_;
    GLuint vao_ = 0, vbo_ = 0;
    GLsizei vertexCount_ = 0;
    GLint uVP_ = -1, uView_ = -1, uWire_ = -1, uWireWidth_ = -1;
};
