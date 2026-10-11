#pragma once

#include "render/GL.h"
#include "render/Shader.h"

#include <string>

// The coordinate grid shown while no mesh is open: a floor grid in the XZ plane (cells of 0.1, a stronger line every 1.0)
// that fades with the distance, and the three axes through the origin (X red, Y green, Z blue).
class GridRenderer {
public:
    bool init(std::string* errorLog);
    void destroy();
    // darkBackground: light lines on a dark background or the other way round.
    void draw(const float viewProj[16], bool darkBackground) const;

private:
    ShaderProgram shader_;
    GLuint vao_ = 0, vbo_ = 0;
    GLsizei vertexCount_ = 0;
    GLint uVP_ = -1, uLine_ = -1;
};
