#pragma once

#include "render/Shader.h"

// Rotating colored triangle: checks that VAO/VBO and 410 core shaders work.
// To be removed once mesh rendering is stable.
class DebugTriangle {
public:
    bool init(std::string* errorLog = nullptr);
    void draw(float angle, float aspect) const;
    void destroy();

private:
    ShaderProgram shader_;
    GLuint vao_ = 0;
    GLuint vbo_ = 0;
    GLint  uAngle_ = -1;
    GLint  uAspect_ = -1;
};
