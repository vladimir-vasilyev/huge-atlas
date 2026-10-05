#pragma once

#include "render/Shader.h"

// Вращающийся цветной треугольник: проверяет, что VAO/VBO и шейдеры 410 core работают.
// Уберём на этапе 1, когда появится отрисовка меша.
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
