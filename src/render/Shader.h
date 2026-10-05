#pragma once

#include "render/GL.h"

#include <string>

// Владеющая обёртка над шейдерной программой GL. Некопируемая, перемещаемая.
// Исходники шейдеров храним строками в коде: не нужно возиться с путями к ресурсам в Xcode.
class ShaderProgram {
public:
    ShaderProgram() = default;
    ~ShaderProgram();

    ShaderProgram(const ShaderProgram&) = delete;
    ShaderProgram& operator=(const ShaderProgram&) = delete;
    ShaderProgram(ShaderProgram&& other) noexcept;
    ShaderProgram& operator=(ShaderProgram&& other) noexcept;

    // Компилирует и линкует программу. При ошибке возвращает false и пишет лог в errorLog.
    bool build(const char* vertexSrc, const char* fragmentSrc, std::string* errorLog = nullptr);

    void use() const;
    GLint uniform(const char* name) const;
    GLuint id() const { return program_; }
    bool valid() const { return program_ != 0; }

    void destroy();

private:
    GLuint program_ = 0;
};
