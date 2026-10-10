#pragma once

#include "render/GL.h"

#include <string>

// Owning wrapper around a GL shader program. Non-copyable, movable.
// Shader sources are kept as strings in code: no resource paths to manage in Xcode.
class ShaderProgram {
public:
    ShaderProgram() = default;
    ~ShaderProgram();

    ShaderProgram(const ShaderProgram&) = delete;
    ShaderProgram& operator=(const ShaderProgram&) = delete;
    ShaderProgram(ShaderProgram&& other) noexcept;
    ShaderProgram& operator=(ShaderProgram&& other) noexcept;

    // Compiles and links the program. On failure returns false and writes the log to errorLog.
    // geometrySrc: optional geometry shader (GL 3.2+).
    bool build(const char* vertexSrc, const char* fragmentSrc, std::string* errorLog = nullptr, const char* geometrySrc = nullptr);

    void use() const;
    GLint uniform(const char* name) const;
    GLuint id() const { return program_; }
    bool valid() const { return program_ != 0; }

    void destroy();

private:
    GLuint program_ = 0;
};
