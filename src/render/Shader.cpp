#include "render/Shader.h"

#include <utility>
#include <vector>

namespace {

GLuint compileStage(GLenum type, const char* src, std::string& log)
{
    GLuint sh = glCreateShader(type);
    glShaderSource(sh, 1, &src, nullptr);
    glCompileShader(sh);

    GLint ok = GL_FALSE;
    glGetShaderiv(sh, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        GLint len = 0;
        glGetShaderiv(sh, GL_INFO_LOG_LENGTH, &len);
        std::vector<char> buf(static_cast<size_t>(len > 1 ? len : 1));
        glGetShaderInfoLog(sh, len, nullptr, buf.data());
        log += (type == GL_VERTEX_SHADER ? "[vertex] " : type == GL_GEOMETRY_SHADER ? "[geometry] " : "[fragment] ");
        log += buf.data();
        glDeleteShader(sh);
        return 0;
    }
    return sh;
}

} // namespace

ShaderProgram::~ShaderProgram() { destroy(); }

ShaderProgram::ShaderProgram(ShaderProgram&& other) noexcept
    : program_(std::exchange(other.program_, 0)) {}

ShaderProgram& ShaderProgram::operator=(ShaderProgram&& other) noexcept
{
    if (this != &other) {
        destroy();
        program_ = std::exchange(other.program_, 0);
    }
    return *this;
}

bool ShaderProgram::build(const char* vertexSrc, const char* fragmentSrc, std::string* errorLog, const char* geometrySrc)
{
    destroy();
    std::string log;

    GLuint vs = compileStage(GL_VERTEX_SHADER, vertexSrc, log);
    GLuint fs = compileStage(GL_FRAGMENT_SHADER, fragmentSrc, log);
    GLuint gs = geometrySrc ? compileStage(GL_GEOMETRY_SHADER, geometrySrc, log) : 0;
    if (!vs || !fs || (geometrySrc && !gs)) {
        if (vs) glDeleteShader(vs);
        if (fs) glDeleteShader(fs);
        if (gs) glDeleteShader(gs);
        if (errorLog) *errorLog = log;
        return false;
    }

    GLuint prog = glCreateProgram();
    glAttachShader(prog, vs);
    glAttachShader(prog, fs);
    if (gs) glAttachShader(prog, gs);
    glLinkProgram(prog);
    glDeleteShader(vs);  // flagged for deletion, freed together with the program
    glDeleteShader(fs);
    if (gs) glDeleteShader(gs);

    GLint ok = GL_FALSE;
    glGetProgramiv(prog, GL_LINK_STATUS, &ok);
    if (!ok) {
        GLint len = 0;
        glGetProgramiv(prog, GL_INFO_LOG_LENGTH, &len);
        std::vector<char> buf(static_cast<size_t>(len > 1 ? len : 1));
        glGetProgramInfoLog(prog, len, nullptr, buf.data());
        log += "[link] ";
        log += buf.data();
        glDeleteProgram(prog);
        if (errorLog) *errorLog = log;
        return false;
    }

    program_ = prog;
    GL_CHECK();
    return true;
}

void ShaderProgram::use() const { glUseProgram(program_); }

GLint ShaderProgram::uniform(const char* name) const
{
    return glGetUniformLocation(program_, name);
}

void ShaderProgram::destroy()
{
    if (program_) {
        glDeleteProgram(program_);
        program_ = 0;
    }
}
