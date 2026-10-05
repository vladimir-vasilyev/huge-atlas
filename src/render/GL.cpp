#include "render/GL.h"

#include <cstdio>

namespace gl {

static const char* errorName(GLenum e)
{
    switch (e) {
        case GL_INVALID_ENUM:                  return "GL_INVALID_ENUM";
        case GL_INVALID_VALUE:                 return "GL_INVALID_VALUE";
        case GL_INVALID_OPERATION:             return "GL_INVALID_OPERATION";
        case GL_INVALID_FRAMEBUFFER_OPERATION: return "GL_INVALID_FRAMEBUFFER_OPERATION";
        case GL_OUT_OF_MEMORY:                 return "GL_OUT_OF_MEMORY";
        default:                               return "unknown GL error";
    }
}

bool checkError(const char* file, int line)
{
    bool ok = true;
    for (GLenum e = glGetError(); e != GL_NO_ERROR; e = glGetError()) {
        std::fprintf(stderr, "[GL] %s (0x%04X) at %s:%d\n", errorName(e), e, file, line);
        ok = false;
    }
    return ok;
}

} // namespace gl
