#pragma once

// Single entry point for OpenGL. No loader is needed on macOS:
// OpenGL.framework exports all 4.1 Core functions directly.
#ifdef __APPLE__
  #ifndef GL_SILENCE_DEPRECATION
    #define GL_SILENCE_DEPRECATION
  #endif
  #include <OpenGL/gl3.h>
#else
  #error "Only macOS is supported at the moment"
#endif

namespace gl {

// Prints all pending GL errors with the call site. Returns true if there were none.
bool checkError(const char* file, int line);

} // namespace gl

#ifndef NDEBUG
  #define GL_CHECK() ::gl::checkError(__FILE__, __LINE__)
#else
  #define GL_CHECK() ((void)0)
#endif
