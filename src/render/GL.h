#pragma once

// Единая точка подключения OpenGL. На macOS загрузчик не нужен:
// OpenGL.framework экспортирует все функции 4.1 Core напрямую.
#ifdef __APPLE__
  #ifndef GL_SILENCE_DEPRECATION
    #define GL_SILENCE_DEPRECATION
  #endif
  #include <OpenGL/gl3.h>
#else
  #error "Only macOS is supported at the moment"
#endif

namespace gl {

// Печатает все накопившиеся ошибки GL с местом вызова. Возвращает true, если ошибок не было.
bool checkError(const char* file, int line);

} // namespace gl

#ifndef NDEBUG
  #define GL_CHECK() ::gl::checkError(__FILE__, __LINE__)
#else
  #define GL_CHECK() ((void)0)
#endif
