cmake_minimum_required(VERSION 3.20)

# Должно быть задано до project()
if(APPLE)
    set(CMAKE_OSX_DEPLOYMENT_TARGET "11.0" CACHE STRING "Minimum macOS version")
endif()

project(VSAParam LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT APPLE)
    message(FATAL_ERROR "Пока поддерживается только macOS (OpenGL 4.1 Core через OpenGL.framework)")
endif()

set(EXT_DIR ${CMAKE_SOURCE_DIR}/external)

# ---------------------------------------------------------------- GLFW
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS    OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_DOCS     OFF CACHE BOOL "" FORCE)
set(GLFW_INSTALL        OFF CACHE BOOL "" FORCE)
add_subdirectory(${EXT_DIR}/glfw EXCLUDE_FROM_ALL)

find_package(OpenGL REQUIRED)

# ---------------------------------------------------------------- Dear ImGui (ветка docking)
set(IMGUI_DIR ${EXT_DIR}/imgui)
add_library(imgui STATIC
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
    ${IMGUI_DIR}/imgui_demo.cpp
    ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
    ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
)
target_include_directories(imgui PUBLIC ${IMGUI_DIR} ${IMGUI_DIR}/backends)
target_compile_definitions(imgui PUBLIC GL_SILENCE_DEPRECATION GLFW_INCLUDE_NONE)
target_link_libraries(imgui PUBLIC glfw OpenGL::GL)

# ---------------------------------------------------------------- Приложение
set(APP_SOURCES
    src/app/main.cpp
    src/app/App.h
    src/app/App.cpp
    src/render/GL.h
    src/render/GL.cpp
    src/render/Shader.h
    src/render/Shader.cpp
    src/render/DebugTriangle.h
    src/render/DebugTriangle.cpp
)

add_executable(vsa_app ${APP_SOURCES})
target_include_directories(vsa_app PRIVATE ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(vsa_app PRIVATE imgui glfw OpenGL::GL)
target_compile_options(vsa_app PRIVATE -Wall -Wextra -Wno-unused-parameter)

# Удобства для Xcode: группы файлов как в дереве, схема с рабочей папкой = корень проекта
source_group(TREE ${CMAKE_SOURCE_DIR}/src PREFIX "src" FILES ${APP_SOURCES})
set_target_properties(vsa_app PROPERTIES
    XCODE_GENERATE_SCHEME TRUE
    XCODE_SCHEME_WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)
