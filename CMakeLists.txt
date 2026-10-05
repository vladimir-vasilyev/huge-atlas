cmake_minimum_required(VERSION 3.20)

# Must be set before project()
if(APPLE)
    set(CMAKE_OSX_DEPLOYMENT_TARGET "11.0" CACHE STRING "Minimum macOS version")
endif()

project(VSAParam LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT APPLE)
    message(FATAL_ERROR "Only macOS is supported for now (OpenGL 4.1 Core via OpenGL.framework)")
endif()

set(EXT_DIR ${CMAKE_SOURCE_DIR}/external)

# ---------------------------------------------------------------- GLFW
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS    OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_DOCS     OFF CACHE BOOL "" FORCE)
set(GLFW_INSTALL        OFF CACHE BOOL "" FORCE)
add_subdirectory(${EXT_DIR}/glfw EXCLUDE_FROM_ALL)

find_package(OpenGL REQUIRED)

# ---------------------------------------------------------------- Dear ImGui (docking branch)
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

# ---------------------------------------------------------------- ImGuiFileDialog
add_library(ImGuiFileDialog STATIC ${EXT_DIR}/ImGuiFileDialog/ImGuiFileDialog.cpp)
target_include_directories(ImGuiFileDialog PUBLIC ${EXT_DIR}/ImGuiFileDialog)
target_link_libraries(ImGuiFileDialog PUBLIC imgui)

# ---------------------------------------------------------------- Eigen (header-only)
add_library(eigen INTERFACE)
target_include_directories(eigen SYSTEM INTERFACE ${EXT_DIR}/eigen)

# ---------------------------------------------------------------- OpenMesh (static)
set(OPENMESH_BUILD_SHARED OFF CACHE BOOL "" FORCE)
set(BUILD_APPS OFF CACHE BOOL "" FORCE)
set(OPENMESH_BUILD_UNIT_TESTS OFF CACHE BOOL "" FORCE)
set(OPENMESH_DOCS OFF CACHE BOOL "" FORCE)
set(DISABLE_QMAKE_BUILD ON CACHE BOOL "" FORCE)
set(OPENMESH_BUILD_PYTHON_BINDINGS OFF CACHE BOOL "" FORCE)
add_subdirectory(${EXT_DIR}/OpenMesh EXCLUDE_FROM_ALL)

# ---------------------------------------------------------------- Core (no GL/ImGui)
add_library(vsa_core STATIC
    src/core/MeshTypes.h
    src/core/MeshPrep.h
    src/core/MeshPrep.cpp
    src/core/Metrics.h
    src/core/Metrics.cpp
    src/core/Proxy.h
    src/core/Proxy.cpp
)
target_include_directories(vsa_core PUBLIC ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(vsa_core PUBLIC OpenMeshCoreStatic OpenMeshToolsStatic eigen)
# OM_STATIC_BUILD: explicit registration of OpenMesh readers (otherwise the linker drops them from the static library)
target_compile_definitions(vsa_core PUBLIC _USE_MATH_DEFINES OM_STATIC_BUILD)
target_compile_options(vsa_core PRIVATE -Wall -Wextra -Wno-unused-parameter)

# ---------------------------------------------------------------- Tests
enable_testing()
add_executable(test_meshprep tests/test_meshprep.cpp)
target_link_libraries(test_meshprep PRIVATE vsa_core)
add_test(NAME meshprep COMMAND test_meshprep ${CMAKE_SOURCE_DIR}/data/cube.obj)

add_executable(test_metrics tests/test_metrics.cpp)
target_link_libraries(test_metrics PRIVATE vsa_core)
add_test(NAME metrics COMMAND test_metrics ${CMAKE_SOURCE_DIR}/data/cube.obj)

# ---------------------------------------------------------------- Application
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
    src/render/Camera.h
    src/render/Camera.cpp
    src/render/MeshRenderer.h
    src/render/MeshRenderer.cpp
)

add_executable(vsa_app ${APP_SOURCES})
target_include_directories(vsa_app PRIVATE ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(vsa_app PRIVATE vsa_core ImGuiFileDialog imgui glfw OpenGL::GL)
target_compile_options(vsa_app PRIVATE -Wall -Wextra -Wno-unused-parameter)

# Xcode conveniences: file groups mirror the tree, scheme working directory = project root
source_group(TREE ${CMAKE_SOURCE_DIR}/src PREFIX "src" FILES ${APP_SOURCES})
set_target_properties(vsa_app PROPERTIES
    XCODE_GENERATE_SCHEME TRUE
    XCODE_SCHEME_WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)
