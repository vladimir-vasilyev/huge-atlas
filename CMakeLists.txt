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

# ---------------------------------------------------------------- meshoptimizer (simplifier only)
set(MO_DIR ${EXT_DIR}/meshoptimizer/src)
add_library(meshopt STATIC ${MO_DIR}/simplifier.cpp ${MO_DIR}/allocator.cpp ${MO_DIR}/indexgenerator.cpp)
target_include_directories(meshopt SYSTEM PUBLIC ${MO_DIR})

# ---------------------------------------------------------------- Core (no GL/ImGui)
add_library(vsa_core STATIC
    src/core/MeshTypes.h
    src/core/MeshPrep.h
    src/core/MeshPrep.cpp
    src/core/ObjLoader.h
    src/core/ObjLoader.cpp
    src/core/Metrics.h
    src/core/Metrics.cpp
    src/core/Proxy.h
    src/core/Proxy.cpp
    src/core/Partition.h
    src/core/Partition.cpp
    src/core/Operations.h
    src/core/Operations.cpp
    src/core/BoundaryMetrics.h
    src/core/BoundaryMetrics.cpp
    src/core/Smoothing.h
    src/core/Smoothing.cpp
    src/param/PatchTopology.h
    src/param/PatchTopology.cpp
    src/param/PatchExtractor.h
    src/param/PatchExtractor.cpp
    src/param/LinABF.h
    src/param/LinABF.cpp
    src/param/AnglesToUV.h
    src/param/AnglesToUV.cpp
    src/param/Predicates.h
    src/param/Predicates.cpp
    src/param/Injectivity.h
    src/param/Injectivity.cpp
    src/param/Distortion.h
    src/param/Distortion.cpp
    src/param/PatchFix.h
    src/param/PatchFix.cpp
    src/param/ParamPipeline.h
    src/param/ParamPipeline.cpp
    src/param/AtlasPacker.h
    src/param/AtlasPacker.cpp
    src/param/ThreadPool.h
    src/param/ThreadPool.cpp
    src/param/HorizonPacker.h
    src/param/HorizonPacker.cpp
    src/param/AtlasMask.h
    src/param/AtlasMask.cpp
    src/param/AtlasBake.h
    src/param/AtlasBake.cpp
    src/param/UVMetrics.h
    src/param/UVMetrics.cpp
    src/param/MeshWriters.h
    src/param/MeshWriters.cpp
    src/core/TextParse.h
    src/param/FlatLoader.h
    src/param/FlatLoader.cpp
    src/param/FlatAdjacency.h
    src/param/FlatAdjacency.cpp
    src/param/FlatBake.h
    src/param/FlatBake.cpp
    src/param/MipChain.h
    src/param/FlatPipeline.h
    src/param/FlatPipeline.cpp
    src/param/CoarsePipeline.h
    src/param/CoarsePipeline.cpp
    src/param/LiftParam.h
    src/param/LiftParam.cpp
    src/param/HierarchyMap.h
    src/param/HierarchyMap.cpp
    src/param/MeshSimplify.h
    src/param/MeshSimplify.cpp
    src/param/Refinement.h
    src/param/Refinement.cpp
)
target_include_directories(vsa_core PUBLIC ${CMAKE_SOURCE_DIR}/src)
find_package(ZLIB REQUIRED)
find_package(Threads REQUIRED)
target_link_libraries(vsa_core PUBLIC OpenMeshCoreStatic OpenMeshToolsStatic eigen meshopt ZLIB::ZLIB Threads::Threads)
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

add_executable(test_partition tests/test_partition.cpp)
target_link_libraries(test_partition PRIVATE vsa_core)
add_executable(test_operations tests/test_operations.cpp)
target_link_libraries(test_operations PRIVATE vsa_core)
add_test(NAME operations COMMAND test_operations)

add_test(NAME partition COMMAND test_partition ${CMAKE_SOURCE_DIR}/data/hemisphere.obj)

add_executable(test_smoothing tests/test_smoothing.cpp)
target_link_libraries(test_smoothing PRIVATE vsa_core)
add_test(NAME smoothing COMMAND test_smoothing ${CMAKE_SOURCE_DIR}/data/hemisphere.obj)

add_executable(test_parallel tests/test_parallel.cpp)
target_link_libraries(test_parallel PRIVATE vsa_core)
add_test(NAME parallel COMMAND test_parallel ${CMAKE_SOURCE_DIR}/data/bunny.ply)

add_executable(test_simplify tests/test_simplify.cpp)
target_link_libraries(test_simplify PRIVATE vsa_core)
add_test(NAME simplify COMMAND test_simplify ${CMAKE_SOURCE_DIR}/data/bunny.ply)

add_executable(test_hierarchy tests/test_hierarchy.cpp)
target_link_libraries(test_hierarchy PRIVATE vsa_core)
add_test(NAME hierarchy COMMAND test_hierarchy ${CMAKE_SOURCE_DIR}/data/bunny.ply)

add_executable(test_coarse tests/test_coarse.cpp)
target_link_libraries(test_coarse PRIVATE vsa_core)
add_test(NAME coarse COMMAND test_coarse ${CMAKE_SOURCE_DIR}/data/bunny.ply)

add_executable(test_flat tests/test_flat.cpp)
target_link_libraries(test_flat PRIVATE vsa_core)
add_test(NAME flat COMMAND test_flat ${CMAKE_SOURCE_DIR}/data/bunny.ply)

add_executable(test_flatload tests/test_flatload.cpp)
target_link_libraries(test_flatload PRIVATE vsa_core)
add_test(NAME flatload COMMAND test_flatload ${CMAKE_SOURCE_DIR}/data/bunny.ply)

add_executable(test_linabf tests/test_linabf.cpp)
target_link_libraries(test_linabf PRIVATE vsa_core)
add_test(NAME linabf COMMAND test_linabf)

add_executable(test_uv tests/test_uv.cpp)
target_link_libraries(test_uv PRIVATE vsa_core)
add_test(NAME uv COMMAND test_uv)

add_executable(test_injectivity tests/test_injectivity.cpp)
target_link_libraries(test_injectivity PRIVATE vsa_core)
add_test(NAME injectivity COMMAND test_injectivity)

add_executable(test_pipeline tests/test_pipeline.cpp)
target_link_libraries(test_pipeline PRIVATE vsa_core)
add_test(NAME pipeline COMMAND test_pipeline)

add_executable(test_atlas tests/test_atlas.cpp)
target_link_libraries(test_atlas PRIVATE vsa_core)
add_test(NAME atlas COMMAND test_atlas)

add_executable(test_refinement tests/test_refinement.cpp)
target_link_libraries(test_refinement PRIVATE vsa_core)
add_test(NAME refinement COMMAND test_refinement)
# the bunny run takes minutes in a Debug build: label it slow (ctest -LE slow skips it)
add_test(NAME refinement_bunny COMMAND test_refinement ${CMAKE_SOURCE_DIR}/data/bunny.ply)
set_tests_properties(refinement_bunny PROPERTIES LABELS slow)

add_executable(test_objloader tests/test_objloader.cpp)
target_link_libraries(test_objloader PRIVATE vsa_core)
add_test(NAME objloader COMMAND test_objloader ${CMAKE_SOURCE_DIR}/data/hemisphere.obj)

add_executable(test_save tests/test_save.cpp)
target_link_libraries(test_save PRIVATE vsa_core)
add_test(NAME save COMMAND test_save)

add_executable(test_horizon tests/test_horizon.cpp)
target_link_libraries(test_horizon PRIVATE vsa_core)
add_test(NAME horizon COMMAND test_horizon)

add_executable(test_patches tests/test_patches.cpp)
target_link_libraries(test_patches PRIVATE vsa_core)
add_test(NAME patches COMMAND test_patches ${CMAKE_SOURCE_DIR}/data/hemisphere.obj)

# Picking needs a GL context, so it links the render sources and GLFW
add_executable(test_picking tests/test_picking.cpp
    src/render/MeshRenderer.cpp src/render/Shader.cpp src/render/GL.cpp src/render/Camera.cpp)
target_include_directories(test_picking PRIVATE ${CMAKE_SOURCE_DIR}/src ${CMAKE_SOURCE_DIR}/tests)
target_link_libraries(test_picking PRIVATE vsa_core glfw OpenGL::GL)
target_compile_definitions(test_picking PRIVATE GL_SILENCE_DEPRECATION GLFW_INCLUDE_NONE)
add_test(NAME picking COMMAND test_picking)

# ---------------------------------------------------------------- Application
set(APP_SOURCES
    src/app/main.cpp
    src/app/App.h
    src/app/App.cpp
    src/app/UiTips.h
    src/app/UiTips.cpp
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
    src/render/SceneFramebuffer.h
    src/render/SceneFramebuffer.cpp
    src/render/ImageFile.h
    src/render/ImageFile.cpp
)

add_executable(vsa_app ${APP_SOURCES})
target_include_directories(vsa_app PRIVATE ${CMAKE_SOURCE_DIR}/src ${EXT_DIR}/ImGuiFileDialog)
target_link_libraries(vsa_app PRIVATE vsa_core ImGuiFileDialog imgui glfw OpenGL::GL)
target_compile_options(vsa_app PRIVATE -Wall -Wextra -Wno-unused-parameter)

# Xcode conveniences: file groups mirror the tree, scheme working directory = project root
source_group(TREE ${CMAKE_SOURCE_DIR}/src PREFIX "src" FILES ${APP_SOURCES})
set_target_properties(vsa_app PROPERTIES
    XCODE_GENERATE_SCHEME TRUE
    XCODE_SCHEME_WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)
