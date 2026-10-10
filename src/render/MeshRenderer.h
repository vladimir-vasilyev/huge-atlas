#pragma once

#include "core/MeshTypes.h"
#include "render/GL.h"
#include "param/MeshSimplify.h"
#include "render/Shader.h"

#include <cstdint>
#include <string>
#include <vector>

// Indexed triangle mesh: one vertex per distinct (position, uv, offset), so large meshes share their
// vertices (the unrolled layout used before cost 44 bytes per corner). The color of a face lives in a
// texture buffer indexed by gl_PrimitiveID, so changing the colors (regions, stretch, ...) re-uploads
// 4 bytes per face and never rebuilds the geometry. Flat shading comes from screen space derivatives.
// The area weighted mean normal of every patch, per face (3 floats per face; a face without patch (label < 0) gets zero).
std::vector<float> patchNormals(const std::vector<int>& patchOfFace, const std::vector<double>& faceArea,
                                const std::vector<Vec3>& faceNormal);
std::vector<float> patchNormals(const std::vector<int>& patchOfFace, const FlatMesh& mesh);

class MeshRenderer {
public:
    bool init(std::string* errorLog);
    void destroy();

    // faceColors: rgb per face (indexed by face.idx()); an empty array means gray.
    // faceOffsets: xyz per face, added to its three vertices (used to explode patches); may be empty.
    // faceUV: u, v of the three corners of every face (6 floats, corners in face-loop order); may be
    // empty. A face whose u is negative counts as "no texture". With checkerCells > 0 in draw() the uv
    // is shown as a checkerboard, with useTexture as the texture set by setTexture().
    // The geometry is rebuilt only when the mesh was invalidated or the uv / offsets changed.
    void upload(const TriMesh& mesh, const std::vector<float>& faceColors = {},
                const std::vector<float>& faceOffsets = {}, const std::vector<float>& faceUV = {});

    // Geometry straight from flat arrays, without a TriMesh (meshes of tens of millions of faces).
    // cornerUV: 6 floats per face (corner k = vertex idx[3 f + k]), NaN = none; may be empty. Corners with the same (vertex, uv)
    // share a GPU vertex. faceRgba: 4 bytes per face; empty = gray. The GPU buffers hold 20 bytes per vertex and 12 per face.
    void uploadFlat(const FlatMesh& mesh, const std::vector<float>& cornerUV = {},
                    const std::vector<unsigned char>& faceRgba = {});
    // New face colors for the current geometry (4 bytes per face); empty = gray.
    void setFaceColorsRgba(const std::vector<unsigned char>& faceRgba);

    // The next upload() rebuilds the geometry (call after the mesh was replaced or edited).
    void invalidateGeometry() { ++geometryVersion_; }

    // RGBA8 image, row 0 = v = 0 (bottom). Mip maps are generated; anisotropic filtering if available.
    bool setTexture(int width, int height, const unsigned char* rgba);
    bool hasTexture() const { return texture_ != 0; }
    GLuint textureId() const { return texture_; }

    // wireframe is a second pass with polygon mode LINE (one pixel wide: wide lines do not exist in GL 4.1 core).
    // cullBackFaces: counter-clockwise faces only. On a closed or scanned shell that skips about half of the
    // triangles before rasterization; on an open surface seen from behind it shows holes.
    void draw(const float viewProj[16], const float view[16], bool wireframe, float checkerCells = 0.0f,
              bool useTexture = false, bool cullBackFaces = false) const;

    // "Presentation": the patches are drawn once as usual (draw(), with the patch colors), then a second time translucent and moved
    // along the normal of their patch (drawGhosts). The normal of the patch of every face (3 floats per face, e.g. from
    // patchNormals()) goes to a texture buffer, the displacement is applied by a geometry shader, so offset and opacity are live.
    void setPatchNormals(const std::vector<float>& normalsPerFace);
    bool hasPatchNormals() const { return patchNormalCount_ == faceCount_ && faceCount_ > 0; }
    // offset in model units (the models are normalized to a unit box), alpha 0..1
    void drawGhosts(const float viewProj[16], const float view[16], float offset, float alpha, bool cullBackFaces = false,
                    bool useTexture = false) const;

    bool hasMesh() const { return faceCount_ > 0; }
    size_t faceCount() const { return faceCount_; }
    size_t vertexCount() const { return vertexCount_; }

    // Line layer drawn over the mesh with a width in pixels (GL 4.1 core has no wide lines, so each
    // segment is expanded into a screen-space quad). Segment = 2 points + rgb, 9 floats.
    void uploadLines(const std::vector<float>& segments);
    void drawLines(const float viewProj[16], int fbWidth, int fbHeight, float widthPx) const;
    bool hasLines() const { return lineVertexCount_ > 0; }

    // Picking by an ID buffer: the mesh is rendered with the face index as color into an offscreen
    // target of the framebuffer size; the pixels in a disc are decoded.
    // (cx, cy) is the disc center in framebuffer pixels, origin at the bottom-left.
    // Appends the unique ids of the visible faces within `radius` pixels to `out`.
    void pickFaces(const float viewProj[16], int fbWidth, int fbHeight, int cx, int cy, int radius,
                   std::vector<int>& out);

private:
    void ensurePickTarget(int w, int h);
    void buildGeometry(const TriMesh& mesh, const std::vector<float>& offsets, const std::vector<float>& uv);
    void uploadColors(size_t faces, const std::vector<float>& colors);
    void uploadColorBytes(size_t faces, const std::vector<unsigned char>& rgba);
    void createGeometry(const std::vector<float>& verts, const uint32_t* indices, size_t indexCount);

    ShaderProgram shader_, wireShader_, pickShader_, lineShader_, ghostShader_;
    GLuint patchNormBuffer_ = 0, patchNormTexture_ = 0;
    size_t patchNormalCount_ = 0;
    GLint uGhostVP_ = -1, uGhostView_ = -1, uGhostOffset_ = -1, uGhostAlpha_ = -1, uGhostUseTex_ = -1, uGhostTex_ = -1, uGhostColors_ = -1, uGhostNormals_ = -1, uGhostUseColors_ = -1;
    GLuint lineVao_ = 0, lineVbo_ = 0;
    GLsizei lineVertexCount_ = 0;
    GLint uLineVP_ = -1, uLineViewport_ = -1, uLineWidth_ = -1;
    GLuint pickFbo_ = 0, pickColor_ = 0, pickDepth_ = 0;
    int pickW_ = 0, pickH_ = 0;
    GLint uPickVP_ = -1;

    GLuint vao_ = 0, vbo_ = 0, ibo_ = 0;
    GLuint colorBuffer_ = 0, colorTexture_ = 0;   // face colors: a buffer object seen as a texture buffer
    GLuint texture_ = 0;
    size_t faceCount_ = 0, vertexCount_ = 0;
    GLsizei indexCount_ = 0;
    bool useFaceColors_ = false;
    GLint uVP_ = -1, uView_ = -1, uChecker_ = -1, uUseTexture_ = -1, uUseFaceColors_ = -1, uFaceColors_ = -1, uTex_ = -1;
    GLint uWireVP_ = -1;

    // what the current geometry was built from
    unsigned geometryVersion_ = 0, builtVersion_ = ~0u;
    uint64_t builtUvHash_ = 0, builtOffsetHash_ = 0;
    size_t builtFaces_ = 0, builtVertices_ = 0;
};
