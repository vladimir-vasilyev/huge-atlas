#pragma once

#include "core/MeshTypes.h"
#include "param/PatchExtractor.h"

#include <Eigen/Dense>

#include <string>
#include <vector>

// PLY writers. (The OBJ writer for the atlas is exportAtlasOBJ in AtlasPacker.h.)

// The patches of an atlas as one mesh: the vertices of every patch (so seams are duplicated, each with its
// own texture coordinates), faces in patch order. Vertex properties x y z s t (float) and the comment
// `TextureFile <name>` when textureFile is not empty. binary: little endian, ASCII otherwise.
// atlasUV[i] belongs to patches[i]; a null entry skips the patch. Returns the number of written faces, -1 on I/O error.
int writePlyAtlas(const std::string& path, const std::vector<const Patch*>& patches,
                  const std::vector<const std::vector<Eigen::Vector2d>*>& atlasUV, const std::string& textureFile,
                  bool binary, const std::string& comment = "");

// The geometry alone (no texture coordinates).
bool writePlyMesh(const std::string& path, const TriMesh& mesh, bool binary);
bool writeObjMesh(const std::string& path, const TriMesh& mesh);
