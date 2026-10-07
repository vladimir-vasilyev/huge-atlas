#include "core/ObjLoader.h"

#include "core/TextParse.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <unordered_map>

namespace {

using namespace textparse;

std::string dirOf(const std::string& path)
{
    const size_t s = path.find_last_of('/');
    return s == std::string::npos ? std::string(".") : path.substr(0, s);
}

bool readFile(const std::string& path, std::string& out)
{
    FILE* f = std::fopen(path.c_str(), "rb");
    if (!f) return false;
    std::fseek(f, 0, SEEK_END);
    const long size = std::ftell(f);
    std::fseek(f, 0, SEEK_SET);
    out.resize(size > 0 ? size_t(size) : 0);
    const size_t got = out.empty() ? 0 : std::fread(&out[0], 1, out.size(), f);
    std::fclose(f);
    return got == out.size();
}

} // namespace

std::vector<ObjMaterial> parseMtl(const std::string& text)
{
    std::vector<ObjMaterial> mats;
    const char* p = text.data();
    const char* end = p + text.size();
    while (p < end) {
        const char* lineEnd = static_cast<const char*>(std::memchr(p, '\n', size_t(end - p)));
        if (!lineEnd) lineEnd = end;
        const char* q = skipSpaces(p, lineEnd);
        auto isKey = [&](const char* key) {
            const size_t n = std::strlen(key);
            return size_t(lineEnd - q) > n && std::strncmp(q, key, n) == 0 && isSpace(q[n]);
        };
        auto rest = [&](const char* key) {
            const char* r = skipSpaces(q + std::strlen(key), lineEnd);
            const char* e = lineEnd;
            while (e > r && (isSpace(e[-1]))) --e;
            return std::string(r, e);
        };
        if (isKey("newmtl")) {
            ObjMaterial m;
            m.name = rest("newmtl");
            mats.push_back(m);
        } else if (isKey("map_Kd") && !mats.empty()) {
            std::string name = rest("map_Kd");
            // options such as "-s 1 1 1" precede the file name: take the last token that is not a number or an option
            const size_t cut = name.find_last_of(' ');
            if (cut != std::string::npos && name.rfind('-', 0) == 0) name = name.substr(cut + 1);
            mats.back().diffuseMap = name;
        }
        p = lineEnd + 1;
    }
    return mats;
}

bool loadObj(const std::string& path, TriMesh& mesh, ObjExtras* extras, std::string* error)
{
    auto fail = [&](const std::string& msg) { if (error) *error = msg; mesh.clean(); return false; };
    mesh.clean();
    std::string text;
    if (!readFile(path, text)) return fail("failed to read file: " + path);

    ObjExtras local;
    ObjExtras& ex = extras ? *extras : local;
    ex = ObjExtras();
    ex.directory = dirOf(path);

    std::vector<TriMesh::VertexHandle> vertices;
    std::vector<float> uvs;                       // 2 per vt
    std::unordered_map<std::string, int> materialIndex;
    std::string mtllib;
    int currentMaterial = -1;

    const char* p = text.data();
    const char* end = p + text.size();
    std::vector<Corner> poly;
    std::vector<TriMesh::VertexHandle> tri(3);

    while (p < end) {
        const char* lineEnd = static_cast<const char*>(std::memchr(p, '\n', size_t(end - p)));
        if (!lineEnd) lineEnd = end;
        const char* q = skipSpaces(p, lineEnd);
        if (q < lineEnd && *q != '#') {
            if (q[0] == 'v' && q + 1 < lineEnd && isSpace(q[1])) {
                const char* r = q + 1;
                double x, y, z;
                r = skipSpaces(r, lineEnd);
                if (parseDouble(r, lineEnd, x) && (r = skipSpaces(r, lineEnd), parseDouble(r, lineEnd, y)) &&
                    (r = skipSpaces(r, lineEnd), parseDouble(r, lineEnd, z)))
                    vertices.push_back(mesh.add_vertex(Vec3(x, y, z)));
            } else if (q[0] == 'v' && q + 2 < lineEnd && q[1] == 't' && isSpace(q[2])) {
                const char* r = skipSpaces(q + 2, lineEnd);
                double u = 0, v = 0;
                parseDouble(r, lineEnd, u);
                r = skipSpaces(r, lineEnd);
                parseDouble(r, lineEnd, v);
                uvs.push_back(float(u));
                uvs.push_back(float(v));
            } else if (q[0] == 'f' && q + 1 < lineEnd && isSpace(q[1])) {
                const char* r = skipSpaces(q + 1, lineEnd);
                poly.clear();
                Corner c;
                while (r < lineEnd && parseCorner(r, lineEnd, c)) {
                    poly.push_back(c);
                    r = skipSpaces(r, lineEnd);
                }
                auto vid = [&](long i) -> long { return i > 0 ? i - 1 : long(vertices.size()) + i; };
                for (size_t k = 1; k + 1 < poly.size(); ++k) {     // triangle fan
                    const Corner cs[3] = {poly[0], poly[k], poly[k + 1]};
                    bool ok = true;
                    long ids[3];
                    for (int j = 0; j < 3; ++j) {
                        ids[j] = vid(cs[j].v);
                        if (ids[j] < 0 || ids[j] >= long(vertices.size())) ok = false;
                    }
                    if (!ok || ids[0] == ids[1] || ids[1] == ids[2] || ids[0] == ids[2]) { ++ex.skippedFaces; continue; }
                    for (int j = 0; j < 3; ++j) tri[j] = vertices[ids[j]];
                    const TriMesh::FaceHandle f = mesh.add_face(tri);
                    if (!f.is_valid()) { ++ex.skippedFaces; continue; }

                    ex.faceMaterial.push_back(currentMaterial);
                    if (!uvs.empty() || ex.hasUV()) {
                        // corners in the order the mesh iterates them
                        float uv[6] = {0, 0, 0, 0, 0, 0};
                        int slot = 0;
                        for (auto v : mesh.fv_range(f)) {
                            for (int j = 0; j < 3; ++j)
                                if (ids[j] == v.idx()) {
                                    const long ti = cs[j].vt > 0 ? cs[j].vt - 1 : long(uvs.size() / 2) + cs[j].vt;
                                    if (cs[j].vt != 0 && ti >= 0 && ti * 2 + 1 < long(uvs.size())) {
                                        uv[slot * 2] = uvs[ti * 2];
                                        uv[slot * 2 + 1] = uvs[ti * 2 + 1];
                                    }
                                }
                            ++slot;
                        }
                        // faces before the first vt get zeros: pad so that the array stays parallel to the faces
                        const size_t want = size_t(f.idx()) * 6;
                        if (ex.cornerUV.size() < want) ex.cornerUV.resize(want, 0.0f);
                        ex.cornerUV.insert(ex.cornerUV.end(), uv, uv + 6);
                    }
                }
            } else if (std::strncmp(q, "mtllib", 6) == 0 && isSpace(q[6])) {
                const char* r = skipSpaces(q + 6, lineEnd);
                const char* e = lineEnd;
                while (e > r && (isSpace(e[-1]))) --e;
                mtllib.assign(r, e);
            } else if (std::strncmp(q, "usemtl", 6) == 0 && isSpace(q[6])) {
                const char* r = skipSpaces(q + 6, lineEnd);
                const char* e = lineEnd;
                while (e > r && (isSpace(e[-1]))) --e;
                const std::string name(r, e);
                auto it = materialIndex.find(name);
                if (it == materialIndex.end()) it = materialIndex.emplace(name, int(materialIndex.size())).first;
                currentMaterial = it->second;
            }
        }
        p = lineEnd + 1;
    }
    if (mesh.n_faces() == 0) return fail("the file contains no faces");
    if (ex.hasUV()) ex.cornerUV.resize(mesh.n_faces() * 6, 0.0f);

    // materials: the library gives the maps; indices follow the order of the first usemtl of each name
    std::vector<ObjMaterial> library;
    if (!mtllib.empty()) {
        std::string mtlText;
        if (readFile(ex.directory + "/" + mtllib, mtlText)) library = parseMtl(mtlText);
    }
    ex.materials.assign(materialIndex.size(), ObjMaterial());
    for (const auto& [name, idx] : materialIndex) {
        ex.materials[idx].name = name;
        for (const ObjMaterial& m : library)
            if (m.name == name) ex.materials[idx].diffuseMap = m.diffuseMap;
    }
    return true;
}
