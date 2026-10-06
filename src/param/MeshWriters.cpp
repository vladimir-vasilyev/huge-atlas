#include "param/MeshWriters.h"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>

namespace {

bool hostIsLittleEndian()
{
    const uint16_t one = 1;
    return *reinterpret_cast<const unsigned char*>(&one) == 1;
}

template <class T>
void putLE(std::vector<unsigned char>& out, T value)
{
    unsigned char bytes[sizeof(T)];
    std::memcpy(bytes, &value, sizeof(T));
    if (!hostIsLittleEndian())
        for (size_t i = 0; i < sizeof(T) / 2; ++i) std::swap(bytes[i], bytes[sizeof(T) - 1 - i]);
    out.insert(out.end(), bytes, bytes + sizeof(T));
}

void header(std::ostream& f, bool binary, size_t vertices, size_t faces, bool withUV, const std::string& textureFile,
            const std::string& comment)
{
    f << "ply\nformat " << (binary ? "binary_little_endian" : "ascii") << " 1.0\n";
    f << "comment VSA + LinABF\n";
    if (!comment.empty()) f << "comment " << comment << "\n";
    if (!textureFile.empty()) f << "comment TextureFile " << textureFile << "\n";
    f << "element vertex " << vertices << "\nproperty float x\nproperty float y\nproperty float z\n";
    if (withUV) f << "property float s\nproperty float t\n";
    f << "element face " << faces << "\nproperty list uchar int vertex_indices\nend_header\n";
}

} // namespace

int writePlyAtlas(const std::string& path, const std::vector<const Patch*>& patches,
                  const std::vector<const std::vector<Eigen::Vector2d>*>& uv, const std::string& textureFile, bool binary,
                  const std::string& comment)
{
    size_t vertices = 0, faces = 0;
    std::vector<size_t> offset(patches.size(), 0);
    std::vector<bool> used(patches.size(), false);
    for (size_t i = 0; i < patches.size(); ++i) {
        if (!uv[i] || uv[i]->size() != patches[i]->mesh.n_vertices()) continue;
        used[i] = true;
        offset[i] = vertices;
        vertices += patches[i]->mesh.n_vertices();
        faces += patches[i]->mesh.n_faces();
    }
    std::ofstream f(path, std::ios::binary);
    if (!f) return -1;
    header(f, binary, vertices, faces, true, textureFile, comment);

    if (binary) {
        std::vector<unsigned char> buf;
        buf.reserve(1 << 20);
        auto flush = [&] { f.write(reinterpret_cast<const char*>(buf.data()), std::streamsize(buf.size())); buf.clear(); };
        for (size_t i = 0; i < patches.size(); ++i) {
            if (!used[i]) continue;
            for (auto v : patches[i]->mesh.vertices()) {
                const auto p = patches[i]->mesh.point(v);
                putLE<float>(buf, float(p[0]));
                putLE<float>(buf, float(p[1]));
                putLE<float>(buf, float(p[2]));
                putLE<float>(buf, float((*uv[i])[v.idx()].x()));
                putLE<float>(buf, float((*uv[i])[v.idx()].y()));
                if (buf.size() > (1 << 20) - 64) flush();
            }
        }
        for (size_t i = 0; i < patches.size(); ++i) {
            if (!used[i]) continue;
            for (auto face : patches[i]->mesh.faces()) {
                buf.push_back(3);
                for (auto v : patches[i]->mesh.fv_range(face)) putLE<int32_t>(buf, int32_t(offset[i] + v.idx()));
                if (buf.size() > (1 << 20) - 64) flush();
            }
        }
        flush();
    } else {
        char line[160];
        for (size_t i = 0; i < patches.size(); ++i) {
            if (!used[i]) continue;
            for (auto v : patches[i]->mesh.vertices()) {
                const auto p = patches[i]->mesh.point(v);
                std::snprintf(line, sizeof(line), "%.9g %.9g %.9g %.9g %.9g\n", p[0], p[1], p[2], (*uv[i])[v.idx()].x(),
                              (*uv[i])[v.idx()].y());
                f << line;
            }
        }
        for (size_t i = 0; i < patches.size(); ++i) {
            if (!used[i]) continue;
            for (auto face : patches[i]->mesh.faces()) {
                f << "3";
                for (auto v : patches[i]->mesh.fv_range(face)) f << " " << offset[i] + v.idx();
                f << "\n";
            }
        }
    }
    return f.good() ? int(faces) : -1;
}

bool writePlyMesh(const std::string& path, const TriMesh& mesh, bool binary)
{
    std::ofstream f(path, std::ios::binary);
    if (!f) return false;
    header(f, binary, mesh.n_vertices(), mesh.n_faces(), false, "", "");
    if (binary) {
        std::vector<unsigned char> buf;
        for (auto v : mesh.vertices()) {
            const auto p = mesh.point(v);
            putLE<float>(buf, float(p[0]));
            putLE<float>(buf, float(p[1]));
            putLE<float>(buf, float(p[2]));
        }
        for (auto face : mesh.faces()) {
            buf.push_back(3);
            for (auto v : mesh.fv_range(face)) putLE<int32_t>(buf, int32_t(v.idx()));
        }
        f.write(reinterpret_cast<const char*>(buf.data()), std::streamsize(buf.size()));
    } else {
        char line[128];
        for (auto v : mesh.vertices()) {
            const auto p = mesh.point(v);
            std::snprintf(line, sizeof(line), "%.9g %.9g %.9g\n", p[0], p[1], p[2]);
            f << line;
        }
        for (auto face : mesh.faces()) {
            f << "3";
            for (auto v : mesh.fv_range(face)) f << " " << v.idx();
            f << "\n";
        }
    }
    return f.good();
}

bool writeObjMesh(const std::string& path, const TriMesh& mesh)
{
    std::ofstream f(path);
    if (!f) return false;
    f << "# VSA + LinABF\n";
    char line[128];
    for (auto v : mesh.vertices()) {
        const auto p = mesh.point(v);
        std::snprintf(line, sizeof(line), "v %.9g %.9g %.9g\n", p[0], p[1], p[2]);
        f << line;
    }
    for (auto face : mesh.faces()) {
        f << "f";
        for (auto v : mesh.fv_range(face)) f << " " << v.idx() + 1;
        f << "\n";
    }
    return f.good();
}
