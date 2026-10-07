#include "param/FlatLoader.h"

#include "core/TextParse.h"

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <sstream>

namespace {

using namespace textparse;

class MappedFile {
public:
    explicit MappedFile(const std::string& path)
    {
        fd_ = ::open(path.c_str(), O_RDONLY);
        if (fd_ < 0) return;
        struct stat st;
        if (::fstat(fd_, &st) != 0 || st.st_size <= 0) return;
        size_ = size_t(st.st_size);
        void* p = ::mmap(nullptr, size_, PROT_READ, MAP_PRIVATE, fd_, 0);
        if (p != MAP_FAILED) data_ = static_cast<const char*>(p);
    }
    ~MappedFile()
    {
        if (data_) ::munmap(const_cast<char*>(data_), size_);
        if (fd_ >= 0) ::close(fd_);
    }
    MappedFile(const MappedFile&) = delete;
    MappedFile& operator=(const MappedFile&) = delete;
    const char* data() const { return data_; }
    size_t size() const { return size_; }
    bool ok() const { return data_ != nullptr; }
private:
    int fd_ = -1;
    const char* data_ = nullptr;
    size_t size_ = 0;
};

// ---------------------------------------------------------------- PLY

enum class Scalar { Int8, UInt8, Int16, UInt16, Int32, UInt32, Float32, Float64, Bad };

Scalar scalarOf(const std::string& s)
{
    if (s == "char" || s == "int8") return Scalar::Int8;
    if (s == "uchar" || s == "uint8") return Scalar::UInt8;
    if (s == "short" || s == "int16") return Scalar::Int16;
    if (s == "ushort" || s == "uint16") return Scalar::UInt16;
    if (s == "int" || s == "int32") return Scalar::Int32;
    if (s == "uint" || s == "uint32") return Scalar::UInt32;
    if (s == "float" || s == "float32") return Scalar::Float32;
    if (s == "double" || s == "float64") return Scalar::Float64;
    return Scalar::Bad;
}

size_t sizeOf(Scalar s)
{
    switch (s) {
        case Scalar::Int8: case Scalar::UInt8: return 1;
        case Scalar::Int16: case Scalar::UInt16: return 2;
        case Scalar::Int32: case Scalar::UInt32: case Scalar::Float32: return 4;
        case Scalar::Float64: return 8;
        default: return 0;
    }
}

struct Property {
    std::string name;
    Scalar type = Scalar::Bad;
    bool isList = false;
    Scalar countType = Scalar::Bad;
};

struct Element {
    std::string name;
    size_t count = 0;
    std::vector<Property> props;
};

// One value of the given type at p, converted to double (byte swapped when `swap`).
double readScalar(const char* p, Scalar t, bool swap)
{
    unsigned char b[8];
    const size_t n = sizeOf(t);
    std::memcpy(b, p, n);
    if (swap) std::reverse(b, b + n);
    switch (t) {
        case Scalar::Int8: return double(int8_t(b[0]));
        case Scalar::UInt8: return double(b[0]);
        case Scalar::Int16: { int16_t v; std::memcpy(&v, b, 2); return v; }
        case Scalar::UInt16: { uint16_t v; std::memcpy(&v, b, 2); return v; }
        case Scalar::Int32: { int32_t v; std::memcpy(&v, b, 4); return v; }
        case Scalar::UInt32: { uint32_t v; std::memcpy(&v, b, 4); return v; }
        case Scalar::Float32: { float v; std::memcpy(&v, b, 4); return v; }
        case Scalar::Float64: { double v; std::memcpy(&v, b, 8); return v; }
        default: return 0.0;
    }
}

bool fail(std::string* error, const std::string& msg)
{
    if (error) *error = msg;
    return false;
}

bool loadPly(const char* data, size_t size, FlatMesh& out, FlatLoadInfo& info, std::string* error, FlatSurface* surface)
{
    // ---- header
    size_t pos = 0;
    auto nextLine = [&](std::string& line) {
        if (pos >= size) return false;
        size_t e = pos;
        while (e < size && data[e] != '\n') ++e;
        line.assign(data + pos, data + e);
        if (!line.empty() && line.back() == '\r') line.pop_back();
        pos = e < size ? e + 1 : e;
        return true;
    };
    std::string line;
    if (!nextLine(line) || line != "ply") return fail(error, "not a PLY file");
    enum { Ascii, Little, Big } fmt = Ascii;
    std::vector<Element> elements;
    bool ended = false;
    while (nextLine(line)) {
        std::istringstream in(line);
        std::string key;
        in >> key;
        if (key == "format") {
            std::string f;
            in >> f;
            if (f == "ascii") fmt = Ascii;
            else if (f == "binary_little_endian") fmt = Little;
            else if (f == "binary_big_endian") fmt = Big;
            else return fail(error, "unknown PLY format " + f);
            info.format = "ply " + f;
        } else if (key == "comment") {
            std::string word, name;
            in >> word >> name;
            if (surface && (word == "TextureFile" || word == "texturefile")) surface->textureFile = name;
        } else if (key == "element") {
            Element e;
            in >> e.name >> e.count;
            elements.push_back(e);
        } else if (key == "property") {
            if (elements.empty()) return fail(error, "PLY property before any element");
            Property p;
            std::string t;
            in >> t;
            if (t == "list") {
                std::string ct, it;
                in >> ct >> it >> p.name;
                p.isList = true;
                p.countType = scalarOf(ct);
                p.type = scalarOf(it);
                if (p.countType == Scalar::Bad || p.type == Scalar::Bad) return fail(error, "bad PLY list type");
            } else {
                p.type = scalarOf(t);
                in >> p.name;
                if (p.type == Scalar::Bad) return fail(error, "bad PLY property type " + t);
            }
            elements.back().props.push_back(p);
        } else if (key == "end_header") {
            ended = true;
            break;
        }
    }
    if (!ended) return fail(error, "PLY header does not end");

    const bool binary = fmt != Ascii;
    const bool swap = fmt == Big;   // the host is little endian
    const Element* vertexEl = nullptr;
    const Element* faceEl = nullptr;
    for (const Element& e : elements) {
        if (e.name == "vertex" && !vertexEl) vertexEl = &e;
        if (e.name == "face" && !faceEl) faceEl = &e;
    }
    if (!vertexEl || !faceEl) return fail(error, "PLY without vertex or face elements");
    const size_t nV = vertexEl->count, nFaceRecords = faceEl->count;
    if (nV > UINT32_MAX) return fail(error, "more than 2^32 vertices");

    out.pos.assign(nV * 3, 0.f);
    out.idx.clear();
    out.idx.reserve(nFaceRecords * 3);
    std::vector<float> vertexUV;   // per vertex, when the vertices carry texture coordinates
    if (surface) surface->cornerUV.clear();

    auto isFixed = [](const Element& e) {
        for (const Property& p : e.props) if (p.isList) return false;
        return true;
    };
    auto fixedSize = [](const Element& e) {
        size_t s = 0;
        for (const Property& p : e.props) s += sizeOf(p.type);
        return s;
    };

    const char* p = data + pos;
    const char* const end = data + size;

    // ---- elements in file order
    for (const Element& e : elements) {
        if (&e == vertexEl) {
            int ix = -1, iy = -1, iz = -1;
            for (size_t i = 0; i < e.props.size(); ++i) {
                if (e.props[i].name == "x") ix = int(i);
                if (e.props[i].name == "y") iy = int(i);
                if (e.props[i].name == "z") iz = int(i);
            }
            if (ix < 0 || iy < 0 || iz < 0) return fail(error, "PLY vertices without x y z");
            int iu = -1, iv = -1;
            for (size_t i = 0; i < e.props.size(); ++i) {
                const std::string& n = e.props[i].name;
                if (n == "s" || n == "u" || n == "texture_u" || n == "texture_s") iu = int(i);
                if (n == "t" || n == "v" || n == "texture_v" || n == "texture_t") iv = int(i);
            }
            if (surface && iu >= 0 && iv >= 0) vertexUV.assign(nV * 2, 0.f);
            if (binary) {
                if (!isFixed(e)) return fail(error, "PLY vertex element with a list property");
                std::vector<size_t> off(e.props.size());
                size_t s = 0;
                for (size_t i = 0; i < e.props.size(); ++i) { off[i] = s; s += sizeOf(e.props[i].type); }
                if (size_t(end - p) < s * nV) return fail(error, "PLY file is truncated (vertices)");
                const int sel[3] = {ix, iy, iz};
                for (size_t v = 0; v < nV; ++v) {
                    for (int c = 0; c < 3; ++c)
                        out.pos[3 * v + c] = float(readScalar(p + s * v + off[sel[c]], e.props[sel[c]].type, swap));
                    if (!vertexUV.empty()) {
                        vertexUV[2 * v] = float(readScalar(p + s * v + off[iu], e.props[iu].type, swap));
                        vertexUV[2 * v + 1] = float(readScalar(p + s * v + off[iv], e.props[iv].type, swap));
                    }
                }
                p += s * nV;
            } else {
                for (size_t v = 0; v < nV; ++v) {
                    for (size_t i = 0; i < e.props.size(); ++i) {
                        p = skipSpaces(p, end);
                        while (p < end && *p == '\n') p = skipSpaces(p + 1, end);
                        double x;
                        if (!parseDouble(p, end, x)) return fail(error, "PLY: bad vertex value");
                        if (int(i) == ix) out.pos[3 * v] = float(x);
                        if (int(i) == iy) out.pos[3 * v + 1] = float(x);
                        if (int(i) == iz) out.pos[3 * v + 2] = float(x);
                        if (!vertexUV.empty() && int(i) == iu) vertexUV[2 * v] = float(x);
                        if (!vertexUV.empty() && int(i) == iv) vertexUV[2 * v + 1] = float(x);
                    }
                }
            }
        } else if (&e == faceEl) {
            // the vertex index list and the properties around it
            int listIdx = -1;
            for (size_t i = 0; i < e.props.size(); ++i)
                if (e.props[i].isList && (e.props[i].name == "vertex_indices" || e.props[i].name == "vertex_index")) listIdx = int(i);
            if (listIdx < 0) return fail(error, "PLY faces without vertex_indices");
            int texIdx = -1;   // per face corner texture coordinates: "property list uchar float texcoord"
            for (size_t i = 0; i < e.props.size(); ++i)
                if (e.props[i].isList && (e.props[i].name == "texcoord" || e.props[i].name == "texcoords")) texIdx = int(i);
            const bool faceUV = surface && texIdx >= 0;
            const bool wantUV = surface && (faceUV || !vertexUV.empty());
            std::vector<uint32_t> poly;
            size_t pendingFirst = 0, pendingCount = 0;   // the triangles of the face being read (their uv wait for the texcoord list)
            auto emit = [&](const std::vector<uint32_t>& c) {
                pendingCount = 0;
                if (c.size() < 3) { ++info.skippedFaces; return; }
                for (uint32_t v : c) if (v >= nV) { ++info.skippedFaces; return; }
                pendingFirst = out.idx.size() / 3;
                for (size_t k = 1; k + 1 < c.size(); ++k) {
                    out.idx.push_back(c[0]);
                    out.idx.push_back(c[k]);
                    out.idx.push_back(c[k + 1]);
                    ++pendingCount;
                    if (!wantUV) continue;
                    if (faceUV) {
                        for (int q = 0; q < 6; ++q) surface->cornerUV.push_back(std::nanf(""));
                    } else {
                        const uint32_t corner[3] = {c[0], c[k], c[k + 1]};
                        for (uint32_t v : corner) { surface->cornerUV.push_back(vertexUV[2 * size_t(v)]); surface->cornerUV.push_back(vertexUV[2 * size_t(v) + 1]); }
                    }
                }
            };
            // the texture coordinates of the corners of the polygon just emitted: fan triangles (0, k, k + 1)
            auto applyTex = [&](const std::vector<float>& tc) {
                for (size_t k = 0; k < pendingCount; ++k) {
                    const size_t corner[3] = {0, k + 1, k + 2};
                    for (int q = 0; q < 3; ++q)
                        if (2 * corner[q] + 1 < tc.size()) {
                            surface->cornerUV[6 * (pendingFirst + k) + 2 * q] = tc[2 * corner[q]];
                            surface->cornerUV[6 * (pendingFirst + k) + 2 * q + 1] = tc[2 * corner[q] + 1];
                        }
                }
            };
            std::vector<float> texBuf;
            if (binary) {
                for (size_t f = 0; f < e.count; ++f) {
                    for (size_t i = 0; i < e.props.size(); ++i) {
                        const Property& pr = e.props[i];
                        if (!pr.isList) {
                            if (size_t(end - p) < sizeOf(pr.type)) return fail(error, "PLY file is truncated (faces)");
                            p += sizeOf(pr.type);
                            continue;
                        }
                        if (size_t(end - p) < sizeOf(pr.countType)) return fail(error, "PLY file is truncated (faces)");
                        const size_t n = size_t(readScalar(p, pr.countType, swap));
                        p += sizeOf(pr.countType);
                        if (size_t(end - p) < n * sizeOf(pr.type)) return fail(error, "PLY file is truncated (faces)");
                        if (int(i) == listIdx) {
                            poly.resize(n);
                            for (size_t k = 0; k < n; ++k) poly[k] = uint32_t(readScalar(p + k * sizeOf(pr.type), pr.type, swap));
                            emit(poly);
                        } else if (faceUV && int(i) == texIdx && pendingCount > 0) {
                            texBuf.resize(n);
                            for (size_t k = 0; k < n; ++k) texBuf[k] = float(readScalar(p + k * sizeOf(pr.type), pr.type, swap));
                            applyTex(texBuf);
                        }
                        p += n * sizeOf(pr.type);
                    }
                }
            } else {
                for (size_t f = 0; f < e.count; ++f) {
                    for (size_t i = 0; i < e.props.size(); ++i) {
                        const Property& pr = e.props[i];
                        p = skipSpaces(p, end);
                        while (p < end && *p == '\n') p = skipSpaces(p + 1, end);
                        if (!pr.isList) { double x; if (!parseDouble(p, end, x)) return fail(error, "PLY: bad face value"); continue; }
                        long n;
                        if (!parseInt(p, end, n) || n < 0) return fail(error, "PLY: bad face size");
                        if (int(i) == texIdx && faceUV) {
                            texBuf.resize(size_t(n));
                            for (long k = 0; k < n; ++k) {
                                double x;
                                p = skipSpaces(p, end);
                                if (!parseDouble(p, end, x)) return fail(error, "PLY: bad texcoord");
                                texBuf[size_t(k)] = float(x);
                            }
                            if (pendingCount > 0) applyTex(texBuf);
                            continue;
                        }
                        poly.resize(size_t(n));
                        for (long k = 0; k < n; ++k) {
                            long v;
                            p = skipSpaces(p, end);
                            if (!parseInt(p, end, v)) return fail(error, "PLY: bad face index");
                            poly[size_t(k)] = uint32_t(v);
                        }
                        if (int(i) == listIdx) emit(poly);
                    }
                }
            }
        } else if (binary) {   // another element: skip it (fixed size records only)
            if (!isFixed(e)) return fail(error, "PLY element " + e.name + " has lists: cannot skip it");
            const size_t s = fixedSize(e) * e.count;
            if (size_t(end - p) < s) return fail(error, "PLY file is truncated");
            p += s;
        } else {
            // ascii: skip e.count lines
            for (size_t r = 0; r < e.count && p < end; ++r) {
                while (p < end && *p != '\n') ++p;
                if (p < end) ++p;
            }
        }
        if (&e == vertexEl || &e == faceEl) {
            if (!binary) { /* position already past the last value */ }
        }
    }
    return true;
}

// ---------------------------------------------------------------- OBJ

bool readTextFile(const std::string& path, std::string& out)
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

bool loadObjFlat(const char* data, size_t size, const std::string& directory, FlatMesh& out, FlatLoadInfo& info,
                 std::string* error, FlatSurface* surface)
{
    info.format = "obj";
    const char* p = data;
    const char* const end = data + size;
    std::vector<long> poly, polyVt;
    std::vector<float> vt;                 // u v per texture coordinate
    int material = -1;
    bool sawUV = false;
    if (surface) { surface->directory = directory; surface->cornerUV.clear(); surface->faceMaterial.clear(); surface->materials.clear(); }
    while (p < end) {
        const char* lineEnd = static_cast<const char*>(std::memchr(p, '\n', size_t(end - p)));
        if (!lineEnd) lineEnd = end;
        const char* q = skipSpaces(p, lineEnd);
        if (q + 1 < lineEnd && q[0] == 'v' && isSpace(q[1])) {
            q += 1;
            double v[3];
            bool ok = true;
            for (int c = 0; c < 3 && ok; ++c) {
                q = skipSpaces(q, lineEnd);
                ok = parseDouble(q, lineEnd, v[c]);
            }
            if (!ok) return fail(error, "OBJ: bad vertex");
            for (int c = 0; c < 3; ++c) out.pos.push_back(float(v[c]));
        } else if (surface && q + 2 < lineEnd && q[0] == 'v' && q[1] == 't' && isSpace(q[2])) {
            q += 2;
            double a = 0, b = 0;
            q = skipSpaces(q, lineEnd);
            parseDouble(q, lineEnd, a);
            q = skipSpaces(q, lineEnd);
            parseDouble(q, lineEnd, b);
            vt.push_back(float(a));
            vt.push_back(float(b));
        } else if (q + 1 < lineEnd && q[0] == 'f' && isSpace(q[1])) {
            q += 1;
            poly.clear();
            polyVt.clear();
            const long nV = long(out.vertexCount()), nVt = long(vt.size() / 2);
            for (;;) {
                q = skipSpaces(q, lineEnd);
                if (q >= lineEnd) break;
                Corner c;
                if (!parseCorner(q, lineEnd, c)) break;
                poly.push_back(c.v > 0 ? c.v - 1 : nV + c.v);
                polyVt.push_back(c.vt == 0 ? -1 : (c.vt > 0 ? c.vt - 1 : nVt + c.vt));
                q = skipToken(q, lineEnd);   // whatever follows the index (v/vt/vn leftovers)
            }
            bool ok = poly.size() >= 3;
            for (long v : poly) ok = ok && v >= 0 && v < nV;
            if (!ok) { ++info.skippedFaces; }
            else
                for (size_t k = 1; k + 1 < poly.size(); ++k) {
                    out.idx.push_back(uint32_t(poly[0]));
                    out.idx.push_back(uint32_t(poly[k]));
                    out.idx.push_back(uint32_t(poly[k + 1]));
                    if (!surface) continue;
                    const size_t corner[3] = {0, k, k + 1};
                    for (size_t cidx : corner) {
                        const long t = polyVt[cidx];
                        if (t >= 0 && t < long(vt.size() / 2)) {
                            surface->cornerUV.push_back(vt[2 * size_t(t)]);
                            surface->cornerUV.push_back(vt[2 * size_t(t) + 1]);
                            sawUV = true;
                        } else {
                            surface->cornerUV.push_back(std::nanf(""));
                            surface->cornerUV.push_back(std::nanf(""));
                        }
                    }
                    surface->faceMaterial.push_back(material < 0 ? uint8_t(255) : uint8_t(std::min(material, 254)));
                }
        } else if (surface && lineEnd - q > 7 && std::strncmp(q, "mtllib", 6) == 0 && isSpace(q[6])) {
            const char* a = skipSpaces(q + 6, lineEnd);
            const char* b = lineEnd;
            while (b > a && (b[-1] == ' ' || b[-1] == '\r' || b[-1] == '\t')) --b;
            std::string text;
            if (readTextFile(directory + "/" + std::string(a, b), text)) {
                std::vector<ObjMaterial> m = parseMtl(text);
                surface->materials.insert(surface->materials.end(), m.begin(), m.end());
            }
        } else if (surface && lineEnd - q > 7 && std::strncmp(q, "usemtl", 6) == 0 && isSpace(q[6])) {
            const char* a = skipSpaces(q + 6, lineEnd);
            const char* b = lineEnd;
            while (b > a && (b[-1] == ' ' || b[-1] == '\r' || b[-1] == '\t')) --b;
            const std::string name(a, b);
            material = -1;
            for (size_t i = 0; i < surface->materials.size(); ++i)
                if (surface->materials[i].name == name) { material = int(i); break; }
        }
        p = lineEnd < end ? lineEnd + 1 : end;
    }
    if (surface && !sawUV) { surface->cornerUV.clear(); surface->faceMaterial.clear(); }
    return true;
}

} // namespace

bool loadFlatMesh(const std::string& path, FlatMesh& out, FlatLoadInfo* infoOut, std::string* error, FlatSurface* surface)
{
    const auto t0 = std::chrono::steady_clock::now();
    FlatLoadInfo info;
    out = FlatMesh();
    MappedFile file(path);
    if (!file.ok()) return fail(error, "cannot read " + path);
    const std::string ext = [&] {
        const size_t d = path.find_last_of('.');
        std::string e = d == std::string::npos ? "" : path.substr(d + 1);
        for (char& c : e) c = char(std::tolower(static_cast<unsigned char>(c)));
        return e;
    }();
    bool ok = false;
    const size_t slash = path.find_last_of('/');
    const std::string directory = slash == std::string::npos ? std::string(".") : path.substr(0, slash);
    if (surface) *surface = FlatSurface();
    if (surface) surface->directory = directory;
    if (ext == "ply") ok = loadPly(file.data(), file.size(), out, info, error, surface);
    else if (ext == "obj") ok = loadObjFlat(file.data(), file.size(), directory, out, info, error, surface);
    else return fail(error, "unsupported format ." + ext + " (ply and obj)");
    if (ok && out.faceCount() == 0) ok = fail(error, "no faces in " + path);
    info.seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    if (infoOut) *infoOut = info;
    return ok;
}

FlatTransform normalizeFlatToUnitBox(FlatMesh& m)
{
    FlatTransform t;
    if (m.vertexCount() == 0) return t;
    double lo[3] = {1e300, 1e300, 1e300}, hi[3] = {-1e300, -1e300, -1e300};
    for (size_t v = 0; v < m.vertexCount(); ++v)
        for (int c = 0; c < 3; ++c) {
            lo[c] = std::min(lo[c], double(m.pos[3 * v + c]));
            hi[c] = std::max(hi[c], double(m.pos[3 * v + c]));
        }
    double ext = 0.0;
    for (int c = 0; c < 3; ++c) { t.centre[c] = 0.5 * (lo[c] + hi[c]); ext = std::max(ext, hi[c] - lo[c]); }
    t.scale = ext > 0.0 ? 1.0 / ext : 1.0;
    for (size_t v = 0; v < m.vertexCount(); ++v)
        for (int c = 0; c < 3; ++c) m.pos[3 * v + c] = float((double(m.pos[3 * v + c]) - t.centre[c]) * t.scale);
    return t;
}
