#include "core/MeshPrep.h"
#include "core/ObjLoader.h"

#include <OpenMesh/Core/IO/MeshIO.hh>

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <random>
#include <sstream>

static int failures = 0;
#define CHECK(c) do { if (!(c)) { std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); ++failures; } } while (0)

static std::string writeTemp(const std::string& name, const std::string& content)
{
    const std::string path = "/tmp/vsa_test_" + name;
    std::ofstream f(path, std::ios::binary);
    f << content;
    return path;
}

// the faces of a mesh as sorted vertex triples
static std::vector<std::array<int, 3>> faceSet(const TriMesh& m)
{
    std::vector<std::array<int, 3>> out;
    for (auto f : m.faces()) {
        std::array<int, 3> t;
        int k = 0;
        for (auto v : m.fv_range(f)) t[k++] = v.idx();
        std::sort(t.begin(), t.end());
        out.push_back(t);
    }
    std::sort(out.begin(), out.end());
    return out;
}

static void testAgainstOpenMesh(const std::string& path)
{
    TriMesh a, b;
    std::string err;
    OpenMesh::IO::Options opt;
    CHECK(OpenMesh::IO::read_mesh(a, path, opt));   // the stream based reader of OpenMesh as the reference
    CHECK(loadObj(path, b, nullptr, &err));
    CHECK(a.n_vertices() == b.n_vertices() && a.n_faces() == b.n_faces());
    for (size_t i = 0; i < a.n_vertices() && i < b.n_vertices(); ++i)
        CHECK((a.point(TriMesh::VertexHandle(int(i))) - b.point(TriMesh::VertexHandle(int(i)))).norm() < 1e-5);   // OpenMesh reads OBJ coordinates as float
    CHECK(faceSet(a) == faceSet(b));
}

static void testNumbers()
{
    // many spellings of the same kind of numbers must equal strtod bit for bit
    std::mt19937_64 rng(5);
    std::uniform_real_distribution<double> U(-1.0, 1.0);
    std::ostringstream os;
    std::vector<double> expected;
    const char* formats[] = {"%.6f", "%.9f", "%.15g", "%.17g", "%.3e", "%g", "%.12f"};
    char buf[64];
    for (int i = 0; i < 3000; ++i) {
        for (int k = 0; k < 3; ++k) {
            const double v = U(rng) * std::pow(10.0, int(rng() % 9) - 4);
            std::snprintf(buf, sizeof(buf), formats[(i + k) % 7], v);
            os << buf << (k < 2 ? " " : "\n");
            expected.push_back(std::strtod(buf, nullptr));
        }
    }
    // three vertices at the end make faces
    std::string text = "# numbers\n";
    std::istringstream is(os.str());
    std::string line;
    int vcount = 0;
    while (std::getline(is, line)) { text += "v " + line + "\n"; ++vcount; }
    text += "f 1 2 3\nf -1 -2 -3\n";
    const std::string path = writeTemp("numbers.obj", text);
    TriMesh m;
    std::string err;
    CHECK(loadObj(path, m, nullptr, &err));
    CHECK(int(m.n_vertices()) == vcount);
    int mismatches = 0;
    for (int i = 0; i < vcount; ++i)
        for (int k = 0; k < 3; ++k)
            mismatches += m.point(TriMesh::VertexHandle(i))[k] != expected[size_t(i) * 3 + k];
    CHECK(mismatches == 0);
    std::remove(path.c_str());
}

static void testSyntax()
{
    // CRLF, comments, a 4th coordinate, tabs, a quad (fan), negative indices, v//vn and v/vt/vn corners, a missing newline at the end
    const std::string text =
        "# comment\r\n"
        "v 0 0 0 1\r\n"
        "v\t1 0 0\r\n"
        "v 1 1 0\r\n"
        "v 0 1 0\r\n"
        "vn 0 0 1\r\n"
        "vt 0 0\nvt 1 0\nvt 1 1\nvt 0 1\n"
        "f 1/1/1 2/2/1 3/3/1 4/4/1\r\n"
        "v 2 0 0\nv 2 1 0\n"
        "f -3 -2 -1\n"       // vertices 4, 5, 6 -> the relative indices count from the end: 4 5 6
        "f 2//1 6 5";        // no newline at the end
    const std::string path = writeTemp("syntax.obj", text);
    TriMesh m;
    ObjExtras ex;
    std::string err;
    CHECK(loadObj(path, m, &ex, &err));
    CHECK(m.n_vertices() == 6);
    CHECK(m.n_faces() == 4);                // the quad gives two triangles
    CHECK(ex.skippedFaces == 0);
    CHECK(ex.hasUV() && ex.cornerUV.size() == m.n_faces() * 6);
    // the quad fan (1 2 3)(1 3 4): the corner uv follow the vt of the quad
    bool seen = false;
    for (auto f : m.faces()) {
        int k = 0;
        bool hasV1 = false, hasV3 = false;
        std::vector<int> ids;
        for (auto v : m.fv_range(f)) { ids.push_back(v.idx()); ++k; }
        for (int id : ids) { hasV1 |= id == 0; hasV3 |= id == 2; }
        if (hasV1 && hasV3) {
            for (size_t c = 0; c < 3; ++c) {
                if (ids[c] == 2) CHECK(ex.cornerUV[f.idx() * 6 + c * 2] == 1.0f && ex.cornerUV[f.idx() * 6 + c * 2 + 1] == 1.0f);
                if (ids[c] == 0) CHECK(ex.cornerUV[f.idx() * 6 + c * 2] == 0.0f && ex.cornerUV[f.idx() * 6 + c * 2 + 1] == 0.0f);
            }
            seen = true;
        }
    }
    CHECK(seen);
    std::remove(path.c_str());
}

static void testMaterialsAndErrors()
{
    const std::string mtl = writeTemp("scene.mtl",
        "newmtl wall\nKd 1 1 1\nmap_Kd wall tex.jpg\n\nnewmtl plain\nKd 0.5 0.5 0.5\nnewmtl floor\r\nmap_Kd -s 1 1 1 floor.png\r\n");
    const std::string obj = writeTemp("scene.obj",
        "mtllib vsa_test_scene.mtl\n"
        "v 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0\nv 2 0 0\nv 2 1 0\n"
        "vt 0 0\nvt 1 0\nvt 1 1\nvt 0 1\n"
        "usemtl wall\nf 1/1 2/2 3/3\nf 1/1 3/3 4/4\n"
        "usemtl plain\nf 2 5 6\n"
        "usemtl floor\nf 2 6 3\n");
    TriMesh m;
    ObjExtras ex;
    std::string err;
    CHECK(loadObj(obj, m, &ex, &err));
    CHECK(m.n_faces() == 4 && ex.faceMaterial.size() == 4);
    CHECK(ex.materials.size() == 3);
    CHECK(ex.materials[0].name == "wall" && ex.materials[0].diffuseMap == "wall tex.jpg");   // a name with a space
    CHECK(ex.materials[1].name == "plain" && ex.materials[1].diffuseMap.empty());
    CHECK(ex.materials[2].name == "floor" && ex.materials[2].diffuseMap == "floor.png");     // the option is skipped
    CHECK(ex.faceMaterial[0] == 0 && ex.faceMaterial[1] == 0 && ex.faceMaterial[2] == 1 && ex.faceMaterial[3] == 2);
    CHECK(ex.directory == "/tmp");
    const auto lib = parseMtl("newmtl a\nmap_Kd x.jpg\nnewmtl b\n");
    CHECK(lib.size() == 2 && lib[0].diffuseMap == "x.jpg" && lib[1].diffuseMap.empty());

    // a face that the mesh rejects (it would make an edge non-manifold) and a malformed one are skipped
    const std::string bad = writeTemp("bad.obj",
        "v 0 0 0\nv 1 0 0\nv 0 1 0\nv 0 0 1\nv 1 1 1\n"
        "f 1 2 3\nf 2 1 4\nf 1 2 5\n"          // the third face would use the edge 1->2 a second time
        "f 1 1 2\nf 1 2 99\nf 1\n");          // a repeated vertex, an index out of range, too few corners
    CHECK(loadObj(bad, m, &ex, &err));
    CHECK(m.n_faces() == 2 && ex.skippedFaces == 3);

    CHECK(!loadObj("/tmp/vsa_no_such_file.obj", m, &ex, &err) && !err.empty());
    const std::string empty = writeTemp("empty.obj", "# nothing\nv 0 0 0\n");
    CHECK(!loadObj(empty, m, &ex, &err));
    for (const std::string& f : {mtl, obj, bad, empty}) std::remove(f.c_str());
}

int main(int argc, char** argv)
{
    const std::string hemisphere = argc > 1 ? argv[1] : "data/hemisphere.obj";
    testAgainstOpenMesh(hemisphere);
    testAgainstOpenMesh(hemisphere.substr(0, hemisphere.find_last_of('/') + 1) + "cube.obj");
    TriMesh viaLoadMesh;
    CHECK(loadMesh(hemisphere, viaLoadMesh) && viaLoadMesh.n_faces() > 0);   // loadMesh uses the fast reader for .obj
    testNumbers();
    testSyntax();
    testMaterialsAndErrors();
    std::printf(failures ? "FAILED (%d)\n" : "OK\n", failures);
    return failures ? 1 : 0;
}
