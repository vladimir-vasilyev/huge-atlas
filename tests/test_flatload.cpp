// Flat mesh readers: ascii PLY with extra properties (the bunny), binary little and big endian PLY, OBJ, polygons.
#include "core/MeshPrep.h"
#include "param/FlatLoader.h"
#include "param/MeshWriters.h"
#include "TestUtil.h"

#include <cstdio>
#include <cstring>
#include <fstream>

static void same(const FlatMesh& a, const TriMesh& m, double tol, const char* what)
{
    CHECK(a.vertexCount() == m.n_vertices());
    CHECK(a.faceCount() == m.n_faces());
    double worst = 0;
    for (auto v : m.vertices())
        for (int c = 0; c < 3; ++c) worst = std::max(worst, std::abs(a.pos[3 * size_t(v.idx()) + c] - m.point(v)[c]));
    std::printf("%s: %zu vertices, %zu faces, worst position difference %.2e\n", what, a.vertexCount(), a.faceCount(), worst);
    CHECK(worst <= tol);
    // the same triangles in the same order
    size_t k = 0, bad = 0;
    for (auto f : m.faces()) {
        int ids[3], i = 0;
        for (auto v : m.fv_range(f)) ids[i++] = v.idx();
        // same cyclic order; the start may differ
        bool ok = false;
        for (int r = 0; r < 3 && !ok; ++r)
            ok = a.idx[3 * k] == uint32_t(ids[r]) && a.idx[3 * k + 1] == uint32_t(ids[(r + 1) % 3]) && a.idx[3 * k + 2] == uint32_t(ids[(r + 2) % 3]);
        bad += !ok;
        ++k;
    }
    CHECK(bad == 0);
}

int main(int argc, char** argv)
{
    const std::string bunny = argc > 1 ? argv[1] : "data/bunny.ply";
    TriMesh m;
    CHECK(loadMesh(bunny, m));

    FlatMesh f;
    FlatLoadInfo info;
    std::string err;
    CHECK(loadFlatMesh(bunny, f, &info, &err));
    same(f, m, 1e-6, "ascii ply with extra properties");

    // binary little endian written by us
    const std::string tmp = "/tmp/vsa_flatload_test";
    CHECK(writePlyMesh(tmp + ".ply", m, true));
    FlatMesh fb;
    CHECK(loadFlatMesh(tmp + ".ply", fb, &info, &err));
    CHECK(info.format == "ply binary_little_endian");
    same(fb, m, 1e-6, "binary little endian ply");

    CHECK(writeObjMesh(tmp + ".obj", m));
    FlatMesh fo;
    CHECK(loadFlatMesh(tmp + ".obj", fo, &info, &err));
    same(fo, m, 1e-5, "obj");

    // big endian, with a quad (fanned into two triangles), a face with an index out of range and an extra
    // per-face property before the list
    {
        auto be32 = [](std::string& s, uint32_t v) { for (int i = 3; i >= 0; --i) s.push_back(char((v >> (8 * i)) & 0xff)); };
        auto bef = [&](std::string& s, float x) { uint32_t u; std::memcpy(&u, &x, 4); be32(s, u); };
        std::string body;
        const float vs[5][3] = {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}, {2, 2, 2}};
        for (auto& v : vs) for (float c : v) bef(body, c);
        // face 1: quad 0 1 2 3, flag 7; face 2: triangle 1 2 9 (bad); face 3: triangle 2 3 4
        auto face = [&](uint8_t flag, std::vector<uint32_t> idx) {
            body.push_back(char(flag));
            body.push_back(char(idx.size()));
            for (uint32_t v : idx) be32(body, v);
        };
        face(7, {0, 1, 2, 3});
        face(1, {1, 2, 9});
        face(2, {2, 3, 4});
        std::string file = "ply\nformat binary_big_endian 1.0\nelement vertex 5\nproperty float x\nproperty float y\nproperty float z\n"
                           "element face 3\nproperty uchar flag\nproperty list uchar uint vertex_indices\nend_header\n" + body;
        std::ofstream(tmp + "_be.ply", std::ios::binary).write(file.data(), std::streamsize(file.size()));
        FlatMesh fbe;
        CHECK(loadFlatMesh(tmp + "_be.ply", fbe, &info, &err));
        CHECK(fbe.vertexCount() == 5);
        CHECK(fbe.faceCount() == 3);   // quad -> 2, bad one skipped, triangle -> 1
        CHECK(info.skippedFaces == 1);
        CHECK(fbe.pos[3 * 4 + 2] == 2.0f);
        CHECK(fbe.idx[3] == 0 && fbe.idx[4] == 2 && fbe.idx[5] == 3);
    }

    // texture coordinates, materials
    {
        { std::ofstream mtl(tmp + "_tex.mtl"); mtl << "newmtl wall\nmap_Kd wall.png\nnewmtl floor\nmap_Kd floor.png\n"; }
        {
            std::ofstream o(tmp + "_tex.obj");
            o << "mtllib vsa_flatload_test_tex.mtl\nv 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0\n"
                 "vt 0 0\nvt 1 0\nvt 1 1\nvt 0 1\nusemtl wall\nf 1/1 2/2 3/3 4/4\nusemtl floor\nf 1/1 3/3 4\n";
        }
        FlatMesh fm;
        FlatSurface sf;
        CHECK(loadFlatMesh(tmp + "_tex.obj", fm, nullptr, &err, &sf));
        CHECK(fm.faceCount() == 3);   // a quad (two triangles) and a triangle
        CHECK(sf.cornerUV.size() == 18 && sf.faceMaterial.size() == 3 && sf.materials.size() == 2);
        CHECK(sf.faceMaterial[0] == 0 && sf.faceMaterial[1] == 0 && sf.faceMaterial[2] == 1);
        CHECK(sf.materials[1].diffuseMap == "floor.png");
        CHECK(sf.cornerUV[2] == 1.f && sf.cornerUV[3] == 0.f);          // face 0, corner 1: vt 2
        CHECK(sf.cornerUV[6 + 2] == 1.f && sf.cornerUV[6 + 3] == 1.f);  // face 1 = fan (0, 2, 3): corner 1 is vt 3 -> (1, 1)
        CHECK(sf.cornerUV[6 + 4] == 0.f && sf.cornerUV[6 + 5] == 1.f);  //                          corner 2 is vt 4 -> (0, 1)
        CHECK(std::isnan(sf.cornerUV[12 + 4]));                          // face 2, corner 2 has no vt
    }
    {
        std::ofstream o(tmp + "_vuv.ply");
        o << "ply\nformat ascii 1.0\ncomment TextureFile skin.png\nelement vertex 3\nproperty float x\nproperty float y\nproperty float z\n"
             "property float s\nproperty float t\nelement face 1\nproperty list uchar int vertex_indices\nend_header\n"
             "0 0 0 0.1 0.2\n1 0 0 0.3 0.4\n0 1 0 0.5 0.6\n3 0 1 2\n";
        o.close();
        FlatMesh fm;
        FlatSurface sf;
        CHECK(loadFlatMesh(tmp + "_vuv.ply", fm, nullptr, &err, &sf));
        CHECK(sf.textureFile == "skin.png");
        CHECK(sf.cornerUV.size() == 6 && std::abs(sf.cornerUV[2] - 0.3f) < 1e-6f && std::abs(sf.cornerUV[5] - 0.6f) < 1e-6f);
    }
    {
        // per face corner texture coordinates, binary: two triangles as one quad
        std::string body;
        auto f32 = [&](float x) { body.append(reinterpret_cast<const char*>(&x), 4); };
        auto i32 = [&](int32_t x) { body.append(reinterpret_cast<const char*>(&x), 4); };
        for (float c : {0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 1.f, 1.f, 0.f, 0.f, 1.f, 0.f}) f32(c);
        body.push_back(char(4)); for (int v : {0, 1, 2, 3}) i32(v);
        body.push_back(char(8)); for (float c : {0.f, 0.f, 1.f, 0.f, 1.f, 1.f, 0.f, 1.f}) f32(c);
        std::string file = "ply\nformat binary_little_endian 1.0\nelement vertex 4\nproperty float x\nproperty float y\nproperty float z\n"
                           "element face 1\nproperty list uchar int vertex_indices\nproperty list uchar float texcoord\nend_header\n" + body;
        std::ofstream(tmp + "_fuv.ply", std::ios::binary).write(file.data(), std::streamsize(file.size()));
        FlatMesh fm;
        FlatSurface sf;
        CHECK(loadFlatMesh(tmp + "_fuv.ply", fm, nullptr, &err, &sf));
        CHECK(fm.faceCount() == 2 && sf.cornerUV.size() == 12);
        // fan (0, 1, 2) and (0, 2, 3): corners take the texcoords 0 1 2 and 0 2 3
        CHECK(sf.cornerUV[0] == 0.f && sf.cornerUV[2] == 1.f && sf.cornerUV[3] == 0.f && sf.cornerUV[4] == 1.f && sf.cornerUV[5] == 1.f);
        CHECK(sf.cornerUV[6 + 2] == 1.f && sf.cornerUV[6 + 3] == 1.f && sf.cornerUV[6 + 4] == 0.f && sf.cornerUV[6 + 5] == 1.f);
    }

    // errors
    FlatMesh x;
    CHECK(!loadFlatMesh("/nonexistent/file.ply", x, nullptr, &err));
    CHECK(!loadFlatMesh(tmp + ".obj.nothing", x, nullptr, &err));

    // normalization agrees with the TriMesh one
    FlatMesh fn = fb;
    normalizeFlatToUnitBox(fn);
    TriMesh mn = m;
    normalizeToUnitBox(mn);
    double worst = 0;
    for (auto v : mn.vertices())
        for (int c = 0; c < 3; ++c) worst = std::max(worst, std::abs(fn.pos[3 * size_t(v.idx()) + c] - mn.point(v)[c]));
    std::printf("normalization: worst difference %.2e\n", worst);
    CHECK(worst < 1e-6);

    if (failures) { std::printf("%d failures\n", failures); return 1; }
    std::printf("flat loader: OK\n");
    return 0;
}
