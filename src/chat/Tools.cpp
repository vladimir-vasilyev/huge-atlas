#include "chat/Tools.h"

#include <algorithm>
#include <cstdlib>
#include <filesystem>

namespace {

Json prop(const char* type, const char* description)
{
    Json p = Json::object();
    p["type"] = type;
    p["description"] = description;
    return p;
}

Json tool(const char* name, const char* description, Json properties, std::initializer_list<const char*> required)
{
    Json params = Json::object();
    params["type"] = "object";
    params["properties"] = std::move(properties);
    Json req = Json::array();
    for (const char* r : required) req.push_back(r);
    params["required"] = std::move(req);
    Json fn = Json::object();
    fn["name"] = name;
    fn["description"] = description;
    fn["parameters"] = std::move(params);
    Json t = Json::object();
    t["type"] = "function";
    t["function"] = std::move(fn);
    return t;
}

} // namespace

const char* chatSystemPrompt()
{
    return
        "You help to make a UV atlas for a triangle mesh in an application. The mesh is opened by the user (get_state tells what is open); "
        "open_mesh opens another file. make_atlas runs the whole pipeline (it takes seconds to minutes) and returns the numbers; read them, "
        "and when they are poor change the settings and run it again; repack changes only the texture size / mip levels (fast). "
        "save_atlas writes the result into the output folder.\n"
        "What the numbers mean:\n"
        "- efficiency: the share of the texture that holds charts; below 0.35 means too many small charts or a big mip gap.\n"
        "- unresolved_patches must be 0 (else raise max_depth, or patches, or quality); faces_without_uv must be 0.\n"
        "- l2_p95 and conformal_p95 (get_layout): distortion, 1.0 is perfect; above 1.3 the charts are stretched: more patches (quality 2).\n"
        "- seam_ratio: the share of the edges that are seams; it grows with the number of patches.\n"
        "- mip_levels: how many mip levels stay clean; the gap between charts is 2 * (ceil(1.5 * 2^L) - 1) texels (L = 0..5: 2, 4, 10, 22, 46, 94).\n"
        "More patches (quality 2, or patches=N): less distortion, more seams; fewer patches: the opposite. A bigger texture_size or fewer "
        "mip_levels: better efficiency. Answer briefly; say what you changed and why.";
}

Json chatToolSpecs()
{
    Json specs = Json::array();
    {
        Json p = Json::object();
        p["path"] = prop("string", "Path of an OBJ or PLY file");
        specs.push_back(tool("open_mesh", "Open a mesh file in the application (it replaces the open one).", std::move(p), {"path"}));
    }
    {
        Json p = Json::object();
        p["quality"] = prop("integer", "0 fast, 1 balanced, 2 detailed: how many patches to start from");
        p["patches"] = prop("integer", "Set the number of patches yourself (overrides quality)");
        p["texture_size"] = prop("integer", "Atlas size in texels: 256, 512, 1024, 2048, 4096 or 8192");
        p["mip_levels"] = prop("integer", "0..5: how many mip levels stay clean (the gap between charts follows from it)");
        p["smoothing"] = prop("number", "Border smoothness of the patches, 0 = off (default 0.1)");
        p["normal_iterations"] = prop("integer", "Passes of smoothing of the normal field before the segmentation (default 0)");
        p["normal_sigma"] = prop("number", "Angle of that smoothing in degrees (default 25)");
        p["lloyd_iterations"] = prop("integer", "Iterations of the segmentation (default 30)");
        p["max_depth"] = prop("integer", "How many times a bad patch may be split again (default 10)");
        p["allow_lscm"] = prop("boolean", "LSCM before the last resort (default true)");
        p["repair_near_misses"] = prop("boolean", "Repair a map with a few flips locally (default true)");
        p["simplify_percent"] = prop("number", "Big scans only: size of the simplified copy in percent (0 = by quality)");
        specs.push_back(tool("make_atlas",
                             "Make the atlas of the open mesh with these settings (the ones you leave out keep their values from the application). "
                             "Blocks until it is done and returns the numbers: patches, unresolved_patches, efficiency, texture_size, gutter_texels, seconds. "
                             "Read them: efficiency below 0.35 = too many small charts or a big mip gap; unresolved_patches must be 0; more patches mean "
                             "less distortion and more seams.",
                             std::move(p), {}));
    }
    {
        Json p = Json::object();
        p["texture_size"] = prop("integer", "New atlas size");
        p["mip_levels"] = prop("integer", "New number of clean mip levels (leave out to keep)");
        specs.push_back(tool("repack", "Pack and bake again with another texture size / mip levels; the segmentation and the unfolding stay (fast).",
                             std::move(p), {"texture_size"}));
    }
    specs.push_back(tool("get_state", "What is open and what was made: the file, faces, texture, the settings of the application and the numbers of the atlas.",
                         Json::object(), {}));
    specs.push_back(tool("get_layout",
                         "Distortion (l2_*, conformal_p95), seams, texel density, flipped faces and mip safety of the new atlas, and of the old layout of the file "
                         "for comparison (null when the file had no texture coordinates). Needs an atlas.",
                         Json::object(), {}));
    specs.push_back(tool("get_report", "What the last run did, as text: patches, how they were unfolded, splits, packing.", Json::object(), {}));
    {
        Json p = Json::object();
        p["file_name"] = prop("string", "File name with .obj or .ply, no folders");
        specs.push_back(tool("save_atlas", "Write the mesh with the atlas coordinates and <name>_atlas.png into the output folder.", std::move(p), {"file_name"}));
    }
    return specs;
}

Json layoutToJson(const LayoutComparison& c)
{
    auto one = [](const UVLayoutStats& l, const MipSafetyResult& m) {
        Json d = Json::object();
        d["charts"] = l.charts;
        d["seam_ratio"] = l.seamRatio();
        d["flipped_faces"] = l.flipped;
        d["faces_without_uv"] = l.facesWithoutUV;
        d["l2_mean"] = l.l2Mean;
        d["l2_p95"] = l.l2P95;
        d["l2_p99"] = l.l2P99;
        d["conformal_p95"] = l.conformalP95;
        d["density_p05"] = l.densityP05;
        d["density_p50"] = l.densityP50;
        d["density_p95"] = l.densityP95;
        d["covered"] = l.covered;
        d["overlap"] = l.overlap;
        d["safe_mip_level"] = m.safeLevel;
        d["gap_min_texels"] = m.gapMin;
        d["gap_p05_texels"] = m.gapP05;
        d["level_by_gap_p05"] = m.levelByGapP05;
        return d;
    };
    Json out = Json::object();
    out["new"] = one(c.ours, c.mipOurs);
    out["original"] = c.hadOriginal ? one(c.original, c.mipOriginal) : Json();
    if (c.hasRetexture) { out["retexture_psnr_db"] = c.retexture.psnr; out["retexture_psnr_full_db"] = c.retexture.psnrFull; }
    return out;
}

std::string chatOutputDirectory()
{
    const char* env = std::getenv("HUGE_ATLAS_OUTPUT");
    const char* home = std::getenv("HOME");
    std::filesystem::path dir = env && *env ? std::filesystem::path(env) : std::filesystem::path(home ? home : ".") / "Documents" / "huge-atlas" / "output";
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    return dir.string();
}

std::string chatSafeFileName(const std::string& requested)
{
    std::string name = std::filesystem::path(requested).filename().string();   // no folders: only the output folder is written to
    if (name.empty() || name == "." || name == "..") name = "atlas";
    std::string lower = name;
    std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char c) { return char(std::tolower(c)); });
    const bool ok = lower.size() > 4 && (lower.compare(lower.size() - 4, 4, ".obj") == 0 || lower.compare(lower.size() - 4, 4, ".ply") == 0);
    return ok ? name : name + ".obj";
}

Json callTool(AtlasHost& host, const std::string& name, const Json& args)
{
    auto error = [](const std::string& m) { Json e = Json::object(); e["error"] = m; return e; };
    if (name == "open_mesh") {
        const std::string path = args.getString("path");
        if (path.empty()) return error("path is required");
        return host.openMesh(path);
    }
    if (name == "make_atlas") return host.makeAtlas(args);
    if (name == "repack") {
        if (!args.has("texture_size")) return error("texture_size is required");
        return host.repack(args.getInt("texture_size"), args.has("mip_levels") ? args.getInt("mip_levels") : -1);
    }
    if (name == "get_state") return host.state();
    if (name == "get_layout") return host.layout();
    if (name == "get_report") return host.report();
    if (name == "save_atlas") {
        const std::string f = args.getString("file_name");
        if (f.empty()) return error("file_name is required");
        return host.save(f);
    }
    return error("unknown tool " + name);
}
