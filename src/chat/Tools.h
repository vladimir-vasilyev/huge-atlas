#pragma once

#include "chat/Json.h"
#include "param/LayoutReport.h"

#include <string>

// What a language model can do in the application. The application implements AtlasHost (the calls come from the thread of the
// agent: the host moves them to its own thread where it must); `callTool` turns a tool call of the model into host calls.
class AtlasHost {
public:
    virtual ~AtlasHost() = default;
    // Each returns a JSON object; {"error": "..."} when it did not work.
    virtual Json openMesh(const std::string& path) = 0;
    // settings: quality, patches, texture_size, mip_levels and the developer options (see the tool specification); blocks until the run ends
    virtual Json makeAtlas(const Json& settings) = 0;
    virtual Json repack(int textureSize, int mipLevels) = 0;     // mipLevels < 0: keep
    virtual Json state() = 0;                                    // the open mesh, the settings, the numbers of the atlas
    virtual Json layout() = 0;                                   // distortion, seams, mip safety of the new atlas and of the old layout
    virtual Json report() = 0;                                   // {"report": text}
    virtual Json save(const std::string& fileName) = 0;          // into the output folder only
    virtual Json cancelRun() { return Json(); }
};

// The tools as the "tools" array of an OpenAI-compatible request.
Json chatToolSpecs();
// The instructions for the model (what the numbers mean and what to turn).
const char* chatSystemPrompt();
// Runs one tool call; unknown tools and bad arguments come back as {"error": ...}.
Json callTool(AtlasHost& host, const std::string& name, const Json& args);

// Helpers for the hosts: the numbers of the layout comparison as JSON.
Json layoutToJson(const LayoutComparison& c);
// The folder the application writes to (created when missing): ~/Documents/huge-atlas/output
std::string chatOutputDirectory();
// A file name without folders, with the extension .obj or .ply
std::string chatSafeFileName(const std::string& requested);
