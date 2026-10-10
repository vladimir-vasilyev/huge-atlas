#include "app/Paths.h"

#include <mach-o/dyld.h>

#include <cstdlib>
#include <filesystem>
#include <vector>

namespace fs = std::filesystem;

namespace paths {

namespace {

std::string executablePath()
{
    uint32_t size = 0;
    _NSGetExecutablePath(nullptr, &size);
    std::vector<char> buf(size + 1, 0);
    if (_NSGetExecutablePath(buf.data(), &size) != 0) return {};
    std::error_code ec;
    return fs::weakly_canonical(buf.data(), ec).string();
}

std::string homeDir()
{
    const char* h = std::getenv("HOME");
    return h ? h : "/tmp";
}

} // namespace

bool runningFromBundle()
{
    static const bool bundled = executablePath().find(".app/Contents/MacOS/") != std::string::npos;
    return bundled;
}

std::string resourcesDir()
{
    if (!runningFromBundle()) return {};
    return (fs::path(executablePath()).parent_path().parent_path() / "Resources").string();
}

std::string userDataDir()
{
    if (!runningFromBundle()) return ".";
    const fs::path dir = fs::path(homeDir()) / "Library" / "Application Support" / "huge-atlas";
    std::error_code ec;
    fs::create_directories(dir, ec);
    return dir.string();
}

std::string debugOutDir()
{
    return runningFromBundle() ? (fs::path(homeDir()) / "Documents" / "huge-atlas" / "DebugOut").string() : "DebugOut";
}

std::string defaultMesh()
{
    std::error_code ec;
    if (fs::exists("data/cube.obj", ec)) return "data/cube.obj";
    const std::string res = resourcesDir();
    return res.empty() ? "data/cube.obj" : (fs::path(res) / "data" / "cube.obj").string();
}

std::string startDir()
{
    return runningFromBundle() ? (fs::path(homeDir()) / "Documents").string() : ".";
}

} // namespace paths
