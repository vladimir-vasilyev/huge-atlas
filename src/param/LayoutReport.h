#pragma once

#include "param/AtlasBake.h"
#include "param/MipSafety.h"
#include "param/UVMetrics.h"

#include <string>

// What `compare` of a session measured: the layout of the file (if it had one) and the new atlas.
struct LayoutComparison {
    UVLayoutStats original, ours;
    MipSafetyResult mipOriginal, mipOurs;
    bool hadOriginal = false;
    RetextureReport retexture;   // how well the new texture reproduces the old one (only when the file had a texture and it was baked)
    bool hasRetexture = false;
};

// The measures of the original UV layout and of the new atlas side by side, as monospaced text (three columns).
// hadOriginal = false: the file has no layout, only the new atlas is listed.
std::string formatLayoutComparison(const UVLayoutStats& original, const UVLayoutStats& ours, const MipSafetyResult& mipOriginal,
                                   const MipSafetyResult& mipOurs, bool hadOriginal);
