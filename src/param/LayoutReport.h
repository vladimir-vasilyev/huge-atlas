#pragma once

#include "param/MipSafety.h"
#include "param/UVMetrics.h"

#include <string>

// The measures of the original UV layout and of the new atlas side by side, as monospaced text (three columns).
// hadOriginal = false: the file has no layout, only the new atlas is listed.
std::string formatLayoutComparison(const UVLayoutStats& original, const UVLayoutStats& ours, const MipSafetyResult& mipOriginal,
                                   const MipSafetyResult& mipOurs, bool hadOriginal);
