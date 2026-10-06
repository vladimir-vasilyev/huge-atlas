#pragma once

#include <string>
#include <vector>

// Decodes an image file (JPEG, PNG, ...) into RGBA8. Rows are flipped so that row 0 is the bottom of the
// picture, which is where v = 0 is in OBJ and OpenGL texture coordinates.
bool loadImageRGBA(const std::string& path, int& width, int& height, std::vector<unsigned char>& pixels,
                   std::string* error = nullptr);
