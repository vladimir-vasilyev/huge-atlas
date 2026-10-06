#include "render/ImageFile.h"

// stb_image (public domain, shipped with ImGuiFileDialog, whose thumbnails are off): the implementation lives here
#define STB_IMAGE_IMPLEMENTATION
#define STBI_NO_GIF
#define STBI_NO_PSD
#define STBI_NO_PIC
#define STBI_NO_PNM
#define STBI_NO_HDR
#include "stb/stb_image.h"

#include <algorithm>

bool loadImageRGBA(const std::string& path, int& w, int& h, std::vector<unsigned char>& pixels, std::string* error)
{
    int channels = 0;
    unsigned char* data = stbi_load(path.c_str(), &w, &h, &channels, 4);
    if (!data) {
        if (error) *error = std::string("cannot decode the image: ") + path + " (" + stbi_failure_reason() + ")";
        return false;
    }
    pixels.resize(size_t(w) * h * 4);
    const size_t row = size_t(w) * 4;
    for (int y = 0; y < h; ++y)   // flip: row 0 of the result is the bottom of the picture
        std::copy(data + size_t(h - 1 - y) * row, data + size_t(h - y) * row, pixels.begin() + size_t(y) * row);
    stbi_image_free(data);
    return true;
}
