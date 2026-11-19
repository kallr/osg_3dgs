#ifndef SPARK_BUFFER_IMAGE_LAYOUT_H
#define SPARK_BUFFER_IMAGE_LAYOUT_H

#include <algorithm>
#include <cstddef>
#include <limits>
#include <stdexcept>

namespace SparkBufferImage {

struct Layout {
    int width;
    int height;
    size_t bytes;
};

// OSG 3.6 computes row widths in signed 32-bit *bits*, even on x64. Keep
// rows small and store the linear TBO payload across contiguous image rows.
// Its total image byte count is unsigned 32-bit, so reject larger buffers.
inline Layout layout(size_t texels, size_t bytesPerTexel)
{
    const size_t maxBytes = std::numeric_limits<unsigned int>::max();
    const size_t maxRowBytes = size_t(std::numeric_limits<int>::max()) / 8;
    if (!texels || !bytesPerTexel || bytesPerTexel > maxRowBytes)
        throw std::invalid_argument("Invalid texture buffer image dimensions");
    if (texels > maxBytes / bytesPerTexel)
        throw std::length_error("Texture buffer exceeds OSG image byte capacity");
    const size_t width = std::min(texels, std::min(size_t(65536), maxRowBytes / bytesPerTexel));
    const size_t height = 1 + (texels - 1) / width;
    if (height > maxBytes / bytesPerTexel / width)
        throw std::length_error("Padded texture buffer exceeds OSG image byte capacity");
    return { int(width), int(height), width * height * bytesPerTexel };
}

} // namespace SparkBufferImage

#endif
