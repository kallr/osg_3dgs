#include "SparkGpuResources.h"
#include "GaussianDrawObj.h"
#include "SparkPackedEncoding.h"
#include "SparkBufferImageLayout.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <limits>

#include <osg/TextureBuffer>
#include <osg/State>
#include <osg/GLExtensions>
#include <osg/GraphicsContext>

#ifndef GL_RED_INTEGER
#define GL_RED_INTEGER 0x8D94
#endif
#ifndef GL_R32I
#define GL_R32I 0x8235
#endif
#ifndef GL_RGBA_INTEGER
#define GL_RGBA_INTEGER 0x8D99
#endif
#ifndef GL_RGBA32UI
#define GL_RGBA32UI 0x8D70
#endif

namespace
{
    void allocateBufferImage(osg::Image& image, size_t texels, GLenum format, GLenum type)
    {
        const size_t bytesPerTexel = osg::Image::computePixelSizeInBits(format, type) / 8;
        const auto dimensions = SparkBufferImage::layout(texels, bytesPerTexel);
        image.allocateImage(dimensions.width, dimensions.height, 1, format, type);
        if (!image.data() || image.getTotalSizeInBytes() != dimensions.bytes)
            throw std::runtime_error("OSG texture buffer image allocation has an invalid size");
    }

    bool validImageRange(const osg::Image* image, size_t offset, size_t bytes)
    {
        return image && image->data() && offset <= image->getTotalSizeInBytes() &&
            bytes <= image->getTotalSizeInBytes() - offset;
    }

    void writeCovSplatTexels(const MI_GaussianPoint& g, float* dst)
    {
        float qx = g.rotation.x(), qy = g.rotation.y(), qz = g.rotation.z(), qw = g.rotation.w();
        float x2 = qx + qx, y2 = qy + qy, z2 = qz + qz;
        float xx = qx * x2, xy = qx * y2, xz = qx * z2;
        float yy = qy * y2, yz = qy * z2, zz = qz * z2;
        float wx = qw * x2, wy = qw * y2, wz = qw * z2;

        float R00 = 1.0f - (yy + zz), R01 = xy + wz,          R02 = xz - wy;
        float R10 = xy - wz,          R11 = 1.0f - (xx + zz), R12 = yz + wx;
        float R20 = xz + wy,          R21 = yz - wx,          R22 = 1.0f - (xx + yy);

        float sx = g.scale.x(), sy = g.scale.y(), sz = g.scale.z();

        float RS00 = R00 * sx, RS01 = R01 * sy, RS02 = R02 * sz;
        float RS10 = R10 * sx, RS11 = R11 * sy, RS12 = R12 * sz;
        float RS20 = R20 * sx, RS21 = R21 * sy, RS22 = R22 * sz;

        float cov_xx = RS00 * RS00 + RS01 * RS01 + RS02 * RS02;
        float cov_yy = RS10 * RS10 + RS11 * RS11 + RS12 * RS12;
        float cov_zz = RS20 * RS20 + RS21 * RS21 + RS22 * RS22;
        float cov_xy = RS00 * RS10 + RS01 * RS11 + RS02 * RS12;
        float cov_xz = RS00 * RS20 + RS01 * RS21 + RS02 * RS22;
        float cov_yz = RS10 * RS20 + RS11 * RS21 + RS12 * RS22;

        dst[0] = cov_xx; dst[1] = cov_yy; dst[2] = cov_zz; dst[3] = cov_xy;
        dst[4] = cov_xz; dst[5] = cov_yz; dst[6] = 0.0f;   dst[7] = 0.0f;
    }
}

int SparkGpuResources::nextPowerOfTwo(int value)
{
    int power = 1;
    while (power < value)
        power <<= 1;
    return power;
}

void SparkGpuResources::setRadMode(bool enabled)
{
    _isRadMode = enabled;
}

void SparkGpuResources::setSplatEncoding(SplatEncoding encoding)
{
    _splatEncoding = encoding;
}

int SparkGpuResources::packedTexelsPerSplat() const
{
    if (_splatEncoding == SplatEncoding::PackedSplat) return 1;
    if (_splatEncoding == SplatEncoding::ExtSplat) return 2;
    return 0;
}

void SparkGpuResources::createPackedDataTexture(int initialCapacity)
{
    initialCapacity = std::max(1, initialCapacity);
    _packedDataTextureCapacity = initialCapacity;

    int texelsPerSplat = packedTexelsPerSplat();
    if (texelsPerSplat <= 0) {
        std::cerr << "createPackedDataTexture requires PackedSplat or ExtSplat encoding." << std::endl;
        return;
    }
    size_t width = size_t(texelsPerSplat) * initialCapacity;

    osg::ref_ptr<osg::Image> image = new osg::Image;
    allocateBufferImage(*image, width, GL_RGBA_INTEGER, GL_UNSIGNED_INT);
    unsigned int* ptr = reinterpret_cast<unsigned int*>(image->data(0));
    if (ptr)
        std::fill(ptr, ptr + width * 4, 0u);

    // Assign a PixelBufferObject so we can map it or use glBufferSubData
    osg::ref_ptr<osg::PixelBufferObject> pbo = new osg::PixelBufferObject(image.get());
    pbo->setUsage(GL_DYNAMIC_DRAW);
    image->setPixelBufferObject(pbo.get());

    osg::ref_ptr<osg::TextureBuffer> tbo = new osg::TextureBuffer;
    tbo->setImage(image.get());
    tbo->setInternalFormat(GL_RGBA32UI);
    _packedDataTexture = tbo;
}

void SparkGpuResources::updatePackedDataRange(const std::vector<MI_GaussianPoint>& points, int firstSlot, int endSlot, bool isRadFile)
{
    osg::Image* image = packedDataImage();
    if (!image)
        return;
    firstSlot = std::max(0, firstSlot);
    endSlot = std::min(endSlot, (int)points.size());
    if (endSlot > _packedDataTextureCapacity) {
        std::cerr << "WARNING: packed data update exceeds capacity: endSlot=" << endSlot
                  << ", capacity=" << _packedDataTextureCapacity
                  << ", residentPoints=" << points.size()
                  << std::endl;
        endSlot = _packedDataTextureCapacity;
    }
    if (firstSlot >= endSlot)
        return;

    unsigned int* basePtr = reinterpret_cast<unsigned int*>(image->data(0));
    if (!basePtr)
        return;

    int texelsPerSplat = packedTexelsPerSplat();

    for (int i = firstSlot; i < endSlot; i++) {
        const MI_GaussianPoint& g = points[i];
        float alpha = g.color.a();
        
        float r = g.color.r();
        float gr = g.color.g();
        float b = g.color.b();

        if (!isRadFile) {
            alpha = std::max(0.0f, std::min(1.0f, alpha));
            float colorAlpha = std::min(alpha, 1.0f);
            r *= colorAlpha;
            gr *= colorAlpha;
            b *= colorAlpha;
        }

        if (_splatEncoding == SplatEncoding::PackedSplat) {
            SparkPacked::PackedSplat ps;
            ps.encode(g.position.x(), g.position.y(), g.position.z(),
                      g.scale.x(), g.scale.y(), g.scale.z(),
                      g.rotation.x(), g.rotation.y(), g.rotation.z(), g.rotation.w(),
                      r, gr, b, alpha);
            unsigned int* dst = basePtr + size_t(i) * texelsPerSplat * 4;
            std::memcpy(dst, ps.words, sizeof(ps.words));
        }
        else if (_splatEncoding == SplatEncoding::ExtSplat) {
            SparkPacked::ExtSplat es;
            es.encode(g.position.x(), g.position.y(), g.position.z(),
                      g.scale.x(), g.scale.y(), g.scale.z(),
                      g.rotation.x(), g.rotation.y(), g.rotation.z(), g.rotation.w(),
                      r, gr, b, alpha);
            unsigned int* dst = basePtr + size_t(i) * texelsPerSplat * 4;
            std::memcpy(dst, es.extA, sizeof(es.extA));
            std::memcpy(dst + 4, es.extB, sizeof(es.extB));
        }
    }
    image->dirty();
}

void SparkGpuResources::ensurePackedDataCapacity(const std::vector<MI_GaussianPoint>& points, int requiredSlots)
{
    if (_splatEncoding == SplatEncoding::Legacy)
        return;
    requiredSlots = std::max(1, requiredSlots);
    if (_packedDataTextureCapacity >= requiredSlots)
        return;

    int newCapacity = nextPowerOfTwo(requiredSlots);
    int texelsPerSplat = packedTexelsPerSplat();
    size_t width = size_t(texelsPerSplat) * newCapacity;
    int oldCapacity = _packedDataTextureCapacity;

    osg::ref_ptr<osg::Image> image = new osg::Image;
    allocateBufferImage(*image, width, GL_RGBA_INTEGER, GL_UNSIGNED_INT);
    unsigned int* ptr = reinterpret_cast<unsigned int*>(image->data(0));
    if (ptr)
        std::fill(ptr, ptr + width * 4, 0u);
    _packedDataTextureCapacity = newCapacity;
    _packedDataTexture->setImage(image.get());
    updatePackedDataRange(points, 0, (int)points.size(), true);
    std::cout << "RAD GPU resource grow: packedDataCapacity "
              << oldCapacity << " -> " << newCapacity
              << ", residentPoints=" << points.size()
              << std::endl;
}

void SparkGpuResources::createIndexTexture(const std::vector<int>& indices, int initialCapacity)
{
    initialCapacity = std::max(1, initialCapacity);
    _indexTextureCapacity = initialCapacity;

    osg::ref_ptr<osg::Image> image = new osg::Image;
    allocateBufferImage(*image, size_t(initialCapacity), GL_RED_INTEGER, GL_INT);
    int* ptr = reinterpret_cast<int*>(image->data(0));
    if (ptr)
        std::fill(ptr, ptr + initialCapacity, -1);
    osg::ref_ptr<osg::TextureBuffer> tbo = new osg::TextureBuffer;
    tbo->setImage(image.get());
    tbo->setInternalFormat(GL_R32I);
    _indexTexture = tbo;
    updateIndexRange(indices, 0, (int)indices.size());
}

void SparkGpuResources::createDataTexture(const std::vector<MI_GaussianPoint>& points, int initialCapacity, bool isRadFile)
{
    initialCapacity = std::max(1, initialCapacity);
    _dataTextureCapacity = initialCapacity;

    osg::ref_ptr<osg::Image> image = new osg::Image;
    allocateBufferImage(*image, size_t(4) * initialCapacity, GL_RGBA, GL_FLOAT);

    osg::ref_ptr<osg::TextureBuffer> tbo = new osg::TextureBuffer;
    tbo->setImage(image.get());
    tbo->setInternalFormat(GL_RGBA32F_ARB);
    _dataTexture = tbo;
    updateDataRange(points, 0, (int)points.size(), isRadFile);
}

void SparkGpuResources::createSHTexture(const std::vector<MI_GaussianPoint>& points, int initialCapacity)
{
    initialCapacity = std::max(1, initialCapacity);
    _shTextureCapacity = initialCapacity;

    osg::ref_ptr<osg::Image> image = new osg::Image;
    allocateBufferImage(*image, size_t(12) * initialCapacity, GL_RGBA, GL_FLOAT);

    osg::ref_ptr<osg::TextureBuffer> tbo = new osg::TextureBuffer;
    tbo->setImage(image.get());
    tbo->setInternalFormat(GL_RGBA32F_ARB);
    _shTexture = tbo;
    updateSHRange(points, 0, (int)points.size());
}

void SparkGpuResources::createSortBuffers(int capacity, const std::vector<int>& activeIndices)
{
    capacity = std::max(1, capacity);

    _activeIndexBuffer = new osg::IntArray(capacity);
    std::fill(_activeIndexBuffer->begin(), _activeIndexBuffer->end(), -1);

    _activeIndexSSBO = new osg::ShaderStorageBufferObject;
    _activeIndexBuffer->setBufferObject(_activeIndexSSBO.get());
    _activeIndexBuffer->setPreserveDataType(true);
    updateActiveIndexBuffer(activeIndices, capacity);

    _depthKeyBuffer = new osg::UIntArray(capacity);
    std::fill(_depthKeyBuffer->begin(), _depthKeyBuffer->end(), 0u);

    _depthKeySSBO = new osg::ShaderStorageBufferObject;
    _depthKeyBuffer->setBufferObject(_depthKeySSBO.get());
    _depthKeyBuffer->setPreserveDataType(true);

    _gpuSortIndexBuffer = new osg::IntArray(capacity);
    std::fill(_gpuSortIndexBuffer->begin(), _gpuSortIndexBuffer->end(), 0);

    _gpuSortIndexSSBO = new osg::ShaderStorageBufferObject;
    _gpuSortIndexBuffer->setBufferObject(_gpuSortIndexSSBO.get());
    _gpuSortIndexBuffer->setPreserveDataType(true);
}

void SparkGpuResources::createTileBuffers(int tileCount, int tileListCapacity)
{
    tileCount = std::max(1, tileCount);
    tileListCapacity = std::max(1, tileListCapacity);

    _tileCountBuffer = new osg::UIntArray(tileCount);
    std::fill(_tileCountBuffer->begin(), _tileCountBuffer->end(), 0u);

    _tileCountSSBO = new osg::ShaderStorageBufferObject;
    _tileCountBuffer->setBufferObject(_tileCountSSBO.get());
    _tileCountBuffer->setPreserveDataType(true);

    _tileCursorBuffer = new osg::UIntArray(tileCount);
    std::fill(_tileCursorBuffer->begin(), _tileCursorBuffer->end(), 0u);

    _tileCursorSSBO = new osg::ShaderStorageBufferObject;
    _tileCursorBuffer->setBufferObject(_tileCursorSSBO.get());
    _tileCursorBuffer->setPreserveDataType(true);

    _tileListBuffer = new osg::IntArray(tileListCapacity);
    std::fill(_tileListBuffer->begin(), _tileListBuffer->end(), -1);

    _tileListSSBO = new osg::ShaderStorageBufferObject;
    _tileListBuffer->setBufferObject(_tileListSSBO.get());
    _tileListBuffer->setPreserveDataType(true);

    _tileStatsBuffer = new osg::UIntArray(4);
    std::fill(_tileStatsBuffer->begin(), _tileStatsBuffer->end(), 0u);

    _tileStatsSSBO = new osg::ShaderStorageBufferObject;
    _tileStatsBuffer->setBufferObject(_tileStatsSSBO.get());
    _tileStatsBuffer->setPreserveDataType(true);
}

void SparkGpuResources::updateActiveIndexBuffer(const std::vector<int>& activeIndices, int capacity)
{
    if (!_activeIndexBuffer)
        return;

    int count = std::min((int)activeIndices.size(), capacity);
    for (int i = 0; i < count; ++i)
        (*_activeIndexBuffer)[i] = activeIndices[i];
    for (int i = count; i < capacity; ++i)
        (*_activeIndexBuffer)[i] = -1;
    _activeIndexBuffer->dirty();
}

void SparkGpuResources::ensureIndexCapacity(const std::vector<int>& indices, int requiredIndices)
{
    if (!_isRadMode || !_indexTexture)
        return;
    requiredIndices = std::max(1, requiredIndices);
    if (_indexTextureCapacity >= requiredIndices)
        return;

    int newCapacity = nextPowerOfTwo(requiredIndices);
    osg::ref_ptr<osg::Image> image = new osg::Image;
    allocateBufferImage(*image, size_t(newCapacity), GL_RED_INTEGER, GL_INT);
    int* ptr = reinterpret_cast<int*>(image->data(0));
    if (ptr)
        std::fill(ptr, ptr + newCapacity, -1);
    _indexTextureCapacity = newCapacity;
    _indexTexture->setImage(image.get());
    updateIndexChangedRanges(indices, std::vector<int>());
}

void SparkGpuResources::ensureDataCapacity(const std::vector<MI_GaussianPoint>& points, int requiredSlots, bool hasSH)
{
    if (!_isRadMode)
        return;
    requiredSlots = std::max(1, requiredSlots);

    // Legacy path
    if (_splatEncoding == SplatEncoding::Legacy) {
        if (_dataTextureCapacity >= requiredSlots && (!hasSH || _shTextureCapacity >= requiredSlots))
            return;
        int oldDataCapacity = _dataTextureCapacity;
        int oldShCapacity = _shTextureCapacity;
        int newCapacity = nextPowerOfTwo(requiredSlots);
        if (_dataTexture) {
            osg::ref_ptr<osg::Image> dataImage = new osg::Image;
            allocateBufferImage(*dataImage, size_t(4) * newCapacity, GL_RGBA, GL_FLOAT);
            _dataTextureCapacity = newCapacity;
            _dataTexture->setImage(dataImage.get());
            updateDataRange(points, 0, (int)points.size(), true);
        }
        if (hasSH && _shTexture) {
            osg::ref_ptr<osg::Image> shImage = new osg::Image;
            allocateBufferImage(*shImage, size_t(12) * newCapacity, GL_RGBA, GL_FLOAT);
            _shTextureCapacity = newCapacity;
            _shTexture->setImage(shImage.get());
            updateSHRange(points, 0, (int)points.size());
        }
        std::cout << "RAD GPU resource grow: dataCapacity "
                  << oldDataCapacity << " -> " << _dataTextureCapacity
                  << ", shCapacity " << oldShCapacity << " -> " << _shTextureCapacity
                  << ", residentPoints=" << points.size()
                  << std::endl;
    }
    // Packed path
    else {
        ensurePackedDataCapacity(points, requiredSlots);
        // Also expand SH texture if needed (for RAD files with packed encoding + SH)
        if (hasSH && _shTexture && _shTextureCapacity < requiredSlots) {
            int newCapacity = nextPowerOfTwo(requiredSlots);
            osg::ref_ptr<osg::Image> shImage = new osg::Image;
            allocateBufferImage(*shImage, size_t(12) * newCapacity, GL_RGBA, GL_FLOAT);
            _shTextureCapacity = newCapacity;
            _shTexture->setImage(shImage.get());
            updateSHRange(points, 0, (int)points.size());
        }
    }
}

void SparkGpuResources::updateDataRange(const std::vector<MI_GaussianPoint>& points, int firstSlot, int endSlot, bool isRadFile)
{
    // Packed encoding path
    if (_splatEncoding != SplatEncoding::Legacy) {
        updatePackedDataRange(points, firstSlot, endSlot, isRadFile);
        return;
    }

    // Legacy path
    osg::Image* image = dataImage();
    if (!image)
        return;
    firstSlot = std::max(0, firstSlot);
    endSlot = std::min(endSlot, (int)points.size());
    if (endSlot > _dataTextureCapacity) {
        std::cerr << "WARNING: data update exceeds capacity: endSlot=" << endSlot
                  << ", capacity=" << _dataTextureCapacity
                  << ", residentPoints=" << points.size()
                  << std::endl;
        endSlot = _dataTextureCapacity;
    }
    if (firstSlot >= endSlot)
        return;

    const int stride = 4;
    osg::Vec4f* base = reinterpret_cast<osg::Vec4f*>(image->data());
    if (!base) return;
    for (int i = firstSlot; i < endSlot; i++) {
        const MI_GaussianPoint& g = points[i];
        osg::Vec4f* ptr = base + size_t(stride) * i;

        float alpha = g.color.a();
        float r = g.color.r();
        float gr = g.color.g();
        float b = g.color.b();

        if (!isRadFile) {
            alpha = std::max(0.0f, std::min(1.0f, alpha));
            float colorAlpha = std::min(alpha, 1.0f);
            r *= colorAlpha;
            gr *= colorAlpha;
            b *= colorAlpha;
        }

        ptr[0].set(g.position.x(), g.position.y(), g.position.z(), alpha);
        ptr[1].set(g.scale.x(), g.scale.y(), g.scale.z(), r);
        ptr[2].set(g.rotation.x(), g.rotation.y(), g.rotation.z(), gr);
        ptr[3].set(g.rotation.w(), (float)g.shOrder, 0.0f, b);
    }
    image->dirty();
}

void SparkGpuResources::updateSHRange(const std::vector<MI_GaussianPoint>& points, int firstSlot, int endSlot)
{
    osg::Image* image = shImage();
    if (!image)
        return;
    firstSlot = std::max(0, firstSlot);
    endSlot = std::min(endSlot, (int)points.size());
    if (endSlot > _shTextureCapacity) {
        std::cerr << "WARNING: SH update exceeds capacity: endSlot=" << endSlot
                  << ", capacity=" << _shTextureCapacity
                  << ", residentPoints=" << points.size()
                  << std::endl;
        endSlot = _shTextureCapacity;
    }
    if (firstSlot >= endSlot)
        return;

    const int shStride = 12;
    osg::Vec4f* base = reinterpret_cast<osg::Vec4f*>(image->data());
    if (!base) return;
    for (int i = firstSlot; i < endSlot; i++) {
        const MI_GaussianPoint& g = points[i];
        osg::Vec4f* ptr = base + size_t(shStride) * i;

        float* shPtr = reinterpret_cast<float*>(ptr);
        for (int k = 0; k < 45; ++k)
            shPtr[k] = g.sh[k];
        shPtr[45] = shPtr[46] = shPtr[47] = 0.0f;
    }
    image->dirty();
}

void SparkGpuResources::updateIndexRange(const std::vector<int>& indices, int firstIndex, int endIndex)
{
    osg::Image* image = indexImage();
    if (!image)
        return;
    firstIndex = std::max(0, firstIndex);
    int* ptr = reinterpret_cast<int*>(image->data(0));
    if (!ptr)
        return;
    endIndex = std::min(endIndex, _indexTextureCapacity);
    if (firstIndex >= endIndex)
        return;

    int validEnd = std::min(endIndex, (int)indices.size());
    for (int i = firstIndex; i < validEnd; i++)
        ptr[i] = indices[i];
    for (int i = validEnd; i < endIndex; i++)
        ptr[i] = -1;
    image->dirty();
}

void SparkGpuResources::updateIndexChangedRanges(const std::vector<int>& current, const std::vector<int>& previous)
{
    if (!indexImage())
        return;
    int oldCount = (int)previous.size();
    int newCount = (int)current.size();
    ensureIndexCapacity(current, newCount);
    int commonCount = std::min(oldCount, newCount);
    int firstChanged = commonCount;
    for (int i = 0; i < commonCount; ++i) {
        if (previous[i] != current[i]) {
            firstChanged = i;
            break;
        }
    }
    int lastChanged = commonCount - 1;
    while (lastChanged >= firstChanged && previous[lastChanged] == current[lastChanged])
        --lastChanged;
    if (firstChanged <= lastChanged)
        updateIndexRange(current, firstChanged, lastChanged + 1);
    if (newCount > oldCount)
        updateIndexRange(current, oldCount, newCount);
    else if (oldCount > newCount) {
        osg::Image* image = indexImage();
        int* ptr = image ? reinterpret_cast<int*>(image->data(0)) : nullptr;
        if (!ptr)
            return;
        int clearEnd = std::min(oldCount, _indexTextureCapacity);
        for (int i = newCount; i < clearEnd; ++i)
            ptr[i] = -1;
        image->dirty();
    }
}

void SparkGpuResources::createCovSplatTexture(int initialCapacity)
{
    initialCapacity = std::max(1, initialCapacity);
    _covSplatTextureCapacity = initialCapacity;

    // 2 texels RGBA32F per splat: texel0 = (xx, yy, zz, xy), texel1 = (xz, yz, 0, 0)
    size_t width = size_t(2) * initialCapacity;

    osg::ref_ptr<osg::Image> image = new osg::Image;
    allocateBufferImage(*image, width, GL_RGBA, GL_FLOAT);
    float* ptr = reinterpret_cast<float*>(image->data(0));
    if (ptr)
        std::fill(ptr, ptr + width * 4, 0.0f);

    osg::ref_ptr<osg::TextureBuffer> tbo = new osg::TextureBuffer;
    tbo->setImage(image.get());
    tbo->setInternalFormat(GL_RGBA32F_ARB);
    _covSplatTexture = tbo;
}

void SparkGpuResources::ensureCovSplatCapacity(const std::vector<MI_GaussianPoint>& points, int requiredSlots)
{
    if (!_isRadMode || !_covSplatTexture)
        return;

    requiredSlots = std::max(1, requiredSlots);
    if (_covSplatTextureCapacity >= requiredSlots)
        return;

    int oldCapacity = _covSplatTextureCapacity;
    int newCapacity = nextPowerOfTwo(requiredSlots);
    size_t width = size_t(2) * newCapacity;

    osg::ref_ptr<osg::Image> image = new osg::Image;
    allocateBufferImage(*image, width, GL_RGBA, GL_FLOAT);
    float* ptr = reinterpret_cast<float*>(image->data(0));
    if (ptr)
        std::fill(ptr, ptr + width * 4, 0.0f);

    _covSplatTextureCapacity = newCapacity;
    _covSplatTexture->setImage(image.get());
    updateCovSplatRange(points, 0, (int)points.size());

    std::cout << "RAD GPU resource grow: covSplatCapacity "
              << oldCapacity << " -> " << newCapacity
              << ", residentPoints=" << points.size()
              << std::endl;
}

void SparkGpuResources::updateCovSplatRange(const std::vector<MI_GaussianPoint>& points, int firstSlot, int endSlot)
{
    osg::Image* image = covSplatImage();
    if (!image)
        return;
    firstSlot = std::max(0, firstSlot);
    endSlot = std::min(endSlot, (int)points.size());
    if (endSlot > _covSplatTextureCapacity) {
        std::cerr << "WARNING: CovSplat update exceeds capacity: endSlot=" << endSlot
                  << ", capacity=" << _covSplatTextureCapacity
                  << ", residentPoints=" << points.size()
                  << std::endl;
        endSlot = _covSplatTextureCapacity;
    }
    if (firstSlot >= endSlot)
        return;

    float* basePtr = reinterpret_cast<float*>(image->data(0));
    if (!basePtr)
        return;

    for (int i = firstSlot; i < endSlot; i++) {
        float* dst = basePtr + size_t(i) * 2 * 4;
        writeCovSplatTexels(points[i], dst);
    }
    image->dirty();
}

// ========================================================================
// Phase 1: Spark Paged GPU Resource Pool Implementation
// ========================================================================

void SparkGpuResources::initPagedPool(int maxPages, int pageSize, int maxActiveIndices)
{
    if (_pagedPoolInitialized) {
        std::cout << "Paged Pool already initialized." << std::endl;
        return;
    }

    if (_splatEncoding == SplatEncoding::Legacy) {
        std::cerr << "Paged Pool requires packed encoding; forcing PackedSplat." << std::endl;
        _splatEncoding = SplatEncoding::PackedSplat;
    }

    _maxPages = std::max(1, maxPages);
    _pageSize = std::max(1, pageSize);
    int64_t totalCapacity64 = (int64_t)_maxPages * (int64_t)_pageSize;
    if (totalCapacity64 <= 0 || totalCapacity64 > std::numeric_limits<int>::max()) {
        std::cerr << "Paged GPU Pool capacity overflow: maxPages=" << _maxPages
                  << ", pageSize=" << _pageSize
                  << ", totalCapacity=" << totalCapacity64 << std::endl;
        _maxPages = 0;
        _pageSize = 0;
        return;
    }
    int totalCapacity = (int)totalCapacity64;

    std::cout << "Initializing Paged GPU Pool: maxPages=" << _maxPages
              << ", pageSize=" << _pageSize 
              << ", totalCapacity=" << totalCapacity << " splats." << std::endl;

    _pageOccupied.resize(_maxPages, false);
    _freePagesList.reserve(_maxPages);
    for (int i = _maxPages - 1; i >= 0; --i) {
        _freePagesList.push_back(i);
    }

    // Create the packed data texture with the total capacity
    createPackedDataTexture(totalCapacity);
    
    // Also create the index texture to hold the global sorted active list
    // The draw list scales with the active budget, not the resident cache.
    const int indexCapacity = maxActiveIndices > 0 ? std::min(maxActiveIndices, totalCapacity) : totalCapacity;
    createIndexTexture(std::vector<int>(), indexCapacity);

    // Note: We don't allocate dataTexture or shTexture because the paged pool
    // requires PackedSplat or ExtSplat encoding.

    _pagedPoolInitialized = true;
}

int SparkGpuResources::allocatePage()
{
    std::lock_guard<std::mutex> lock(_pageMutex);
    if (_freePagesList.empty())
        return -1;
    int page = _freePagesList.back();
    _freePagesList.pop_back();
    _pageOccupied[page] = true;
    return page;
}

void SparkGpuResources::freePage(int pageIndex)
{
    std::lock_guard<std::mutex> lock(_pageMutex);
    if (pageIndex < 0 || pageIndex >= _maxPages) return;
    if (_pageOccupied[pageIndex]) {
        _pageOccupied[pageIndex] = false;
        _freePagesList.push_back(pageIndex);
    }
}

void SparkGpuResources::updatePage(int pageIndex, const void* data, int count)
{
    if (!_pagedPoolInitialized || !_packedDataTexture) return;
    if (pageIndex < 0 || pageIndex >= _maxPages) return;

    osg::Image* image = packedDataImage();
    if (!image || !data) return;

    count = std::min(count, _pageSize);
    if (count <= 0) return;

    int texelsPerSplat = packedTexelsPerSplat();
    
    // Each texel is RGBA32UI (16 bytes)
    size_t bytesPerSplat = texelsPerSplat * 16;
    size_t offsetBytes = (size_t)pageIndex * _pageSize * bytesPerSplat;
    size_t sizeBytes = count * bytesPerSplat;

    if (!validImageRange(image, offsetBytes, sizeBytes))
        throw std::out_of_range("Packed page exceeds its texture buffer image");

    unsigned char* dstPtr = image->data(0) + offsetBytes;
    std::memcpy(dstPtr, data, sizeBytes);

    {
        std::lock_guard<std::mutex> lock(_pageMutex);
        _pendingPackedUpdates.push_back({pageIndex, count});
    }
}

void SparkGpuResources::updatePageSH(int pageIndex, const float* shData, int count)
{
    if (!_pagedPoolInitialized) return;
    if (pageIndex < 0 || pageIndex >= _maxPages) return;

    // Ensure SH texture exists with the total capacity of the pool
    if (!_shTexture) {
        int64_t totalCapacity64 = (int64_t)_maxPages * (int64_t)_pageSize;
        if (totalCapacity64 <= 0 || totalCapacity64 > std::numeric_limits<int>::max()) {
            std::cerr << "Paged GPU Pool SH capacity overflow: maxPages=" << _maxPages
                      << ", pageSize=" << _pageSize
                      << ", totalCapacity=" << totalCapacity64 << std::endl;
            return;
        }
        int totalCapacity = (int)totalCapacity64;
        // Don't arbitrarily round up to next power of two if it risks >2GB allocation.
        // A single SH texel block is 12 * 16 bytes = 192 bytes per splat.
        // E.g. 16M splats * 192 = 3GB. We just allocate what we exactly need.
        int newCapacity = totalCapacity; 
        
        osg::ref_ptr<osg::Image> shImage = new osg::Image;
        allocateBufferImage(*shImage, size_t(12) * newCapacity, GL_RGBA, GL_FLOAT);
        std::memset(shImage->data(0), 0, shImage->getTotalSizeInBytesIncludingMipmaps());
        _shTextureCapacity = newCapacity;

        osg::ref_ptr<osg::PixelBufferObject> pbo = new osg::PixelBufferObject(shImage.get());
        pbo->setUsage(GL_DYNAMIC_DRAW);
        shImage->setPixelBufferObject(pbo.get());

        osg::ref_ptr<osg::TextureBuffer> tbo = new osg::TextureBuffer;
        tbo->setImage(shImage.get());
        tbo->setInternalFormat(GL_RGBA32F_ARB);
        _shTexture = tbo;
    }

    osg::Image* image = shImage();
    if (!image || !shData) return;

    count = std::min(count, _pageSize);
    if (count <= 0) return;

    const size_t shStrideBytes = 12 * 16; // 12 texels * 16 bytes/texel (RGBA32F)
    size_t offsetBytes = (size_t)pageIndex * _pageSize * shStrideBytes;

    if (!validImageRange(image, offsetBytes, size_t(count) * shStrideBytes))
        throw std::out_of_range("SH page exceeds its texture buffer image");

    std::memcpy(image->data(0) + offsetBytes, shData, count * shStrideBytes);

    {
        std::lock_guard<std::mutex> lock(_pageMutex);
        _pendingSHUpdates.push_back({pageIndex, count});
    }
}

void SparkGpuResources::updatePageCovSplat(int pageIndex, const MI_GaussianPoint* points, int count)
{
    if (!_pagedPoolInitialized || !points) return;
    if (pageIndex < 0 || pageIndex >= _maxPages) return;

    if (!_covSplatTexture) {
        createCovSplatTexture(_maxPages * _pageSize);
    }

    osg::Image* image = covSplatImage();
    if (!image) return;

    count = std::min(count, _pageSize);
    if (count <= 0) return;

    const size_t covStrideBytes = 2 * 16; // 2 texels * 16 bytes/texel (RGBA32F)
    size_t offsetBytes = (size_t)pageIndex * _pageSize * covStrideBytes;
    if (!validImageRange(image, offsetBytes, size_t(count) * covStrideBytes))
        throw std::out_of_range("Covariance page exceeds its texture buffer image");
    float* dstBase = reinterpret_cast<float*>(image->data(0) + offsetBytes);

    for (int i = 0; i < count; ++i) {
        writeCovSplatTexels(points[i], dstBase + i * 8);
    }

    {
        std::lock_guard<std::mutex> lock(_pageMutex);
        _pendingCovSplatUpdates.push_back({pageIndex, count});
    }
}

void SparkGpuResources::clearPageCovSplat(int pageIndex)
{
    if (!_pagedPoolInitialized || !_covSplatTexture) return;
    if (pageIndex < 0 || pageIndex >= _maxPages) return;

    osg::Image* image = covSplatImage();
    if (!image) return;

    const size_t covStrideBytes = 2 * 16; // 2 texels * 16 bytes/texel (RGBA32F)
    size_t offsetBytes = (size_t)pageIndex * _pageSize * covStrideBytes;
    if (!validImageRange(image, offsetBytes, size_t(_pageSize) * covStrideBytes))
        throw std::out_of_range("Covariance page clear exceeds its texture buffer image");
    std::memset(image->data(0) + offsetBytes, 0, (size_t)_pageSize * covStrideBytes);

    {
        std::lock_guard<std::mutex> lock(_pageMutex);
        _pendingCovSplatUpdates.push_back({pageIndex, _pageSize});
    }
}

void SparkGpuResources::setPageOccupied(int pageIndex, bool occupied)
{
    std::lock_guard<std::mutex> lock(_pageMutex);
    if (pageIndex < 0 || pageIndex >= _maxPages) return;

    bool currentlyOccupied = _pageOccupied[pageIndex];
    if (currentlyOccupied != occupied) {
        _pageOccupied[pageIndex] = occupied;
        if (!occupied) {
            _freePagesList.push_back(pageIndex);
        } else {
            // Remove from free list if it was manually occupied
            auto it = std::find(_freePagesList.begin(), _freePagesList.end(), pageIndex);
            if (it != _freePagesList.end()) {
                _freePagesList.erase(it);
            }
        }
    }
}

bool SparkGpuResources::isPageOccupied(int pageIndex) const
{
    std::lock_guard<std::mutex> lock(_pageMutex);
    if (pageIndex >= 0 && pageIndex < _maxPages) {
        return _pageOccupied[pageIndex];
    }
    return false;
}

int SparkGpuResources::findFreePage() const
{
    // Deprecated: prefer allocatePage() for atomic find+reserve.
    // Kept for backward compatibility.
    std::lock_guard<std::mutex> lock(_pageMutex);
    if (!_freePagesList.empty()) {
        return _freePagesList.back();
    }
    return -1; // Pool full
}

void SparkGpuResources::flushPendingPageUpdates(osg::State& state)
{
    std::vector<PageUpdate> packedUpdates, shUpdates, covSplatUpdates;
    {
        std::lock_guard<std::mutex> lock(_pageMutex);
        packedUpdates.swap(_pendingPackedUpdates);
        shUpdates.swap(_pendingSHUpdates);
        covSplatUpdates.swap(_pendingCovSplatUpdates);
    }

    auto uploadPages = [&](osg::Image* image, std::vector<PageUpdate>& updates, size_t stride) {
        if (!image || updates.empty()) return;
        osg::BufferObject* bo = image->getBufferObject();
        osg::GLBufferObject* glbo = bo ? bo->getGLBufferObject(state.getContextID()) : nullptr;
        const osg::GLExtensions* ext = state.get<osg::GLExtensions>();
        bool incremental = glbo && !glbo->isDirty() && ext && ext->glBufferSubData;
        // Other existing contexts need OSG's normal modified-count propagation.
        if (incremental) {
            for (unsigned int id = 0; id <= osg::GraphicsContext::getMaxContextID(); ++id) {
                if (id != state.getContextID() && bo->getGLBufferObject(id)) {
                    incremental = false;
                    break;
                }
            }
        }
        if (!incremental) {
            image->dirty();
            return;
        }
        std::sort(updates.begin(), updates.end(), [](const PageUpdate& a, const PageUpdate& b) {
            return a.pageIndex < b.pageIndex;
        });
        // COPY_WRITE avoids disturbing the array/PBO bindings cached by OSG.
        const GLenum copyWriteBuffer = 0x8F37;
        GLint previousBuffer = 0;
        glGetIntegerv(copyWriteBuffer, &previousBuffer);
        ext->glBindBuffer(copyWriteBuffer, glbo->getGLObjectID());
        const size_t baseOffset = (size_t)glbo->getOffset(image->getBufferIndex());
        size_t begin = 0, end = 0;
        auto upload = [&]() {
            if (end > begin)
                ext->glBufferSubData(copyWriteBuffer, baseOffset + begin, end - begin, image->data() + begin);
        };
        for (const PageUpdate& update : updates) {
            size_t first = (size_t)update.pageIndex * _pageSize * stride;
            size_t last = first + (size_t)update.count * stride;
            if (!validImageRange(image, first, last - first))
                throw std::out_of_range("GPU page upload exceeds its texture buffer image");
            if (first > end) {
                upload();
                begin = first;
                end = last;
            } else {
                end = std::max(end, last);
            }
        }
        upload();
        ext->glBindBuffer(copyWriteBuffer, (GLuint)previousBuffer);
    };
    uploadPages(packedDataImage(), packedUpdates, (size_t)packedTexelsPerSplat() * 16);
    uploadPages(shImage(), shUpdates, 12 * 16);
    uploadPages(covSplatImage(), covSplatUpdates, 2 * 16);

    // State caches texture attribute pointers across frames. Image::dirty only
    // reaches the GPU when TextureBuffer::apply runs, so invalidate the bound
    // TBO attributes even when only the sorted indices changed. Unmodified
    // buffers are rebound without uploading their contents again.
    osg::TextureBuffer* textures[] = { indexTexture(), dataTexture(), shTexture(),
        packedDataTexture(), covSplatTexture() };
    for (unsigned int unit = 0; unit < 5; ++unit) {
        if (textures[unit])
            state.haveAppliedTextureAttribute(unit, osg::StateAttribute::TEXTURE);
    }
}
