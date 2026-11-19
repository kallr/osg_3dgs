#ifndef SPARK_GPU_RESOURCES_H
#define SPARK_GPU_RESOURCES_H

#include <osg/Image>
#include <osg/Array>
#include <osg/BufferObject>
#include <osg/TextureBuffer>
#include <osg/ref_ptr>

#include <vector>
#include <mutex>

struct MI_GaussianPoint;

enum class SplatEncoding
{
    Legacy,         // 4x vec4f = 64 bytes/splat (current path)
    PackedSplat,    // 4x uint32 = 16 bytes/splat
    ExtSplat        // 8x uint32 = 32 bytes/splat
};

class SparkGpuResources
{
public:
    void setRadMode(bool enabled);
    void setSplatEncoding(SplatEncoding encoding);
    SplatEncoding splatEncoding() const { return _splatEncoding; }

    void createIndexTexture(const std::vector<int>& indices, int initialCapacity);
    void createDataTexture(const std::vector<MI_GaussianPoint>& points, int initialCapacity, bool isRadFile);
    void createSHTexture(const std::vector<MI_GaussianPoint>& points, int initialCapacity);

    void ensureIndexCapacity(const std::vector<int>& indices, int requiredIndices);
    void ensureDataCapacity(const std::vector<MI_GaussianPoint>& points, int requiredSlots, bool hasSH);

    void updateDataRange(const std::vector<MI_GaussianPoint>& points, int firstSlot, int endSlot, bool isRadFile);
    void updateSHRange(const std::vector<MI_GaussianPoint>& points, int firstSlot, int endSlot);
    void updateIndexRange(const std::vector<int>& indices, int firstIndex, int endIndex);
    void updateIndexChangedRanges(const std::vector<int>& current, const std::vector<int>& previous);

    void createSortBuffers(int capacity, const std::vector<int>& activeIndices);
    void createTileBuffers(int tileCount, int tileListCapacity);
    void updateActiveIndexBuffer(const std::vector<int>& activeIndices, int capacity);

    osg::TextureBuffer* indexTexture() const { return _indexTexture.get(); }
    osg::TextureBuffer* dataTexture() const { return _dataTexture.get(); }
    osg::TextureBuffer* shTexture() const { return _shTexture.get(); }
    osg::TextureBuffer* packedDataTexture() const { return _packedDataTexture.get(); }
    osg::TextureBuffer* covSplatTexture() const { return _covSplatTexture.get(); }

    osg::IntArray* activeIndexBuffer() const { return _activeIndexBuffer.get(); }
    osg::UIntArray* depthKeyBuffer() const { return _depthKeyBuffer.get(); }
    osg::IntArray* gpuSortIndexBuffer() const { return _gpuSortIndexBuffer.get(); }
    osg::UIntArray* tileCountBuffer() const { return _tileCountBuffer.get(); }
    osg::UIntArray* tileCursorBuffer() const { return _tileCursorBuffer.get(); }
    osg::IntArray* tileListBuffer() const { return _tileListBuffer.get(); }
    osg::UIntArray* tileStatsBuffer() const { return _tileStatsBuffer.get(); }

    osg::Image* indexImage() const { return _indexTexture.valid() ? _indexTexture->getImage() : nullptr; }
    osg::Image* dataImage() const { return _dataTexture.valid() ? _dataTexture->getImage() : nullptr; }
    osg::Image* shImage() const { return _shTexture.valid() ? _shTexture->getImage() : nullptr; }
    osg::Image* packedDataImage() const { return _packedDataTexture.valid() ? _packedDataTexture->getImage() : nullptr; }
    osg::Image* covSplatImage() const { return _covSplatTexture.valid() ? _covSplatTexture->getImage() : nullptr; }

    int indexCapacity() const { return _indexTextureCapacity; }
    int dataCapacity() const { return _dataTextureCapacity; }
    int shCapacity() const { return _shTextureCapacity; }
    int packedDataCapacity() const { return _packedDataTextureCapacity; }
    int covSplatCapacity() const { return _covSplatTextureCapacity; }

    // Packed data texels per splat: 1 for PackedSplat, 2 for ExtSplat
    int packedTexelsPerSplat() const;

    // Packed data texture management (public for direct use by GaussianDrawObj)
    void createPackedDataTexture(int initialCapacity);
    void updatePackedDataRange(const std::vector<MI_GaussianPoint>& points, int firstSlot, int endSlot, bool isRadFile);

    // CovSplat texture management (2 texels RGBA32F per splat: xxyyzz+xyxzyz)
    void createCovSplatTexture(int initialCapacity);
    void ensureCovSplatCapacity(const std::vector<MI_GaussianPoint>& points, int requiredSlots);
    void updateCovSplatRange(const std::vector<MI_GaussianPoint>& points, int firstSlot, int endSlot);

    // ========================================================================
    // Phase 1: Spark Paged GPU Resource Pool (For RAD only)
    // ========================================================================
    // Initialize a fixed-size GPU resource pool for RAD pages.
    void initPagedPool(int maxPages, int pageSize = 65536, int maxActiveIndices = 0);
    
    // Check if the paged pool is initialized.
    bool hasPagedPool() const { return _pagedPoolInitialized; }
    
    // Get max number of pages in the pool.
    int maxPages() const { return _maxPages; }
    
    // Get number of splats per page.
    int pageSize() const { return _pageSize; }
    
    // Update a specific page in the pool with raw packed data.
    // data: Pointer to packed data (16B or 32B per splat depending on _splatEncoding).
    // count: Number of splats to write (must be <= pageSize).
    void updatePage(int pageIndex, const void* data, int count);

    // Update SH data for a specific page in the pool.
    void updatePageSH(int pageIndex, const float* shData, int count);

    // Update CovSplat data for a specific page in the pool.
    void updatePageCovSplat(int pageIndex, const MI_GaussianPoint* points, int count);

    // Clear CovSplat data for a specific page in the pool.
    void clearPageCovSplat(int pageIndex);

    // Atomically allocate a free page. Returns -1 if full.
    // Combines findFreePage + setPageOccupied(true) under internal lock.
    int allocatePage();

    // Mark a page as free.
    void freePage(int pageIndex);

    // Mark a page as occupied (kept for backward compat, prefer allocatePage/freePage).
    void setPageOccupied(int pageIndex, bool occupied);
    bool isPageOccupied(int pageIndex) const;

    // Find an empty page index. Returns -1 if full.
    int findFreePage() const;
    
    // Process pending partial updates to GPU (should be called before rendering)
    void flushPendingPageUpdates(osg::State& state);

private:
    struct PageUpdate {
        int pageIndex;
        int count;
    };
    std::vector<PageUpdate> _pendingPackedUpdates;
    std::vector<PageUpdate> _pendingSHUpdates;
    std::vector<PageUpdate> _pendingCovSplatUpdates;
    void ensurePackedDataCapacity(const std::vector<MI_GaussianPoint>& points, int requiredSlots);

    static int nextPowerOfTwo(int value);

    bool _isRadMode = false;
    SplatEncoding _splatEncoding = SplatEncoding::Legacy;
    osg::ref_ptr<osg::TextureBuffer> _indexTexture;
    osg::ref_ptr<osg::TextureBuffer> _dataTexture;
    osg::ref_ptr<osg::TextureBuffer> _shTexture;
    osg::ref_ptr<osg::TextureBuffer> _packedDataTexture;
    osg::ref_ptr<osg::TextureBuffer> _covSplatTexture;
    osg::ref_ptr<osg::IntArray> _activeIndexBuffer;
    osg::ref_ptr<osg::ShaderStorageBufferObject> _activeIndexSSBO;
    osg::ref_ptr<osg::UIntArray> _depthKeyBuffer;
    osg::ref_ptr<osg::ShaderStorageBufferObject> _depthKeySSBO;
    osg::ref_ptr<osg::IntArray> _gpuSortIndexBuffer;
    osg::ref_ptr<osg::ShaderStorageBufferObject> _gpuSortIndexSSBO;
    osg::ref_ptr<osg::UIntArray> _tileCountBuffer;
    osg::ref_ptr<osg::ShaderStorageBufferObject> _tileCountSSBO;
    osg::ref_ptr<osg::UIntArray> _tileCursorBuffer;
    osg::ref_ptr<osg::ShaderStorageBufferObject> _tileCursorSSBO;
    osg::ref_ptr<osg::IntArray> _tileListBuffer;
    osg::ref_ptr<osg::ShaderStorageBufferObject> _tileListSSBO;
    osg::ref_ptr<osg::UIntArray> _tileStatsBuffer;
    osg::ref_ptr<osg::ShaderStorageBufferObject> _tileStatsSSBO;
    int _indexTextureCapacity = 0;
    int _dataTextureCapacity = 0;
    int _shTextureCapacity = 0;
    int _packedDataTextureCapacity = 0;
    int _covSplatTextureCapacity = 0;

    // Phase 1 Paged Pool State
    bool _pagedPoolInitialized = false;
    int _maxPages = 0;
    int _pageSize = 65536;
    std::vector<bool> _pageOccupied;
    std::vector<int> _freePagesList; // Cache for O(1) allocation
    mutable std::mutex _pageMutex;   // Protects _pageOccupied, _freePagesList and pending page update queues
};

#endif
