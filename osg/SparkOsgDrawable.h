#ifndef SPARK_OSG_DRAWABLE_H
#define SPARK_OSG_DRAWABLE_H

#include "RadTypes.h"
#include "SparkRadStreamer.h"
#include "SparkGpuResources.h"
#include "SparkSortPass.h"
#include "SparkLodWorker.h"
#include "SparkRadQuality.h"
#include "SparkRadProtectedPages.h"
#include "SparkRadPendingLod.h"

#include <osg/Image>
#include <osg/Matrix>
#include <osg/BoundingBox>

#include <array>
#include <cstdint>
#include <string>
#include <vector>

struct MI_GaussianPoint;

class SparkOsgDrawable
{
    friend class GaussianDrawObj;

public:
    SparkOsgDrawable();
    ~SparkOsgDrawable();

    // Set external shared resources (owned by GaussianDrawObj)
    void setSharedResources(SparkGpuResources* gpuResources, SparkSortPass* sortPass);

    // Initialize RAD pipeline state (called after readRadFile populates radFileInfo/radChunks)
    void initializeRad(const std::string& filename,
                       std::streamoff chunkDataBase,
                       int totalPointCount,
                       int residentCapacity);

    // Streamer lifecycle
    void startRadLoader();
    void stopRadLoader();

    // Per-frame RAD LOD update
    bool updateRadNativeLOD(const osg::Matrix& viewMatrix, const osg::Camera* cam);

    // Sort orchestration
    bool shouldSort(const osg::Matrix& currentView, bool firstSort);
    void requestSort(const osg::Matrix& viewMatrix);
    bool applySortResult(osg::Image* indexImage);

    // Active set access
    const std::vector<int>& activeIndices() const { return _activeIndices; }
    const std::vector<int>& depthIndex() const { return _depthIndex; }
    int drawActiveCount() const { return _drawActiveCount; }
    int activeCount() const { return _drawActiveCount; }
    bool isRadFile() const { return _isRadFile; }
    int currentLOD() const { return _lodLevel; }
    int originalPointCount() const { return _radTotalPointCount; }
    bool firstSort() const { return _firstSort; }
    void setFirstSort(bool v) { _firstSort = v; }

    // Shared resource access (owned by GaussianDrawObj)
    SparkGpuResources& gpuResources() { return *_gpuResources; }
    const SparkGpuResources& gpuResources() const { return *_gpuResources; }
    SparkSortPass& sortPass() { return *_sortPass; }
    const SparkSortPass& sortPass() const { return *_sortPass; }

    // RAD chunk management
    bool loadRadChunk(unsigned int chunkIndex);
    bool applyRadChunk(unsigned int chunkIndex, const std::vector<uint8_t>& chunk, uint32_t loadGeneration,
                       const SparkRad::PreparedChunk* prepared = nullptr);
    bool applyRadChunkPacked(unsigned int chunkIndex, const std::vector<uint32_t>& packedData, uint32_t loadGeneration);
    bool evictRadChunk(unsigned int chunkIndex);

    // Dirty slot management
    void markRadDirtySlots(int firstSlot, int endSlot);
    void flushRadDirtySlots();

    // Shared point data (owned by GaussianDrawObj, referenced here). In paged
    // RAD mode only mapped resident slots have defined point payloads; free
    // slots may retain retired records, as do the packed GPU page buffers.
    void setPoints(std::vector<MI_GaussianPoint>* points) { _points = points; }
    std::vector<MI_GaussianPoint>* points() { return _points; }

    // Bounds (updated by chunk apply/evict)
    osg::BoundingBox& bounds() { return _bounds; }

    // SH metadata
    bool hasSH() const { return _hasSH; }
    void setHasSH(bool v) { _hasSH = v; }
    int getSHOriginalPointCount() const { return _radTotalPointCount; }

    // Diagnostics
    double lastSortMilliseconds() const { return _lastSortMilliseconds; }
    double lastSortItems() const { return _lastSortItems; }

    // RAD info for logging
    int radLoadedChunkCount() const { return _radLoadedChunkCount; }
    int radTotalChunkCount() const { return _radTotalChunkCount; }
    int radEvictedChunkCount() const { return _radEvictedChunkCount; }
    int radResidentCapacity() const { return _radResidentCapacity; }
    int radTotalPointCount() const { return _radTotalPointCount; }
    const std::string& radFilename() const { return _radFilename; }
    std::streamoff radChunkDataBase() const { return _radChunkDataBase; }
    int radChunkSize() const { return _radChunkSize; }
    unsigned int activeSetVersion() const { return _activeSetVersion; }

    // Access to RAD mapping data (needed by SparkLodSelector)
    std::vector<int>& radGlobalToLocal() { return _radGlobalToLocal; }
    std::vector<int>& radSlotToChunk() { return _radSlotToChunk; }
    std::vector<uint16_t>& radChildCounts() { return _radChildCounts; }
    std::vector<uint32_t>& radChildStarts() { return _radChildStarts; }
    std::vector<float>& radLodSizes() { return _radLodSizes; }
    std::vector<RadChunkInfo>& radChunks() { return _radChunks; }
    RadFileInfo& radFileInfo() { return _radFileInfo; }

private:
    int ensureRadFreeSlots(int requiredSlots, const std::vector<int>& protectedChunks);
    void queueRadChunk(uint32_t chunkIndex, float priority, int& queuedCount);
    void updateActiveIndexBuffer();
    void updateIndexChangedRanges(const std::vector<int>& current, const std::vector<int>& previous);
    void ensureRadDataTextureCapacity(int requiredSlots);

    // Shared resources (not owned, owned by GaussianDrawObj)
    SparkGpuResources* _gpuResources = nullptr;
    SparkSortPass* _sortPass = nullptr;

    // Shared point data (not owned)
    std::vector<MI_GaussianPoint>* _points = nullptr;
    osg::BoundingBox _bounds;
    bool _hasSH = false;

    // Core pipeline state
    std::vector<int> _depthIndex;
    std::vector<int> _activeIndices;
    std::vector<int> _sortResultScratch;
    int _drawActiveCount = 0;
    int _lodLevel = 0;
    bool _firstSort = true;

    // Diagnostics
    double _lastSortMilliseconds = 0.0;
    double _lastSortItems = 0.0;

    // RAD state
    bool _isRadFile = false;
    int _radTotalPointCount = 0;
    int _radLoadedChunkCount = 0;
    int _radTotalChunkCount = 0;
    int _radEvictedChunkCount = 0;
    unsigned int _radFrameNumber = 0;
    int _radResidentCapacity = 0;
    int _radChunkSize = 65536;
    bool _radHasLodOpacity = false;
    bool _radLoadRequested = false;
    bool _firstRadLOD = true;
    std::string _radFilename;
    std::streamoff _radChunkDataBase = 0;
    int _radChunkLoadCursor = 0;
    unsigned int _activeSetVersion = 0;

    // RAD LOD state
    osg::Matrix _lastRadLODViewMatrix;
    std::vector<int> _lodScratchIndices;
    std::vector<int> _pendingLODIndices;
    int _pendingLODStableFrames = 0;
    SparkRadPendingLod _radPendingLod;
    SparkLodWorker _radLodWorker;
    SparkRadQualityPolicy _radQuality;
    SparkRadLodRetargetPolicy _radLodRetarget;
    osg::Matrix _radWorkerView;
    bool _radRetargetOnStop = true;
    bool _radWaitSortHandoff = true;
    bool _radClearEvictedPoints = false;
    unsigned int _radSortHandoffWaitFrames = 0;
    SparkRadProjectionState _radProjection;
    SparkRadStreamWait _radStreamWait;
    SparkRadProtectedPages _radDrawProtectedPages;
    std::vector<int> _radActivePageSummary;
    bool _radActivePageSummaryValid = false;
    unsigned int _radActivePageSummaryVersion = 0;
    std::uint64_t _radActivePageSummaryResidency = 0;
    // Display can remain on an older complete frontier while active is sorting.
    unsigned int _radDisplayedActiveSetVersion = 0;
    std::uint64_t _radDisplayRevision = 0;
    std::uint64_t _radResidencyRevision = 0;
    std::uint64_t _radDataEpoch = 0;
    std::uint64_t _radWorkerQualityGeneration = 0;
    std::uint64_t _radWorkerProjectionGeneration = 0;
    float _radWorkerPixelScaleLimit = 0.0f;
    float _radWorkerViewportOutsideWeight = 1.0f;
    bool _radWorkerForceCommit = false;
    int _radLoadedSinceSelection = 0;
    int _radIdleLoadBatchLimit = 32;

    // RAD chunk/slot management
    RadFileInfo _radFileInfo;
    std::vector<RadChunkInfo> _radChunks;
    std::vector<std::vector<int>> _radChunkSlots;
    std::vector<int> _radSlotToChunk;
    std::vector<int> _radFreeSlots;
    std::vector<std::array<int, 2>> _radDirtySlotRanges;
    std::vector<int> _radProtectedChunks;
    std::vector<uint32_t> _radPrefetchedChunks;
    std::vector<int> _radTraversalProtectedChunks;

    // RAD index mapping
    std::vector<uint16_t> _radChildCounts;
    std::vector<uint32_t> _radChildStarts;
    std::vector<uint32_t> _radLocalToGlobal;
    std::vector<int> _radGlobalToLocal;
    std::vector<uint32_t> _radLeafGlobalIndices;
    std::vector<float> _radLodSizes;
    // Compact CPU-only scoring records; full Gaussian records remain in
    // _points for rendering and upload.
    std::vector<SparkLodSelector::LodNode> _radLodNodes;

    // LRU page eviction tracking (Spark-style)
    std::vector<uint32_t> _radLruOrder;       // chunk indices sorted by LRU (oldest first)
    std::vector<uint32_t> _radFreeableChunks;  // chunks not in active set, sorted by LRU

    // Streamer
    SparkRadStreamer _radStreamer;
};

#endif
