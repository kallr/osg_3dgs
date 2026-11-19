#include "SparkOsgDrawable.h"
#include "GaussianDrawObj.h"
#include "SparkGpuResources.h"
#include "SparkSortPass.h"
#include "SparkRadFile.h"
#include "SparkLodSelector.h"
#include "SparkFrameMetrics.h"

#include "osg/TextureBuffer"
#include "osgUtil/CullVisitor"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <numeric>
#include <unordered_set>
#include <utility>
#ifdef HAVE_ZLIB
#include <zlib.h>
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace
{
    const int DEFAULT_RAD_MAX_RESIDENT = 6291456;
    const float DEFAULT_RAD_LOD_LEVEL = 0.786f;
    const float DEFAULT_RAD_LOD_QUALITY_AT_LEVEL = 0.08f;

    bool isEnvEnabled(const char* name)
    {
        std::string text;
#ifdef _MSC_VER
        char* value = nullptr;
        size_t length = 0;
        if (_dupenv_s(&value, &length, name) != 0 || !value)
            return false;
        text.assign(value);
        std::free(value);
#else
        const char* value = std::getenv(name);
        if (!value)
            return false;
        text.assign(value);
#endif
        if (text.empty())
            return false;
        std::transform(text.begin(), text.end(), text.begin(),
            [](unsigned char ch) { return (char)std::tolower(ch); });
        return text != "0" && text != "false" && text != "off" && text != "no";
    }

    int getEnvInt(const char* name, int fallback, int minValue, int maxValue)
    {
        std::string text;
#ifdef _MSC_VER
        char* value = nullptr;
        size_t length = 0;
        if (_dupenv_s(&value, &length, name) != 0 || !value)
            return fallback;
        text.assign(value);
        std::free(value);
#else
        const char* value = std::getenv(name);
        if (!value)
            return fallback;
        text.assign(value);
#endif
        if (text.empty())
            return fallback;
        int parsed = std::atoi(text.c_str());
        return std::max(minValue, std::min(maxValue, parsed));
    }

    float getEnvFloat(const char* name, float fallback, float minValue, float maxValue)
    {
        std::string text;
#ifdef _MSC_VER
        char* value = nullptr;
        size_t length = 0;
        if (_dupenv_s(&value, &length, name) != 0 || !value)
            return fallback;
        text.assign(value);
        std::free(value);
#else
        const char* value = std::getenv(name);
        if (!value)
            return fallback;
        text.assign(value);
#endif
        if (text.empty())
            return fallback;
        float parsed = (float)std::atof(text.c_str());
        return std::max(minValue, std::min(maxValue, parsed));
    }

    bool isEnvSet(const char* name)
    {
#ifdef _MSC_VER
        char* value = nullptr;
        size_t length = 0;
        bool exists = _dupenv_s(&value, &length, name) == 0 && value;
        std::free(value);
        return exists;
#else
        return std::getenv(name) != nullptr;
#endif
    }
}

SparkOsgDrawable::SparkOsgDrawable()
{
}

SparkOsgDrawable::~SparkOsgDrawable()
{
    stopRadLoader();
}

void SparkOsgDrawable::setSharedResources(SparkGpuResources* gpuResources, SparkSortPass* sortPass)
{
    _gpuResources = gpuResources;
    _sortPass = sortPass;
}

void SparkOsgDrawable::initializeRad(const std::string& filename,
                                      std::streamoff chunkDataBase,
                                      int totalPointCount,
                                      int residentCapacity)
{
    _radLodWorker.stop();
    _isRadFile = true;
    ++_radDataEpoch;
    _radChunkSize = _radFileInfo.chunkSize;
    _radFilename = filename;
    _radChunkDataBase = chunkDataBase;
    _radTotalPointCount = totalPointCount;
    _radTotalChunkCount = (int)_radChunks.size();
    // Note: _radLoadedChunkCount and _radGlobalToLocal are NOT reset here
    // because readRadFile() already populated them with initial chunk data.
    _radEvictedChunkCount = 0;
    _radFrameNumber = 0;
    _radResidentCapacity = residentCapacity;
    _radHasLodOpacity = true;
    _firstRadLOD = true;
    _activeSetVersion = 0;

    const SparkRadQualitySettings quality = SparkRadQualitySettings::fromEnvironment();
    _radQuality.reset(quality);
    _radLodRetarget.reset();
    _radRetargetOnStop = !isEnvSet("OSG_3DGS_RAD_RETARGET_ON_STOP") ||
        isEnvEnabled("OSG_3DGS_RAD_RETARGET_ON_STOP");
    _radWaitSortHandoff = !isEnvSet("OSG_3DGS_RAD_WAIT_SORT_HANDOFF") ||
        isEnvEnabled("OSG_3DGS_RAD_WAIT_SORT_HANDOFF");
    _radClearEvictedPoints = isEnvEnabled("OSG_3DGS_RAD_CLEAR_EVICTED_POINTS");
    _radSortHandoffWaitFrames = 0;
    _radWorkerQualityGeneration = 0;
    _radProjection.reset();
    _radWorkerProjectionGeneration = 0;
    _radWorkerViewportOutsideWeight = 1.0f;
    _radStreamWait.clear();
    _radPendingLod.clear();
    _radDrawProtectedPages.clear();
    _radActivePageSummary.clear();
    _radActivePageSummaryValid = false;
    _radActivePageSummaryVersion = 0;
    _radActivePageSummaryResidency = 0;
    _radDisplayedActiveSetVersion = 0;
    _radDisplayRevision = 0;
    _radResidencyRevision = 0;
    _radLoadedSinceSelection = 0;
    _radIdleLoadBatchLimit = getEnvInt("OSG_3DGS_RAD_IDLE_LOAD_BATCH", 32, 1, 32);
    const size_t activeCapacity = static_cast<size_t>(std::min(quality.capacity(),
        std::max(1, residentCapacity)));
    _activeIndices.reserve(activeCapacity);
    _depthIndex.reserve(activeCapacity);
    _lodScratchIndices.reserve(activeCapacity);
    std::cout << "RAD detail budgets: moving=" << quality.movingSplats
              << ", idle=" << quality.idleSplats
              << ", idleDelayMs=" << quality.idleDelayMs << std::endl;

    _radChunkSlots.resize(_radChunks.size());
}

void SparkOsgDrawable::startRadLoader()
{
    if (_sortPass && !_sortPass->useGpuSort() && !_sortPass->sortDisabled()) {
        // Bootstrap with the complete coarse root, which needs no sorting.
        // The larger bootstrap active list contains hierarchy nodes in file
        // order and must not be presented before its first sorted result.
        const int root = !_radGlobalToLocal.empty() ? _radGlobalToLocal[0] : -1;
        if (root >= 0 && _points && size_t(root) < _points->size())
            _depthIndex.assign(1, root);
        else if (_depthIndex.size() > 1)
            _depthIndex.resize(1);
        _drawActiveCount = int(_depthIndex.size());
        // The root-only preview is not the bootstrap active membership (v0).
        _radDisplayedActiveSetVersion = std::numeric_limits<unsigned int>::max();
        _firstSort = true;
    }
    if (_gpuResources && _gpuResources->hasPagedPool() && _points && _sortPass) {
        // Slot addresses include page padding. Reserve for the actual pool,
        // bounded by the number of chunks, before streaming starts. Reserving
        // leaves size and residency unchanged; missing slots remain invalid.
        const size_t pageCount = std::min(size_t(_gpuResources->maxPages()), _radChunks.size());
        const size_t slotCapacity = pageCount * size_t(_gpuResources->pageSize());
        _points->reserve(slotCapacity);
        _radChildCounts.reserve(slotCapacity);
        _radChildStarts.reserve(slotCapacity);
        _radLocalToGlobal.reserve(slotCapacity);
        _radLodSizes.reserve(slotCapacity);
        _radLodNodes.reserve(slotCapacity);
        _radSlotToChunk.reserve(slotCapacity);
        _sortPass->reservePositionCapacity(int(slotCapacity));
    }
    SparkRadStreamer::PrepareChunk prepare;
#ifdef HAVE_ZLIB
    if (!isEnvEnabled("OSG_3DGS_RAD_SYNC_PREPARE")) {
        SparkRad::PrepareOptions options;
        options.totalPointCount = _radTotalPointCount;
        if (_gpuResources && _gpuResources->hasPagedPool()) {
            options.maxPageSplats = _gpuResources->pageSize();
            options.splatEncoding = (int)_gpuResources->splatEncoding();
        }
        options.timingsEnabled = SparkFrameMetrics::enabled();
        // Capture immutable settings, never Drawable/GPU state or slot arrays.
        prepare = [options](RadRawChunk& raw, const RadChunkInfo& info) {
            auto result = std::make_shared<SparkRad::PreparedChunk>();
            if (!SparkRad::prepareChunk(raw.bytes, info.count, info.base, options, *result))
                return false;
            raw.prepared = std::move(result);
            std::vector<uint8_t>().swap(raw.bytes);
            return true;
        };
    }
#endif
    const size_t readyLimit = getEnvInt("OSG_3DGS_RAD_READY_CHUNKS", prepare ? 4 : 8, 1, 8);
    const size_t prepareWorkers = getEnvInt("OSG_3DGS_RAD_PREPARE_WORKERS", prepare ? 2 : 1, 1, 3);
    _radStreamer.configure(_radFilename, _radChunkDataBase, &_radChunks, std::move(prepare), readyLimit, prepareWorkers);
    if (SparkFrameMetrics::enabled())
        std::cout << "RAD streaming settings: workers=" << prepareWorkers << ", ready=" << readyLimit
                  << ", idleBatch=" << _radIdleLoadBatchLimit << ", retargetOnStop=" << (_radRetargetOnStop ? 1 : 0)
                  << ", waitSortHandoff=" << (_radWaitSortHandoff ? 1 : 0)
                  << std::endl;
    _radStreamer.start();
}

void SparkOsgDrawable::stopRadLoader()
{
    // Join while all borrowed point/LOD arrays still exist.
    _radLodWorker.stop();
    _radStreamer.stop();
}

bool SparkOsgDrawable::shouldSort(const osg::Matrix& currentView, bool firstSort)
{
    float epsilon = _isRadFile ? 2.0e-3f : 1.0e-5f;
    return _sortPass->shouldSort(currentView, firstSort, epsilon);
}

void SparkOsgDrawable::requestSort(const osg::Matrix& viewMatrix)
{
    if (_sortPass->sortDisabled()) {
        _sortPass->setLastViewMatrix(viewMatrix);
        _depthIndex = _activeIndices;
        _radDisplayedActiveSetVersion = _activeSetVersion;
        ++_radDisplayRevision;
        _drawActiveCount = (int)_depthIndex.size();
        _firstSort = false;
        return;
    }

    if (_sortPass->useGpuSort()) {
        _sortPass->noteGpuSortDispatched(viewMatrix);
        updateActiveIndexBuffer();
        _drawActiveCount = (int)_activeIndices.size();
        _firstSort = false;
        return;
    }

    if (_isRadFile && _sortPass->hasPendingCpuSort(_activeSetVersion, viewMatrix, 2.0e-3f))
        return;

    _firstSort = false;
    _sortPass->requestCpuSort(viewMatrix, _activeIndices, _activeSetVersion);
}

bool SparkOsgDrawable::applySortResult(osg::Image* indexImage)
{
    SparkFrameMetrics::Scope profile(SparkFrameMetrics::SortApply);
    if (_sortPass->useGpuSort() || _sortPass->sortDisabled())
        return false;

    const unsigned int previousDisplayVersion = _radDisplayedActiveSetVersion;
    auto reportPresentation = [&]() {
        if (_isRadFile && previousDisplayVersion != _radDisplayedActiveSetVersion && SparkFrameMetrics::enabled())
            std::cout << "RAD sorted frontier displayed: activeSet=" << _radDisplayedActiveSetVersion
                      << ", displayed=" << _drawActiveCount << ", sortMs=" << _lastSortMilliseconds << std::endl;
    };

    // Try to swap the latest sort result into the display buffer
    double sortMilliseconds = 0.0;
    if (_sortPass->swapDisplayResult(&sortMilliseconds)) {
        std::vector<int> previousDepthIndex;
        previousDepthIndex.swap(_depthIndex);
        _depthIndex = _sortPass->displayIndices();
        _radDisplayedActiveSetVersion = _activeSetVersion;
        ++_radDisplayRevision;
        _drawActiveCount = _sortPass->displayCount();
        _lastSortMilliseconds = sortMilliseconds;
        _lastSortItems = (double)_depthIndex.size();
        updateIndexChangedRanges(_depthIndex, previousDepthIndex);
        reportPresentation();
        return true;
    }

    // Fallback: try direct fetch (for compatibility)
    sortMilliseconds = 0.0;
    SparkSortPass::CpuResultStatus resultStatus = _sortPass->fetchCpuResult(_sortResultScratch, _activeSetVersion, sortMilliseconds);
    if (resultStatus == SparkSortPass::CpuResultStatus::None)
        return false;
    if (resultStatus == SparkSortPass::CpuResultStatus::Stale) {
        if (_isRadFile)
            std::cout << "Discarding stale RAD sort result: currentGeneration=" << _sortPass->generation()
                      << ", currentActiveSet=" << _activeSetVersion << std::endl;
        return false;
    }

    std::vector<int> previousDepthIndex;
    previousDepthIndex.swap(_depthIndex);
    _depthIndex.swap(_sortResultScratch);
    _radDisplayedActiveSetVersion = _activeSetVersion;
    ++_radDisplayRevision;
    _drawActiveCount = (int)_depthIndex.size();
    _lastSortMilliseconds = sortMilliseconds;
    _lastSortItems = (double)_depthIndex.size();
    updateIndexChangedRanges(_depthIndex, previousDepthIndex);
    reportPresentation();
    return true;
}

void SparkOsgDrawable::updateActiveIndexBuffer()
{
    _gpuResources->updateActiveIndexBuffer(_activeIndices, _sortPass->gpuSortCapacity());
}

void SparkOsgDrawable::updateIndexChangedRanges(const std::vector<int>& current, const std::vector<int>& previous)
{
    _gpuResources->updateIndexChangedRanges(current, previous);
}

void SparkOsgDrawable::ensureRadDataTextureCapacity(int requiredSlots)
{
    _gpuResources->ensureDataCapacity(*_points, requiredSlots, _hasSH);
}

void SparkOsgDrawable::markRadDirtySlots(int firstSlot, int endSlot)
{
    if (!_isRadFile || firstSlot >= endSlot)
        return;
    firstSlot = std::max(0, firstSlot);
    endSlot = std::max(firstSlot, endSlot);
    _radDirtySlotRanges.push_back({ firstSlot, endSlot });
}

void SparkOsgDrawable::flushRadDirtySlots()
{
    if (!_isRadFile || _radDirtySlotRanges.empty())
        return;
    std::sort(_radDirtySlotRanges.begin(), _radDirtySlotRanges.end(), [](const std::array<int, 2>& lhs, const std::array<int, 2>& rhs) {
        if (lhs[0] != rhs[0])
            return lhs[0] < rhs[0];
        return lhs[1] < rhs[1];
    });

    std::vector<std::array<int, 2>> ranges;
    ranges.reserve(_radDirtySlotRanges.size());
    for (const std::array<int, 2>& dirty : _radDirtySlotRanges) {
        int firstSlot = std::max(0, dirty[0]);
        int endSlot = std::min(dirty[1], (int)_points->size());
        if (firstSlot >= endSlot)
            continue;
        if (!ranges.empty() && firstSlot <= ranges.back()[1])
            ranges.back()[1] = std::max(ranges.back()[1], endSlot);
        else
            ranges.push_back({ firstSlot, endSlot });
    }
    _radDirtySlotRanges.clear();
    if (ranges.empty())
        return;
    ensureRadDataTextureCapacity((int)_points->size());
    if (_gpuResources->covSplatTexture())
        _gpuResources->ensureCovSplatCapacity(*_points, (int)_points->size());

    osg::Image* dataImage = _gpuResources->dataImage();
    osg::Image* shImage = _hasSH ? _gpuResources->shImage() : nullptr;
    osg::Image* covSplatImage = _gpuResources->covSplatImage();
    for (const std::array<int, 2>& range : ranges) {
        if (dataImage || _gpuResources->packedDataImage())
            _gpuResources->updateDataRange(*_points, range[0], range[1], true);
        if (shImage)
            _gpuResources->updateSHRange(*_points, range[0], range[1]);
        if (covSplatImage)
            _gpuResources->updateCovSplatRange(*_points, range[0], range[1]);
        _sortPass->updatePositionRange(_points, range[0], range[1]);
    }
}

bool SparkOsgDrawable::loadRadChunk(unsigned int chunkIndex)
{
    if (_radLodWorker.hasInFlight())
        return false;
#ifndef HAVE_ZLIB
    return false;
#else
    if (chunkIndex >= _radChunks.size())
        return false;
    bool resident = false;
    uint64_t chunkOffset = 0;
    uint64_t chunkBytes = 0;
    {
        std::lock_guard<std::mutex> lock(_radStreamer.stateMutex());
        if (chunkIndex >= _radChunks.size())
            return false;
        resident = _radChunks[chunkIndex].resident;
        chunkOffset = _radChunks[chunkIndex].offset;
        chunkBytes = _radChunks[chunkIndex].bytes;
    }
    if (resident)
        return true;
    if (_radFilename.empty())
        return false;
    std::ifstream file(_radFilename, std::ios::binary);
    if (!file)
        return false;
    file.seekg(_radChunkDataBase + (std::streamoff)chunkOffset);
    std::vector<uint8_t> chunk((size_t)chunkBytes);
    file.read(reinterpret_cast<char*>(chunk.data()), chunk.size());
    if ((size_t)file.gcount() != chunk.size())
        return false;
    uint32_t loadGeneration = 0;
    {
        std::lock_guard<std::mutex> lock(_radStreamer.stateMutex());
        if (chunkIndex < _radChunks.size())
            loadGeneration = _radChunks[chunkIndex].loadGeneration;
    }
    return applyRadChunk(chunkIndex, chunk, loadGeneration);
#endif
}

bool SparkOsgDrawable::applyRadChunkPacked(unsigned int chunkIndex, const std::vector<uint32_t>& packedData, uint32_t loadGeneration)
{
    if (_radLodWorker.hasInFlight())
        return false;
    // WARNING: This function does NOT establish CPU-side LOD/mapping state
    // (_radChunkSlots, _radSlotToChunk, _radLocalToGlobal, _radGlobalToLocal,
    //  _radChildCounts, _radChildStarts, _radLodSizes, _bounds, chunkInfo.base/count/priority).
    // Use applyRadChunk() instead, which handles both GPU upload and CPU mapping.
    // This function is retained for potential future direct-packed-load optimization
    // but must NOT be called until CPU-side state is properly populated.
    std::cerr << "SparkOsgDrawable: applyRadChunkPacked is incomplete - CPU-side LOD mapping not established. "
              << "Use applyRadChunk() instead. chunkIndex=" << chunkIndex << std::endl;
    return false;
#if 0 // Disabled until CPU-side state is properly populated
    {
        std::lock_guard<std::mutex> lock(_radStreamer.stateMutex());
        if (chunkIndex >= _radChunks.size()) return false;
        RadChunkInfo& chunkInfo = _radChunks[chunkIndex];
        if (chunkInfo.loadGeneration != loadGeneration || chunkInfo.resident) {
            return false;
        }
    }

    if (!_gpuResources || !_gpuResources->hasPagedPool()) {
        std::cerr << "SparkOsgDrawable: applyRadChunkPacked failed. Paged Pool not initialized!" << std::endl;
        return false;
    }

    int freePage = _gpuResources->allocatePage();
    if (freePage < 0) {
        // Pool is full, try to evict LRU
        int evictedChunk = -1;
        {
            std::lock_guard<std::mutex> lock(_radStreamer.stateMutex());
            unsigned int oldestFrame = 0xFFFFFFFF;
            for (unsigned int i = 1; i < _radChunks.size(); ++i) {
                if (_radChunks[i].resident && _radChunks[i].pageIndex >= 0) {
                    bool protectedChunk = false;
                    for (int pc : _radProtectedChunks) {
                        if (pc == i) { protectedChunk = true; break; }
                    }
                    if (!protectedChunk && _radChunks[i].lastTouchedFrame < oldestFrame) {
                        oldestFrame = _radChunks[i].lastTouchedFrame;
                        evictedChunk = i;
                    }
                }
            }
        }
        if (evictedChunk >= 0) {
            evictRadChunk(evictedChunk);
            freePage = _gpuResources->allocatePage();
        }

        if (freePage < 0) {
            std::cerr << "SparkOsgDrawable: Paged Pool full. Cannot apply packed chunk " << chunkIndex << std::endl;
            return false;
        }
    }

    // Number of splats is derived from packed data size
    int texelsPerSplat = _gpuResources->packedTexelsPerSplat();
    int count = packedData.size() / (texelsPerSplat * 4);

    // Update GPU memory directly
    _gpuResources->updatePage(freePage, packedData.data(), count);

    {
        std::lock_guard<std::mutex> lock(_radStreamer.stateMutex());
        if (chunkIndex >= _radChunks.size()) {
            _gpuResources->freePage(freePage);
            return false;
        }
        RadChunkInfo& chunkInfo = _radChunks[chunkIndex];
        if (chunkInfo.loadGeneration != loadGeneration) {
            _gpuResources->freePage(freePage);
            return false;
        }
        
        // Update logical mapping
        chunkInfo.pageIndex = freePage;
        chunkInfo.resident = true;
        chunkInfo.queued = false;
        chunkInfo.lastTouchedFrame = _radFrameNumber;
        _radLoadedChunkCount++;
    }

    return true;
#endif // Disabled applyRadChunkPacked
}

bool SparkOsgDrawable::applyRadChunk(unsigned int chunkIndex, const std::vector<uint8_t>& chunk, uint32_t loadGeneration,
                                    const SparkRad::PreparedChunk* prepared)
{
    if (_radLodWorker.hasInFlight())
        return false;
#ifndef HAVE_ZLIB
    return false;
#else
    if (chunkIndex >= _radChunks.size())
        return false;
    RadChunkInfo expectedInfo;
    {
        std::lock_guard<std::mutex> lock(_radStreamer.stateMutex());
        if (chunkIndex >= _radChunks.size())
            return false;
        expectedInfo = _radChunks[chunkIndex];
        if (expectedInfo.loadGeneration != loadGeneration)
            return false;
        if (expectedInfo.resident)
            return true;
    }

    auto rejectPreparation = [&]() {
        std::lock_guard<std::mutex> lock(_radStreamer.stateMutex());
        if (chunkIndex < _radChunks.size() && !_radChunks[chunkIndex].resident &&
            _radChunks[chunkIndex].loadGeneration == loadGeneration)
            _radChunks[chunkIndex].queued = false;
        _radStreamer.notify();
        return false;
    };
    const bool usePagedPool = (_gpuResources && _gpuResources->hasPagedPool());
    const bool preparedInBackground = prepared != nullptr;
    SparkRad::PreparedChunk localPreparation;
    if (!prepared) {
        SparkRad::PrepareOptions options;
        options.totalPointCount = _radTotalPointCount;
        if (usePagedPool) {
            options.maxPageSplats = _gpuResources->pageSize();
            options.splatEncoding = (int)_gpuResources->splatEncoding();
        }
        options.timingsEnabled = SparkFrameMetrics::enabled();
        if (!SparkRad::prepareChunk(chunk, expectedInfo.count, expectedInfo.base, options, localPreparation))
            return rejectPreparation();
        prepared = &localPreparation;
    }
    const int count = prepared->count;
    const int base = prepared->base;
    const auto& decodedSplats = prepared->splats;
    const int encoding = usePagedPool ? (int)_gpuResources->splatEncoding() : 0;
    const size_t expectedWords = usePagedPool
        ? size_t(std::max(0, count)) * size_t(_gpuResources->packedTexelsPerSplat()) * 4u : 0u;
    // Preparation is immutable, but publication can be followed by a canceled
    // view or a reconfigured renderer. Validate before allocating/evicting pages.
    if (_radTotalPointCount <= 0 || base < 0 || count <= 0 || base >= _radTotalPointCount ||
        count > _radTotalPointCount - base || decodedSplats.size() != size_t(count) ||
        prepared->encoding != encoding || (usePagedPool &&
        (count > _gpuResources->pageSize() || prepared->packedData.size() != expectedWords)) ||
        (!prepared->shData.empty() && prepared->shData.size() != size_t(count) * 48u))
        return rejectPreparation();
    if (SparkFrameMetrics::enabled()) {
        auto& timings = SparkFrameMetrics::state().milliseconds;
        if (preparedInBackground) {
            timings[SparkFrameMetrics::PrepareWorker] += prepared->decodeMilliseconds +
                prepared->convertMilliseconds + prepared->packMilliseconds;
        } else {
            timings[SparkFrameMetrics::Decode] += prepared->decodeMilliseconds;
            timings[SparkFrameMetrics::Convert] += prepared->convertMilliseconds;
            timings[SparkFrameMetrics::Pack] += prepared->packMilliseconds;
        }
    }
    SparkFrameMetrics::Scope applyProfile(SparkFrameMetrics::PageApply);
    {
        std::lock_guard<std::mutex> lock(_radStreamer.stateMutex());
        if (chunkIndex >= _radChunks.size())
            return false;
        _radChunks[chunkIndex].properties = prepared->properties;
        _radChunks[chunkIndex].payloadBytes = (uint64_t)prepared->payloadBytes;
    }

    const bool useCovSplats = isEnvEnabled("OSG_3DGS_ENABLE_COVSPLATS");

    if (usePagedPool) {
        // These disjoint CPU stages are nested within PageApply. Image writes
        // update CPU mirrors and queue uploads; they do not time GPU execution.
        int freePage = -1;
        {
            SparkFrameMetrics::Scope allocationProfile(SparkFrameMetrics::PageAllocate);
            freePage = _gpuResources->allocatePage();
            if (freePage < 0) {
                // Pool is full, try to evict LRU
                int evictedChunk = -1;
                {
                    std::lock_guard<std::mutex> lock(_radStreamer.stateMutex());
                    unsigned int oldestFrame = 0xFFFFFFFF;
                    for (unsigned int i = 1; i < _radChunks.size(); ++i) {
                        if (_radChunks[i].resident && _radChunks[i].pageIndex >= 0) {
                            bool protectedChunk = false;
                            for (int pc : _radProtectedChunks) {
                                if (pc == i) { protectedChunk = true; break; }
                            }
                            if (!protectedChunk && _radChunks[i].lastTouchedFrame < oldestFrame) {
                                oldestFrame = _radChunks[i].lastTouchedFrame;
                                evictedChunk = i;
                            }
                        }
                    }
                }
                if (evictedChunk >= 0) {
                    evictRadChunk(evictedChunk);
                    freePage = _gpuResources->allocatePage();
                }

                if (freePage < 0) {
                    std::lock_guard<std::mutex> lock(_radStreamer.stateMutex());
                    if (chunkIndex < _radChunks.size() && !_radChunks[chunkIndex].resident)
                        _radChunks[chunkIndex].queued = false;
                    _radStreamer.notify();
                    return false;
                }
            }
        }

        {
            SparkFrameMetrics::Scope imagesProfile(SparkFrameMetrics::PageImages);
            _gpuResources->updatePage(freePage, prepared->packedData.data(), count);
            if (!prepared->shData.empty())
                _gpuResources->updatePageSH(freePage, prepared->shData.data(), count);

            if (useCovSplats) {
                std::vector<MI_GaussianPoint> covPoints(count);
                for (int i = 0; i < count; ++i)
                    covPoints[i] = decodedSplats[i].point;
                _gpuResources->updatePageCovSplat(freePage, covPoints.data(), count);
            }
        }

        {
            SparkFrameMetrics::Scope cpuProfile(SparkFrameMetrics::PageCpu);
            std::vector<int> slots;
            slots.reserve(count);
            std::lock_guard<std::mutex> lock(_radStreamer.stateMutex());
            if (chunkIndex >= _radChunks.size()) {
                if (useCovSplats)
                    _gpuResources->clearPageCovSplat(freePage);
                _gpuResources->freePage(freePage);
                return false;
            }
            RadChunkInfo& chunkInfo = _radChunks[chunkIndex];
            if (chunkInfo.loadGeneration != loadGeneration) {
                // Was canceled during load, rollback
                if (useCovSplats)
                    _gpuResources->clearPageCovSplat(freePage);
                _gpuResources->freePage(freePage);
                return false;
            }

            int baseLocalIndex = freePage * _gpuResources->pageSize();
            int maxLocalIndex = baseLocalIndex + count;
            // Initialize only a gap preceding this page. New page points can
            // be copy-constructed directly instead of clearing their SH and
            // geometry first and then overwriting ~15 MiB per full page.
            if (baseLocalIndex > (int)_points->size())
                _points->resize(baseLocalIndex);
            if (maxLocalIndex > (int)_radChildCounts.size()) {
                _radChildCounts.resize(maxLocalIndex, 0);
                _radChildStarts.resize(maxLocalIndex, 0);
                _radLocalToGlobal.resize(maxLocalIndex, 0);
                _radLodSizes.resize(maxLocalIndex, 0.0f);
                _radLodNodes.resize(maxLocalIndex);
                _radSlotToChunk.resize(maxLocalIndex, -1);
            }

            for (int i = 0; i < count; ++i) {
                int slot = baseLocalIndex + i;
                const SparkRad::DecodedSplat& decodedSplat = decodedSplats[i];
                if (slot < (int)_points->size())
                    (*_points)[slot] = decodedSplat.point;
                else
                    _points->push_back(decodedSplat.point);
                _radChildCounts[slot] = decodedSplat.childCount;
                _radChildStarts[slot] = decodedSplat.childStart;
                uint32_t globalIndex = (uint32_t)(base + i);
                _radLodSizes[slot] = decodedSplat.lodSize;
                _radLodNodes[slot] = { decodedSplat.point.position, decodedSplat.point.scale,
                    decodedSplat.point.color.a(), decodedSplat.lodSize };
                _radLocalToGlobal[slot] = globalIndex;
                if (globalIndex < _radGlobalToLocal.size())
                    _radGlobalToLocal[globalIndex] = slot;
                _radSlotToChunk[slot] = (int)chunkIndex;
                _bounds.expandBy(decodedSplat.point.position);
                slots.push_back(slot);
            }

            chunkInfo.resident = true;
            chunkInfo.queued = false;
            chunkInfo.base = base;
            chunkInfo.count = count;
            chunkInfo.pageIndex = freePage;
            chunkInfo.priority = std::max(chunkInfo.priority, 0.0f);
            chunkInfo.lastTouchedFrame = _radFrameNumber;

            if (_radChunkSlots.size() <= chunkIndex) {
                _radChunkSlots.resize(chunkIndex + 1);
            }
            // Transfer the page slot list instead of allocating and copying
            // up to 65k integers a second time on every resident page.
            _radChunkSlots[chunkIndex] = std::move(slots);
            ++_radLoadedChunkCount;
        }

        {
            SparkFrameMetrics::Scope positionsProfile(SparkFrameMetrics::PageSortPositions);
            _sortPass->updatePositionRange(_points, freePage * _gpuResources->pageSize(), freePage * _gpuResources->pageSize() + count);
        }

        ++_radResidencyRevision;
        // This page was free; only this chunk's slots became mapped.
        _radDrawProtectedPages.noteChunkMappingChange((int)chunkIndex, _radResidencyRevision);
        _firstRadLOD = true;
        return true;
    }

    // --- Legacy Vector Path ---
    int maxResident = getEnvInt("OSG_3DGS_RAD_MAX_RESIDENT", DEFAULT_RAD_MAX_RESIDENT, 65536, 8388608);
    if (_radResidentCapacity <= 0)
        _radResidentCapacity = maxResident;
    {
        std::lock_guard<std::mutex> lock(_radStreamer.stateMutex());
        if (_radChunkSlots.size() < _radChunks.size())
            _radChunkSlots.resize(_radChunks.size());
    }
    int availableSlots = ensureRadFreeSlots(count, _radProtectedChunks);
    if (availableSlots < count) {
        std::lock_guard<std::mutex> lock(_radStreamer.stateMutex());
        if (chunkIndex < _radChunks.size() && !_radChunks[chunkIndex].resident)
            _radChunks[chunkIndex].queued = false;
        _radStreamer.notify();
        return false;
    }
    int emitCount = count;
    std::vector<int> slots;
    slots.reserve(emitCount);
    {
        std::lock_guard<std::mutex> lock(_radStreamer.stateMutex());
        for (int i = 0; i < emitCount; ++i) {
            int slot = -1;
            if (!_radFreeSlots.empty()) {
                slot = _radFreeSlots.back();
                _radFreeSlots.pop_back();
            } else {
                slot = (int)_points->size();
                _points->push_back(MI_GaussianPoint());
            }
            if (slot >= (int)_radChildCounts.size()) {
                _radChildCounts.resize(slot + 1, 0);
                _radChildStarts.resize(slot + 1, 0);
                _radLocalToGlobal.resize(slot + 1, 0);
                _radLodSizes.resize(slot + 1, 0.0f);
                _radLodNodes.resize(slot + 1);
            }
            if (slot >= (int)_radSlotToChunk.size())
                _radSlotToChunk.resize(slot + 1, -1);
            _radSlotToChunk[slot] = (int)chunkIndex;
            markRadDirtySlots(slot, slot + 1);
            slots.push_back(slot);
        }
        for (int i = 0; i < emitCount; ++i) {
            int slot = slots[i];
            const SparkRad::DecodedSplat& decodedSplat = decodedSplats[i];
            const MI_GaussianPoint& g = decodedSplat.point;
            _radChildCounts[slot] = decodedSplat.childCount;
            _radChildStarts[slot] = decodedSplat.childStart;
            uint32_t globalIndex = (uint32_t)(base + i);
            _radLodSizes[slot] = decodedSplat.lodSize;
            _radLodNodes[slot] = { decodedSplat.point.position, decodedSplat.point.scale,
                decodedSplat.point.color.a(), decodedSplat.lodSize };
            _radLocalToGlobal[slot] = globalIndex;
            if (globalIndex < _radGlobalToLocal.size())
                _radGlobalToLocal[globalIndex] = slot;
            _bounds.expandBy(g.position);
            (*_points)[slot] = g;
        }
    }

    {
        std::lock_guard<std::mutex> lock(_radStreamer.stateMutex());
        if (chunkIndex >= _radChunks.size()) {
            for (int slot : slots) {
                if (slot >= 0)
                    _radFreeSlots.push_back(slot);
            }
            return false;
        }
        RadChunkInfo& chunkInfo = _radChunks[chunkIndex];
        if (chunkInfo.loadGeneration != loadGeneration) {
            for (int i = 0; i < (int)slots.size(); ++i) {
                int slot = slots[i];
                if (slot < 0)
                    continue;
                uint32_t globalIndex = (uint32_t)(base + i);
                if (globalIndex < _radGlobalToLocal.size() && _radGlobalToLocal[globalIndex] == slot)
                    _radGlobalToLocal[globalIndex] = -1;
                if (slot < (int)_radSlotToChunk.size())
                    _radSlotToChunk[slot] = -1;
                if (slot < (int)_radLocalToGlobal.size())
                    _radLocalToGlobal[slot] = 0;
                if (slot < (int)_radChildCounts.size())
                    _radChildCounts[slot] = 0;
                if (slot < (int)_radChildStarts.size())
                    _radChildStarts[slot] = 0;
                if (slot < (int)_radLodSizes.size())
                    _radLodSizes[slot] = 0.0f;
                if (slot < (int)_points->size()) {
                    MI_GaussianPoint emptyPoint;
                    emptyPoint.color.set(0.0f, 0.0f, 0.0f, 0.0f);
                    (*_points)[slot] = emptyPoint;
                    markRadDirtySlots(slot, slot + 1);
                }
                _radFreeSlots.push_back(slot);
            }
            _radStreamer.notify();
            return false;
        }
        if (chunkInfo.resident) {
            for (int i = 0; i < (int)slots.size(); ++i) {
                int slot = slots[i];
                if (slot < 0)
                    continue;
                uint32_t globalIndex = (uint32_t)(base + i);
                if (globalIndex < _radGlobalToLocal.size() && _radGlobalToLocal[globalIndex] == slot)
                    _radGlobalToLocal[globalIndex] = -1;
                if (slot < (int)_radSlotToChunk.size())
                    _radSlotToChunk[slot] = -1;
                if (slot < (int)_radLocalToGlobal.size())
                    _radLocalToGlobal[slot] = 0;
                if (slot < (int)_radChildCounts.size())
                    _radChildCounts[slot] = 0;
                if (slot < (int)_radChildStarts.size())
                    _radChildStarts[slot] = 0;
                if (slot < (int)_radLodSizes.size())
                    _radLodSizes[slot] = 0.0f;
                if (slot < (int)_points->size()) {
                    MI_GaussianPoint emptyPoint;
                    emptyPoint.color.set(0.0f, 0.0f, 0.0f, 0.0f);
                    (*_points)[slot] = emptyPoint;
                    markRadDirtySlots(slot, slot + 1);
                }
                _radFreeSlots.push_back(slot);
            }
            chunkInfo.queued = false;
            _radStreamer.notify();
            ++_radResidencyRevision;
            return true;
        }
        chunkInfo.resident = true;
        chunkInfo.queued = false;
        chunkInfo.base = base;
        chunkInfo.count = emitCount;
        chunkInfo.priority = std::max(chunkInfo.priority, 0.0f);
        chunkInfo.lastTouchedFrame = _radFrameNumber;
    }
    _radStreamer.notify();
    _radChunkSlots[chunkIndex] = std::move(slots);
    ++_radLoadedChunkCount;
    ++_radResidencyRevision;
    flushRadDirtySlots();
    _firstRadLOD = true;
    std::cout << "Loaded RAD chunk " << chunkIndex << " (" << _radLoadedChunkCount << "/" << _radTotalChunkCount << ") residentPoints=" << _points->size() << std::endl;
    return true;
#endif
}

bool SparkOsgDrawable::evictRadChunk(unsigned int chunkIndex)
{
    if (_radLodWorker.hasInFlight())
        return false;
    if (chunkIndex == 0 || chunkIndex >= _radChunks.size())
        return false;
    
    // Fast path for Spark Paged Pool Mode
    if (_gpuResources && _gpuResources->hasPagedPool()) {
        std::lock_guard<std::mutex> lock(_radStreamer.stateMutex());
        RadChunkInfo& chunkInfo = _radChunks[chunkIndex];
        if (!chunkInfo.resident) return false;

        int pageIndex = chunkInfo.pageIndex;
        if (pageIndex >= 0) {
            if (_gpuResources->covSplatTexture())
                _gpuResources->clearPageCovSplat(pageIndex);
            _gpuResources->freePage(pageIndex);
        }

        std::uint64_t skippedClearBytes = 0;
        if (chunkIndex < _radChunkSlots.size()) {
            for (int slot : _radChunkSlots[chunkIndex]) {
                if (slot < (int)_radLocalToGlobal.size()) {
                    uint32_t globalIndex = _radLocalToGlobal[slot];
                    if (globalIndex < _radGlobalToLocal.size() && _radGlobalToLocal[globalIndex] == slot)
                        _radGlobalToLocal[globalIndex] = -1;
                    _radLocalToGlobal[slot] = 0;
                }
                if (slot < (int)_radSlotToChunk.size())
                    _radSlotToChunk[slot] = -1;
                if (slot < (int)_radChildCounts.size())
                    _radChildCounts[slot] = 0;
                if (slot < (int)_radChildStarts.size())
                    _radChildStarts[slot] = 0;
                if (slot < (int)_radLodSizes.size())
                    _radLodSizes[slot] = 0.0f;
                // Like the packed GPU page and sort-position snapshot, an
                // unmapped CPU slot has no valid payload. Active/display page
                // protection keeps live indices out of eviction; apply replaces
                // every new valid slot before publishing residency. Avoid
                // clearing ~15 MiB only to overwrite it on the next page load.
                if (slot >= 0 && slot < (int)_points->size()) {
                    if (_radClearEvictedPoints) {
                        MI_GaussianPoint emptyPoint;
                        emptyPoint.color.set(0.0f, 0.0f, 0.0f, 0.0f);
                        (*_points)[slot] = emptyPoint;
                    } else {
                        skippedClearBytes += sizeof(MI_GaussianPoint);
                    }
                }
            }
            _radChunkSlots[chunkIndex].clear();
        }

        chunkInfo.resident = false;
        chunkInfo.pageIndex = -1;
        chunkInfo.loadGeneration++;
        _radLoadedChunkCount--;
        _radEvictedChunkCount++;
        ++_radResidencyRevision;
        // Notify eviction separately from the subsequent reuse of this page.
        _radDrawProtectedPages.noteChunkMappingChange((int)chunkIndex, _radResidencyRevision);
        if (SparkFrameMetrics::enabled())
            std::cout << "RAD page eviction: chunk=" << chunkIndex
                      << ", skippedClearBytes=" << skippedClearBytes << std::endl;
        return true;
    }

    // --- Legacy Vector Path ---
    if (_radChunkSlots.size() <= chunkIndex)
        return false;
    {
        std::lock_guard<std::mutex> queueLock(_radStreamer.mutex());
        std::lock_guard<std::mutex> stateLock(_radStreamer.stateMutex());
        if (chunkIndex >= _radChunks.size() || !_radChunks[chunkIndex].resident)
            return false;
        RadChunkInfo& chunk = _radChunks[chunkIndex];
        chunk.resident = false;
        chunk.queued = false;
        ++chunk.loadGeneration;
        chunk.priority = 0.0f;
        chunk.lastTouchedFrame = 0;
        _radStreamer.pendingChunks().erase(std::remove(_radStreamer.pendingChunks().begin(), _radStreamer.pendingChunks().end(), chunkIndex), _radStreamer.pendingChunks().end());
        _radStreamer.rawChunkQueue().erase(std::remove_if(_radStreamer.rawChunkQueue().begin(), _radStreamer.rawChunkQueue().end(),
            [chunkIndex](const RadRawChunk& raw) { return raw.chunkIndex == chunkIndex; }),
            _radStreamer.rawChunkQueue().end());

        // Clean up CPU-side mapping state under the same lock
        std::unordered_set<int> freeSlotSet(_radFreeSlots.begin(), _radFreeSlots.end());
        std::unordered_set<int> evictingSlots;
        evictingSlots.reserve(_radChunkSlots[chunkIndex].size());
        for (int slot : _radChunkSlots[chunkIndex]) {
            if (slot >= 0 && freeSlotSet.count(slot) == 0)
                evictingSlots.insert(slot);
        }

        for (int slot : evictingSlots) {
            if (slot < (int)_radLocalToGlobal.size()) {
                uint32_t globalIndex = _radLocalToGlobal[slot];
                if (globalIndex < _radGlobalToLocal.size() && _radGlobalToLocal[globalIndex] == slot)
                    _radGlobalToLocal[globalIndex] = -1;
                _radLocalToGlobal[slot] = 0;
            }
            if (slot < (int)_radSlotToChunk.size())
                _radSlotToChunk[slot] = -1;
            if (slot < (int)_radChildCounts.size())
                _radChildCounts[slot] = 0;
            if (slot < (int)_radChildStarts.size())
                _radChildStarts[slot] = 0;
            if (slot < (int)_radLodSizes.size())
                _radLodSizes[slot] = 0.0f;
            if (slot < (int)_points->size()) {
                MI_GaussianPoint emptyPoint;
                emptyPoint.color.set(0.0f, 0.0f, 0.0f, 0.0f);
                (*_points)[slot] = emptyPoint;
            }
            markRadDirtySlots(slot, slot + 1);
            _radFreeSlots.push_back(slot);
        }
        _radChunkSlots[chunkIndex].clear();
        _radLoadedChunkCount = std::max(0, _radLoadedChunkCount - 1);
        ++_radEvictedChunkCount;
        ++_radResidencyRevision;
    }
    _radStreamer.notify();
    flushRadDirtySlots();
    _firstRadLOD = true;
    return true;
}

int SparkOsgDrawable::ensureRadFreeSlots(int requiredSlots, const std::vector<int>& protectedChunks)
{
    if (requiredSlots <= 0)
        return (int)_radFreeSlots.size();
    int availableSlots = (int)_radFreeSlots.size() + std::max(0, _radResidentCapacity - (int)_points->size());
    if (availableSlots >= requiredSlots)
        return availableSlots;

    // Build protected chunk set for fast lookup
    std::unordered_set<int> protectedSet(protectedChunks.begin(), protectedChunks.end());

    // Phase 1: Evict from freeable chunks (not in active set, sorted by LRU)
    for (uint32_t chunkIndex : _radFreeableChunks) {
        if (chunkIndex == 0 || chunkIndex >= _radChunks.size())
            continue;
        if (protectedSet.count((int)chunkIndex))
            continue;
        bool resident = false;
        {
            std::lock_guard<std::mutex> lock(_radStreamer.stateMutex());
            resident = _radChunks[chunkIndex].resident;
        }
        if (!resident)
            continue;
        if (evictRadChunk(chunkIndex)) {
            _firstRadLOD = true;
            availableSlots = (int)_radFreeSlots.size() + std::max(0, _radResidentCapacity - (int)_points->size());
            if (availableSlots >= requiredSlots)
                return availableSlots;
        }
    }

    // Phase 2: Evict from LRU order (oldest first) - includes active but low-priority chunks
    for (uint32_t chunkIndex : _radLruOrder) {
        if (chunkIndex == 0 || chunkIndex >= _radChunks.size())
            continue;
        if (protectedSet.count((int)chunkIndex))
            continue;
        bool resident = false;
        unsigned int lastTouched = 0;
        {
            std::lock_guard<std::mutex> lock(_radStreamer.stateMutex());
            if (chunkIndex < _radChunks.size()) {
                resident = _radChunks[chunkIndex].resident;
                lastTouched = _radChunks[chunkIndex].lastTouchedFrame;
            }
        }
        if (!resident)
            continue;
        // Grace period: don't evict chunks touched in the last 2 frames
        const unsigned int TOUCH_GRACE_FRAMES = 2;
        if (lastTouched != 0 && _radFrameNumber >= lastTouched && _radFrameNumber - lastTouched <= TOUCH_GRACE_FRAMES)
            continue;
        if (evictRadChunk(chunkIndex)) {
            _firstRadLOD = true;
            availableSlots = (int)_radFreeSlots.size() + std::max(0, _radResidentCapacity - (int)_points->size());
            if (availableSlots >= requiredSlots)
                return availableSlots;
        }
    }

    return availableSlots;
}

void SparkOsgDrawable::queueRadChunk(uint32_t chunkIndex, float priority, int& queuedCount)
{
    if (chunkIndex >= _radChunks.size())
        return;

    std::lock_guard<std::mutex> queueLock(_radStreamer.mutex());
    std::lock_guard<std::mutex> stateLock(_radStreamer.stateMutex());
    RadChunkInfo& chunk = _radChunks[chunkIndex];
    float oldPriority = chunk.priority;
    chunk.priority = std::max(chunk.priority, priority);
    if (chunk.resident)
        return;

    const size_t MAX_PENDING_CHUNKS = 96;
    auto insertByPriority = [&](uint32_t value) {
        auto insertPos = _radStreamer.pendingChunks().begin();
        for (; insertPos != _radStreamer.pendingChunks().end(); ++insertPos) {
            uint32_t pending = *insertPos;
            float pendingPriority = pending < _radChunks.size() ? _radChunks[pending].priority : 0.0f;
            if (chunk.priority > pendingPriority)
                break;
        }
        _radStreamer.pendingChunks().insert(insertPos, value);
    };

    if (chunk.queued) {
        if (chunk.priority > oldPriority) {
            auto existing = std::find(_radStreamer.pendingChunks().begin(), _radStreamer.pendingChunks().end(), chunkIndex);
            if (existing != _radStreamer.pendingChunks().end()) {
                _radStreamer.pendingChunks().erase(existing);
                insertByPriority(chunkIndex);
            }
        }
        return;
    }

    if (_radStreamer.pendingChunks().size() >= MAX_PENDING_CHUNKS) {
        auto lowest = _radStreamer.pendingChunks().end();
        float lowestPriority = priority;
        for (auto it = _radStreamer.pendingChunks().begin(); it != _radStreamer.pendingChunks().end(); ++it) {
            uint32_t pending = *it;
            float pendingPriority = pending < _radChunks.size() ? _radChunks[pending].priority : 0.0f;
            if (lowest == _radStreamer.pendingChunks().end() || pendingPriority < lowestPriority) {
                lowest = it;
                lowestPriority = pendingPriority;
            }
        }
        if (lowest == _radStreamer.pendingChunks().end() || chunk.priority <= lowestPriority)
            return;
        uint32_t dropped = *lowest;
        if (dropped < _radChunks.size())
            _radChunks[dropped].queued = false;
        _radStreamer.pendingChunks().erase(lowest);
    }

    chunk.queued = true;
    insertByPriority(chunkIndex);
    ++queuedCount;
}

bool SparkOsgDrawable::updateRadNativeLOD(const osg::Matrix& viewMatrix, const osg::Camera* cam)
{
    // Observe projection/viewport changes before the settled-cache and in-flight
    // early returns. FOV changes and window resizes need a new selection even
    // while the camera pose is stationary and no streamed bytes are waiting.
    SparkRadProjectionState::Stamp projectionStamp;
    if (cam) {
        projectionStamp.matrix = cam->getProjectionMatrix();
        if (cam->getViewport()) {
            projectionStamp.x = cam->getViewport()->x();
            projectionStamp.y = cam->getViewport()->y();
            projectionStamp.width = cam->getViewport()->width();
            projectionStamp.height = cam->getViewport()->height();
            projectionStamp.valid = std::isfinite(projectionStamp.x) && std::isfinite(projectionStamp.y) &&
                std::isfinite(projectionStamp.width) && std::isfinite(projectionStamp.height) &&
                projectionStamp.width > 0.0 && projectionStamp.height > 0.0;
        }
    }
    const bool projectionChanged = _radProjection.update(projectionStamp);
    if (projectionChanged) {
        _radLodWorker.requestCancel();
        _firstRadLOD = true;
        _radPendingLod.clear();
        _radLoadedSinceSelection = 0;
        _radStreamWait.clear();
    }
    const bool traceRadLod = isEnvEnabled("OSG_3DGS_RAD_LOD_TRACE");
    if (!_isRadFile || !_points || _points->empty() || _radLodSizes.empty() || _radChunkSize <= 0) {
        if (traceRadLod) {
            std::cout << "RAD LOD trace: skip invalid state"
                      << ", isRad=" << (_isRadFile ? 1 : 0)
                      << ", points=" << (_points ? (int)_points->size() : -1)
                      << ", lodSizes=" << (int)_radLodSizes.size()
                      << ", chunkSize=" << _radChunkSize
                      << std::endl;
        }
        return false;
    }

    auto getRadQueueSizes = [&]() {
        std::pair<size_t, size_t> sizes(0, 0);
        std::lock_guard<std::mutex> lock(_radStreamer.mutex());
        sizes.first = _radStreamer.pendingChunks().size();
        sizes.second = _radStreamer.rawChunkQueue().size();
        return sizes;
    };

    const std::uint64_t previousQualityGeneration = _radQuality.generation();
    const double nowMs = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    if (_radRetargetOnStop) {
        _radLodRetarget.update(viewMatrix, nowMs);
        if (_radLodRetarget.shouldCancel(_radLodWorker.hasInFlight(), _radWorkerView)) {
            _radLodWorker.requestCancel();
            if (SparkFrameMetrics::enabled())
                std::cout << "RAD LOD retarget: settledMs=80" << std::endl;
        }
    }
    const int MAX_ACTIVE_RAD_SPLATS = _radQuality.update(viewMatrix, nowMs);
    const bool qualityChanged = _radQuality.generation() != previousQualityGeneration;
    if (qualityChanged) {
        // A stationary camera still needs a new traversal when its detail
        // budget changes; likewise movement must promptly restore its budget.
        _radLodWorker.requestCancel();
        _firstRadLOD = true;
        _radPendingLod.clear();
        _radLoadedSinceSelection = 0;
    }
    const bool forceRadLODTraceUpdate = isEnvEnabled("OSG_3DGS_RAD_LOD_FORCE_UPDATE");
    ++_radFrameNumber;
    const bool asynchronousLod = !isEnvEnabled("OSG_3DGS_RAD_SYNC_LOD");
    SparkLodSelector::Result selectorResult;
    osg::Matrix selectedView = viewMatrix;
    bool completedLod = false;
    bool completedForceCommit = false;
    float completedPixelScaleLimit = 0.0f;
    float completedViewportOutsideWeight = 1.0f;
    if (_radPendingLod.pending()) {
        SparkRadPendingLod::Candidate candidate;
        const SparkRadPendingLod::State state{ _radQuality.generation(),
            _radProjection.generation(), _radResidencyRevision };
        const auto status = _radPendingLod.poll(viewMatrix, state, _radFrameNumber, candidate);
        if (status == SparkRadPendingLod::Status::Waiting)
            return false;
        if (status == SparkRadPendingLod::Status::Ready) {
            selectorResult = std::move(candidate.result);
            selectedView = candidate.view;
            completedPixelScaleLimit = candidate.pixelScaleLimit;
            completedViewportOutsideWeight = candidate.viewportOutsideWeight;
            completedLod = true;
            completedForceCommit = true;
            _radStreamWait.clear();
        } else if (status == SparkRadPendingLod::Status::Invalidated) {
            _firstRadLOD = true;
        }
    }
    if (!completedLod && _radLodWorker.hasInFlight()) {
        double workerMilliseconds = 0.0;
        if (!_radLodWorker.tryTake(selectorResult, selectedView, workerMilliseconds))
            return false;
        _radLoadedSinceSelection = 0;
        if (selectorResult.canceled) {
            if (traceRadLod || SparkFrameMetrics::enabled())
                std::cout << "RAD LOD canceled: workerMs=" << workerMilliseconds << std::endl;
            _firstRadLOD = true;
            return false;
        }
        if (!_radQuality.accepts(_radWorkerQualityGeneration) ||
            !_radProjection.accepts(_radWorkerProjectionGeneration)) {
            // tryTake has released borrowed inputs, but the old selection must
            // not commit or pin pages after a budget or projection transition.
            // Pose-only changes deliberately do not reject every old result:
            // that would starve selection while continuously moving.
            _firstRadLOD = true;
            return false;
        }
        completedLod = true;
        completedForceCommit = _radWorkerForceCommit;
        completedPixelScaleLimit = _radWorkerPixelScaleLimit;
        completedViewportOutsideWeight = _radWorkerViewportOutsideWeight;
        _radStreamWait.clear();
        if (SparkFrameMetrics::enabled())
            SparkFrameMetrics::state().milliseconds[SparkFrameMetrics::LodWorker] = workerMilliseconds;
    }
    bool viewChanged = false;
    for (int row = 0; row < 4 && !viewChanged; ++row)
        for (int column = 0; column < 4; ++column)
            if (std::abs(_lastRadLODViewMatrix(row, column) - viewMatrix(row, column)) > 1.0e-3f) {
                viewChanged = true;
                break;
            }
    if (viewChanged || !_radQuality.refiningIdle())
        _radLoadedSinceSelection = 0;
    auto waitForSortHandoff = [&](bool capacityBlocked) {
        if (!_radWaitSortHandoff || completedLod)
            return false;
        SparkRadSortHandoffWait::State state;
        state.capacityBlocked = capacityBlocked;
        state.loadedSinceSelection = _radLoadedSinceSelection;
        state.gpuSort = _sortPass->useGpuSort();
        state.sortDisabled = _sortPass->sortDisabled();
        state.sortPending = _sortPass->hasPendingCpuSort(_activeSetVersion, viewMatrix, 2.0e-3f);
        state.activeVersion = _activeSetVersion;
        state.displayVersion = _radDisplayedActiveSetVersion;
        state.qualityChanged = qualityChanged;
        state.projectionChanged = projectionChanged;
        if (!SparkRadSortHandoffWait::shouldWait(viewMatrix, _lastRadLODViewMatrix, state))
            return false;
        if (++_radSortHandoffWaitFrames == 1 && (traceRadLod || SparkFrameMetrics::enabled()))
            std::cout << "RAD LOD wait for sort handoff: activeSet=" << _activeSetVersion
                      << ", displayedSet=" << _radDisplayedActiveSetVersion
                      << ", loaded=" << _radLoadedSinceSelection << std::endl;
        return true;
    };
    if (_radWaitSortHandoff) {
        const SparkRadStreamWait::State state{ _radQuality.generation(), _activeSetVersion,
            _radDisplayRevision, _radResidencyRevision };
        // Partial page application deliberately keeps _firstRadLOD set. Honor
        // this narrow display wait before that flag bypasses the ordinary
        // settled-view stream wait below; display revision changes wake it.
        if (waitForSortHandoff(_radStreamWait.blockedFor(viewMatrix, state)))
            return false;
        if (_radSortHandoffWaitFrames) {
            if (traceRadLod || SparkFrameMetrics::enabled())
                std::cout << "RAD LOD sort handoff resumed: activeSet=" << _activeSetVersion
                          << ", displayedSet=" << _radDisplayedActiveSetVersion
                          << ", loaded=" << _radLoadedSinceSelection
                          << ", waitFrames=" << _radSortHandoffWaitFrames << std::endl;
            _radSortHandoffWaitFrames = 0;
        }
    }
    if (!completedLod && !_firstRadLOD && !viewChanged &&
        !_radPendingLod.pending() && _radPrefetchedChunks.empty() && !forceRadLODTraceUpdate) {
        const SparkRadStreamWait::State state{ _radQuality.generation(), _activeSetVersion,
            _radDisplayRevision, _radResidencyRevision };
        if (_radStreamWait.blockedFor(viewMatrix, state))
            return false;
        std::lock_guard<std::mutex> lock(_radStreamer.mutex());
        if (_radStreamer.rawChunkQueue().empty())
            return false;
    }
    // Page protection is needed only when consuming a traversal or applying
    // streamed data. In a settled view this avoids scanning millions of active
    // and displayed indices on every frame, including during idle refinement.
    std::vector<unsigned char> protectedChunkFlags(_radChunks.size(), 0);
    auto protectChunk = [&](int chunkIndex) {
        if (chunkIndex < 0 || chunkIndex >= (int)protectedChunkFlags.size() || protectedChunkFlags[chunkIndex])
            return;
        protectedChunkFlags[chunkIndex] = 1;
        _radProtectedChunks.push_back(chunkIndex);
    };
    auto deduplicateProtectedChunks = [&]() {
        if (_radProtectedChunks.empty())
            return;
        std::sort(_radProtectedChunks.begin(), _radProtectedChunks.end());
        _radProtectedChunks.erase(std::unique(_radProtectedChunks.begin(), _radProtectedChunks.end()), _radProtectedChunks.end());
    };
    {
        SparkFrameMetrics::Scope profile(SparkFrameMetrics::Protect);
        _radProtectedChunks.clear();
        for (int chunkIndex : _radTraversalProtectedChunks)
            protectChunk(chunkIndex);
        // Keep the old sorted frontier resident until the replacement sort is
        // ready. Camera-only sorts still leave both membership versions intact.
        // This summary belongs to the exact committed membership and mapping.
        // A deferred result is accepted only while its residency stamp matches;
        // page application cannot overlap the worker or a completed result.
        const std::vector<int>* activePageSummary = _radActivePageSummaryValid &&
            _radActivePageSummaryVersion == _activeSetVersion &&
            _radActivePageSummaryResidency == _radResidencyRevision ? &_radActivePageSummary : nullptr;
        const auto summariesBefore = _radDrawProtectedPages.summaryUseCount();
        const auto& drawPages = _radDrawProtectedPages.update(_activeIndices, _depthIndex,
            _radSlotToChunk, _radChunks.size(), _activeSetVersion, _radResidencyRevision,
            _radDisplayedActiveSetVersion, activePageSummary);
        if (SparkFrameMetrics::enabled() && _radDrawProtectedPages.summaryUseCount() != summariesBefore)
            std::cout << "RAD active page summary: activeSet=" << _activeSetVersion
                      << ", pages=" << _radActivePageSummary.size()
                      << ", avoidedIndices=" << _activeIndices.size() << std::endl;
        for (int chunkIndex : drawPages)
            protectChunk(chunkIndex);
        deduplicateProtectedChunks();
    }

    int loadedChunksThisUpdate = 0;
    bool streamProgressBlocked = false;
    bool streamCapacityBlocked = false;
    int maxChunkLoadsPerUpdate = getEnvInt("OSG_3DGS_RAD_LOAD_CHUNKS_PER_UPDATE", 2, 0, 4);
    if (!_sortPass->useGpuSort() && !_sortPass->sortDisabled()) {
        const int activeCount = (int)_activeIndices.size();
        const bool refiningIdle = _radQuality.refiningIdle();
        if (!refiningIdle && (_lastSortMilliseconds >= 6.0 || activeCount >= 300000)) {
            maxChunkLoadsPerUpdate = std::min(maxChunkLoadsPerUpdate, 1);
        } else if (_lastSortMilliseconds >= 3.0 || activeCount >= 180000) {
            maxChunkLoadsPerUpdate = std::min(maxChunkLoadsPerUpdate, 2);
        }
    }
    if (!completedLod) {
        SparkFrameMetrics::Scope profile(SparkFrameMetrics::Stream);
        // Bound attempted work as well as successful uploads: corrupt or canceled
        // chunks must not let the streaming thread refill an unbounded decode loop.
        int attemptedChunksThisUpdate = 0;
        while (attemptedChunksThisUpdate < maxChunkLoadsPerUpdate) {
            RadRawChunk raw;
            {
                std::lock_guard<std::mutex> lock(_radStreamer.mutex());
                if (_radStreamer.rawChunkQueue().empty())
                    break;
                if (_gpuResources && _gpuResources->hasPagedPool() && _gpuResources->findFreePage() < 0) {
                    const RadRawChunk& next = _radStreamer.rawChunkQueue().front();
                    std::lock_guard<std::mutex> stateLock(_radStreamer.stateMutex());
                    if (next.chunkIndex < _radChunks.size() &&
                        !_radChunks[next.chunkIndex].resident &&
                        _radChunks[next.chunkIndex].loadGeneration == next.loadGeneration) {
                        bool canEvict = false;
                        for (size_t i = 1; i < _radChunks.size(); ++i) {
                            if (_radChunks[i].resident && _radChunks[i].pageIndex >= 0 && !protectedChunkFlags[i]) {
                                canEvict = true;
                                break;
                            }
                        }
                        // Leave compressed data queued until a page becomes usable.
                        // No eviction or allocation happens until decode succeeds.
                        if (!canEvict) {
                            streamProgressBlocked = true;
                            streamCapacityBlocked = true;
                            const SparkRadStreamWait::State state{ _radQuality.generation(), _activeSetVersion,
                                _radDisplayRevision, _radResidencyRevision };
                            _radStreamWait.park(viewMatrix, state);
                            std::cout << "RAD refinement waiting for cache capacity: active=" << _activeIndices.size()
                                      << ", residentChunks=" << _radLoadedChunkCount
                                      << ", budget=" << MAX_ACTIVE_RAD_SPLATS << std::endl;
                            break;
                        }
                    }
                }
                raw = std::move(_radStreamer.rawChunkQueue().front());
                _radStreamer.rawChunkQueue().erase(_radStreamer.rawChunkQueue().begin());
                _radStreamer.notify();
            }
            ++attemptedChunksThisUpdate;
            if (applyRadChunk(raw.chunkIndex, raw.bytes, raw.loadGeneration, raw.prepared.get())) {
                _radPrefetchedChunks.push_back(raw.chunkIndex);
                ++loadedChunksThisUpdate;
            } else {
                streamProgressBlocked = true;
            }
        }
        if (!_radPrefetchedChunks.empty()) {
            _firstRadLOD = true;
            _radPrefetchedChunks.clear();
        }
        _radLoadedSinceSelection += loadedChunksThisUpdate;
    }
    if (waitForSortHandoff(streamCapacityBlocked))
        return false;

    // Traverse once per small idle batch instead of after every two uploads.
    // The old complete draw frontier stays visible and per-frame upload work
    // remains bounded. Required pages (including missing ones) from the last
    // standard traversal protect each new page throughout this batch.
    // While the small ready queue is being refilled, keep rendering the previous
    // complete frontier instead of restarting LOD after every one or two pages.
    // No thread waits here. Failed/blocked uploads and view/budget/projection
    // changes immediately resume selection, including a full-cache repair.
    if (!completedLod && !viewChanged && !qualityChanged && !projectionChanged && _radQuality.refiningIdle() &&
        !streamProgressBlocked && _radLoadedSinceSelection > 0 && _radLoadedSinceSelection < _radIdleLoadBatchLimit &&
        _gpuResources && _gpuResources->hasPagedPool() &&
        !isEnvEnabled("OSG_3DGS_LOD_DYNAMIC_TRAVERSE")) {
        bool hasReadyRequestedChunk = false;
        {
            std::lock_guard<std::mutex> queueLock(_radStreamer.mutex());
            std::lock_guard<std::mutex> stateLock(_radStreamer.stateMutex());
            for (const RadRawChunk& raw : _radStreamer.rawChunkQueue()) {
                if (raw.chunkIndex >= _radChunks.size())
                    continue;
                const RadChunkInfo& chunk = _radChunks[raw.chunkIndex];
                if (chunk.queued && !chunk.resident && chunk.loadGeneration == raw.loadGeneration &&
                    protectedChunkFlags[raw.chunkIndex]) {
                    hasReadyRequestedChunk = true;
                    break;
                }
            }
        }
        if (hasReadyRequestedChunk || _radStreamer.hasPendingWork()) {
            if (traceRadLod) {
                std::cout << "RAD LOD trace: continue idle upload batch"
                          << ", loaded=" << _radLoadedSinceSelection
                          << ", limit=" << _radIdleLoadBatchLimit << std::endl;
            }
            return false;
        }
    }

    const bool gpuSortActive = _sortPass->useGpuSort();
    const bool forceRadLODCommit = completedLod ? completedForceCommit :
        (_firstRadLOD || (loadedChunksThisUpdate > 0 && !gpuSortActive));
    if (!completedLod && !_firstRadLOD) {
        if (!viewChanged && !_radPendingLod.pending() && !forceRadLODTraceUpdate) {
            if (traceRadLod) {
                std::pair<size_t, size_t> queues = getRadQueueSizes();
                std::cout << "RAD LOD trace: skip no view change"
                          << ", frame=" << _radFrameNumber
                          << ", active=" << (int)_activeIndices.size()
                          << ", loadedThisUpdate=" << loadedChunksThisUpdate
                          << ", loadedChunks=" << _radLoadedChunkCount << "/" << _radTotalChunkCount
                          << ", pending=" << queues.first
                          << ", rawQueue=" << queues.second
                          << std::endl;
            }
            return false;
        }
    }
    float pixelScaleLimit = 0.0018f;
    float tanHalfFovX = 0.0f, tanHalfFovY = 0.0f;
    if (projectionStamp.valid) {
        double fovy = 90.0, aspect = 1.0, zNear = 0.1, zFar = 10000.0;
        if (projectionStamp.matrix.getPerspective(fovy, aspect, zNear, zFar)) {
            const float tanY = std::tan(0.5f * (float)fovy * (float)M_PI / 180.0f);
            const float h = std::max(1.0f, (float)projectionStamp.height);
            pixelScaleLimit = (2.0f * tanY) / h;
            // The optional viewport weighting assumes a symmetric perspective
            // frustum. Other projection types retain the previous LOD policy.
            if (std::isfinite(tanY) && tanY > 0.0f && std::isfinite(aspect) && aspect > 0.0 &&
                std::abs(projectionStamp.matrix(2, 0)) < 1.0e-7 &&
                std::abs(projectionStamp.matrix(2, 1)) < 1.0e-7) {
                tanHalfFovX = tanY * (float)aspect;
                tanHalfFovY = tanY;
            }
        }
    }
    float lodLevel = getEnvFloat("OSG_3DGS_RAD_LOD_LEVEL", DEFAULT_RAD_LOD_LEVEL, 0.25f, 2.0f);
    float lodQuality = DEFAULT_RAD_LOD_QUALITY_AT_LEVEL * (DEFAULT_RAD_LOD_LEVEL / std::max(0.25f, lodLevel));
    lodQuality = std::max(0.01f, std::min(1.0f, lodQuality));
    if (isEnvSet("OSG_3DGS_RAD_LOD_QUALITY"))
        lodQuality = getEnvFloat("OSG_3DGS_RAD_LOD_QUALITY", lodQuality, 0.01f, 1.0f);
    pixelScaleLimit *= lodQuality;

    SparkLodSelector::Input selectorInput;
    selectorInput.points = _points;
    selectorInput.lodNodes = &_radLodNodes;
    selectorInput.useCompactLodRecords = !isEnvSet("OSG_3DGS_RAD_LOD_COMPACT_RECORDS") ||
        isEnvEnabled("OSG_3DGS_RAD_LOD_COMPACT_RECORDS");
    selectorInput.globalToLocal = &_radGlobalToLocal;
    selectorInput.slotToChunk = &_radSlotToChunk;
    selectorInput.chunks = &_radChunks;
    selectorInput.childCounts = &_radChildCounts;
    selectorInput.childStarts = &_radChildStarts;
    selectorInput.lodSizes = &_radLodSizes;
    selectorInput.viewMatrix = viewMatrix;
    selectorInput.tanHalfFovX = tanHalfFovX;
    selectorInput.tanHalfFovY = tanHalfFovY;
    selectorInput.viewportOutsideWeight = getEnvFloat("OSG_3DGS_RAD_VIEWPORT_WEIGHT", 0.1f, 0.01f, 1.0f);
    // These GPU paths can move or inflate a splat beyond its CPU support bound.
    // Match the material's disable-variable precedence when checking inflation.
    const bool lodInflate = isEnvSet("OSG_3DGS_DISABLE_LOD_INFLATE")
        ? !isEnvEnabled("OSG_3DGS_DISABLE_LOD_INFLATE") : isEnvEnabled("OSG_3DGS_LOD_INFLATE");
    if (lodInflate || isEnvEnabled("OSG_3DGS_ENABLE_SPLAT_EDIT") || isEnvEnabled("OSG_3DGS_ENABLE_SKINNING") ||
        getEnvFloat("OSG_3DGS_SUPPORT_RADIUS_SCALE", 1.0f, 0.1f, 2.0f) > 1.0f)
        selectorInput.viewportOutsideWeight = 1.0f;
    // These optional counts classify CPU centers, not exact visible ellipses.
    selectorInput.collectViewportStats = isEnvEnabled("OSG_3DGS_RAD_VIEWPORT_STATS");
    selectorInput.cachePagedPriorities = asynchronousLod &&
        (!isEnvSet("OSG_3DGS_RAD_PRIORITY_CACHE") || isEnvEnabled("OSG_3DGS_RAD_PRIORITY_CACHE"));
    selectorInput.cachePagedScoringInputs = asynchronousLod && isEnvEnabled("OSG_3DGS_RAD_SCORING_CACHE");
    selectorInput.useFourAryHeap = !isEnvSet("OSG_3DGS_RAD_FOUR_ARY_HEAP") ||
        isEnvEnabled("OSG_3DGS_RAD_FOUR_ARY_HEAP");
    selectorInput.bypassTerminalHeap = !isEnvSet("OSG_3DGS_RAD_BYPASS_TERMINAL_HEAP") ||
        isEnvEnabled("OSG_3DGS_RAD_BYPASS_TERMINAL_HEAP");
    selectorInput.resumePagedTraversal = asynchronousLod && isEnvEnabled("OSG_3DGS_RAD_RESUME_TRAVERSAL");
    selectorInput.useCanonicalTerminalRanges = isEnvEnabled("OSG_3DGS_RAD_TERMINAL_RANGES");
    selectorInput.collectActiveChunks = asynchronousLod &&
        (!isEnvSet("OSG_3DGS_RAD_ACTIVE_PAGE_SUMMARY") || isEnvEnabled("OSG_3DGS_RAD_ACTIVE_PAGE_SUMMARY"));
    selectorInput.dataEpoch = _radDataEpoch;
    selectorInput.options.maxActiveSplats = MAX_ACTIVE_RAD_SPLATS;
    selectorInput.options.totalPointCount = _radTotalPointCount;
    selectorInput.options.chunkSize = _radChunkSize;
    selectorInput.options.totalChunkCount = _radTotalChunkCount;
    selectorInput.options.pixelScaleLimit = pixelScaleLimit;
    if (_gpuResources && _gpuResources->hasPagedPool() && !isEnvEnabled("OSG_3DGS_LOD_DYNAMIC_TRAVERSE"))
        selectorInput.options.maxResidentChunks = _gpuResources->maxPages();
    // Foveated LOD parameters (Spark-style defaults)
    selectorInput.coneFov0 = getEnvFloat("OSG_3DGS_CONE_FOV0", 90.0f, 0.0f, 180.0f);
    selectorInput.coneFov = getEnvFloat("OSG_3DGS_CONE_FOV", 120.0f, 0.0f, 180.0f);
    selectorInput.coneFoveate = getEnvFloat("OSG_3DGS_CONE_FOVEATE", 0.4f, 0.0f, 1.0f);
    selectorInput.behindFoveate = getEnvFloat("OSG_3DGS_BEHIND_FOVEATE", 0.2f, 0.0f, 1.0f);
    // Ensure coneFov0 <= coneFov to avoid division by zero in LOD interpolation
    selectorInput.coneFov0 = std::min(selectorInput.coneFov0, selectorInput.coneFov);

    SparkLodSelector selector;
    SparkLodSelector::TraverseMode traverseMode = SparkLodSelector::TraverseMode::Standard;
    if (isEnvEnabled("OSG_3DGS_LOD_DYNAMIC_TRAVERSE"))
        traverseMode = SparkLodSelector::TraverseMode::Dynamic;
    if (completedLod) {
        // No page application/eviction has occurred since this task was
        // submitted, so every returned local index still names the same splat.
        pixelScaleLimit = completedPixelScaleLimit;
    } else {
        SparkFrameMetrics::Scope profile(SparkFrameMetrics::Select);
        std::lock_guard<std::mutex> lock(_radStreamer.stateMutex());
        if (_gpuResources && _gpuResources->hasPagedPool()) {
            const int pageSize = _gpuResources->pageSize();
            bool contiguousPages = pageSize > 0 && _radChunkSize > 0 && _radTotalPointCount > 0;
            // Both the preview loader and applyRadChunk populate each slot
            // vector consecutively. A compact preview can start inside a GPU
            // page, so only certify mappings aligned to that page's base.
            // Every resident logical range must also be canonical: an unusual
            // overlapping payload could otherwise overwrite another chunk's
            // interior globalToLocal entries while leaving its endpoints intact.
            for (size_t i = 0; contiguousPages && i < _radChunks.size(); ++i) {
                const RadChunkInfo& chunk = _radChunks[i];
                if (!chunk.resident)
                    continue;
                const int64_t globalBase = int64_t(i) * _radChunkSize;
                if (globalBase >= _radTotalPointCount || chunk.base != globalBase ||
                    chunk.count != std::min<int64_t>(_radChunkSize, int64_t(_radTotalPointCount) - globalBase) ||
                    chunk.pageIndex < 0 || chunk.count <= 0 || chunk.count > pageSize ||
                    i >= _radChunkSlots.size() || _radChunkSlots[i].size() != size_t(chunk.count)) {
                    contiguousPages = false;
                    break;
                }
                const int64_t base = int64_t(chunk.pageIndex) * pageSize;
                contiguousPages = _radChunkSlots[i].front() == base &&
                    _radChunkSlots[i].back() == base + chunk.count - 1;
            }
            if (contiguousPages)
                selectorInput.pagedPageSize = pageSize;
        }
        if (asynchronousLod) {
            // The worker snapshots chunk metadata. All other input vectors
            // remain frozen until tryTake above succeeds (or stop joins it).
            if (!_radLodWorker.submit(selectorInput, traverseMode))
                return false;
            _radWorkerView = viewMatrix;
            _radLoadedSinceSelection = 0;
            _radWorkerPixelScaleLimit = pixelScaleLimit;
            _radWorkerViewportOutsideWeight = selectorInput.viewportOutsideWeight;
            _radWorkerForceCommit = forceRadLODCommit;
            _radWorkerQualityGeneration = _radQuality.generation();
            _radWorkerProjectionGeneration = _radProjection.generation();
            return false;
        }
        selectorResult = selector.select(selectorInput, traverseMode);
        _radLoadedSinceSelection = 0;
        _radStreamWait.clear();
    }
    SparkFrameMetrics::Scope commitProfile(SparkFrameMetrics::LodCommit);
    bool selectedViewMatchesCurrent = true;
    for (int row = 0; row < 4 && selectedViewMatchesCurrent; ++row)
        for (int column = 0; column < 4; ++column)
            if (std::abs(selectedView(row, column) - viewMatrix(row, column)) > 1.0e-5) {
                selectedViewMatchesCurrent = false;
                break;
            }
    _lastRadLODViewMatrix = selectedView;
    _firstRadLOD = false;
    _lodScratchIndices = std::move(selectorResult.activeIndices);
    if (SparkFrameMetrics::enabled() && (selectorResult.priorityCacheHits || selectorResult.priorityCacheMisses))
        std::cout << "RAD LOD priority cache: hits=" << selectorResult.priorityCacheHits
                  << ", misses=" << selectorResult.priorityCacheMisses
                  << ", scoringBuilt=" << selectorResult.scoringCachePointsBuilt
                  << ", scoringBytes=" << selectorResult.scoringCacheBytes
                  << ", bypassedTerminals=" << selectorResult.bypassedTerminalCount << std::endl;
    if (SparkFrameMetrics::enabled() && selectorInput.resumePagedTraversal)
        std::cout << "RAD LOD checkpoint: prefix=" << selectorResult.checkpointPrefixItems
                  << ", saved=" << selectorResult.checkpointSavedItems
                  << ", resumed=" << selectorResult.checkpointResumedItems
                  << ", bytes=" << selectorResult.checkpointBytes << std::endl;
    if (SparkFrameMetrics::enabled() && selectorInput.useCanonicalTerminalRanges)
        std::cout << "RAD LOD terminal ranges: nodes=" << selectorResult.canonicalTerminalRangeNodes
                  << ", leaves=" << selectorResult.canonicalTerminalRangeLeaves << std::endl;
    if (selectorInput.collectViewportStats && selectorResult.viewportInsideCount +
        selectorResult.viewportOutsideCount + selectorResult.viewportBehindCount > 0) {
        std::cout << "RAD viewport centers: inside=" << selectorResult.viewportInsideCount
                  << ", outside=" << selectorResult.viewportOutsideCount
                  << ", behind=" << selectorResult.viewportBehindCount
                  << ", active=" << _lodScratchIndices.size()
                  << ", weight=" << (completedLod ? completedViewportOutsideWeight : selectorInput.viewportOutsideWeight)
                  << std::endl;
    }
    bool candidateComplete = selectorResult.candidateComplete;
    int radStopActiveCap = selectorResult.stopActiveCap;
    int radStopMissingChildren = selectorResult.stopMissingChildren;
    int radStopPixelLimit = selectorResult.stopPixelLimit;
    const bool boundedPaging = selectorInput.options.maxResidentChunks > 0 && !selectorResult.requiredChunks.empty();
    int radQueuedChunks = 0;
    float radMaxFrontierPixelScale = selectorResult.maxFrontierPixelScale;

    if (traceRadLod) {
        std::cout << "RAD LOD trace: selector"
                  << ", frame=" << _radFrameNumber
                  << ", scratch=" << (int)_lodScratchIndices.size()
                  << ", active=" << (int)_activeIndices.size()
                  << ", candidateComplete=" << (candidateComplete ? 1 : 0)
                  << ", missing=" << (int)selectorResult.missingChunks.size()
                  << ", touched=" << (int)selectorResult.touchedChunks.size()
                  << ", pixelLimit=" << pixelScaleLimit
                  << ", maxFrontier=" << radMaxFrontierPixelScale
                  << ", missingStops=" << radStopMissingChildren
                  << ", activeCapStops=" << radStopActiveCap
                  << ", residentCapStops=" << selectorResult.stopResidentCap
                  << ", requiredPages=" << selectorResult.requiredChunks.size()
                  << ", pixelStops=" << radStopPixelLimit
                  << ", loadedThisUpdate=" << loadedChunksThisUpdate
                  << ", loadedChunks=" << _radLoadedChunkCount << "/" << _radTotalChunkCount
                  << ", residentPoints=" << (int)_points->size()
                  << std::endl;
    }

    // Only the render thread changes residency. Snapshot once after uploads;
    // the streamer concurrently changes queue flags, not residency or slots.
    std::vector<unsigned char> residentChunks(_radChunks.size(), 0);
    {
        SparkFrameMetrics::Scope profile(SparkFrameMetrics::Residency);
        std::lock_guard<std::mutex> lock(_radStreamer.stateMutex());
        for (size_t i = 0; i < _radChunks.size(); ++i)
            residentChunks[i] = _radChunks[i].resident;
    }
    auto isResidentRadLocal = [&](int localIndex) {
        if (localIndex < 0 || localIndex >= (int)_points->size())
            return false;
        if (localIndex >= (int)_radSlotToChunk.size())
            return false;
        int chunkIndex = _radSlotToChunk[localIndex];
        if (chunkIndex < 0 || chunkIndex >= (int)_radChunks.size())
            return false;
        return residentChunks[chunkIndex] != 0;
    };

    auto clearStaleRadQueuedLoads = [&](const char* reason) {
        size_t pendingBefore = 0;
        size_t rawBefore = 0;
        size_t canceledQueued = 0;
        {
            std::lock_guard<std::mutex> lock(_radStreamer.mutex());
            std::lock_guard<std::mutex> stateLock(_radStreamer.stateMutex());
            pendingBefore = _radStreamer.pendingChunks().size();
            rawBefore = _radStreamer.rawChunkQueue().size();

            _radStreamer.pendingChunks().clear();
            _radStreamer.rawChunkQueue().clear();

            for (RadChunkInfo& chunk : _radChunks) {
                if (!chunk.resident && chunk.queued) {
                    chunk.queued = false;
                    ++chunk.loadGeneration;
                    ++canceledQueued;
                }
            }

            if (pendingBefore || rawBefore || canceledQueued)
                _radStreamer.notify();
        }

        if (traceRadLod && (pendingBefore || rawBefore || canceledQueued)) {
            std::cout << "RAD LOD trace: clear stale queued loads"
                      << ", reason=" << reason
                      << ", pendingBefore=" << pendingBefore
                      << ", rawBefore=" << rawBefore
                      << ", canceledQueued=" << canceledQueued
                      << std::endl;
        }
    };

    for (const SparkLodSelector::ChunkPriority& touched : selectorResult.touchedChunks) {
        int chunkIndex = (int)touched.chunkIndex;
        if (chunkIndex < 0 || chunkIndex >= (int)_radChunks.size())
            continue;
        {
            std::lock_guard<std::mutex> lock(_radStreamer.stateMutex());
            if (chunkIndex < (int)_radChunks.size()) {
                RadChunkInfo& chunk = _radChunks[chunkIndex];
                chunk.priority = std::max(chunk.priority, touched.priority);
                chunk.lastTouchedFrame = _radFrameNumber;
            }
        }
        _radProtectedChunks.push_back(chunkIndex);
    }

    // Only legacy vector allocation consumes these lists. The paged allocator
    // selects its victim directly from chunk metadata and protected pages.
    if (!_gpuResources || !_gpuResources->hasPagedPool()) {
        // Build set of chunks referenced by active indices (needed chunks)
        std::vector<unsigned char> neededChunks(_radChunks.size(), 0);
        for (int localIndex : _lodScratchIndices) {
            if (localIndex < 0 || localIndex >= (int)_radSlotToChunk.size())
                continue;
            int chunkIndex = _radSlotToChunk[localIndex];
            if (chunkIndex > 0 && chunkIndex < (int)_radChunks.size())
                neededChunks[chunkIndex] = 1;
        }
        // Also protect chunks that are in the current active set
        for (int localIndex : _activeIndices) {
            if (localIndex < 0 || localIndex >= (int)_radSlotToChunk.size())
                continue;
            int chunkIndex = _radSlotToChunk[localIndex];
            if (chunkIndex > 0 && chunkIndex < (int)_radChunks.size())
                neededChunks[chunkIndex] = 1;
        }

        // Build LRU order: all resident chunks sorted by lastTouchedFrame (oldest first)
        std::vector<std::pair<unsigned int, uint32_t>> lruEntries; // (lastTouchedFrame, chunkIndex)
        {
            std::lock_guard<std::mutex> stateLock(_radStreamer.stateMutex());
            for (unsigned int i = 1; i < _radChunks.size(); ++i) {
                if (!_radChunks[i].resident)
                    continue;
                lruEntries.push_back({ _radChunks[i].lastTouchedFrame, i });
            }
        }
        std::sort(lruEntries.begin(), lruEntries.end());
        _radLruOrder.clear();
        _radFreeableChunks.clear();
        for (const auto& entry : lruEntries) {
            _radLruOrder.push_back(entry.second);
            if (!neededChunks[entry.second])
                _radFreeableChunks.push_back(entry.second);
        }
    }

    // A complete frontier with no missing children already meets this view's
    // budget. Old requests from coarser traversals would otherwise keep loading
    // pages and invalidate the active set long after refinement has finished.
    const bool clearedResidentFrontier = selectorResult.candidateComplete &&
        !_lodScratchIndices.empty() && selectorResult.missingChunks.empty();
    if (clearedResidentFrontier)
        clearStaleRadQueuedLoads("current frontier resident");

    bool queuedChunk = false;
    if (boundedPaging && candidateComplete) {
        // Missing dependencies count against the same physical page budget as
        // resident ones. Drop old-view requests so those admitted pages can
        // replace lower-priority pages after this complete frontier is staged.
        std::vector<std::pair<uint32_t, float>> required;
        required.reserve(selectorResult.requiredChunks.size());
        for (const auto& page : selectorResult.requiredChunks)
            required.emplace_back(page.chunkIndex, page.priority);
        radQueuedChunks = _radStreamer.setRequiredChunks(required);
        queuedChunk = radQueuedChunks > 0;
    } else {
        for (const SparkLodSelector::ChunkPriority& missing : selectorResult.missingChunks) {
            int beforeQueued = radQueuedChunks;
            queueRadChunk(missing.chunkIndex, missing.priority, radQueuedChunks);
            queuedChunk = queuedChunk || (radQueuedChunks != beforeQueued);
        }
    }
    if (queuedChunk)
        _radStreamer.notify();
    if (_lodScratchIndices.empty() && !selectorResult.missingChunks.empty()) {
        if (traceRadLod) {
            std::pair<size_t, size_t> queues = getRadQueueSizes();
            std::cout << "RAD LOD trace: skip empty candidate waiting for chunks"
                      << ", missing=" << (int)selectorResult.missingChunks.size()
                      << ", queued=" << radQueuedChunks
                      << ", pending=" << queues.first
                      << ", rawQueue=" << queues.second
                      << std::endl;
        }
        return false;
    }

    deduplicateProtectedChunks();
    if (!_lodScratchIndices.empty()) {
        // Carry only this traversal's dependencies into the next update. The
        // combined protection set also contains the previous traversal and the
        // current draw lists; retaining that union would pin old pages forever.
        // Active/display indices are protected independently on every update.
        _radTraversalProtectedChunks.clear();
        const auto& dependencies = boundedPaging ? selectorResult.requiredChunks : selectorResult.touchedChunks;
        _radTraversalProtectedChunks.reserve(dependencies.size());
        for (const SparkLodSelector::ChunkPriority& touched : dependencies) {
            if (touched.chunkIndex < _radChunks.size())
                _radTraversalProtectedChunks.push_back((int)touched.chunkIndex);
        }
    }

    bool changed = _lodScratchIndices != _activeIndices;
    if (changed) {
        const int defaultStableFrames = gpuSortActive ? 5 : 4;
        const int RAD_LOD_STABLE_FRAME_COUNT = getEnvInt("OSG_3DGS_RAD_ACTIVE_STABLE_FRAMES", defaultStableFrames, 1, 12);
        const int RAD_LOD_MIN_CHANGE_ABS = getEnvInt("OSG_3DGS_RAD_ACTIVE_MIN_CHANGE_ABS", 8192, 0, 131072);
        const float defaultMinChangeFrac = gpuSortActive ? 0.08f : 0.04f;
        const float RAD_LOD_MIN_CHANGE_FRAC = getEnvFloat("OSG_3DGS_RAD_ACTIVE_MIN_CHANGE_FRAC", defaultMinChangeFrac, 0.0f, 0.25f);
        int changedEntries = std::abs((int)_lodScratchIndices.size() - (int)_activeIndices.size());
        int compareCount = std::min((int)_lodScratchIndices.size(), (int)_activeIndices.size());
        for (int i = 0; i < compareCount; ++i) {
            if (_lodScratchIndices[i] != _activeIndices[i])
                ++changedEntries;
        }
        int currentCount = std::max(1, (int)_activeIndices.size());
        int materialChangeThreshold = std::max(RAD_LOD_MIN_CHANGE_ABS,
            (int)std::ceil(RAD_LOD_MIN_CHANGE_FRAC * (float)currentCount));
        float changeRatio = (float)changedEntries / (float)std::max(1, std::max((int)_lodScratchIndices.size(), (int)_activeIndices.size()));
        bool enoughChange = changedEntries >= materialChangeThreshold;
        bool activeCapLimitedResidentComplete =
            !candidateComplete &&
            radStopActiveCap > 0 &&
            radStopMissingChildren == 0 &&
            selectorResult.missingChunks.empty() &&
            !_lodScratchIndices.empty();
        if (activeCapLimitedResidentComplete) {
            SparkFrameMetrics::Scope profile(SparkFrameMetrics::Residency);
            for (int localIndex : _lodScratchIndices) {
                if (!isResidentRadLocal(localIndex)) {
                    activeCapLimitedResidentComplete = false;
                    break;
                }
            }
        }
        bool candidateCommitReady = candidateComplete || activeCapLimitedResidentComplete;
        const bool ordinaryCommit = candidateCommitReady && (forceRadLODCommit || enoughChange);
        bool forceRepairCommit = _activeIndices.empty();
        // A complete replacement already being committed needs no repair test
        // of the old set. Keep that test for incomplete/deferred candidates.
        if (!ordinaryCommit && !forceRepairCommit) {
            SparkFrameMetrics::Scope profile(SparkFrameMetrics::Residency);
            for (int localIndex : _activeIndices) {
                if (!isResidentRadLocal(localIndex)) {
                    forceRepairCommit = true;
                    break;
                }
            }
        }
        bool commitImmediately = forceRepairCommit || ordinaryCommit;
        if (!candidateCommitReady && !forceRepairCommit) {
            if (traceRadLod) {
                std::pair<size_t, size_t> queues = getRadQueueSizes();
                std::cout << "RAD LOD trace: skip incomplete candidate"
                          << ", scratch=" << (int)_lodScratchIndices.size()
                          << ", active=" << (int)_activeIndices.size()
                          << ", missing=" << (int)selectorResult.missingChunks.size()
                          << ", queued=" << radQueuedChunks
                          << ", pending=" << queues.first
                          << ", rawQueue=" << queues.second
                          << ", missingStops=" << radStopMissingChildren
                          << ", activeCapStops=" << radStopActiveCap
                          << ", pixelStops=" << radStopPixelLimit
                          << ", activeCapResidentComplete=" << (activeCapLimitedResidentComplete ? 1 : 0)
                          << std::endl;
            }
            _radPendingLod.clear();
            return false;
        }
        if (!commitImmediately && RAD_LOD_STABLE_FRAME_COUNT > 1) {
            SparkRadPendingLod::Candidate candidate;
            selectorResult.activeIndices = std::move(_lodScratchIndices);
            candidate.result = std::move(selectorResult);
            candidate.view = selectedView;
            candidate.pixelScaleLimit = pixelScaleLimit;
            candidate.viewportOutsideWeight = completedLod
                ? completedViewportOutsideWeight : selectorInput.viewportOutsideWeight;
            const SparkRadPendingLod::State state{ _radQuality.generation(),
                _radProjection.generation(), _radResidencyRevision };
            if (!_radPendingLod.defer(std::move(candidate), viewMatrix, state,
                    _radFrameNumber, RAD_LOD_STABLE_FRAME_COUNT)) {
                _firstRadLOD = true;
            }
            if (traceRadLod)
                std::cout << "RAD LOD trace: defer completed candidate"
                          << ", renderFrames=" << RAD_LOD_STABLE_FRAME_COUNT
                          << ", retained=" << (_radPendingLod.pending() ? 1 : 0)
                          << ", changedEntries=" << changedEntries
                          << ", threshold=" << materialChangeThreshold << std::endl;
            return false;
        }
        _activeIndices = _lodScratchIndices;
        _pendingLODIndices.clear();
        _radPendingLod.clear();
        ++_activeSetVersion;
        _radActivePageSummaryValid = selectorResult.activeChunksValid;
        _radActivePageSummary = std::move(selectorResult.activeChunks);
        _radActivePageSummaryVersion = _activeSetVersion;
        _radActivePageSummaryResidency = _radResidencyRevision;
        _sortPass->noteActiveSetChanged();
        _lodLevel = ((int)_activeIndices.size() >= _radTotalPointCount) ? 0 : 1;
        // Update display buffer active set version so swapDisplayResult can match
        _sortPass->updateDisplayActiveSetVersion(_activeSetVersion);
        if (_sortPass->useGpuSort() || _sortPass->sortDisabled()) {
            std::vector<int> previousDepthIndex;
            previousDepthIndex.swap(_depthIndex);
            _depthIndex = _activeIndices;
            _drawActiveCount = (int)_depthIndex.size();
            _radDisplayedActiveSetVersion = _activeSetVersion;
            ++_radDisplayRevision;
            if (_sortPass->useGpuSort())
                updateActiveIndexBuffer();
            if (_gpuResources->indexTexture() && _gpuResources->indexTexture()->getImage())
                updateIndexChangedRanges(_depthIndex, previousDepthIndex);
        }
        // CPU sorting presents this complete active set only after its matching
        // result arrives. Appending unsorted new nodes to surviving old nodes
        // temporarily reverses alpha order and causes large visible flashes.
        // Keep the old complete display, its instance count and page protection.
        size_t pendingChunks = 0;
        size_t rawChunks = 0;
        {
            std::lock_guard<std::mutex> lock(_radStreamer.mutex());
            pendingChunks = _radStreamer.pendingChunks().size();
            rawChunks = _radStreamer.rawChunkQueue().size();
        }
        std::cout << "RAD Spark-style LOD active=" << _activeIndices.size()
                  << ", activeSet=" << _activeSetVersion
                  << ", selectedViewMatchesCurrent=" << (selectedViewMatchesCurrent ? 1 : 0)
                  << ", displayed=" << _drawActiveCount
                  << ", awaitingSort=" << (_radDisplayedActiveSetVersion != _activeSetVersion ? 1 : 0)
                  << ", budget=" << MAX_ACTIVE_RAD_SPLATS
                  << ", residentCapStops=" << selectorResult.stopResidentCap
                  << ", requiredPages=" << selectorResult.requiredChunks.size()
                  << ", pixelLimit=" << pixelScaleLimit
                  << ", loadedChunks=" << _radLoadedChunkCount << "/" << _radTotalChunkCount
                  << ", residentPoints=" << _points->size()
                  << ", freeSlots=" << _radFreeSlots.size()
                  << ", pending=" << pendingChunks
                  << ", rawQueue=" << rawChunks
                  << ", loadedThisUpdate=" << loadedChunksThisUpdate
                  << ", evicted=" << _radEvictedChunkCount
                  << ", queued=" << radQueuedChunks
                  << ", missingStops=" << radStopMissingChildren
                  << ", activeCapStops=" << radStopActiveCap
                  << ", pixelStops=" << radStopPixelLimit
                  << ", maxFrontier=" << radMaxFrontierPixelScale
                  << ", changeRatio=" << changeRatio
                  << ", lastSortMs=" << _lastSortMilliseconds
                  << ", lastSortItems=" << _lastSortItems
                  << std::endl;
    } else {
        if (traceRadLod || SparkFrameMetrics::enabled()) {
            std::pair<size_t, size_t> queues = getRadQueueSizes();
            std::cout << "RAD LOD trace: unchanged"
                      << ", active=" << (int)_activeIndices.size()
                      << ", activeSet=" << _activeSetVersion
                      << ", budget=" << MAX_ACTIVE_RAD_SPLATS
                      << ", selectedViewMatchesCurrent=" << (selectedViewMatchesCurrent ? 1 : 0)
                      << ", awaitingSort=" << (_radDisplayedActiveSetVersion != _activeSetVersion ? 1 : 0)
                      << ", scratch=" << (int)_lodScratchIndices.size()
                      << ", candidateComplete=" << (candidateComplete ? 1 : 0)
                      << ", pixelLimit=" << pixelScaleLimit
                      << ", maxFrontier=" << radMaxFrontierPixelScale
                      << ", missingStops=" << radStopMissingChildren
                      << ", residentCapStops=" << selectorResult.stopResidentCap
                      << ", requiredPages=" << selectorResult.requiredChunks.size()
                      << ", loadedChunks=" << _radLoadedChunkCount
                      << ", activeCapStops=" << radStopActiveCap
                      << ", pixelStops=" << radStopPixelLimit
                      << ", pending=" << queues.first
                      << ", rawQueue=" << queues.second
                      << std::endl;
        }
        _pendingLODIndices.clear();
        _radPendingLod.clear();
    }
    const bool clearStaleQueuesAtActiveCap =
        getEnvInt("OSG_3DGS_RAD_CLEAR_STALE_QUEUE_ON_ACTIVE_CAP", 1, 0, 1) != 0;
    if (clearStaleQueuesAtActiveCap && !(clearedResidentFrontier && !queuedChunk) &&
        selectorResult.missingChunks.empty() &&
        radStopActiveCap > 0 &&
        radStopMissingChildren == 0 &&
        !_lodScratchIndices.empty()) {
        bool residentComplete = true;
        {
            SparkFrameMetrics::Scope profile(SparkFrameMetrics::Residency);
            for (int localIndex : _lodScratchIndices) {
                bool isResident = false;
                if (localIndex >= 0 && localIndex < (int)_radSlotToChunk.size()) {
                    int chk = _radSlotToChunk[localIndex];
                    if (chk > 0 && chk < (int)_radChunks.size()) {
                        isResident = residentChunks[chk] != 0;
                    }
                }
                if (!isResident) {
                    residentComplete = false;
                    break;
                }
            }
        }
        if (residentComplete)
            clearStaleRadQueuedLoads(changed ? "active-cap-commit" : "active-cap-unchanged");
    }

    return changed;
}
