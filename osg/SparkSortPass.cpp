#include "SparkSortPass.h"
#include "GaussianDrawObj.h"
#include "SparkFrameMetrics.h"

#include <algorithm>
#include <cfloat>
#include <chrono>
#include <iostream>
#include <cstdlib>

// Maximum GPU sort capacity in elements.
// Raised from 262144 to 524288 to accommodate OSG_3DGS_RAD_MAX_ACTIVE=400000
// without truncation.  Bitonic sort on 524288 elements requires 19 stages
// (2^19 = 524288), which is within practical GPU dispatch limits.
static const int MAX_GPU_SORT_CAPACITY = 524288;

int SparkSortPass::nextPowerOfTwo(int value)
{
    int power = 1;
    while (power < value)
        power <<= 1;
    return power;
}

static bool isEnvEnabled(const char* name)
{
    const char* val = std::getenv(name);
    return val && (std::atoi(val) != 0 || val[0] == '1' || val[0] == 'y' || val[0] == 'Y');
}

void SparkSortPass::configureSort(int requestedActiveCapacity, bool isRadFile)
{
    int requestedCapacity = nextPowerOfTwo(std::max(1, requestedActiveCapacity));
    int capacity = std::min(requestedCapacity, MAX_GPU_SORT_CAPACITY);
    _gpuSortCapacity = capacity;

    // Determine sort mode from environment variables
    bool disableSort = isRadFile && isEnvEnabled("OSG_3DGS_RAD_DISABLE_SORT");
    bool forceCpuSort = isRadFile && isEnvEnabled("OSG_3DGS_RAD_FORCE_CPU_SORT");
    bool enableGpuSort = false;

    if (isRadFile && isEnvEnabled("OSG_3DGS_RAD_ENABLE_GPU_SORT"))
        enableGpuSort = true;

    if (isRadFile && isEnvEnabled("OSG_3DGS_RAD_ENABLE_SORT"))
        disableSort = false;

    if (disableSort) {
        _sortMode = SortMode::Disabled;
    } else if (forceCpuSort) {
        _sortMode = SortMode::CpuOnly;
    } else if (enableGpuSort) {
        _sortMode = SortMode::GpuPreferred;
    } else {
        _sortMode = SortMode::CpuOnly;
    }

    // Pre-compute bitonic sort pass count for GPU path
    _gpuSortPassCount = 0;
    for (int stage = 2; stage <= _gpuSortCapacity; stage <<= 1)
        for (int pass = stage >> 1; pass > 0; pass >>= 1)
            ++_gpuSortPassCount;

    std::cout << "GPU sort: requestedCapacity=" << requestedCapacity
              << ", capacity=" << _gpuSortCapacity
              << ", passes=" << _gpuSortPassCount
              << ", mode=" << (_sortMode == SortMode::GpuPreferred ? "GpuPreferred" :
                               _sortMode == SortMode::Disabled ? "Disabled" : "CpuOnly")
              << (isRadFile ? " (RAD now defaults to CPU sort for stable rendering; set OSG_3DGS_RAD_ENABLE_GPU_SORT=1 to enable GPU sort, OSG_3DGS_RAD_FORCE_CPU_SORT=1 to force CPU sort, OSG_3DGS_RAD_DISABLE_SORT=1 to disable)" : "") << std::endl;
}

void SparkSortPass::fallbackToCpu()
{
    if (_sortMode == SortMode::GpuPreferred) {
        _sortMode = SortMode::CpuOnly;
        std::cout << "SparkSortPass: falling back from GpuPreferred to CpuOnly" << std::endl;
    }
}

SparkSortPass::SparkSortPass() {}

SparkSortPass::~SparkSortPass() { stop(); }

void SparkSortPass::start()
{
    _running = true;
    _thread = std::thread(&SparkSortPass::run, this);
}

void SparkSortPass::stop()
{
    _running = false;
    _cv.notify_one();
    if (_thread.joinable())
        _thread.join();
}

bool SparkSortPass::shouldSort(const osg::Matrix& currentView, bool firstSort, float epsilon) const
{
    if (firstSort)
        return true;

    // Sort throttle: skip if not enough time has elapsed since last sort
    if (_minSortIntervalMs > 0.0f) {
        auto now = std::chrono::steady_clock::now();
        float elapsedMs = std::chrono::duration<float, std::milli>(now - _lastSortTime).count();
        if (elapsedMs < _minSortIntervalMs)
            return false;
    }

    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            if (std::abs(_lastViewMatrix(i, j) - currentView(i, j)) > epsilon)
                return true;
    return false;
}

void SparkSortPass::setLastViewMatrix(const osg::Matrix& viewMatrix)
{
    _lastViewMatrix = viewMatrix;
}

void SparkSortPass::noteGpuSortDispatched(const osg::Matrix& viewMatrix)
{
    _lastViewMatrix = viewMatrix;
    _lastSortTime = std::chrono::steady_clock::now();
}

bool SparkSortPass::hasPendingCpuSort(unsigned int activeSetVersion, const osg::Matrix& viewMatrix, float epsilon) const
{
    if (!_cpuSortPending || _cpuSortPendingActiveSetVersion != activeSetVersion)
        return false;

    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            if (std::abs(_lastViewMatrix(i, j) - viewMatrix(i, j)) > epsilon)
                return false;
    return true;
}

void SparkSortPass::invalidateCpuSort()
{
    std::lock_guard<std::mutex> lock(_taskMutex);
    _activeSortSnapshot.reset();
    _cpuSortPending = false;
    ++_generation;
    ++_invalidationVersion;
}

unsigned int SparkSortPass::noteActiveSetChanged()
{
    invalidateCpuSort();
    return _generation;
}

unsigned int SparkSortPass::requestCpuSort(const osg::Matrix& modelView, const std::vector<int>& activeIndices, unsigned int activeSetVersion)
{
    SparkFrameMetrics::Scope profile(SparkFrameMetrics::SortRequest);
    _lastViewMatrix = modelView;
    _lastSortTime = std::chrono::steady_clock::now();
    ++_generation;
    _lastRequestActiveSetVersion = activeSetVersion;
    _cpuSortPending = true;
    _cpuSortPendingActiveSetVersion = activeSetVersion;
    unsigned int generation = _generation;

    {
        std::lock_guard<std::mutex> lock(_taskMutex);
        _taskModelView = modelView;
        const bool sameActiveVersion = _activeSortSnapshot &&
            _activeSortSnapshot->activeSetVersion == activeSetVersion;
        const bool changedIndices = sameActiveVersion && _activeSortSnapshot->indices != activeIndices;
        // Non-RAD LOD callers may reuse the external version for a new set.
        // The old ordering then refers to a different active set even though
        // fetchCpuResult's public version check would otherwise accept it.
        if (changedIndices)
            ++_invalidationVersion;
        if (!_activeSortSnapshot ||
            !sameActiveVersion ||
            _activeSortSnapshot->positionRevision != _positionRevision ||
            changedIndices) {
            auto snapshot = std::make_shared<ActiveSortSnapshot>();
            snapshot->activeSetVersion = activeSetVersion;
            snapshot->positionRevision = _positionRevision;
            snapshot->positionCount = (int)_positionSnapshot.size();
            snapshot->indices = activeIndices;
            snapshot->positions.reserve(activeIndices.size());
            for (int index : activeIndices) {
                snapshot->positions.push_back(index >= 0 && index < snapshot->positionCount
                    ? _positionSnapshot[index] : osg::Vec3f());
            }
            _activeSortSnapshot = std::move(snapshot);
        }
        _taskSnapshot = _activeSortSnapshot;
        _taskGeneration = generation;
        _taskActiveSetVersion = activeSetVersion;
        _taskInvalidationVersion = _invalidationVersion;
        _hasTask = true;
    }
    _cv.notify_one();
    return generation;
}

void SparkSortPass::setPositions(const std::vector<MI_GaussianPoint>* points, int numPoints)
{
    std::lock_guard<std::mutex> lock(_taskMutex);
    _points = points;
    _numPoints = numPoints;
    ++_positionRevision;
    _activeSortSnapshot.reset();
    _positionSnapshot.clear();
    if (points) {
        _positionSnapshot.reserve(points->size());
        for (const MI_GaussianPoint& point : *points)
            _positionSnapshot.push_back(point.position);
    }
    // Worker-owned scratch buffers are reserved by run(), never while a
    // previous immutable snapshot may still be sorting on the worker.
}

void SparkSortPass::reservePositionCapacity(int capacity)
{
    if (capacity <= 0)
        return;
    std::lock_guard<std::mutex> lock(_taskMutex);
    _positionSnapshot.reserve(static_cast<size_t>(capacity));
}

void SparkSortPass::updatePositionRange(const std::vector<MI_GaussianPoint>* points, int firstSlot, int endSlot)
{
    std::lock_guard<std::mutex> lock(_taskMutex);
    if (!points)
        return;
    firstSlot = std::max(0, firstSlot);
    endSlot = std::min(endSlot, (int)points->size());
    if (firstSlot >= endSlot)
        return;
    if ((int)_positionSnapshot.size() < endSlot)
        _positionSnapshot.resize(endSlot);
    for (int i = firstSlot; i < endSlot; ++i)
        _positionSnapshot[i] = (*points)[i].position;
    ++_positionRevision;
    _activeSortSnapshot.reset();
    _points = points;
    _numPoints = (int)points->size();
}

SparkSortPass::CpuResultStatus SparkSortPass::fetchCpuResult(std::vector<int>& outIndices, unsigned int activeSetVersion, double& sortMilliseconds)
{
    std::lock_guard<std::mutex> lock(_resultMutex);
    if (!_hasResult)
        return CpuResultStatus::None;

    unsigned int resultGeneration = _resultGeneration;
    unsigned int resultActiveSetVersion = _resultActiveSetVersion;
    sortMilliseconds = _resultSortMilliseconds;
    // A newer camera request does not change what these indices reference.
    // Present the completed ordering while the worker catches up, as Spark's
    // display/current accumulators do. Mapping changes remain strict barriers.
    if (resultActiveSetVersion != activeSetVersion || _resultInvalidationVersion != _invalidationVersion) {
        _hasResult = false;
        if (resultGeneration == _generation)
            _cpuSortPending = false;
        return CpuResultStatus::Stale;
    }

    outIndices.swap(_resultIndices);
    _hasResult = false;
    if (resultGeneration == _generation)
        _cpuSortPending = false;
    return CpuResultStatus::Ready;
}

bool SparkSortPass::swapDisplayResult(double* sortMilliseconds)
{
    std::vector<int> newIndices;
    double sortMs = 0.0;
    CpuResultStatus status = fetchCpuResult(newIndices, _displayActiveSetVersion, sortMs);
    if (status == CpuResultStatus::Ready) {
        _displayIndices.swap(newIndices);
        _displayCount = (int)_displayIndices.size();
        if (sortMilliseconds)
            *sortMilliseconds = sortMs;
        return true;
    }
    return false;
}

void SparkSortPass::run()
{
    while (_running)
    {
        {
            std::unique_lock<std::mutex> lock(_taskMutex);
            _cv.wait(lock, [this] { return _hasTask.load() || !_running.load(); });
        }

        if (!_running) break;
        if (!_hasTask) continue;

        osg::Matrix localMV;
        unsigned int localGeneration = 0;
        unsigned int localActiveSetVersion = 0;
        unsigned int localInvalidationVersion = 0;
        std::shared_ptr<const ActiveSortSnapshot> snapshot;
        {
            std::lock_guard<std::mutex> lock(_taskMutex);
            localMV = _taskModelView;
            snapshot = std::move(_taskSnapshot);
            localGeneration = _taskGeneration;
            localActiveSetVersion = _taskActiveSetVersion;
            localInvalidationVersion = _taskInvalidationVersion;
            _hasTask = false;
        }

        int localNum = snapshot ? (int)snapshot->indices.size() : 0;
        if (localNum <= 0) {
            {
                std::lock_guard<std::mutex> lock(_resultMutex);
                _resultIndices.clear();
                _resultGeneration = localGeneration;
                _resultActiveSetVersion = localActiveSetVersion;
                _resultInvalidationVersion = localInvalidationVersion;
                _resultSortMilliseconds = 0.0;
                _hasResult = true;
            }
            continue;
        }

        auto sortStart = std::chrono::steady_clock::now();
        _keys.clear();
        _values.clear();
        _keys.reserve(localNum);
        _values.reserve(localNum);

        for (int i = 0; i < localNum; ++i)
        {
            int splatIndex = snapshot->indices[i];
            if (splatIndex < 0 || splatIndex >= snapshot->positionCount)
                continue;
            const osg::Vec3f& p = snapshot->positions[i];
            float ez = (float)(p.x() * localMV(0, 2) + p.y() * localMV(1, 2) +
                                p.z() * localMV(2, 2) + localMV(3, 2));
            float d;
            if (_sortRadial) {
                // Radial distance from camera
                float ex = (float)(p.x() * localMV(0, 0) + p.y() * localMV(1, 0) +
                                    p.z() * localMV(2, 0) + localMV(3, 0));
                float ey = (float)(p.x() * localMV(0, 1) + p.y() * localMV(1, 1) +
                                    p.z() * localMV(2, 1) + localMV(3, 1));
                d = std::sqrt(ex * ex + ey * ey + ez * ez);
            } else {
                // Z-depth (distance along -Z)
                d = (ez > 0.0f) ? FLT_MAX : std::max(0.0f, -ez);
            }
            union { float f; uint32_t u; } un;
            un.f = d;
            // Nonnegative float keys preserve numeric order. Invert them so
            // stable ascending radix passes produce far-to-near alpha order.
            _keys.push_back(~un.u);
            _values.push_back(splatIndex);
        }

        localNum = (int)_values.size();
        if (localNum <= 0) {
            {
                std::lock_guard<std::mutex> lock(_resultMutex);
                _resultIndices.clear();
                _resultGeneration = localGeneration;
                _resultActiveSetVersion = localActiveSetVersion;
                _resultInvalidationVersion = localInvalidationVersion;
                _resultSortMilliseconds = 0.0;
                _hasResult = true;
            }
            continue;
        }

        const int RADIX_BITS = 8;
        const int RADIX_SIZE = 1 << RADIX_BITS;
        const int RADIX_MASK = RADIX_SIZE - 1;

        _tempKeys.resize(localNum);
        _tempValues.resize(localNum);

        for (int pass = 0; pass < 4; ++pass)
        {
            int shift = pass * RADIX_BITS;
            _radixCount.fill(0);

            for (int i = 0; i < localNum; ++i)
                ++_radixCount[(_keys[i] >> shift) & RADIX_MASK];

            for (int i = 1; i < RADIX_SIZE; ++i)
                _radixCount[i] += _radixCount[i - 1];

            for (int i = localNum - 1; i >= 0; --i)
            {
                int bucket = (_keys[i] >> shift) & RADIX_MASK;
                int dest = --_radixCount[bucket];
                _tempKeys[dest] = _keys[i];
                _tempValues[dest] = _values[i];
            }

            _keys.swap(_tempKeys);
            _values.swap(_tempValues);
        }

        _sorted.resize(localNum);
        for (int i = 0; i < localNum; ++i)
            _sorted[i] = _values[i]; // Back-to-front for premultiplied alpha quads.

        {
            auto sortEnd = std::chrono::steady_clock::now();
            double sortMilliseconds = std::chrono::duration<double, std::milli>(sortEnd - sortStart).count();
            std::lock_guard<std::mutex> lock(_resultMutex);
            _resultIndices.swap(_sorted);
            _resultGeneration = localGeneration;
            _resultActiveSetVersion = localActiveSetVersion;
            _resultInvalidationVersion = localInvalidationVersion;
            _resultSortMilliseconds = sortMilliseconds;
            _hasResult = true;
        }
    }
}
