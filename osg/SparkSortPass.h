#ifndef SPARK_SORT_PASS_H
#define SPARK_SORT_PASS_H

#include <osg/Matrix>
#include <osg/Vec3f>

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

struct MI_GaussianPoint;

class SparkSortPass
{
public:
    enum class CpuResultStatus
    {
        None,
        Stale,
        Ready
    };

    enum class SortMode
    {
        CpuOnly,      // CPU radix sort fallback / explicit override
        GpuPreferred, // GPU bitonic sort with CPU fallback
        Disabled      // No sorting (depth index = active index order)
    };

    SparkSortPass();
    ~SparkSortPass();

    void start();
    void stop();

    // Sort mode and capacity configuration
    void configureSort(int requestedActiveCapacity, bool isRadFile);
    SortMode sortMode() const { return _sortMode; }
    bool useGpuSort() const { return _sortMode == SortMode::GpuPreferred; }
    bool sortDisabled() const { return _sortMode == SortMode::Disabled; }
    int gpuSortCapacity() const { return _gpuSortCapacity; }
    int gpuSortPassCount() const { return _gpuSortPassCount; }
    void fallbackToCpu();

    // CPU sort interface
    void setPositions(const std::vector<MI_GaussianPoint>* points, int numPoints);
    // Preallocate resident positions without changing their valid range or
    // invalidating immutable snapshots held by queued/active sort requests.
    void reservePositionCapacity(int capacity);
    void updatePositionRange(const std::vector<MI_GaussianPoint>* points, int firstSlot, int endSlot);
    bool shouldSort(const osg::Matrix& currentView, bool firstSort, float epsilon) const;
    void setLastViewMatrix(const osg::Matrix& viewMatrix);
    void noteGpuSortDispatched(const osg::Matrix& viewMatrix);
    bool hasPendingCpuSort(unsigned int activeSetVersion, const osg::Matrix& viewMatrix, float epsilon) const;
    void invalidateCpuSort();
    unsigned int noteActiveSetChanged();
    unsigned int requestCpuSort(const osg::Matrix& modelView, const std::vector<int>& activeIndices, unsigned int activeSetVersion);
    CpuResultStatus fetchCpuResult(std::vector<int>& outIndices, unsigned int activeSetVersion, double& sortMilliseconds);
    unsigned int generation() const { return _generation; }

    // Sort radial mode: sort by radial distance instead of Z-depth
    bool sortRadial() const { return _sortRadial; }
    void setSortRadial(bool radial) { _sortRadial = radial; }

    // Sort throttle: minimum interval between sorts in milliseconds
    float minSortIntervalMs() const { return _minSortIntervalMs; }
    void setMinSortIntervalMs(float ms) { _minSortIntervalMs = ms; }

    // Double-buffered sort result access
    const std::vector<int>& displayIndices() const { return _displayIndices; }
    int displayCount() const { return _displayCount; }
    bool swapDisplayResult(double* sortMilliseconds = nullptr);  // Try to swap current sort result into display buffer
    void updateDisplayActiveSetVersion(unsigned int version) { _displayActiveSetVersion = version; }

private:
    friend class SparkSortPassTestAccess;
    static int nextPowerOfTwo(int value);

    void run();

    // Sort mode and capacity
    SortMode _sortMode = SortMode::CpuOnly;
    int _gpuSortCapacity = 1;
    int _gpuSortPassCount = 0;

    std::thread _thread;
    std::mutex _taskMutex;
    std::mutex _resultMutex;
    std::condition_variable _cv;
    std::atomic<bool> _running{false};
    std::atomic<bool> _hasTask{false};
    std::atomic<bool> _hasResult{false};

    unsigned int _generation = 0;
    // Changes only when resident data/active mappings become invalid. Camera
    // requests may supersede one another without invalidating usable indices.
    unsigned int _invalidationVersion = 0;
    unsigned int _lastRequestActiveSetVersion = 0;
    bool _cpuSortPending = false;
    unsigned int _cpuSortPendingActiveSetVersion = 0;
    osg::Matrix _lastViewMatrix;

    const std::vector<MI_GaussianPoint>* _points = nullptr;
    std::vector<osg::Vec3f> _positionSnapshot;
    uint64_t _positionRevision = 0;
    struct ActiveSortSnapshot {
        unsigned int activeSetVersion = 0;
        uint64_t positionRevision = 0;
        int positionCount = 0;
        // Keep original indices (including invalid entries) to preserve the
        // API for callers that change active indices without changing version.
        std::vector<int> indices;
        std::vector<osg::Vec3f> positions;
    };
    // Immutable snapshots can be shared by camera requests and the worker.
    // Streaming replaces the cache without changing an in-flight snapshot.
    std::shared_ptr<const ActiveSortSnapshot> _activeSortSnapshot;
    std::shared_ptr<const ActiveSortSnapshot> _taskSnapshot;
    int _numPoints = 0;
    osg::Matrix _taskModelView;
    unsigned int _taskGeneration = 0;
    unsigned int _taskActiveSetVersion = 0;
    unsigned int _taskInvalidationVersion = 0;
    unsigned int _resultGeneration = 0;
    unsigned int _resultActiveSetVersion = 0;
    unsigned int _resultInvalidationVersion = 0;
    double _resultSortMilliseconds = 0.0;

    std::vector<int> _resultIndices;
    std::vector<uint32_t> _keys;
    std::vector<int> _values;
    std::vector<uint32_t> _tempKeys;
    std::vector<int> _tempValues;
    std::vector<int> _sorted;
    std::array<int, 256> _radixCount{};

    // Double-buffered display indices (separate from sort result)
    std::vector<int> _displayIndices;  // Currently rendering
    int _displayCount = 0;
    unsigned int _displayActiveSetVersion = 0;

    // Sort radial mode
    bool _sortRadial = false;

    // Sort throttle
    float _minSortIntervalMs = 0.0f;
    mutable std::chrono::steady_clock::time_point _lastSortTime{};
};

#endif
