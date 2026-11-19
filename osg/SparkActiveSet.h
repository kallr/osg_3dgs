#ifndef SPARK_ACTIVE_SET_H
#define SPARK_ACTIVE_SET_H

#include <numeric>
#include <vector>

// Owns the non-RAD active/depth index state used by GaussianDrawObj.
// GaussianDrawObj keeps compatibility references during the first extraction slice.
class SparkActiveSet
{
public:
    void clear()
    {
        depthIndex.clear();
        activeIndices.clear();
        sortResultScratch.clear();
        lodScratchIndices.clear();
        pendingLODIndices.clear();
        drawActiveCount = 0;
        firstSort = true;
        activeSetVersion = 0;
        pendingLODStableFrames = 0;
        lastSortMilliseconds = 0.0;
        lastSortItems = 0.0;
        lodLevel = 0;
    }

    void reserveScratch(int count)
    {
        lodScratchIndices.reserve(count);
        pendingLODIndices.reserve(count);
    }

    void initializeSequential(int count)
    {
        depthIndex.resize(count);
        std::iota(depthIndex.begin(), depthIndex.end(), 0);
        activeIndices = depthIndex;
        drawActiveCount = (int)activeIndices.size();
    }

    void commitSortDisabled()
    {
        depthIndex = activeIndices;
        drawActiveCount = (int)depthIndex.size();
        firstSort = false;
    }

    void commitGpuSortDispatched()
    {
        drawActiveCount = (int)activeIndices.size();
        firstSort = false;
    }

    void noteCpuSortRequested()
    {
        firstSort = false;
    }

    bool stageLOD(const std::vector<int>& nextIndices, int stableFrameCount)
    {
        if (nextIndices == pendingLODIndices) {
            ++pendingLODStableFrames;
        } else {
            pendingLODIndices = nextIndices;
            pendingLODStableFrames = 1;
        }
        return pendingLODStableFrames >= stableFrameCount;
    }

    void clearPendingLOD()
    {
        pendingLODIndices.clear();
        pendingLODStableFrames = 0;
    }

    std::vector<int> commitPendingLOD(int originalPointCount)
    {
        std::vector<int> previousDepthIndex;
        previousDepthIndex.swap(depthIndex);
        activeIndices.swap(pendingLODIndices);
        depthIndex = activeIndices;
        drawActiveCount = (int)activeIndices.size();
        pendingLODIndices.clear();
        pendingLODStableFrames = 0;
        lodLevel = ((int)activeIndices.size() >= originalPointCount) ? 0 : 1;
        return previousDepthIndex;
    }

    void applyCpuSortResult(double sortMilliseconds)
    {
        depthIndex.swap(sortResultScratch);
        drawActiveCount = (int)depthIndex.size();
        lastSortMilliseconds = sortMilliseconds;
        lastSortItems = (double)depthIndex.size();
    }

    std::vector<int> applyCpuSortResultWithPrevious(double sortMilliseconds)
    {
        std::vector<int> previousDepthIndex;
        previousDepthIndex.swap(depthIndex);
        applyCpuSortResult(sortMilliseconds);
        return previousDepthIndex;
    }

    const std::vector<int>& getDepthIndex() const { return depthIndex; }
    std::vector<int>& getDepthIndex() { return depthIndex; }
    const std::vector<int>& getActiveIndices() const { return activeIndices; }
    std::vector<int>& getActiveIndices() { return activeIndices; }
    const std::vector<int>& getSortResultScratch() const { return sortResultScratch; }
    std::vector<int>& getSortResultScratch() { return sortResultScratch; }
    const std::vector<int>& getLodScratchIndices() const { return lodScratchIndices; }
    std::vector<int>& getLodScratchIndices() { return lodScratchIndices; }
    const std::vector<int>& getPendingLODIndices() const { return pendingLODIndices; }
    std::vector<int>& getPendingLODIndices() { return pendingLODIndices; }
    bool isFirstSort() const { return firstSort; }
    void setFirstSort(bool value) { firstSort = value; }
    int getPendingLODStableFrames() const { return pendingLODStableFrames; }
    void setPendingLODStableFrames(int value) { pendingLODStableFrames = value; }
    unsigned int getActiveSetVersion() const { return activeSetVersion; }
    void setActiveSetVersion(unsigned int value) { activeSetVersion = value; }
    double getLastSortMilliseconds() const { return lastSortMilliseconds; }
    void setLastSortMilliseconds(double value) { lastSortMilliseconds = value; }
    double getLastSortItems() const { return lastSortItems; }
    void setLastSortItems(double value) { lastSortItems = value; }
    int getLODLevel() const { return lodLevel; }
    void setLODLevel(int value) { lodLevel = value; }
    int getDrawActiveCount() const { return drawActiveCount; }
    void setDrawActiveCount(int value) { drawActiveCount = value; }

private:
    std::vector<int> depthIndex;
    std::vector<int> activeIndices;
    std::vector<int> sortResultScratch;
    std::vector<int> lodScratchIndices;
    std::vector<int> pendingLODIndices;

    bool firstSort = true;
    int pendingLODStableFrames = 0;
    unsigned int activeSetVersion = 0;
    double lastSortMilliseconds = 0.0;
    double lastSortItems = 0.0;
    int lodLevel = 0;
    int drawActiveCount = 0;
};

#endif
